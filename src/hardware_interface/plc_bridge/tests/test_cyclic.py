from dataclasses import replace
from pathlib import Path
import math
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import yaml

from fx5u_controller.cyclic import (
    CyclicWorker, FixedLadderSession, HallEstimator, STOP, Target, load_cyclic_config,
)
from fx5u_controller.exceptions import ConfigurationError
from fx5u_controller.open_loop import OpenLoopSwerveKinematics, VelocityCommand

ROOT = Path(__file__).resolve().parents[1]
CONFIG = load_cyclic_config(ROOT / "config/default.yaml")


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Driver:
    def __init__(self, clock=None):
        self.clock = clock or time.monotonic
        self.calls = []
        self.angles = [90] * 4
        self.hall = [0] * 4
        self.fail = set()

    def record(self, kind, head, values):
        self.calls.append((kind, head, tuple(values), self.clock(), threading.get_ident()))
        if (kind, head, tuple(values)) in self.fail:
            raise OSError(f"injected {kind} {head} {tuple(values)}")

    def connect(self):
        self.record("connect", "", ())

    def disconnect(self):
        self.record("disconnect", "", ())

    def write_bits(self, head, values):
        self.record("bits", head, values)

    def write_words(self, head, values):
        self.record("words", head, values)

    def read_words(self, head, count):
        self.record("read", head, (count,))
        return self.angles if head == "D1030" else self.hall


class CyclicTests(unittest.TestCase):
    def session(self, **overrides):
        clock = Clock()
        driver = Driver(clock)
        session = FixedLadderSession(replace(CONFIG, **overrides), driver, clock, clock.sleep)
        return session, driver, clock

    def test_order_map_and_trigger_hold_times(self):
        session, driver, clock = self.session()
        session.startup()
        calls = [(k, h, v) for k, h, v, *_ in driver.calls]
        self.assertEqual(calls[:6], [
            ("words", "D1000", STOP), ("bits", "M1010", (0,) * 4),
            ("bits", "M1000", (1,) * 4), ("bits", "M1020", (1,)),
            ("words", "D1020", (90,) * 4), ("bits", "M1010", (1,) * 4)])
        driver.calls.clear()
        for angle in (91, 92):
            session.cycle(Target((angle,) * 4, (600,) * 4, clock()))
        writes = [(k, h, v) for k, h, v, *_ in driver.calls if k != "read"]
        for base in (0, 4):
            self.assertEqual([(k, h) for k, h, _ in writes[base:base + 4]], [
                ("words", "D1000"), ("words", "D1020"),
                ("bits", "M1010"), ("bits", "M1010")])
        pulses = [(v[0], t) for k, h, v, t, _ in driver.calls if h == "M1010"]
        self.assertGreaterEqual(pulses[1][1] - pulses[0][1] + 1e-9, CONFIG.on_s)
        self.assertGreaterEqual(pulses[2][1] - pulses[1][1] + 1e-9, CONFIG.off_s)
        self.assertFalse(any(h in ("Y8", "M58", "M222", "M60") for _, h, *_ in driver.calls))

    def test_on_ack_failure_still_attempts_off(self):
        session, driver, _ = self.session()
        driver.fail.add(("bits", "M1010", (1,) * 4))
        with self.assertRaises(OSError):
            session.pulse()
        self.assertEqual(driver.calls[-1][:3], ("bits", "M1010", (0,) * 4))

    def test_off_failure_prevents_next_cycle_and_all_shutdown_steps_attempted(self):
        session, driver, clock = self.session(enable_mode="slmp")
        driver.fail.add(("bits", "M1010", (0,) * 4))
        with self.assertRaises(OSError):
            session.pulse()
        with self.assertRaises(RuntimeError):
            session.shutdown()
        self.assertEqual([h for _, h, *_ in driver.calls[-5:]],
                         ["D1000", "M1010", "Y8", "M1020", "M1000"])

    def test_shutdown_after_speed_stop_failure_still_brakes_and_disables_servo(self):
        session, driver, _ = self.session()
        driver.fail.add(("words", "D1000", STOP))
        with self.assertRaises(RuntimeError):
            session.shutdown()
        self.assertEqual(driver.calls[-1][:3], ("bits", "M1000", (0,) * 4))

    def test_expired_command_stops_without_retriggering_or_steering(self):
        session, driver, clock = self.session()
        session.startup()
        clock.sleep(CONFIG.command_timeout_s + 0.01)
        driver.calls.clear()
        result = session.cycle(Target((180,) * 4, (400,) * 4, 0))
        self.assertEqual(result["applied_raw"], STOP)
        self.assertTrue(result["command_expired"])
        self.assertFalse(any(h in ("D1020", "M1010") for _, h, *_ in driver.calls))

    def test_alignment_to_final_direction_and_stale_feedback_stop_propulsion(self):
        session, driver, clock = self.session()
        session.startup()
        target = Target((91,) * 4, (400,) * 4, clock(), alignment_angles=(270,) * 4)
        self.assertEqual(session.cycle(target)["applied_raw"], STOP)
        clock.sleep(CONFIG.feedback_max_age_s + 0.01)
        result = session.cycle(Target((90,) * 4, (400,) * 4, clock()))
        self.assertTrue(result["feedback_stale"])
        self.assertEqual(result["applied_raw"], STOP)
        self.assertEqual(session.cycle(Target((90,) * 4, (400,) * 4, clock()))["applied_raw"],
                         (400,) * 4)

    def test_angle_error_reduces_drive_without_stopping_each_update(self):
        session, driver, clock = self.session()
        session.startup()
        result = session.cycle(Target((110,) * 4, (400,) * 4, clock()))
        self.assertTrue(all(400 < value < 660 for value in result["applied_raw"]))

    def test_startup_timeout_and_y8_only_enabled_after_angle_convergence(self):
        session, driver, _ = self.session(enable_mode="slmp", startup_timeout_s=0.1)
        driver.angles = [0] * 4
        with self.assertRaises(TimeoutError):
            session.startup()
        self.assertNotIn(("bits", "Y8", (1,)), [c[:3] for c in driver.calls])
        session, driver, _ = self.session(enable_mode="slmp")
        session.startup()
        self.assertEqual(driver.calls[-1][:3], ("bits", "Y8", (1,)))

    def test_stop_holds_steering_and_invalid_target_never_enters_mailbox(self):
        worker = CyclicWorker(CONFIG, Driver())
        worker.submit((100,) * 4, (500,) * 4)
        worker.stop_drive()
        self.assertEqual(worker._target.speeds, STOP)
        self.assertFalse(worker._target.steer)
        for speeds in ((700,) * 4, (True,) * 4, (500,)):
            with self.assertRaises(ValueError):
                worker.submit((100,) * 4, speeds)
            self.assertEqual(worker._target.speeds, STOP)

    def test_worker_uses_one_thread_and_latest_mailbox_during_slow_transaction(self):
        entered, release, applied = threading.Event(), threading.Event(), threading.Event()

        class SlowDriver(Driver):
            block = False

            def write_words(self, head, values):
                super().write_words(head, values)
                if head == "D1000" and self.block:
                    self.block = False
                    entered.set()
                    if not release.wait(2):
                        raise TimeoutError("test release missing")
                if head == "D1020" and tuple(values) == (93,) * 4:
                    applied.set()

        driver = SlowDriver()
        worker = CyclicWorker(replace(CONFIG, servo_wait_s=0.001), driver)
        try:
            worker.start()
            driver.block = True
            worker.submit((91,) * 4, (500,) * 4)
            self.assertTrue(entered.wait(1))
            worker.submit((92,) * 4, (500,) * 4)
            worker.submit((93,) * 4, (500,) * 4)
            # Force one cycle beyond 50 ms without launching another cycle.
            time.sleep(0.065)
            release.set()
            self.assertTrue(applied.wait(1))
        finally:
            release.set()
            worker.close()
        self.assertEqual(len({c[4] for c in driver.calls}), 1)
        self.assertNotEqual(driver.calls[0][4], threading.get_ident())
        self.assertNotIn(("words", "D1020", (92,) * 4), [c[:3] for c in driver.calls])
        self.assertGreaterEqual(worker.status["overruns"], 1)

    def test_read_failure_latches_worker_fault_and_cleans_up_without_reconnect(self):
        driver = Driver()
        driver.fail.add(("read", "D1030", (4,)))
        worker = CyclicWorker(replace(CONFIG, servo_wait_s=0.001), driver)
        with self.assertLogs("fx5u_controller.cyclic", level="ERROR"):
            with self.assertRaises(RuntimeError):
                worker.start()
        with self.assertRaises(RuntimeError):
            worker.close()
        self.assertEqual(sum(c[0] == "connect" for c in driver.calls), 1)
        self.assertEqual(driver.calls[-1][0], "disconnect")
        self.assertIn(("bits", "M1020", (0,)), [c[:3] for c in driver.calls])

    def test_hall_unknown_wrap_window_and_reset_rejection(self):
        self.assertIsNone(HallEstimator(CONFIG).update([20] * 4, 0))
        estimator = HallEstimator(replace(CONFIG, hall_mode="cumulative_u16", hall_window_s=0.5))
        self.assertIsNone(estimator.update([-2] * 4, 0))  # 65534
        speed = estimator.update([3] * 4, 0.5)  # five pulses through wrap
        self.assertAlmostEqual(speed[0], 5 * math.pi * 0.428 / 25 / 0.5)
        self.assertIsNone(estimator.update([0] * 4, 0.6))  # unexpected reset
        self.assertIsNone(estimator.update([1] * 4, 0.7))

    def test_forward_only_kinematics_represents_reverse_by_steering(self):
        kinematics = OpenLoopSwerveKinematics(0.8, 0.7, 0.214, 10, allow_reverse=False)
        result = kinematics.calculate(VelocityCommand(-0.1, 0, 0), (0,) * 4)
        self.assertTrue(all(abs(a) == math.pi for a in result.steering_rad))
        self.assertTrue(all(v > 0 for v in result.wheel_rate_rad_s))

    def test_encoding_scale_and_wrap(self):
        config = replace(CONFIG, units_per_degree=10, raw_zero=-100)
        self.assertEqual(config.encode((90, 0, 359.999, 180)), (800, -100, -100, 1700))
        self.assertEqual(config.decode((800, -100, -100, 1700)), (90, 0, 0, 180))

    def test_config_rejects_bad_map_timing_encoding_unknown_keys_and_enable_mode(self):
        raw = yaml.safe_load((ROOT / "config/default.yaml").read_text())
        mapping = (ROOT / "config/registers_mapping.yaml").read_text()
        changes = [
            ("timing", "trigger_on_s", 0.05), ("timing", "plc_max_scan_s", 0.01),
            ("timing", "trigger_off_s", 0), ("timing", "scan_margin", 1),
            ("steering", "units_per_degree", 1000), ("bldc", "enable_mode", "guess"),
            ("hall", "mode", "rpm"), ("plc", "port", True),
            ("feedback", "max_age_s", 0.05), ("startup", "initial_angle_deg", [90]),
            ("timing", "unrecognized", 1),
        ]
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "default.yaml"
            map_path = Path(directory) / "registers_mapping.yaml"
            map_path.write_text(mapping)
            for section, key, value in changes:
                data = {name: dict(values) if isinstance(values, dict) else values
                        for name, values in raw.items()}
                data[section][key] = value
                config_path.write_text(yaml.safe_dump(data))
                with self.subTest(section=section, key=key):
                    with self.assertRaises(ConfigurationError):
                        load_cyclic_config(config_path)
            config_path.write_text(yaml.safe_dump(raw))
            map_path.write_text(mapping.replace("speed: D1000", "speed: D1020"))
            with self.assertRaisesRegex(ConfigurationError, "register map"):
                load_cyclic_config(config_path)


if __name__ == "__main__":
    unittest.main()
