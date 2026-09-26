from __future__ import annotations

import threading
import unittest
from pathlib import Path
from typing import List, Sequence, Tuple

from fx5u_controller import (
    ControllerStateError,
    MitsubishiWheelController,
    PulseCleanupError,
    SLMPCommunicationError,
    load_application_config,
    load_register_mapping,
)


Call = Tuple[str, str, List[int]]
PROJECT_ROOT = Path(__file__).resolve().parents[1]
WHEELS = load_register_mapping(PROJECT_ROOT / "config" / "registers_mapping.yaml")
SPEED_CONTROL = load_application_config(
    PROJECT_ROOT / "config" / "default.yaml"
).speed_control


class FakeDriver:
    def __init__(self) -> None:
        self.connected = False
        self.calls: List[Call] = []
        self.fail_calls = set()
        self._write_number = 0

    def connect(self) -> None:
        self.connected = True
        self.calls.append(("connect", "", []))

    def disconnect(self) -> None:
        self.connected = False
        self.calls.append(("disconnect", "", []))

    def write_bits(self, head_device: str, values: Sequence[int]) -> None:
        self._write_number += 1
        self.calls.append(("bits", head_device, list(values)))
        if self._write_number in self.fail_calls:
            raise OSError(f"injected write failure {self._write_number}")

    def write_words(self, head_device: str, values: Sequence[int]) -> None:
        self._write_number += 1
        self.calls.append(("words", head_device, list(values)))
        if self._write_number in self.fail_calls:
            raise OSError(f"injected write failure {self._write_number}")


def make_controller(fake: FakeDriver, sleeps: List[float]) -> MitsubishiWheelController:
    return MitsubishiWheelController(
        "192.0.2.10",
        5007,
        trigger_pulse_s=0.05,
        wheels=WHEELS,
        speed_control=SPEED_CONTROL,
        driver=fake,
        sleep_fn=sleeps.append,
    )


class WheelControllerTests(unittest.TestCase):
    def test_confirmed_mapping(self) -> None:
        self.assertEqual([WHEELS[i].trigger for i in WHEELS], ["M100", "M101", "M102", "M103"])
        self.assertEqual([WHEELS[i].servo for i in WHEELS], ["M110", "M111", "M112", "M113"])
        self.assertEqual([WHEELS[i].reset for i in WHEELS], ["M120", "M121", "M122", "M123"])
        self.assertEqual(
            [WHEELS[i].direction for i in WHEELS],
            ["M60", "M65", "M62", "M63"],
        )
        self.assertEqual([WHEELS[i].angle for i in WHEELS], ["D1000", "D1001", "D1002", "D1003"])
        self.assertEqual([WHEELS[i].speed for i in WHEELS], ["D1020", "D1021", "D1022", "D1023"])

    def test_move_angles_batch_order_and_signed_values(self) -> None:
        fake = FakeDriver()
        sleeps: List[float] = []
        controller = make_controller(fake, sleeps)
        controller.connect()

        controller.move_angles([45, 45, -45, -45])

        self.assertEqual(
            fake.calls,
            [
                ("connect", "", []),
                ("bits", "M110", [1, 1, 1, 1]),
                ("words", "D1000", [45, 45, -45, -45]),
                ("bits", "M100", [1, 1, 1, 1]),
                ("bits", "M100", [0, 0, 0, 0]),
            ],
        )
        self.assertEqual(sleeps, [0.05])
        self.assertTrue(all(controller.servo_enabled.values()))

    def test_servo_stays_on_across_commands(self) -> None:
        fake = FakeDriver()
        controller = make_controller(fake, [])
        controller.connect()
        controller.move_angle(3, -30)
        controller.move_speed(3, 100)

        self.assertEqual(
            [call for call in fake.calls if call[0] == "bits" and call[1] == "M112"],
            [("bits", "M112", [1])],
        )
        self.assertTrue(controller.servo_enabled[3])

    def test_speed_uses_drive_motor_only_and_does_not_enable_steering_servo(self) -> None:
        fake = FakeDriver()
        sleeps: List[float] = []
        controller = make_controller(fake, sleeps)
        controller.connect()

        controller.move_speeds([660, 520, 380, 100])

        self.assertEqual(
            fake.calls,
            [
                ("connect", "", []),
                ("words", "D1020", [660, 660, 660, 660]),
                ("bits", "M58", [1]),
                ("words", "D1020", [660, 520, 380, 100]),
            ],
        )
        self.assertEqual(sleeps, [])
        self.assertTrue(controller.speed_control_enabled)

        controller.move_speeds([100, 100, 100, 100])
        self.assertEqual(fake.calls[-1], ("words", "D1020", [100, 100, 100, 100]))
        self.assertEqual(
            [call for call in fake.calls if call[0:2] == ("bits", "M58")],
            [("bits", "M58", [1]), ("bits", "M58", [1])],
        )

    def test_steering_servo_off_does_not_disable_wheel_drive(self) -> None:
        fake = FakeDriver()
        controller = make_controller(fake, [])
        controller.connect()
        controller.move_speeds([600, 600, 600, 600])

        controller.servo_off_all()

        self.assertEqual(fake.calls[-1], ("bits", "M110", [0, 0, 0, 0]))
        self.assertTrue(controller.speed_control_enabled)

    def test_direction_uses_confirmed_nonconsecutive_bits(self) -> None:
        fake = FakeDriver()
        controller = make_controller(fake, [])
        controller.connect()

        controller.set_direction(1, True)
        controller.set_direction(2, False)
        controller.set_directions([True, False, True, False])

        self.assertEqual(
            fake.calls[1:],
            [
                ("bits", "M60", [1]),
                ("bits", "M65", [0]),
                ("bits", "M60", [1]),
                ("bits", "M65", [0]),
                ("bits", "M62", [1]),
                ("bits", "M63", [0]),
            ],
        )
        self.assertEqual(
            controller.direction_clockwise,
            {1: True, 2: False, 3: True, 4: False},
        )

    def test_reset_all_is_one_batch_pulse(self) -> None:
        fake = FakeDriver()
        sleeps: List[float] = []
        controller = make_controller(fake, sleeps)
        controller.connect()
        controller.reset_error_all()

        self.assertEqual(
            fake.calls[-2:],
            [
                ("bits", "M120", [1, 1, 1, 1]),
                ("bits", "M120", [0, 0, 0, 0]),
            ],
        )
        self.assertEqual(sleeps, [0.05])

    def test_off_cleanup_is_attempted_after_trigger_on_failure(self) -> None:
        fake = FakeDriver()
        controller = make_controller(fake, [])
        controller.connect()
        controller.servo_on_all()  # write 1
        fake.fail_calls.add(3)  # angle is write 2, trigger ON is write 3

        with self.assertRaises(SLMPCommunicationError):
            controller.move_angles([1, 2, 3, 4])

        self.assertEqual(fake.calls[-1], ("bits", "M100", [0, 0, 0, 0]))
        self.assertTrue(controller.communication_fault)
        with self.assertRaises(ControllerStateError):
            controller.move_angles([1, 2, 3, 4])

    def test_dual_pulse_failure_reports_cleanup_failure(self) -> None:
        fake = FakeDriver()
        controller = make_controller(fake, [])
        controller.connect()
        controller.servo_on_all()  # write 1
        fake.fail_calls.update((3, 4))

        with self.assertRaises(PulseCleanupError) as caught:
            controller.move_angles([1, 2, 3, 4])

        self.assertIsNotNone(caught.exception.primary_error)
        self.assertIsNotNone(caught.exception.cleanup_error)

    def test_emergency_stop_attempts_all_safety_writes(self) -> None:
        fake = FakeDriver()
        controller = make_controller(fake, [])
        controller.connect()
        controller.servo_on_all()  # write 1
        fake.fail_calls.add(2)  # zero-speed write fails

        with self.assertRaises(SLMPCommunicationError):
            controller.emergency_stop()

        self.assertEqual(fake.calls[-4], ("words", "D1020", [660, 660, 660, 660]))
        self.assertEqual(fake.calls[-3], ("bits", "M58", [0]))
        self.assertEqual(fake.calls[-2], ("bits", "M100", [0, 0, 0, 0]))
        self.assertEqual(fake.calls[-1], ("bits", "M110", [0, 0, 0, 0]))
        self.assertFalse(any(controller.servo_enabled.values()))

    def test_disconnected_emergency_stop_does_not_claim_servo_off(self) -> None:
        fake = FakeDriver()
        controller = make_controller(fake, [])
        controller.servo_enabled[1] = True

        controller.emergency_stop()

        self.assertTrue(controller.servo_enabled[1])
        self.assertEqual(fake.calls, [])

    def test_disconnect_clears_bits_and_servo_state_before_close(self) -> None:
        fake = FakeDriver()
        controller = make_controller(fake, [])
        controller.connect()
        controller.servo_on_all()

        controller.disconnect()

        self.assertEqual(
            fake.calls[-5:],
            [
                ("words", "D1020", [660, 660, 660, 660]),
                ("bits", "M58", [0]),
                ("bits", "M100", [0, 0, 0, 0]),
                ("bits", "M110", [0, 0, 0, 0]),
                ("disconnect", "", []),
            ],
        )
        self.assertFalse(controller.connected)
        self.assertFalse(any(controller.servo_enabled.values()))

    def test_input_validation_happens_before_command_writes(self) -> None:
        fake = FakeDriver()
        controller = make_controller(fake, [])
        controller.connect()

        for bad_wheel in (0, 5, True, 1.0):
            with self.assertRaises(ValueError):
                controller.move_angle(bad_wheel, 10)  # type: ignore[arg-type]
        with self.assertRaises(ValueError):
            controller.move_angles([1, 2, 3])
        with self.assertRaises(TypeError):
            controller.move_angles([1, 2, 3, 4.5])  # type: ignore[list-item]
        with self.assertRaises(ValueError):
            controller.move_angle(1, 32_768)
        with self.assertRaises(ValueError):
            controller.move_speeds([100, 500, 660, 661])
        with self.assertRaises(ValueError):
            controller.move_speed(1, 99)
        with self.assertRaises(TypeError):
            controller.move_speed(1, 10.5)  # type: ignore[arg-type]
        with self.assertRaises(TypeError):
            controller.set_direction(1, 1)  # type: ignore[arg-type]
        with self.assertRaises(ValueError):
            controller.set_directions([True, False, True])

        self.assertEqual(fake.calls, [("connect", "", [])])

    def test_lock_serializes_complete_transactions(self) -> None:
        fake = FakeDriver()
        gate_entered = threading.Event()
        release_gate = threading.Event()

        def blocking_sleep(_: float) -> None:
            gate_entered.set()
            self.assertTrue(release_gate.wait(timeout=1.0))

        controller = MitsubishiWheelController(
            "192.0.2.10",
            5007,
            trigger_pulse_s=0.05,
            wheels=WHEELS,
            speed_control=SPEED_CONTROL,
            driver=fake,
            sleep_fn=blocking_sleep,
        )
        controller.connect()
        controller.servo_on_all()

        first = threading.Thread(target=controller.move_angle, args=(1, 10))
        second = threading.Thread(target=controller.move_angle, args=(2, 20))
        first.start()
        self.assertTrue(gate_entered.wait(timeout=1.0))
        second.start()
        release_gate.set()
        first.join(timeout=1.0)
        second.join(timeout=1.0)

        command_calls = fake.calls[2:]
        self.assertEqual(
            command_calls,
            [
                ("words", "D1000", [10]),
                ("bits", "M100", [1]),
                ("bits", "M100", [0]),
                ("words", "D1001", [20]),
                ("bits", "M101", [1]),
                ("bits", "M101", [0]),
            ],
        )


if __name__ == "__main__":
    unittest.main()
