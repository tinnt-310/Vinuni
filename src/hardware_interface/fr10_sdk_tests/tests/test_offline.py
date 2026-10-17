#!/usr/bin/env python3
"""Exercise admission, dispatch, measured feedback and cleanup without hardware."""
import argparse
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import cnde
import run as runner

PARSER = argparse.ArgumentParser()
PARSER.add_argument("--binary", type=Path, required=True)
ARGS, REST = PARSER.parse_known_args()


class BenchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = json.loads((ROOT / "config.example.json").read_text())
        self.config["duration_s"] = 0.04
        self.config["force"].update(fixture_ready=True, select=[0, 0, 1, 0, 0, 0],
                                  desired_wrench=[0, 0, 1, 0, 0, 0], pid=[0.00001] * 6,
                                  compliance_p=0.00005, compliance_threshold_n=1)
        self.config["torque"].update(prepared=True, torque_nm=[0] * 6, max_abs_torque_nm=[1] * 6,
                                   power_limit_w=[1] * 6, velocity_limit_deg_s=[1] * 6)
        self.config["calibration"].update(stationary_no_contact_confirmed=True,
                                        payload_record_pose_prepared=True, payload_record_completed=True)
        self.config["timeout_s"] = 30
        self.config["force_movel"] = dict(reference_verified=True, expected_tool=0,
            tool_axis=0, distance_mm=1, speed_mm_s=1, acceleration_mm_s2=1,
            motion_timeout_s=2.1, minimum_contact_n=0.5, force_tolerance_n=0.5)

    def probe(self, name, flags=("--execute", "--exclusive-control"), env=None):
        path = self.root / f"config_{name}.json"
        path.write_text(json.dumps(self.config))
        output = self.root / name
        log = self.root / f"{name}.calls"
        variables = {**os.environ, "BENCH_FAKE_LOG": str(log), **(env or {})}
        if name.startswith("servojt"):
            variables["BENCH_FAKE_DRAG"] = "1"
        if name == "ft_movel":
            variables["BENCH_FAKE_CONTACT"] = "1"
        proc = subprocess.run([str(ARGS.binary), "--config", str(path), "--case", name,
                               "--output", str(output), *flags], env=variables,
                              capture_output=True, text=True, timeout=8)
        result = json.loads((output / "result.json").read_text()) if (output / "result.json").exists() else None
        calls = log.read_text().splitlines() if log.exists() else []
        return proc, result, calls

    def test_every_native_case_and_cleanup(self):
        for name in runner.CASES:
            if name == "cnde":
                continue
            with self.subTest(name=name):
                proc, result, calls = self.probe(name)
                self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
                self.assertTrue(result["cleanup_confirmed"])
                self.assertEqual(result["backend"], "OFFLINE_FAKE")
                self.assertGreater(result["fresh_state_frames"], 1)
                self.assertIn("CloseRPC", calls)
                if name.startswith("servocart"):
                    self.assertEqual(result["status"], "MOTION_OBSERVED")
                    self.assertAlmostEqual(result["measured_delta"], 1)
                if name.startswith("ft_control"):
                    variant = name.removeprefix("ft_control_")
                    self.assertIn(f"FT_Control/{variant}/start", calls)
                    self.assertIn(f"FT_Control/{variant}/stop", calls)
                if name == "ft_compliance":
                    self.assertLess(calls.index("FT_Control/extended/start"), calls.index("FT_ComplianceStart"))
                    self.assertIn("FT_ComplianceStop", calls)
                    self.assertIn("FT_Control/extended/stop", calls)

    def test_force_movel_moves_tool_tangent_and_measures_force(self):
        proc, result, calls = self.probe("ft_movel")
        self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
        self.assertLess(calls.index("FT_Control/basic/start"), calls.index("MoveL"))
        self.assertLess(calls.index("MoveL"), calls.index("FT_Control/basic/stop"))
        self.assertTrue(result["movel_complete"])
        # Fake tool Rz=90: tool X -> base Y.
        self.assertAlmostEqual(result["final"]["tcp_mm_deg"][1]-200, 1)
        self.assertAlmostEqual(result["final"]["tcp_mm_deg"][0], 100)
        self.assertGreater(result["force_tracking"]["samples"], 5)
        self.assertAlmostEqual(result["force_tracking"]["rmse_n"], 1)
        self.assertEqual(result["force_tracking"]["fraction_in_tolerance"], 0)
        self.assertEqual(result["status"], "API_ACCEPTED_UNVERIFIED")

    def test_force_movel_wrong_contact_sign_blocks_motion(self):
        self.config["force"]["desired_wrench"][2] = -1
        proc, result, calls = self.probe("ft_movel")
        self.assertEqual(proc.returncode, 1)
        self.assertIn("matching Fz sign", result["error"])
        self.assertNotIn("FT_Control/basic/start", calls)
        self.assertNotIn("MoveL", calls)

    def test_force_movel_missing_contact_blocks_motion(self):
        self.config["force_movel"]["minimum_contact_n"] = 3
        proc, _, calls = self.probe("ft_movel")
        self.assertEqual(proc.returncode, 1)
        self.assertNotIn("MoveL", calls)

    def test_force_movel_unverified_reference_never_connects(self):
        self.config["force_movel"]["reference_verified"] = False
        proc, _, calls = self.probe("ft_movel")
        self.assertEqual(proc.returncode, 2)
        self.assertEqual(calls, [])

    def test_force_movel_no_response_times_out_and_disables_force(self):
        proc, result, calls = self.probe("ft_movel", env={"BENCH_FAKE_NO_RESPONSE": "1"})
        self.assertEqual(proc.returncode, 1)
        self.assertFalse(result["movel_complete"])
        self.assertIn("timeout", result["error"])
        self.assertIn("StopMotion", calls)
        self.assertIn("FT_Control/basic/stop", calls)

    def test_force_movel_rpc_failure_stops_and_disables_force(self):
        proc, result, calls = self.probe("ft_movel", env={"BENCH_FAKE_FAIL": "MoveL"})
        self.assertEqual(proc.returncode, 1)
        self.assertIn("MoveL returned 3", result["error"])
        self.assertIn("StopMotion", calls)
        self.assertIn("FT_Control/basic/stop", calls)

    def test_force_movel_force_spike_stops_and_disables_force(self):
        proc, result, calls = self.probe("ft_movel", env={"BENCH_FAKE_FORCE_SPIKE": "1"})
        self.assertEqual(proc.returncode, 1)
        self.assertIn("Force/torque observation guard", result["error"])
        self.assertIn("FT_Control/basic/stop", calls)

    def test_force_movel_sensor_loss_stops_and_disables_force(self):
        proc, result, calls = self.probe("ft_movel", env={"BENCH_FAKE_SENSOR_LOSS": "1"})
        self.assertEqual(proc.returncode, 1)
        self.assertIn("Force sensor became inactive", result["error"])
        self.assertIn("FT_Control/basic/stop", calls)

    def test_force_movel_contact_loss_stops_and_disables_force(self):
        proc, result, calls = self.probe("ft_movel", env={"BENCH_FAKE_CONTACT_LOSS": "1"})
        self.assertEqual(proc.returncode, 1)
        self.assertIn("Contact lost", result["error"])
        self.assertIn("FT_Control/basic/stop", calls)

    def test_force_movel_wrong_tcp_blocks_force_start(self):
        self.config["force_movel"]["expected_tool"] = 1
        proc, result, calls = self.probe("ft_movel")
        self.assertEqual(proc.returncode, 1)
        self.assertIn("expected_tool", result["error"])
        self.assertNotIn("FT_Control/basic/start", calls)

    def test_no_execute_constructs_no_sdk(self):
        proc, result, calls = self.probe("servoj", flags=())
        self.assertEqual(proc.returncode, 2)
        self.assertIsNone(result)
        self.assertEqual(calls, [])

    def test_no_exclusive_constructs_no_sdk(self):
        proc, _, calls = self.probe("servoj", flags=("--execute",))
        self.assertEqual(proc.returncode, 2)
        self.assertEqual(calls, [])

    def test_invalid_torque_config_before_connection(self):
        self.config["torque"]["torque_nm"][0] = 2
        proc, _, calls = self.probe("servojt")
        self.assertEqual(proc.returncode, 2)
        self.assertEqual(calls, [])

    def test_wrong_version_blocks_motion(self):
        proc, result, calls = self.probe("servoj", env={"BENCH_FAKE_WRONG_VERSION": "1"})
        self.assertEqual(proc.returncode, 1)
        self.assertIn("web version", result["identity_mismatches"])
        self.assertNotIn("ServoMoveStart", calls)

    def test_zero_return_without_motion_is_failure(self):
        proc, result, calls = self.probe("servocart_absolute", env={"BENCH_FAKE_NO_RESPONSE": "1"})
        self.assertEqual(proc.returncode, 1)
        self.assertEqual(result["status"], "NO_MEASURED_RESPONSE")
        self.assertIn("ServoMoveEnd", calls)

    def test_servocart_z_10mm(self):
        self.config["servo"].update(cart_axis=2, cart_delta_mm=10)
        self.config["guards"]["max_tcp_delta_mm"] = 12
        for name in ("servocart_absolute", "servocart_base", "servocart_tool"):
            with self.subTest(name=name):
                proc, result, _ = self.probe(name)
                self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
                self.assertEqual(result["status"], "MOTION_OBSERVED")
                initial = result["initial"]["tcp_mm_deg"]
                final = result["final"]["tcp_mm_deg"]
                self.assertAlmostEqual(final[2] - initial[2], 10)
                for axis in (0, 1, 3, 4, 5):
                    self.assertAlmostEqual(final[axis], initial[axis])

    def test_servocart_10mm_requires_larger_envelope(self):
        self.config["servo"].update(cart_axis=2, cart_delta_mm=10)
        proc, _, calls = self.probe("servocart_absolute", flags=("--validate-only",))
        self.assertEqual(proc.returncode, 2)
        self.assertIn("observation envelope", proc.stderr)
        self.assertEqual(calls, [])

    def test_servocart_z_50mm(self):
        self.config["servo"].update(cart_axis=2, cart_delta_mm=50)
        self.config["guards"]["max_tcp_delta_mm"] = 55
        proc, result, _ = self.probe("servocart_absolute")
        self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
        self.assertEqual(result["status"], "MOTION_OBSERVED")
        initial = result["initial"]["tcp_mm_deg"]
        final = result["final"]["tcp_mm_deg"]
        self.assertAlmostEqual(final[2] - initial[2], 50)
        for axis in (0, 1, 3, 4, 5):
            self.assertAlmostEqual(final[axis], initial[axis])

    def test_servocart_over_50mm_rejected_before_connection(self):
        self.config["servo"].update(cart_axis=2, cart_delta_mm=50.1)
        self.config["guards"]["max_tcp_delta_mm"] = 55
        proc, _, calls = self.probe("servocart_absolute", flags=("--validate-only",))
        self.assertEqual(proc.returncode, 2)
        self.assertIn("cart_delta_mm", proc.stderr)
        self.assertEqual(calls, [])

    def test_rejected_start_still_attempts_cleanup(self):
        proc, result, calls = self.probe("servocart_base", env={"BENCH_FAKE_FAIL": "ServoMoveStart"})
        self.assertEqual(proc.returncode, 1)
        self.assertIn("returned 3", result["error"])
        self.assertIn("StopMotion", calls)
        self.assertIn("ServoMoveEnd", calls)

    def test_force_start_failure_still_disables(self):
        proc, result, calls = self.probe("ft_control_mb", env={"BENCH_FAKE_FAIL": "FT_Control/mb/start"})
        self.assertEqual(proc.returncode, 1)
        self.assertIn("FT_Control/mb/stop", calls)
        self.assertTrue(result["cleanup_confirmed"])

    def test_stop_failure_is_not_success(self):
        proc, result, _ = self.probe("servoj", env={"BENCH_FAKE_FAIL": "StopMotion"})
        self.assertEqual(proc.returncode, 1)
        self.assertEqual(result["status"], "CLEANUP_UNCONFIRMED")

    def test_stale_packets_block_motion(self):
        proc, result, calls = self.probe("servoj", env={"BENCH_FAKE_STALE": "1"})
        self.assertEqual(proc.returncode, 1)
        self.assertIn("Stale", result["error"])
        self.assertNotIn("ServoMoveStart", calls)

    def test_inactive_sensor_is_not_working_force(self):
        proc, result, calls = self.probe("ft_linear", env={"BENCH_FAKE_SENSOR_OFF": "1"})
        self.assertEqual(proc.returncode, 1)
        self.assertNotIn("FT_LinInsertion", calls)

    def test_validate_only_no_connection(self):
        proc, _, calls = self.probe("ft_compliance", flags=("--validate-only",))
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(calls, [])

    def test_runner_dry_run_never_constructs_sdk(self):
        log = self.root / "dry.calls"
        proc = subprocess.run([sys.executable, str(ROOT / "run.py"), "--suite", "all",
                               "--binary", str(ARGS.binary), "--output", str(self.root / "dry")],
                              env={**os.environ, "BENCH_FAKE_LOG": str(log)}, capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        report = json.loads((self.root / "dry/summary.json").read_text())
        self.assertEqual(len(report["results"]), len(runner.CASES))
        self.assertFalse(log.exists())


class FakeSocket:
    def __init__(self, *args):
        self.sent = []
        self.queue = []
        self.streaming = False
        self.counter = 253

    def connect(self, endpoint):
        self.endpoint = endpoint

    def settimeout(self, value):
        self.timeout = value

    def send(self, data):
        self.sent.append(data)
        _, kind, payload = cnde.parse(data)
        if kind == 1:
            self.queue.append(cnde.frame(6, b"\0DOUBLE_6,DOUBLE_6"))
        else:
            self.queue.append(cnde.frame(6, b"\0"))
        self.streaming = kind == 2

    def recv(self, size):
        if self.queue:
            return self.queue.pop(0)
        if self.streaming:
            time.sleep(0.002)
            self.counter = (self.counter + 1) % 256
            return cnde.frame(4, struct.pack("<12d", *range(12)), self.counter)
        raise TimeoutError("No scripted packet")

    def close(self):
        pass


class CndeTests(unittest.TestCase):
    def test_frame_length_and_trailer(self):
        count, kind, body = cnde.parse(cnde.frame(1, b"abc", 255))
        self.assertEqual((count, kind, body), (255, 1, b"abc"))
        with self.assertRaises(ValueError):
            cnde.parse(cnde.frame(1, b"abc")[:-1])

    def test_negotiation_decode_wrap_and_stop(self):
        config = json.loads((ROOT / "config.example.json").read_text())
        config["duration_s"] = 0.04
        sock = FakeSocket()
        with tempfile.TemporaryDirectory() as temp:
            result = cnde.run(config, temp, lambda *args: sock)
            self.assertEqual(result["status"], "OBSERVED", result)
            self.assertTrue(result["cleanup_confirmed"])
            self.assertEqual(result["estimated_lost_packets"], 0)
            self.assertEqual(cnde.parse(sock.sent[-1])[1], 3)
            rows = [json.loads(line) for line in (Path(temp) / "samples.jsonl").read_text().splitlines()]
            self.assertEqual(rows[0]["actual_joint_pos"], list(range(6)))

    def test_unknown_field_rejected_before_socket(self):
        config = json.loads((ROOT / "config.example.json").read_text())
        config["cnde"]["fields"] = ["made_up_field"]
        with self.assertRaises(ValueError):
            cnde.run(config, "/unused", lambda *args: self.fail("Must not open socket"))

    def test_negotiated_type_mismatch_is_failure_and_stops(self):
        class WrongTypeSocket(FakeSocket):
            def send(self, data):
                super().send(data)
                if cnde.parse(data)[1] == 1:
                    self.queue[-1] = cnde.frame(6, b"\0DOUBLE_6,UINT8")
        config = json.loads((ROOT / "config.example.json").read_text())
        sock = WrongTypeSocket()
        with tempfile.TemporaryDirectory() as temp:
            result = cnde.run(config, temp, lambda *args: sock)
            self.assertEqual(result["status"], "FAILED")
            self.assertIn("negotiated types", result["error"])
            self.assertTrue(result["cleanup_confirmed"])
            self.assertNotIn(2, [cnde.parse(data)[1] for data in sock.sent])

    def test_cnde_stop_failure_is_not_success(self):
        class BadStopSocket(FakeSocket):
            def send(self, data):
                super().send(data)
                if cnde.parse(data)[1] == 3 and len(self.sent) > 1:
                    self.queue[-1] = cnde.frame(6, b"\3stop failed")
        config = json.loads((ROOT / "config.example.json").read_text())
        config["duration_s"] = 0.04
        with tempfile.TemporaryDirectory() as temp:
            result = cnde.run(config, temp, lambda *args: BadStopSocket())
            self.assertEqual(result["status"], "CLEANUP_UNCONFIRMED")
            self.assertFalse(result["cleanup_confirmed"])


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0], *REST])
