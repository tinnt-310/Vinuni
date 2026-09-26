#!/usr/bin/env python3
"""Extended bounded target, validation, and timing tests on ros2_control FakeSystem."""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import random
import re
import sys
import time

from metrics import position_error, quaternion_error, rate_metrics
from common import (
    ManagedLaunch,
    ProbeContext,
    axis_angle_quaternion,
    make_results_dir,
    quaternion_multiply,
    write_json,
)


def status_number(status: str, key: str, default=math.nan):
    match = re.search(rf"(?:^| ){re.escape(key)}=([0-9.eE+-]+)", status)
    return float(match.group(1)) if match else default


def generation(status: str) -> int:
    value = status_number(status, "generation", 0.0)
    return int(value) if math.isfinite(value) else 0


def line_deviation(start, goal, sample) -> float:
    direction = [b - a for a, b in zip(start, goal)]
    relative = [p - a for p, a in zip(sample, start)]
    norm = math.sqrt(sum(value * value for value in direction))
    if norm <= 1e-12:
        return math.sqrt(sum(value * value for value in relative))
    cross = [
        relative[1] * direction[2] - relative[2] * direction[1],
        relative[2] * direction[0] - relative[0] * direction[2],
        relative[0] * direction[1] - relative[1] * direction[0],
    ]
    return math.sqrt(sum(value * value for value in cross)) / norm


class ExtendedFakeSuite:
    def __init__(self, domain_id: int, results_dir: pathlib.Path, seed: int):
        self.domain_id = domain_id
        self.results_dir = results_dir
        self.seed = seed
        self.outcomes = {}
        self.launch = None
        self.context = None
        self.probe = None

    def record(self, test_id: str, passed: bool, **evidence):
        self.outcomes[test_id] = {
            "result": "PASS" if passed else "FAIL", **evidence}
        print(f"{'PASS' if passed else 'FAIL':>5} {test_id}")
        return passed

    def wait_holding(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if "state=HOLDING" in self.probe.status():
                return True
            time.sleep(0.02)
        return False

    def stream_case(self, rate_hz: float, axis: int):
        test_id = f"FAKE_STREAM_{int(rate_hz)}HZ"
        start = self.probe.tcp_pose()
        before_generation = generation(self.probe.status())
        history_start = len(self.probe.tcp_history)
        submissions = int(rate_hz)
        target = list(start[0])
        target[axis] += 0.002
        started = time.monotonic()
        for index in range(submissions):
            fraction = (index + 1) / submissions
            current = list(start[0])
            current[axis] += 0.002 * fraction
            self.probe.publish_pose(current, start[1])
            deadline = started + (index + 1) / rate_hz
            time.sleep(max(0.0, deadline - time.monotonic()))
        last_publish = time.monotonic()
        reached = self.probe.wait_pose(
            target, start[1], position_tolerance=0.0005,
            angular_tolerance=0.005, timeout=20.0)
        holding = reached and self.wait_holding()
        final = self.probe.tcp_pose()
        status = self.probe.status()
        after_generation = generation(status)
        with self.probe.lock:
            tcp_rows = list(self.probe.tcp_history[history_start:])
        deviations = [line_deviation(start[0], target, row[1])
                      for row in tcp_rows]
        accepted = max(0, after_generation - before_generation)
        passed = (
            reached and holding and self.launch.process.poll() is None
            and "sdk_errors=0" in status
            and 120.0 <= status_number(status, "loop_hz", 0.0) <= 130.0
        )
        return self.record(
            test_id, passed, submissions=submissions,
            measured_input_hz=submissions / (last_publish - started),
            accepted_generation_delta=accepted,
            coalesced_or_deadband=max(0, submissions - accepted),
            settle_latency_s=time.monotonic() - last_publish,
            endpoint_position_error_m=position_error(final[0], target),
            endpoint_orientation_error_rad=quaternion_error(final[1], start[1]),
            max_perpendicular_error_m=max(deviations, default=0.0),
            status=status)

    def noise_case(self, kind: str, seed_offset: int):
        test_id = f"FAKE_NOISE_{kind.upper()}"
        rng = random.Random(self.seed + seed_offset)
        start = self.probe.tcp_pose()
        before_generation = generation(self.probe.status())
        history_start = len(self.probe.tcp_history)
        submissions = 20
        last_position = list(start[0])
        last_quaternion = list(start[1])
        started = time.monotonic()
        for index in range(submissions):
            position = list(start[0])
            quaternion = list(start[1])
            if kind in ("translation", "combined"):
                position = [value + rng.gauss(0.0, 0.0005)
                            for value in position]
            if kind in ("angular", "combined"):
                perturbation = axis_angle_quaternion(
                    2, rng.gauss(0.0, math.radians(0.5)))
                quaternion = quaternion_multiply(perturbation, quaternion)
            self.probe.publish_pose(position, quaternion)
            last_position, last_quaternion = position, quaternion
            deadline = started + (index + 1) * 0.05
            time.sleep(max(0.0, deadline - time.monotonic()))
        time.sleep(1.0)
        status = self.probe.status()
        accepted = max(0, generation(status) - before_generation)
        with self.probe.lock:
            positions = [row[1] for row in self.probe.tcp_history[history_start:]]
        amplitude = max((position_error(row, start[0]) for row in positions),
                        default=0.0)
        # More than 90% accepted means the current deadband is functioning as
        # a change detector, not as a useful filter for this configured noise.
        bounded = self.launch.process.poll() is None and "sdk_errors=0" in status
        filtered = accepted <= int(0.90 * submissions)
        return self.record(
            test_id, bounded and filtered, seed=self.seed + seed_offset,
            submissions=submissions, accepted_generation_delta=accepted,
            ignored_or_coalesced=max(0, submissions - accepted),
            accepted_fraction=accepted / submissions,
            tcp_motion_amplitude_m=amplitude,
            last_target_position=last_position,
            last_target_quaternion=last_quaternion,
            criterion="accepted_fraction <= 0.90 engineering test threshold",
            status=status)

    def identical_and_deadband(self):
        current = self.probe.tcp_pose()
        baseline = list(current[0])
        baseline[0] += 0.001
        self.probe.publish_pose(baseline, current[1])
        self.probe.wait_pose(baseline, current[1], timeout=15.0)
        self.wait_holding()
        baseline_generation = generation(self.probe.status())
        for _ in range(20):
            self.probe.publish_pose(baseline, current[1])
            time.sleep(0.02)
        identical_delta = generation(self.probe.status()) - baseline_generation
        self.record("FAKE_REPEATED_IDENTICAL", identical_delta == 0,
                    submissions=20, generation_delta=identical_delta)

        for index in range(20):
            inside = list(baseline)
            inside[1] += (1 if index % 2 == 0 else -1) * 0.00002
            orientation = quaternion_multiply(
                axis_angle_quaternion(2, math.radians(0.05)), current[1])
            self.probe.publish_pose(inside, orientation)
            time.sleep(0.02)
        inside_delta = generation(self.probe.status()) - baseline_generation
        self.record("FAKE_INSIDE_DEADBAND", inside_delta == 0,
                    submissions=20, generation_delta=inside_delta,
                    position_amplitude_m=0.00002,
                    angular_amplitude_rad=math.radians(0.05))

        outside = list(baseline)
        outside[1] += 0.00010
        self.probe.publish_pose(outside, current[1])
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and \
                generation(self.probe.status()) <= baseline_generation:
            time.sleep(0.02)
        outside_delta = generation(self.probe.status()) - baseline_generation
        self.record("FAKE_OUTSIDE_DEADBAND", outside_delta >= 1,
                    generation_delta=outside_delta,
                    position_change_m=0.00010)

    def invalid_cases(self):
        current = self.probe.tcp_pose()
        cases = [
            ("FAKE_NAN_POSE", [math.nan, *current[0][1:]], current[1], "base_link"),
            ("FAKE_INF_POSE", [math.inf, *current[0][1:]], current[1], "base_link"),
            ("FAKE_ZERO_QUAT", current[0], [0.0, 0.0, 0.0, 0.0], "base_link"),
            ("FAKE_NAN_QUAT", current[0], [math.nan, 0.0, 0.0, 1.0], "base_link"),
            ("FAKE_INVALID_FRAME", current[0], current[1], "unsupported_frame"),
        ]
        for test_id, position, quaternion, frame in cases:
            before = generation(self.probe.status())
            self.probe.publish_pose_message(position, quaternion, frame, True)
            time.sleep(0.3)
            after = generation(self.probe.status())
            self.record(test_id, after == before,
                        generation_before=before, generation_after=after,
                        frame=frame)

    def silence_and_rejected_freshness(self):
        start = self.probe.tcp_pose()
        target = list(start[0])
        target[2] += 0.002
        for index in range(10):
            point = list(start[0])
            point[2] += 0.002 * (index + 1) / 10.0
            self.probe.publish_pose(point, start[1])
            time.sleep(0.05)
        last_publish = time.monotonic()
        accepted_generation = generation(self.probe.status())
        reached = self.probe.wait_pose(target, start[1], timeout=20.0)
        holding = reached and self.wait_holding()
        status = self.probe.status()
        self.record(
            "FAKE_STREAM_SILENCE", reached and holding and
            "SENSOR_TIMEOUT" not in status,
            time_to_holding_s=time.monotonic() - last_publish,
            final_generation=generation(status), status=status)

        start = self.probe.tcp_pose()
        valid = list(start[0])
        valid[0] += 0.001
        self.probe.publish_pose(valid, start[1])
        deadline = time.monotonic() + 3.0
        before = generation(self.probe.status())
        while time.monotonic() < deadline and generation(self.probe.status()) <= before:
            time.sleep(0.02)
        accepted = generation(self.probe.status())
        for _ in range(10):
            self.probe.publish_pose_message(
                [math.nan, valid[1], valid[2]], start[1], "base_link", True)
            time.sleep(0.1)
        reached = self.probe.wait_pose(valid, start[1], timeout=15.0)
        holding = reached and self.wait_holding()
        after_invalid = generation(self.probe.status())
        self.record(
            "FAKE_REJECTED_TARGET_FRESHNESS",
            reached and holding and after_invalid == accepted,
            accepted_generation=accepted,
            generation_after_invalid_stream=after_invalid,
            invalid_duration_s=1.0,
            semantic="invalid messages do not replace/refresh the last accepted goal")

    def execute(self):
        self.results_dir.mkdir(parents=True, exist_ok=True)
        os.environ["ROS_LOG_DIR"] = str(self.results_dir / "ros_log_probe")
        os.environ["RMW_LOCALHOST_ONLY"] = "1"
        self.launch = ManagedLaunch.start(
            self.results_dir, "192.168.58.2", self.domain_id,
            use_fake_hardware=True)
        self.context = ProbeContext(self.domain_id)
        result_code = 1
        try:
            self.probe = self.context.__enter__()
            ready = self.probe.wait_ready(timeout=20.0)
            if self.record("FAKE_STARTUP", ready):
                timing_start = len(self.probe.joint_stamps)
                for index, rate in enumerate((10.0, 20.0, 50.0, 100.0)):
                    self.stream_case(rate, index % 3)
                self.noise_case("translation", 1)
                self.noise_case("angular", 2)
                self.noise_case("combined", 3)
                self.identical_and_deadband()
                self.invalid_cases()
                self.silence_and_rejected_freshness()
                with self.probe.lock:
                    stamps = list(self.probe.joint_stamps[timing_start:])
                timing = rate_metrics(stamps)
                self.outcomes["FAKE_CONTROL_TIMING"] = {
                    "result": (
                        "PASS" if 120.0 <= timing["mean_hz"] <= 130.0
                        else "FAIL"),
                    **timing,
                    "nominal_period_s": 0.008,
                    "environment": "ros2_control FakeSystem",
                }
                print(
                    f"{self.outcomes['FAKE_CONTROL_TIMING']['result']:>5} "
                    "FAKE_CONTROL_TIMING")
                result_code = 0 if all(
                    item["result"] == "PASS"
                    for item in self.outcomes.values()) else 1
        finally:
            if self.context is not None and self.context.node is not None:
                self.context.__exit__(None, None, None)
            if self.launch is not None:
                self.outcomes["FAKE_SHUTDOWN"] = self.launch.stop()
                self.outcomes["FAKE_SHUTDOWN"]["result"] = (
                    "PASS" if self.outcomes["FAKE_SHUTDOWN"]["returncode"] == 0
                    and not self.outcomes["FAKE_SHUTDOWN"]["forced"]
                    and not self.outcomes["FAKE_SHUTDOWN"]["segfault"] else "FAIL")
                if self.outcomes["FAKE_SHUTDOWN"]["result"] != "PASS":
                    result_code = 1
            write_json(self.results_dir / "extended_fake_runtime.json", self.outcomes)
        return result_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=184)
    parser.add_argument("--results-dir")
    parser.add_argument("--seed", type=int, default=20260829)
    args = parser.parse_args()
    results = pathlib.Path(args.results_dir) if args.results_dir else make_results_dir()
    suite = ExtendedFakeSuite(args.domain_id, results, args.seed)
    code = suite.execute()
    print(json.dumps(suite.outcomes, indent=2, sort_keys=True))
    return code


if __name__ == "__main__":
    sys.exit(main())
