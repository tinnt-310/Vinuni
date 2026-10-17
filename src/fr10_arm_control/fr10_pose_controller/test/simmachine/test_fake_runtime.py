#!/usr/bin/env python3
"""Small end-to-end Joint/Pose smoke test using ros2_control FakeSystem."""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import re
import threading
import sys
import time

from std_msgs.msg import Float64MultiArray

from common import (
    ManagedLaunch,
    ProbeContext,
    generate,
    make_results_dir,
    max_abs_error,
    position_error,
    quaternion_error,
    write_json,
)


def fake_health_packet(sequence: int):
    """Test-only healthy heartbeat for ros2_control GenericSystem."""
    if not isinstance(sequence, int) or sequence <= 0 or sequence > 2**53:
        raise ValueError('health sequence must be a positive exact integer')
    data = [0.0] * 16
    data[0] = 1.0
    data[1] = float(sequence)
    data[6] = 1.0
    return data


class FakeHealthHeartbeat:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(
            Float64MultiArray, '/fairino_driver/health', 10)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._publish, daemon=True)

    def start(self):
        self.thread.start()

    def _publish(self):
        sequence = 1
        while not self.stop_event.is_set():
            message = Float64MultiArray()
            message.data = fake_health_packet(sequence)
            self.publisher.publish(message)
            sequence += 1
            self.stop_event.wait(0.005)

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.node.destroy_publisher(self.publisher)


def run_fake_runtime(domain_id: int, results_dir: pathlib.Path) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(results_dir / "ros_log_probe")
    os.environ["RMW_LOCALHOST_ONLY"] = "1"
    launch = ManagedLaunch.start(
        results_dir, "192.168.58.2", domain_id, use_fake_hardware=True)
    probe_context = ProbeContext(domain_id)
    health_heartbeat = None
    outcome = {}

    try:
        probe = probe_context.__enter__()
        health_heartbeat = FakeHealthHeartbeat(probe)
        health_heartbeat.start()
        ready = probe.wait_ready(timeout=20.0)
        outcome["startup"] = "PASS" if ready else "FAIL"
        outcome["startup_feedback"] = {
            "joint_seen": probe.joint_position() is not None,
            "tcp_seen": probe.tcp_pose() is not None,
            "status": probe.status(),
        }
        if not ready:
            return 1

        joint_start = probe.joint_position()
        safe_joint = generate(joint_start, 0, 2.0)
        outcome["joint_start_rad"] = joint_start
        outcome["joint_target_rad"] = safe_joint.target
        if not safe_joint.allowed:
            outcome["joint"] = safe_joint.blocked_reason
            return 1

        joint_started = time.monotonic()
        probe.publish_joint(safe_joint.target)
        joint_reached = probe.wait_joint(safe_joint.target, timeout=10.0)
        hold_deadline = time.monotonic() + 2.0
        while (joint_reached and time.monotonic() < hold_deadline
               and "state=HOLDING" not in probe.status()):
            time.sleep(0.02)
        joint_final = probe.joint_position()
        joint_status = probe.status()
        joint_error = max_abs_error(joint_final, safe_joint.target)
        joint_pass = (
            joint_reached
            and joint_error <= 0.002
            and "state=HOLDING" in joint_status
            and "owner=JOINT_OTG" in joint_status
            and "sdk_errors=0" in joint_status
        )
        outcome["joint"] = {
            "result": "PASS" if joint_pass else "FAIL",
            "duration_s": time.monotonic() - joint_started,
            "max_error_rad": joint_error,
            "status": joint_status,
            "command_state": probe.command_state(),
        }
        if not joint_pass:
            return 1

        pose_start = probe.tcp_pose()
        pose_target = list(pose_start[0])
        pose_target[0] += 0.002
        outcome["pose_start"] = pose_start
        outcome["pose_target"] = [pose_target, pose_start[1]]
        pose_started = time.monotonic()
        # One publication must be a complete goal. Do not refresh it while the
        # robot is moving; this catches any accidental stream-timeout behavior.
        probe.publish_pose(pose_target, pose_start[1])
        pose_reached = probe.wait_pose(
            pose_target, pose_start[1], position_tolerance=0.0005,
            angular_tolerance=0.005, timeout=30.0,
            republish_period_s=None)
        hold_deadline = time.monotonic() + 3.0
        while (pose_reached and time.monotonic() < hold_deadline
               and "state=HOLDING" not in probe.status()):
            time.sleep(0.02)
        pose_final = probe.tcp_pose()
        pose_status = probe.status()
        pose_position_error = position_error(pose_final[0], pose_target)
        pose_angular_error = quaternion_error(pose_final[1], pose_start[1])
        pose_pass = (
            pose_reached
            and pose_position_error <= 0.001
            and pose_angular_error <= 0.01
            and "state=HOLDING" in pose_status
            and "owner=POSE_PATH" in pose_status
            and "sdk_errors=0" in pose_status
        )
        one_shot_pass = (
            pose_pass and "SENSOR_TIMEOUT" not in pose_status
        )
        outcome["pose_one_shot"] = {
            "result": "PASS" if one_shot_pass else "FAIL",
            "duration_s": time.monotonic() - pose_started,
            "position_error_m": pose_position_error,
            "angular_error_rad": pose_angular_error,
            "status": pose_status,
        }
        if not one_shot_pass:
            return 1

        generation_match = re.search(r"generation=(\d+)", pose_status)
        one_shot_generation = (
            int(generation_match.group(1)) if generation_match else 0
        )

        # Now use the exact same topic as a changing 10 Hz target stream. Each
        # update is a new goal; after the final publication, no more messages
        # are sent and the robot must still complete that last goal and HOLD.
        stream_start = probe.tcp_pose()
        stream_target = list(stream_start[0])
        stream_target[1] += 0.004
        stream_started = time.monotonic()
        submissions = 20
        for index in range(submissions):
            fraction = (index + 1) / submissions
            target = [
                stream_start[0][0] + 0.0005 * math.sin(math.pi * fraction),
                stream_start[0][1] + 0.004 * fraction,
                stream_start[0][2],
            ]
            probe.publish_pose(target, stream_start[1])
            time.sleep(max(
                0.0,
                stream_started + (index + 1) * 0.1 - time.monotonic(),
            ))

        stream_publish_finished = time.monotonic()
        stream_reached = probe.wait_pose(
            stream_target, stream_start[1], position_tolerance=0.0005,
            angular_tolerance=0.005, timeout=35.0,
            republish_period_s=None)
        stream_hold_deadline = time.monotonic() + 3.0
        while (stream_reached and time.monotonic() < stream_hold_deadline
               and "state=HOLDING" not in probe.status()):
            time.sleep(0.02)
        stream_final = probe.tcp_pose()
        stream_status = probe.status()
        stream_position_error = position_error(stream_final[0], stream_target)
        stream_angular_error = quaternion_error(
            stream_final[1], stream_start[1])
        stream_generation_match = re.search(
            r"generation=(\d+)", stream_status)
        stream_generation = (
            int(stream_generation_match.group(1))
            if stream_generation_match else 0
        )
        velocity_jump_match = re.search(
            r"max_delta_qdot_rad_s=([0-9.eE+-]+)", stream_status)
        max_velocity_jump = (
            float(velocity_jump_match.group(1))
            if velocity_jump_match else math.inf
        )
        stream_pass = (
            stream_reached
            and stream_position_error <= 0.001
            and stream_angular_error <= 0.01
            and "state=HOLDING" in stream_status
            and "owner=POSE_PATH" in stream_status
            and "SENSOR_TIMEOUT" not in stream_status
            and "sdk_errors=0" in stream_status
            and stream_generation >= one_shot_generation + 5
            and max_velocity_jump <= 0.05
        )
        outcome["pose_stream_10hz"] = {
            "result": "PASS" if stream_pass else "FAIL",
            "submissions": submissions,
            "input_hz": submissions / (
                stream_publish_finished - stream_started),
            "settle_after_last_pose_s": (
                time.monotonic() - stream_publish_finished),
            "one_shot_generation": one_shot_generation,
            "final_generation": stream_generation,
            "position_error_m": stream_position_error,
            "angular_error_rad": stream_angular_error,
            "max_delta_qdot_rad_s": max_velocity_jump,
            "status": stream_status,
        }
        return 0 if stream_pass else 1
    finally:
        if health_heartbeat is not None:
            health_heartbeat.stop()
        if probe_context.node is not None:
            probe_context.__exit__(None, None, None)
        outcome["shutdown"] = launch.stop()
        write_json(results_dir / "fake_runtime.json", outcome)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=int, default=181)
    parser.add_argument("--results-dir")
    args = parser.parse_args()
    results = (
        pathlib.Path(args.results_dir)
        if args.results_dir else make_results_dir()
    )
    code = run_fake_runtime(args.domain_id, results)
    print((results / "fake_runtime.json").read_text())
    sys.exit(code)


if __name__ == "__main__":
    main()
