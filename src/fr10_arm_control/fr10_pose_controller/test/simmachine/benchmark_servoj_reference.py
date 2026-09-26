#!/usr/bin/env python3
"""Quantitative reference benchmark for the production Pose-to-ServoJ path."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import random
import statistics
import sys
import time

from common import (
    ManagedLaunch,
    ProbeContext,
    analyze_log,
    find_motion_owners,
    position_error,
    quaternion_error,
    rate_metrics,
    write_json,
)


def point_line_error_m(point, start, target):
    direction = [target[i] - start[i] for i in range(3)]
    relative = [point[i] - start[i] for i in range(3)]
    norm = math.sqrt(sum(value * value for value in direction))
    if norm <= 1e-12:
        return position_error(point, start)
    cross = [
        relative[1] * direction[2] - relative[2] * direction[1],
        relative[2] * direction[0] - relative[0] * direction[2],
        relative[0] * direction[1] - relative[1] * direction[0],
    ]
    return math.sqrt(sum(value * value for value in cross)) / norm


def rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # R(q) * v, ROS quaternion order XYZW.
    return [
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - z * w) * vy
        + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx + (1 - 2 * (x * x + z * z)) * vy
        + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx + 2 * (y * z + x * w) * vy
        + (1 - 2 * (x * x + y * y)) * vz,
    ]


def snapshot_rows(probe, start_index):
    with probe.lock:
        return list(probe.tcp_history[start_index:])


def wait_owner_holding(probe, target, quaternion, timeout=15.0):
    deadline = time.monotonic() + timeout
    observed_owner = False
    while time.monotonic() < deadline:
        status = probe.status()
        actual = probe.tcp_pose()
        observed_owner = observed_owner or "owner=POSE_PATH" in status
        if (observed_owner and "state=HOLDING" in status and actual
                and position_error(actual[0], target) <= 0.001
                and quaternion_error(actual[1], quaternion) <= 0.01):
            return True
        if "state=FAULT" in status:
            return False
        time.sleep(0.02)
    return False


def path_result(name, start, target, quaternion, rows, final, reached, status):
    errors = [point_line_error_m(position, start, target)
              for _, position, _ in rows]
    angular = [quaternion_error(actual_q, quaternion)
               for _, _, actual_q in rows]
    return {
        "test": name,
        "result": "PASS" if reached and "sdk_errors=0" in status else "FAIL",
        "sample_count": len(rows),
        "feedback_rate_hz": rate_metrics([row[0] for row in rows])["mean_hz"],
        "e_perp_max_mm": max(errors, default=0.0) * 1000.0,
        "rms_line_error_mm": (
            math.sqrt(statistics.fmean(value * value for value in errors)) * 1000.0
            if errors else 0.0),
        "endpoint_error_mm": position_error(final[0], target) * 1000.0,
        "orientation_error_deg": math.degrees(quaternion_error(final[1], quaternion)),
        "status": status,
    }


def run_path(probe, name, delta):
    initial = probe.tcp_pose()
    start, quaternion = list(initial[0]), list(initial[1])
    target = [start[index] + delta[index] for index in range(3)]
    with probe.lock:
        history_start = len(probe.tcp_history)
    probe.publish_pose(target, quaternion)
    reached = wait_owner_holding(probe, target, quaternion)
    final = probe.tcp_pose()
    rows = snapshot_rows(probe, history_start)
    result = path_result(
        name, start, target, quaternion, rows, final, reached, probe.status())
    return result, rows, (start, quaternion)


def return_to(probe, pose):
    position, quaternion = pose
    probe.publish_pose(position, quaternion)
    return wait_owner_holding(probe, position, quaternion)


def retarget(probe, high_hz):
    name = f"SJ_RETARGET_{int(high_hz)}HZ"
    anchor = probe.tcp_pose()
    update_period = 1.0 / high_hz
    begun = time.monotonic()
    next_update = begun
    update_index = 0
    with probe.lock:
        history_start = len(probe.tcp_history)
    while time.monotonic() - begun < 2.0:
        now = time.monotonic()
        if now >= next_update:
            target = list(anchor[0])
            target[0] += 0.0005 if update_index % 2 == 0 else -0.0005
            probe.publish_pose(target, anchor[1])
            update_index += 1
            next_update = begun + update_index * update_period
        time.sleep(0.001)
    rows = snapshot_rows(probe, history_start)
    max_step = max(
        (position_error(previous[1], current[1])
         for previous, current in zip(rows, rows[1:])), default=0.0)
    target = list(anchor[0])
    target[0] += -0.0005 if (update_index - 1) % 2 else 0.0005
    final = probe.tcp_pose()
    result = {
        "test": name,
        "result": "MEASURED" if "sdk_errors=0" in probe.status()
                  and "state=FAULT" not in probe.status() else "FAIL",
        "high_level_hz": update_index / max(time.monotonic() - begun, 1e-9),
        "low_level_hz": 125.0,
        "tcp_feedback_hz": rate_metrics([row[0] for row in rows])["mean_hz"],
        "tcp_samples": len(rows),
        "max_observed_tcp_step_mm": max_step * 1000.0,
        "final_error_to_last_target_mm": position_error(final[0], target) * 1000.0,
        "retarget_latency_ms": None,
        "latency_note": "not identifiable below the 10 Hz TCP telemetry period",
        "status": probe.status(),
    }
    return_to(probe, anchor)
    return result, rows


def noise_test(probe):
    anchor = probe.tcp_pose()
    rng = random.Random(393)
    begun = time.monotonic()
    submissions = 0
    with probe.lock:
        history_start = len(probe.tcp_history)
    while time.monotonic() - begun < 2.0:
        target = [anchor[0][axis] + rng.gauss(0.0, 0.0005) for axis in range(3)]
        # Small-angle deterministic quaternion perturbation is intentionally
        # omitted here: changing translation and orientation together makes a
        # failed IK/retarget impossible to attribute to one variable.
        probe.publish_pose(target, anchor[1])
        submissions += 1
        time.sleep(max(0.0, begun + submissions * 0.1 - time.monotonic()))
    rows = snapshot_rows(probe, history_start)
    displacement = [position_error(row[1], anchor[0]) for row in rows]
    result = {
        "test": "SJ_NOISE_TRANSLATION",
        "result": "MEASURED" if "sdk_errors=0" in probe.status()
                  and "state=FAULT" not in probe.status() else "FAIL",
        "seed": 393,
        "input_translation_sigma_mm": 0.5,
        "high_level_hz": submissions / max(time.monotonic() - begun, 1e-9),
        "actual_rms_displacement_mm": (
            math.sqrt(statistics.fmean(value * value for value in displacement)) * 1000.0
            if displacement else 0.0),
        "actual_max_displacement_mm": max(displacement, default=0.0) * 1000.0,
        "tcp_samples": len(rows),
        "status": probe.status(),
    }
    return_to(probe, anchor)
    return result, rows


def write_tcp_csv(path, grouped_rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["test", "monotonic_s", "x_m", "y_m", "z_m",
                         "qx", "qy", "qz", "qw"])
        for name, rows in grouped_rows:
            for stamp, position, quaternion in rows:
                writer.writerow([name, stamp, *position, *quaternion])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=194)
    parser.add_argument("--results-dir", required=True)
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)

    owners = find_motion_owners()
    if not owners["pass"]:
        write_json(results_dir / "duplicate_owners.json", owners)
        print(json.dumps(owners, indent=2))
        return 2

    launch = ManagedLaunch.start(results_dir, args.robot_ip, args.domain_id)
    probe_context = ProbeContext(args.domain_id)
    probe = None
    outcomes = []
    grouped_rows = []
    try:
        probe = probe_context.__enter__()
        if not probe.wait_ready(timeout=25.0) or launch.process.poll() is not None:
            print(launch.log_path.read_text(errors="replace")[-8000:])
            return 3

        with probe.lock:
            rate_start = len(probe.joint_stamps)
        time.sleep(2.0)
        with probe.lock:
            timing = rate_metrics(list(probe.joint_stamps[rate_start:]))
        write_json(results_dir / "servo_j_rate.json", timing)

        path_cases = [
            ("SJ_STATIC_TARGET_2MM", [0.002, 0.0, 0.0]),
            ("SJ_STRAIGHT_PATH_2MM", [0.0, 0.002, 0.0]),
            ("SJ_POS_X", [0.002, 0.0, 0.0]),
            ("SJ_NEG_X", [-0.002, 0.0, 0.0]),
            ("SJ_POS_Y", [0.0, 0.002, 0.0]),
            ("SJ_NEG_Y", [0.0, -0.002, 0.0]),
            ("SJ_POS_Z", [0.0, 0.0, 0.002]),
            ("SJ_NEG_Z", [0.0, 0.0, -0.002]),
        ]
        for name, delta in path_cases:
            result, rows, initial = run_path(probe, name, delta)
            outcomes.append(result)
            grouped_rows.append((name, rows))
            return_to(probe, initial)

        for local_name, local_axis in (
                ("SJ_LOCAL_POS_Z", [0.0, 0.0, 1.0]),
                ("SJ_LOCAL_POS_X", [1.0, 0.0, 0.0])):
            initial = probe.tcp_pose()
            direction = rotate_vector(initial[1], local_axis)
            result, rows, start = run_path(
                probe, local_name, [0.002 * value for value in direction])
            outcomes.append(result)
            grouped_rows.append((local_name, rows))
            return_to(probe, start)

        for high_hz in (10.0, 20.0):
            result, rows = retarget(probe, high_hz)
            outcomes.append(result)
            grouped_rows.append((result["test"], rows))

        interruption_initial = probe.tcp_pose()
        result, rows, start = run_path(
            probe, "SJ_HIGH_LEVEL_STREAM_INTERRUPTION", [0.002, 0.0, 0.0])
        result["behavior"] = (
            "last accepted Pose completed to HOLDING while low-level ServoJ continued"
            if result["result"] == "PASS" else "last target did not complete")
        outcomes.append(result)
        grouped_rows.append((result["test"], rows))
        return_to(probe, start)

        outcomes.append({
            "test": "SJ_CONTROLLED_STOP",
            "result": "NOT_IMPLEMENTED",
            "reason": "no public controlled-stop command; existing core regression is red",
        })
        noise, rows = noise_test(probe)
        outcomes.append(noise)
        grouped_rows.append((noise["test"], rows))

        write_json(results_dir / "servo_j_results.json", {
            "timing": timing,
            "tcp_feedback_limitation": "10 Hz diagnostic FK telemetry",
            "results": outcomes,
        })
        write_tcp_csv(results_dir / "servo_j_tcp_samples.csv", grouped_rows)
        return 0
    finally:
        if probe is not None:
            probe_context.__exit__(None, None, None)
        shutdown = launch.stop(timeout=20.0)
        write_json(results_dir / "shutdown.json", shutdown)
        write_json(results_dir / "sdk_errors.json", analyze_log(launch.log_path))


if __name__ == "__main__":
    sys.exit(main())
