#!/usr/bin/env python3
"""Single-pose Docker SimMachine proof for JOINT/POSE public modes."""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import time

from common import (
    ManagedLaunch,
    ProbeContext,
    analyze_log,
    check_preflight,
    make_results_dir,
    position_error,
    quaternion_error,
    write_json,
)


def rotate_local_z(quaternion):
    x, y, z, w = quaternion
    # Third column of the unit-quaternion rotation matrix.
    return [
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    ]


def line_deviation(point, start, goal):
    direction = [goal[i] - start[i] for i in range(3)]
    offset = [point[i] - start[i] for i in range(3)]
    length2 = sum(value * value for value in direction)
    if length2 < 1e-12:
        return math.inf
    scale = sum(offset[i] * direction[i] for i in range(3)) / length2
    residual = [offset[i] - scale * direction[i] for i in range(3)]
    return math.sqrt(sum(value * value for value in residual))


def segment_speeds(rows):
    speeds = []
    for previous, current in zip(rows, rows[1:]):
        dt = current[0] - previous[0]
        if dt > 1e-6:
            distance = position_error(previous[1], current[1])
            speeds.append(distance / dt)
    return speeds


def run(robot_ip: str, domain_id: int, results_dir: pathlib.Path) -> int:
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight.json", preflight)
    if not preflight["all_ok"]:
        print("FAIL: SimMachine preflight failed")
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    context = ProbeContext(domain_id)
    probe = None
    result = {"test": "TWO_MODE_SIMMACHINE"}
    try:
        probe = context.__enter__()
        if not probe.wait_ready(timeout=25.0):
            result.update(result="FAIL", reason="runtime not ready")
            return 1

        initial = probe.tcp_pose()
        axis = rotate_local_z(initial[1])
        pre_insert = [initial[0][0] + 0.080, initial[0][1], initial[0][2]]
        target = [pre_insert[i] + 0.200 * axis[i] for i in range(3)]
        with probe.lock:
            history_start = len(probe.tcp_history)
        started = time.monotonic()
        # Exactly one final Pose target must autonomously stage and insert.
        probe.publish_pose(target, initial[1])
        inserted = probe.wait_pose(
            target, initial[1], position_tolerance=0.001,
            angular_tolerance=math.radians(1.0), timeout=45.0)
        inserted_at = time.monotonic()
        final = probe.tcp_pose()
        with probe.lock:
            rows = list(probe.tcp_history[history_start:])

        closest_pre = min(
            range(len(rows)),
            key=lambda index: position_error(rows[index][1], pre_insert),
            default=0,
        )
        free_rows = rows[:closest_pre + 1]
        insertion_rows = rows[closest_pre:]
        free_speeds = segment_speeds(free_rows)
        insertion_speeds = segment_speeds(insertion_rows)
        max_perpendicular = max(
            (line_deviation(row[1], pre_insert, target)
             for row in insertion_rows),
            default=math.inf,
        )

        probe.publish_retract()
        retracted = probe.wait_pose(
            pre_insert, initial[1], position_tolerance=0.001,
            angular_tolerance=math.radians(1.0), timeout=30.0)
        probe.publish_pose(initial[0], initial[1])
        restored = probe.wait_pose(
            initial[0], initial[1], position_tolerance=0.002,
            angular_tolerance=math.radians(1.0), timeout=35.0)

        runtime_text = launch.log_path.read_text(errors="replace")
        diagnostics_ok = (
            "[TIME_PARAM_DIAGNOSTIC]" in runtime_text
            and "policy=FREE_SPACE" in runtime_text
            and "policy=INSERTION" in runtime_text
            and "[POSE POLICY TRANSITION]" in runtime_text
        )
        sdk = analyze_log(launch.log_path)
        sdk_error_total = sum(sdk.values())
        final_position_error = position_error(final[0], target)
        final_orientation_error = quaternion_error(final[1], initial[1])
        free_peak = max(free_speeds, default=0.0)
        insertion_peak = max(insertion_speeds, default=math.inf)
        passed = (
            inserted and retracted and restored and diagnostics_ok
            and final_position_error <= 0.001
            and final_orientation_error <= math.radians(1.0)
            and max_perpendicular <= 0.002
            and free_peak > 2.0 * insertion_peak
            and insertion_peak <= 0.025
            and sdk_error_total == 0
        )
        result.update(
            result="PASS" if passed else "FAIL",
            inserted=inserted,
            retracted=retracted,
            restored=restored,
            duration_s=inserted_at - started,
            final_position_error_m=final_position_error,
            final_orientation_error_rad=final_orientation_error,
            insertion_max_perpendicular_m=max_perpendicular,
            free_peak_speed_m_s=free_peak,
            insertion_peak_speed_m_s=insertion_peak,
            diagnostics_ok=diagnostics_ok,
            sdk_errors=sdk_error_total,
        )
        with (results_dir / "two_mode_tcp.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["monotonic_s", "x", "y", "z", "qx", "qy", "qz", "qw"])
            for stamp, position, quaternion in rows:
                writer.writerow([stamp, *position, *quaternion])
        return 0 if passed else 1
    finally:
        write_json(results_dir / "two_mode_summary.json", result)
        if probe is not None:
            probe.publish_controlled_stop()
            time.sleep(0.5)
        context.__exit__(None, None, None)
        shutdown = launch.stop(timeout=20.0)
        write_json(results_dir / "shutdown.json", shutdown)
        print(result)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=184)
    parser.add_argument("--results-dir")
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir) if args.results_dir else make_results_dir()
    results_dir.mkdir(parents=True, exist_ok=True)
    return run(args.robot_ip, args.domain_id, results_dir)


if __name__ == "__main__":
    raise SystemExit(main())
