#!/usr/bin/env python3
"""Live deterministic long-distance Cartesian validation on FAIRINO SimMachine."""

from __future__ import annotations

import argparse
import csv
import json
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


CASES = (
    ("LD_01_CROSS_X", [-0.500, 0.400, 0.400], [0.500, 0.400, 0.400]),
    ("LD_02_CROSS_Y", [0.450, -0.500, 0.350], [0.450, 0.500, 0.350]),
    ("LD_03_SPATIAL_DIAGONAL",
     [-0.400, -0.400, 0.200], [0.450, 0.450, 0.750]),
)


def status_value(status: str, key: str):
    prefix = key + "="
    for field in status.split():
        if field.startswith(prefix):
            try:
                return float(field[len(prefix):])
            except ValueError:
                return None
    return None


def max_joint_step(rows) -> float:
    maximum = 0.0
    for previous, current in zip(rows, rows[1:]):
        maximum = max(
            maximum,
            max(abs(a - b) for a, b in zip(previous[1], current[1])),
        )
    return maximum


def wrist_radius(position, quaternion) -> float:
    """Planar radius of wrist3 from the configured wrist3->tool0 transform."""
    x, y, z, w = quaternion
    rotation = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)),
    )
    wrist3_to_tcp = (0.060, 0.0, 0.268)
    offset = [sum(rotation[row][column] * wrist3_to_tcp[column]
                  for column in range(3)) for row in range(3)]
    return math.hypot(position[0] - offset[0], position[1] - offset[1])


def write_tcp_csv(path: pathlib.Path, rows) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["monotonic_s", "x", "y", "z",
                         "qx", "qy", "qz", "qw"])
        for stamp, position, quaternion in rows:
            writer.writerow([stamp, *position, *quaternion])


def run(robot_ip: str, domain_id: int, results_dir: pathlib.Path) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight.json", preflight)
    if not preflight["all_ok"]:
        write_json(results_dir / "long_distance_summary.json", {
            "result": "FAIL", "reason": "preflight failed"})
        return 2

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    context = ProbeContext(domain_id)
    probe = None
    outcomes = {}
    try:
        probe = context.__enter__()
        if not probe.wait_ready(timeout=30.0):
            outcomes["RUNTIME"] = {"result": "FAIL", "reason": "not ready"}
            return 1
        # Keep tool0 +Z along base +Y.  The Cartesian coordinates in this
        # matrix intentionally leave orientation unspecified; inheriting an
        # arbitrary previous-test wrist attitude can turn an otherwise valid
        # endpoint into a self-collision during the mandatory 200 mm staging
        # segment.  This fixed connector attitude makes every run repeatable.
        # RPY [-180, -60, -60] deg.  A deterministic offline scan starts at
        # the live auto-synchronised joint state and validates the complete
        # six-endpoint FREE_SPACE -> INSERTION sequence, not endpoints alone.
        orientation = [-0.75, 0.4330127019, -0.4330127019, -0.25]

        for test_id, start, goal in CASES:
            probe.publish_pose(start, orientation)
            start_reached = probe.wait_pose(
                start, orientation, position_tolerance=0.002,
                angular_tolerance=math.radians(1.0), timeout=150.0)
            if not start_reached:
                outcomes[test_id] = {
                    "result": "FAIL", "stage": "start",
                    "status": probe.status(), "target": start,
                }
                break

            with probe.lock:
                tcp_start = len(probe.tcp_history)
                joint_start = len(probe.joint_history)
                status_start = len(probe.status_history)
            begun = time.monotonic()
            probe.publish_pose(goal, orientation)
            reached = probe.wait_pose(
                goal, orientation, position_tolerance=0.002,
                angular_tolerance=math.radians(1.0), timeout=180.0)
            duration_s = time.monotonic() - begun
            final = probe.tcp_pose()
            with probe.lock:
                tcp_rows = list(probe.tcp_history[tcp_start:])
                joint_rows = list(probe.joint_history[joint_start:])
                statuses = [value for _, value
                            in probe.status_history[status_start:]]

            min_tcp_radius_m = min(
                (math.hypot(row[1][0], row[1][1]) for row in tcp_rows),
                default=math.inf,
            )
            min_wrist_radius_m = min(
                (wrist_radius(row[1], row[2]) for row in tcp_rows),
                default=math.inf,
            )
            maximum_path_deviation_m = max(
                (value for value in
                 (status_value(status, "path_deviation_m")
                  for status in statuses) if value is not None),
                default=math.inf,
            )
            final_position_error_m = (
                position_error(final[0], goal) if final else math.inf)
            final_orientation_error_rad = (
                quaternion_error(final[1], orientation)
                if final else math.inf)
            adjacent_joint_step_rad = max_joint_step(joint_rows)
            status = probe.status()
            fault_seen = any(
                "state=FAULT" in item or "sdk_errors=0" not in item
                for item in statuses)
            passed = (
                reached and final_position_error_m <= 0.002
                and final_orientation_error_rad <= math.radians(1.0)
                and min_wrist_radius_m >= 0.350 - 0.002
                and maximum_path_deviation_m <= 0.050
                and adjacent_joint_step_rad <= 0.10
                and not fault_seen and "state=HOLDING" in status
                and "sdk_errors=0" in status
            )
            outcomes[test_id] = {
                "result": "PASS" if passed else "FAIL",
                "start": start, "goal": goal, "final": final,
                "duration_s": duration_s,
                "final_position_error_m": final_position_error_m,
                "final_orientation_error_rad": final_orientation_error_rad,
                "min_tcp_radius_m": min_tcp_radius_m,
                "min_wrist_radius_m": min_wrist_radius_m,
                "max_path_deviation_m": maximum_path_deviation_m,
                "max_adjacent_joint_step_rad": adjacent_joint_step_rad,
                "status": status,
            }
            write_tcp_csv(results_dir / f"{test_id.lower()}_tcp.csv", tcp_rows)
            if not passed:
                break

        runtime_text = launch.log_path.read_text(errors="replace")
        outcomes["DETERMINISTIC_CYLINDER_EVIDENCE"] = {
            "result": ("PASS" if "deterministic cylindrical" in
                       runtime_text.lower() else "NOT_OBSERVED"),
            "note": "fallback is emitted only when direct route validation fails",
        }
        return 0 if all(
            item.get("result") == "PASS"
            for key, item in outcomes.items()
            if key.startswith("LD_")) and len(
                [key for key in outcomes if key.startswith("LD_")]) == 3 else 1
    finally:
        if probe is not None:
            try:
                probe.publish_controlled_stop()
                time.sleep(0.5)
            except Exception:
                pass
        try:
            context.__exit__(None, None, None)
        finally:
            shutdown = launch.stop(timeout=20.0)
        sdk = analyze_log(launch.log_path)
        outcomes["SDK_AUDIT"] = {
            "result": "PASS" if sum(sdk.values()) == 0 else "FAIL", **sdk}
        outcomes["SHUTDOWN"] = shutdown
        write_json(results_dir / "long_distance_summary.json", outcomes)
        report = ["# Cartesian long-distance SimMachine report", ""]
        for key, value in outcomes.items():
            report.append(f"- {key}: {value.get('result', 'INFO')}")
        (results_dir / "long_distance_report.md").write_text(
            "\n".join(report) + "\n")
        print(json.dumps(outcomes, indent=2, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir")
    args = parser.parse_args()
    results_dir = (pathlib.Path(args.results_dir) if args.results_dir
                   else make_results_dir())
    return run(args.robot_ip, args.domain_id, results_dir)


if __name__ == "__main__":
    raise SystemExit(main())
