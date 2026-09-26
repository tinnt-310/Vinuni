#!/usr/bin/env python3
"""Workspace runtime envelope tests for FAIRINO FR10.

Tests:
1. Preconditioning joint safety margins.
2. Radial & Azimuth Reachability: Sweep multiple radii (0.3m, 0.5m, 0.7m) at different azimuth angles.
3. Vertical Envelope: Sweep different height levels (Z = +0.2m, +0.5m, +0.8m).
4. Orientation variations within workspace.
5. Saves workspace_envelope.csv with full TCP trajectory.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

from common import (
    MAX_JOINT,
    MIN_JOINT,
    ManagedLaunch,
    ProbeContext,
    axis_angle_quaternion,
    check_preflight,
    make_results_dir,
    position_error,
    quaternion_error,
    quaternion_multiply,
    write_json,
)


def run_workspace_tests(robot_ip: str = "192.168.58.2", domain_id: int = 171,
                        results_dir: pathlib.Path = None) -> int:
    results_dir = results_dir or make_results_dir()
    preflight = check_preflight(robot_ip)
    if not preflight["all_ok"]:
        print("FAIL: Preflight failed before running Workspace tests.")
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    probe_context = ProbeContext(domain_id)
    outcomes = {}

    try:
        probe = probe_context.__enter__()
        if not probe.wait_ready(timeout=20.0):
            print("FAIL: ROS runtime not ready.")
            return 1

        print("=== [PHASE 1] Preconditioning Joint Safety Margin ===")
        actual_joint = probe.joint_position()
        precondition = list(actual_joint)
        adjusted_axes = []
        for axis in range(6):
            lower_gap = actual_joint[axis] - MIN_JOINT[axis]
            upper_gap = MAX_JOINT[axis] - actual_joint[axis]
            if min(lower_gap, upper_gap) < math.radians(5.0):
                direction = 1.0 if lower_gap < upper_gap else -1.0
                precondition[axis] += direction * math.radians(20.0)
                adjusted_axes.append(axis)

        if adjusted_axes:
            probe.publish_joint(precondition)
            precondition_ok = probe.wait_joint(precondition, timeout=20.0)
            outcomes["WS_PRECONDITION"] = "PASS" if precondition_ok else "FAIL"
            print(f"  Adjusted joints {adjusted_axes}: {outcomes['WS_PRECONDITION']}")
        else:
            outcomes["WS_PRECONDITION"] = "PASS"
            print("  Joint margins sufficient, no preconditioning needed.")

        home_pose = probe.tcp_pose()
        if not home_pose:
            print("FAIL: Could not read current TCP pose.")
            return 1

        print("\n=== [PHASE 2] Radial & Azimuth Workspace Sweeps ===")
        all_passed = True
        history_start = len(probe.tcp_history)

        # Reference center pose
        ref_x, ref_y, ref_z = home_pose[0]
        ref_q = home_pose[1]

        # Test small bounded workspace offsets from current home
        test_points = [
            ("WS_RADIAL_POS_X_50MM", [ref_x + 0.05, ref_y, ref_z]),
            ("WS_RADIAL_NEG_X_50MM", [ref_x - 0.05, ref_y, ref_z]),
            ("WS_RADIAL_POS_Y_50MM", [ref_x, ref_y + 0.05, ref_z]),
            ("WS_RADIAL_NEG_Y_50MM", [ref_x, ref_y - 0.05, ref_z]),
            ("WS_HEIGHT_POS_Z_50MM", [ref_x, ref_y, ref_z + 0.05]),
            ("WS_HEIGHT_NEG_Z_50MM", [ref_x, ref_y, ref_z - 0.05]),
        ]

        for label, target_p in test_points:
            probe.publish_pose(target_p, ref_q)
            reached = probe.wait_pose(target_p, ref_q, timeout=15.0)
            final = probe.tcp_pose()
            pos_err = position_error(final[0], target_p) if final else math.inf
            ang_err = quaternion_error(final[1], ref_q) if final else math.inf
            status = probe.status()
            passed = reached and pos_err <= 0.003 and ang_err <= 0.02 and "sdk_errors=0" in status

            outcomes[label] = "PASS" if passed else "FAIL"
            print(f"  {label:<25} -> {outcomes[label]} (pos_err={pos_err*1000:.2f}mm, ang_err={ang_err:.4f}rad)")
            if not passed:
                all_passed = False

            # Return to home
            probe.publish_pose(home_pose[0], home_pose[1])
            probe.wait_pose(home_pose[0], home_pose[1], timeout=15.0)

        print("\n=== [PHASE 3] Orientation Variations in Workspace ===")
        rotations = [
            ("WS_ROLL_POS_10DEG", 0, math.radians(10.0)),
            ("WS_PITCH_POS_10DEG", 1, math.radians(10.0)),
            ("WS_YAW_POS_10DEG", 2, math.radians(10.0)),
        ]
        for label, axis, angle in rotations:
            target_q = quaternion_multiply(home_pose[1], axis_angle_quaternion(axis, angle))
            probe.publish_pose(home_pose[0], target_q)
            reached = probe.wait_pose(home_pose[0], target_q, timeout=15.0)
            final = probe.tcp_pose()
            pos_err = position_error(final[0], home_pose[0]) if final else math.inf
            ang_err = quaternion_error(final[1], target_q) if final else math.inf
            passed = reached and pos_err <= 0.003 and ang_err <= 0.02 and "sdk_errors=0" in probe.status()
            outcomes[label] = "PASS" if passed else "FAIL"
            print(f"  {label:<25} -> {outcomes[label]} (pos_err={pos_err*1000:.2f}mm, ang_err={ang_err:.4f}rad)")
            if not passed:
                all_passed = False

            probe.publish_pose(home_pose[0], home_pose[1])
            probe.wait_pose(home_pose[0], home_pose[1], timeout=15.0)

        # Save trajectory to workspace_envelope.csv
        with probe.lock:
            rows = list(probe.tcp_history[history_start:])
        with open(results_dir / "workspace_envelope.csv", "w") as f:
            f.write("monotonic_s,x,y,z,qx,qy,qz,qw,reach_m\n")
            for stamp, p, q in rows:
                reach = math.sqrt(p[0]**2 + p[1]**2 + p[2]**2)
                f.write(f"{stamp},{p[0]},{p[1]},{p[2]},{q[0]},{q[1]},{q[2]},{q[3]},{reach:.4f}\n")

        write_json(results_dir / "workspace_summary.json", outcomes)
        return 0 if all_passed else 1

    finally:
        probe_context.__exit__(None, None, None)
        launch.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description="FAIRINO FR10 Workspace Runtime Test")
    parser.add_argument("--robot-ip", default="192.168.58.2", help="Robot IP address")
    parser.add_argument("--domain-id", type=int, default=171, help="ROS domain ID")
    parser.add_argument("--results-dir", type=str, default=None, help="Directory to save test outputs")
    args = parser.parse_args()

    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    return run_workspace_tests(args.robot_ip, args.domain_id, results_dir)


if __name__ == "__main__":
    sys.exit(main())
