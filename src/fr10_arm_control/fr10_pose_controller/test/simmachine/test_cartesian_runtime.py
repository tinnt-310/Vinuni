#!/usr/bin/env python3
"""Cartesian runtime motion tests for FAIRINO FR10.

Tests:
1. Cartesian Preconditioning (ensure sufficient joint margin).
2. Small Translations (+/- 5mm along X, Y, Z).
3. Diagonal Translation (5mm XYZ combined).
4. Orientation Rotations (5° around RX, RY, RZ).
5. Combined SE3 (5mm translation + 5° rotation).
6. Quaternion Sign Equivalence (q vs -q produces zero motion).
7. Large Cartesian safety audit & limits documentation.
8. Saves cartesian_small.csv and cartesian_large.csv.
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


def run_cartesian_tests(robot_ip: str = "192.168.58.2", domain_id: int = 171,
                        results_dir: pathlib.Path = None) -> int:
    results_dir = results_dir or make_results_dir()
    preflight = check_preflight(robot_ip)
    if not preflight["all_ok"]:
        print("FAIL: Preflight failed before running Cartesian tests.")
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
            outcomes["CART_PRECONDITION"] = "PASS" if precondition_ok else "FAIL"
            print(f"  Adjusted joints {adjusted_axes}: {outcomes['CART_PRECONDITION']}")
        else:
            outcomes["CART_PRECONDITION"] = "PASS"
            print("  Joint margins sufficient, no preconditioning needed.")

        print("\n=== [PHASE 2] Small Axis-Aligned Translations (+/- 5mm) ===")
        all_passed = True
        tests = [
            (0, 0.005, "+X"), (0, -0.005, "-X"),
            (1, 0.005, "+Y"), (1, -0.005, "-Y"),
            (2, 0.005, "+Z"), (2, -0.005, "-Z"),
        ]

        for axis, delta, name in tests:
            start = probe.tcp_pose()
            target_pos = list(start[0])
            target_pos[axis] += delta
            label = "CART_5MM_" + name.replace("+", "POS_").replace("-", "NEG_")

            history_start = len(probe.tcp_history)
            probe.publish_pose(target_pos, start[1])
            reached = probe.wait_pose(target_pos, start[1], timeout=15.0)
            final = probe.tcp_pose()

            pos_err = position_error(final[0], target_pos) if final else math.inf
            ang_err = quaternion_error(final[1], start[1]) if final else math.inf
            status = probe.status()
            passed = reached and pos_err <= 0.002 and ang_err <= 0.02 and "sdk_errors=0" in status

            outcomes[label] = "PASS" if passed else "FAIL"
            print(f"  {label:<25} -> {outcomes[label]} (pos_err={pos_err*1000:.2f}mm, ang_err={ang_err:.4f}rad)")
            if not passed:
                all_passed = False

            if axis == 0 and delta > 0:
                # Save as cartesian_small.csv
                with probe.lock:
                    rows = list(probe.tcp_history[history_start:])
                with open(results_dir / "cartesian_small.csv", "w") as f:
                    f.write("monotonic_s,x,y,z,qx,qy,qz,qw\n")
                    for stamp, p, q in rows:
                        f.write(f"{stamp},{p[0]},{p[1]},{p[2]},{q[0]},{q[1]},{q[2]},{q[3]}\n")

            probe.publish_pose(start[0], start[1])
            probe.wait_pose(start[0], start[1], timeout=15.0)

        print("\n=== [PHASE 3] Quaternion Sign Invariance (q vs -q) ===")
        start = probe.tcp_pose()
        probe.publish_pose(start[0], [-v for v in start[1]])
        time.sleep(1.0)
        final = probe.tcp_pose()
        q_motion = position_error(final[0], start[0]) if final else math.inf
        q_angle = quaternion_error(final[1], start[1]) if final else math.inf
        q_pass = q_motion < 0.001 and q_angle < 0.005 and "sdk_errors=0" in probe.status()
        outcomes["CART_Q_NEG_Q"] = "PASS" if q_pass else "FAIL"
        print(f"  CART_Q_NEG_Q              -> {outcomes['CART_Q_NEG_Q']} (motion={q_motion*1000:.3f}mm)")
        if not q_pass:
            all_passed = False

        print("\n=== [PHASE 4] Diagonal & Rotational SE3 Motions ===")
        # Diagonal 5mm
        start = probe.tcp_pose()
        diagonal = [start[0][i] + 0.005 for i in range(3)]
        probe.publish_pose(diagonal, start[1])
        reached = probe.wait_pose(diagonal, start[1], timeout=15.0)
        final = probe.tcp_pose()
        diag_ok = (reached and position_error(final[0], diagonal) <= 0.002 and
                   quaternion_error(final[1], start[1]) <= 0.02 and "sdk_errors=0" in probe.status())
        outcomes["CART_DIAGONAL_5MM"] = "PASS" if diag_ok else "FAIL"
        print(f"  CART_DIAGONAL_5MM         -> {outcomes['CART_DIAGONAL_5MM']}")
        probe.publish_pose(start[0], start[1])
        probe.wait_pose(start[0], start[1], timeout=15.0)

        # 5 deg orientation rotations
        for axis, name in enumerate(("RX", "RY", "RZ")):
            start = probe.tcp_pose()
            target_q = quaternion_multiply(start[1], axis_angle_quaternion(axis, math.radians(5.0)))
            probe.publish_pose(start[0], target_q)
            reached = probe.wait_pose(start[0], target_q, timeout=20.0)
            final = probe.tcp_pose()
            pos_err = position_error(final[0], start[0]) if final else math.inf
            ang_err = quaternion_error(final[1], target_q) if final else math.inf
            passed = reached and pos_err <= 0.002 and ang_err <= 0.02 and "sdk_errors=0" in probe.status()
            outcomes[f"CART_ORIENTATION_{name}_5DEG"] = "PASS" if passed else "FAIL"
            print(f"  CART_ORIENTATION_{name}_5DEG   -> {outcomes[f'CART_ORIENTATION_{name}_5DEG']}")
            probe.publish_pose(start[0], start[1])
            probe.wait_pose(start[0], start[1], timeout=20.0)

        # Combined 5mm + 5 deg
        start = probe.tcp_pose()
        target_pos = [start[0][0] + 0.005, start[0][1] + 0.005, start[0][2]]
        target_q = quaternion_multiply(start[1], axis_angle_quaternion(2, math.radians(5.0)))
        probe.publish_pose(target_pos, target_q)
        reached = probe.wait_pose(target_pos, target_q, timeout=20.0)
        final = probe.tcp_pose()
        comb_ok = (reached and position_error(final[0], target_pos) <= 0.002 and
                   quaternion_error(final[1], target_q) <= 0.02 and "sdk_errors=0" in probe.status())
        outcomes["CART_COMBINED_5MM_5DEG"] = "PASS" if comb_ok else "FAIL"
        print(f"  CART_COMBINED_5MM_5DEG    -> {outcomes['CART_COMBINED_5MM_5DEG']}")
        probe.publish_pose(start[0], start[1])
        probe.wait_pose(start[0], start[1], timeout=20.0)

        print("\n=== [PHASE 5] Large Cartesian Safety Documentation ===")
        safety_reason = ("runtime validates continuous IK and singularity barriers "
                         "but does not yet expose PlanningScene collision validation")
        for t_id in ("CART_50MM", "CART_100MM", "CART_200MM", "CART_LARGE_3D", "CART_LARGE_SE3"):
            outcomes[t_id] = "BLOCKED"
            print(f"  {t_id:<25} -> BLOCKED ({safety_reason})")

        # Save dummy/blocked record for cartesian_large
        with open(results_dir / "cartesian_large.csv", "w") as f:
            f.write("# Large Cartesian motions require MoveIt collision & singularity validation\n")

        write_json(results_dir / "cartesian_summary.json", outcomes)
        return 0 if all_passed else 1

    finally:
        probe_context.__exit__(None, None, None)
        launch.stop()


def main():
    parser = argparse.ArgumentParser(description="Run Cartesian Runtime Tests")
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    sys.exit(run_cartesian_tests(args.robot_ip, args.domain_id, results_dir))


if __name__ == "__main__":
    main()
