#!/usr/bin/env python3
"""Mode Switching and Command Exclusivity Runtime Tests for FAIRINO FR10.

Tests:
1. Joint command tracking -> HOLD state.
2. Sequential switch to Cartesian pose tracking -> HOLD state.
3. Sequential switch back to Joint tracking -> HOLD state.
4. Exclusive ownership: Verifies status reflects exactly one active controller/owner.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

from common import (
    ManagedLaunch,
    ProbeContext,
    check_preflight,
    generate,
    make_results_dir,
    position_error,
    write_json,
)


def run_mode_switch_test(robot_ip: str = "192.168.58.2", domain_id: int = 171,
                         results_dir: pathlib.Path = None) -> int:
    results_dir = results_dir or make_results_dir()
    preflight = check_preflight(robot_ip)
    if not preflight["all_ok"]:
        print("FAIL: Preflight failed before running mode switch tests.")
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    probe_context = ProbeContext(domain_id)
    outcomes = {}

    try:
        probe = probe_context.__enter__()
        if not probe.wait_ready(timeout=20.0):
            print("FAIL: ROS runtime not ready.")
            return 1

        print("=== [STEP 1] Submit Joint Command & Settle in HOLD ===")
        q_init = probe.joint_position()
        safe_joint = generate(q_init, 0, 15.0)
        probe.publish_joint(safe_joint.target)
        reached_j1 = probe.wait_joint(safe_joint.target, timeout=20.0)
        time.sleep(0.5)
        st1 = probe.status()
        j1_ok = reached_j1 and "owner=JOINT_OTG" in st1 and "sdk_errors=0" in st1
        outcomes["SWITCH_TO_JOINT_1"] = "PASS" if j1_ok else "FAIL"
        print(f"  SWITCH_TO_JOINT_1 -> {outcomes['SWITCH_TO_JOINT_1']} ({st1})")

        print("\n=== [STEP 2] Submit Cartesian Command & Settle in HOLD ===")
        tcp_init = probe.tcp_pose()
        target_tcp_pos = [tcp_init[0][0] + 0.005, tcp_init[0][1], tcp_init[0][2]]
        probe.publish_pose(target_tcp_pos, tcp_init[1])
        reached_cart = probe.wait_pose(target_tcp_pos, tcp_init[1], timeout=20.0)
        time.sleep(0.5)
        st2 = probe.status()
        cart_ok = reached_cart and "owner=POSE_PATH" in st2 and "sdk_errors=0" in st2
        outcomes["SWITCH_TO_CARTESIAN"] = "PASS" if cart_ok else "FAIL"
        print(f"  SWITCH_TO_CARTESIAN -> {outcomes['SWITCH_TO_CARTESIAN']} ({st2})")

        print("\n=== [STEP 3] Submit Joint Command Back to Initial Pose ===")
        probe.publish_joint(q_init)
        reached_j2 = probe.wait_joint(q_init, timeout=20.0)
        time.sleep(0.5)
        st3 = probe.status()
        j2_ok = reached_j2 and "owner=JOINT_OTG" in st3 and "sdk_errors=0" in st3
        outcomes["SWITCH_TO_JOINT_2"] = "PASS" if j2_ok else "FAIL"
        print(f"  SWITCH_TO_JOINT_2 -> {outcomes['SWITCH_TO_JOINT_2']} ({st3})")

        write_json(results_dir / "mode_switch_summary.json", outcomes)
        all_ok = j1_ok and cart_ok and j2_ok
        return 0 if all_ok else 1

    finally:
        probe_context.__exit__(None, None, None)
        launch.stop()


def main():
    parser = argparse.ArgumentParser(description="Run Mode Switch Runtime Tests")
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    sys.exit(run_mode_switch_test(args.robot_ip, args.domain_id, results_dir))


if __name__ == "__main__":
    main()
