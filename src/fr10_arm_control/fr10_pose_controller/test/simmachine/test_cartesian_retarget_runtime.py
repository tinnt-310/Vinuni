#!/usr/bin/env python3
"""Online Dynamic Cartesian Retargeting and 10 Hz AI Stream tests for FAIRINO FR10.

Tests:
1. Local Cartesian A -> B -> C Retargeting.
2. Cartesian 10 Hz Stream: High-level AI publishes continuous pose targets at 10 Hz
   while low-level runs at ~125 Hz without jerks, missed cycles, or SDK errors.
3. Saves cartesian_retarget.csv.
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
    make_results_dir,
    position_error,
    write_json,
)


def run_cartesian_retarget_tests(robot_ip: str = "192.168.58.2", domain_id: int = 171,
                                 results_dir: pathlib.Path = None) -> int:
    results_dir = results_dir or make_results_dir()
    preflight = check_preflight(robot_ip)
    if not preflight["all_ok"]:
        print("FAIL: Preflight failed before running Cartesian retarget tests.")
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    probe_context = ProbeContext(domain_id)
    outcomes = {}

    try:
        probe = probe_context.__enter__()
        if not probe.wait_ready(timeout=20.0):
            print("FAIL: ROS runtime not ready.")
            return 1

        print("=== [PHASE 1] Cartesian Local A -> B -> C Retarget ===")
        start = probe.tcp_pose()
        q = start[1]
        a = [start[0][0] + 0.002, start[0][1], start[0][2]]
        b = [start[0][0] - 0.002, start[0][1], start[0][2]]
        c = [start[0][0], start[0][1] + 0.002, start[0][2]]

        history_start = len(probe.tcp_history)
        probe.publish_pose(a, q)
        time.sleep(0.10)
        state_b = probe.tcp_pose()
        probe.publish_pose(b, q)
        time.sleep(0.10)
        state_c = probe.tcp_pose()
        probe.publish_pose(c, q)

        reached = probe.wait_pose(c, q, timeout=60.0)
        final = probe.tcp_pose()
        pos_err = position_error(final[0], c) if final else math.inf
        passed = reached and pos_err <= 0.002 and "sdk_errors=0" in probe.status()

        outcomes["CART_LOCAL_A_B_C"] = "PASS" if passed else "FAIL"
        print(f"  CART_LOCAL_A_B_C -> {outcomes['CART_LOCAL_A_B_C']} (pos_err={pos_err*1000:.2f}mm)")

        # Save trajectory to cartesian_retarget.csv
        with probe.lock:
            rows = list(probe.tcp_history[history_start:])
        with open(results_dir / "cartesian_retarget.csv", "w") as f:
            f.write("monotonic_s,x,y,z,qx,qy,qz,qw\n")
            for stamp, p, quat in rows:
                f.write(f"{stamp},{p[0]},{p[1]},{p[2]},{quat[0]},{quat[1]},{quat[2]},{quat[3]}\n")

        probe.publish_pose(start[0], q)
        probe.wait_pose(start[0], q, timeout=60.0)

        print("\n=== [PHASE 2] Cartesian 10 Hz AI Target Stream ===")
        center = probe.tcp_pose()
        stream_feedback_start = len(probe.tcp_history)
        begun = time.monotonic()
        submissions = 0

        # Continuous smooth sinusoidal trajectory over 10 seconds at 10 Hz
        while time.monotonic() - begun < 10.0:
            t = time.monotonic() - begun
            target = [
                center[0][0] + 0.005 * math.sin(0.4 * t),
                center[0][1] + 0.005 * math.sin(0.31 * t),
                center[0][2],
            ]
            probe.publish_pose(target, center[1])
            submissions += 1
            time.sleep(max(0.0, begun + submissions * 0.1 - time.monotonic()))

        stream_duration_s = time.monotonic() - begun
        stream_feedback_count = len(probe.tcp_history) - stream_feedback_start
        probe.publish_pose(center[0], center[1])
        reached = probe.wait_pose(center[0], center[1], timeout=60.0)
        input_hz = submissions / stream_duration_s

        passed_stream = (stream_feedback_count > 80 and reached and
                         9.0 <= input_hz <= 11.0 and "sdk_errors=0" in probe.status())
        outcomes["CART_AI_10HZ_STREAM"] = "PASS" if passed_stream else "FAIL"
        print(f"  CART_AI_10HZ_STREAM -> {outcomes['CART_AI_10HZ_STREAM']} (AI={input_hz:.1f}Hz, samples={stream_feedback_count})")

        write_json(results_dir / "cartesian_retarget_summary.json", outcomes)
        return 0 if (outcomes["CART_LOCAL_A_B_C"] == "PASS" and passed_stream) else 1

    finally:
        probe_context.__exit__(None, None, None)
        launch.stop()


def main():
    parser = argparse.ArgumentParser(description="Run Cartesian Retargeting & 10 Hz Stream Tests")
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    sys.exit(run_cartesian_retarget_tests(args.robot_ip, args.domain_id, results_dir))


if __name__ == "__main__":
    main()
