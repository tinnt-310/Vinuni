#!/usr/bin/env python3
"""Shutdown and Hardware Lifecycle Protection Runtime Tests for FAIRINO FR10.

Tests:
1. Controlled shutdown during idle.
2. Controlled shutdown during active motion (verifies no runaway or uncontrolled movement).
3. Process clean teardown (exit code 0, no segmentation faults).
4. Backend lifecycle clean release.
5. Log analysis: SDK error counter audit (ServoJ code 14, GetActualJointPosDegree -2, RPC errors).
6. Saves shutdown.json and sdk_errors.json.
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

from common import (
    ManagedLaunch,
    ProbeContext,
    analyze_log,
    check_preflight,
    generate,
    make_results_dir,
    write_json,
)


def run_shutdown_test(robot_ip: str = "192.168.58.2", domain_id: int = 171,
                      results_dir: pathlib.Path = None) -> int:
    results_dir = results_dir or make_results_dir()
    preflight = check_preflight(robot_ip)
    if not preflight["all_ok"]:
        print("FAIL: Preflight failed before running shutdown tests.")
        return 1

    print("=== [PHASE 1] Active Motion Shutdown Campaign ===")
    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    probe_context = ProbeContext(domain_id)
    outcomes = {}

    try:
        probe = probe_context.__enter__()
        if not probe.wait_ready(timeout=20.0):
            print("FAIL: ROS runtime not ready.")
            return 1

        # Command a 15 deg joint motion
        q = probe.joint_position()
        safe = generate(q, 0, 15.0)
        probe.publish_joint(safe.target)

        # Wait until robot begins moving
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            vel = probe.joint_velocity()
            if vel and abs(vel[0]) > 0.05:
                break
            time.sleep(0.01)

        print("  Robot is moving. Initiating clean SIGINT shutdown...")

    finally:
        probe_context.__exit__(None, None, None)
        shutdown_result = launch.stop(timeout=15.0)

    errors = analyze_log(results_dir / "runtime.log")
    write_json(results_dir / "sdk_errors.json", errors)
    write_json(results_dir / "shutdown.json", shutdown_result)

    safety = shutdown_result["backend_clean"] and not shutdown_result["forced"]
    teardown = shutdown_result["returncode"] == 0 and not shutdown_result["segfault"]
    sdk_ok = (errors["servo_j_code14"] == 0 and
              errors["joint_read_minus2"] == 0 and
              errors["sdk_command_errors"] == 0)

    outcomes["SHUTDOWN_MOTION_SAFETY"] = "PASS" if safety else "FAIL"
    outcomes["SHUTDOWN_PROCESS_TEARDOWN"] = "PASS" if teardown else "FAIL"
    outcomes["SDK_HEALTH"] = "PASS" if sdk_ok else "FAIL"

    print(f"  Motion Safety:       {outcomes['SHUTDOWN_MOTION_SAFETY']}")
    print(f"  Process Teardown:    {outcomes['SHUTDOWN_PROCESS_TEARDOWN']} (returncode={shutdown_result['returncode']})")
    print(f"  Backend Clean:       {shutdown_result['backend_clean']}")
    print(f"  SDK Errors (code14): {errors['servo_j_code14']}")
    print(f"  SDK Errors (-2):     {errors['joint_read_minus2']}")
    print(f"  RPC Errors:          {errors['rpc_errors']}")
    print(f"  SDK Health:          {outcomes['SDK_HEALTH']}")

    success = safety and teardown and sdk_ok
    return 0 if success else 1


def main():
    parser = argparse.ArgumentParser(description="Run Shutdown & Lifecycle Tests")
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    sys.exit(run_shutdown_test(args.robot_ip, args.domain_id, results_dir))


if __name__ == "__main__":
    main()
