#!/usr/bin/env python3
"""Automated ROS2 startup and Auto-Sync validation test for FAIRINO FR10.

Tests:
1. SimMachine Preflight (Docker, Ping, Ports, Process exclusivity)
2. ROS2 Stack launch and lifecycle readiness
3. Topic availability: /joint_states, /fr10_motion/tcp_pose, /controller_status
4. TF fr10_base_link -> charging_tcp live transform lookup
5. AUTO_SYNC_WITHOUT_RVIZ: Verify stack initializes from measured robot pose
   without requiring RViz or dummy trajectories.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

from common import (
    ManagedLaunch,
    ProbeContext,
    check_preflight,
    make_results_dir,
    write_json,
)


def run_startup_test(robot_ip: str = "192.168.58.2", domain_id: int = 171,
                     results_dir: pathlib.Path = None) -> int:
    results_dir = results_dir or make_results_dir()
    outcomes = {}

    print("=== [PHASE 1] Preflight Checks ===")
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight.json", preflight)
    outcomes["DOCKER"] = "PASS" if preflight["docker_ok"] else "FAIL"
    outcomes["NETWORK"] = "PASS" if preflight["network_ok"] else "FAIL"
    outcomes["DUPLICATE_OWNER"] = "PASS" if preflight["owner_ok"] else "FAIL"
    print(f"  Docker running: {outcomes['DOCKER']}")
    print(f"  Network & SDK ports: {outcomes['NETWORK']}")
    print(f"  Process exclusivity: {outcomes['DUPLICATE_OWNER']}")

    if not preflight["all_ok"]:
        print("FAIL: Preflight checks did not pass.")
        write_json(results_dir / "startup.json", outcomes)
        return 1

    print("\n=== [PHASE 2] ROS2 Launch & Topic Readiness ===")
    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    probe_context = ProbeContext(domain_id)
    try:
        probe = probe_context.__enter__()
        ready = probe.wait_ready(timeout=20.0)
        if not ready or launch.process.poll() is not None:
            outcomes["ROS_STARTUP"] = "FAIL"
            print("  FAIL: ROS node did not reach ready state.")
            write_json(results_dir / "startup.json", outcomes)
            return 1

        initial_joint = probe.joint_position()
        initial_tcp = probe.tcp_pose()
        time.sleep(1.0)
        tf_ok = probe.tf_connected()

        outcomes["ROS_STARTUP"] = "PASS"
        outcomes["TF_FR10_BASE_CHARGING_TCP"] = "PASS" if tf_ok else "FAIL"
        outcomes["INITIAL_JOINT_RAD"] = initial_joint
        outcomes["INITIAL_TCP_M"] = initial_tcp
        outcomes["AUTO_SYNC_WITHOUT_RVIZ"] = "PASS" if (initial_joint and initial_tcp and tf_ok) else "FAIL"

        print(f"  ROS Startup: {outcomes['ROS_STARTUP']}")
        print(f"  TF fr10_base_link -> charging_tcp: {outcomes['TF_FR10_BASE_CHARGING_TCP']}")
        print(f"  Initial Joint (rad): {initial_joint}")
        print(f"  Initial TCP (pos, quat): {initial_tcp}")
        print(f"  Auto-sync without RViz: {outcomes['AUTO_SYNC_WITHOUT_RVIZ']}")

        write_json(results_dir / "startup.json", outcomes)
        success = (outcomes["ROS_STARTUP"] == "PASS" and
                   outcomes["TF_FR10_BASE_CHARGING_TCP"] == "PASS" and
                   outcomes["AUTO_SYNC_WITHOUT_RVIZ"] == "PASS")
        return 0 if success else 1

    finally:
        probe_context.__exit__(None, None, None)
        launch.stop()


def main():
    parser = argparse.ArgumentParser(description="Test ROS2 startup and Auto-Sync")
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir", default=None)
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    sys.exit(run_startup_test(args.robot_ip, args.domain_id, results_dir))


if __name__ == "__main__":
    main()
