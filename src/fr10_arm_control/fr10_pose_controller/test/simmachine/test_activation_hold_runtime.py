#!/usr/bin/env python3
"""Verify full ServoJ controller activation while holding measured position.

This stage starts the production SimMachine stack but does not publish a Joint
or Pose target.  It must pass before any displacement test is allowed.
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
    rate_metrics,
    sourced_bash,
    write_json,
)


def run_test(robot_ip: str, domain_id: int, duration_s: float,
             results_dir: pathlib.Path) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    outcome = {
        "test": "SIMMACHINE_CONTROLLER_ACTIVATION_HOLD",
        "robot_ip": robot_ip,
        "ros_domain_id": domain_id,
        "duration_requested_s": duration_s,
        "target_publish_count": 0,
    }
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight_activation.json", preflight)
    if not preflight["all_ok"]:
        outcome.update({"result": "FAIL", "reason": "preflight_failed"})
        write_json(results_dir / "activation_hold.json", outcome)
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    context = ProbeContext(domain_id)
    result_code = 1
    try:
        probe = context.__enter__()
        ready = probe.wait_ready(timeout=25.0)
        if not ready or launch.process.poll() is not None:
            outcome.update({
                "result": "FAIL",
                "reason": "full_stack_not_ready",
                "launch_returncode": launch.process.poll(),
                "status": probe.status(),
            })
        else:
            controllers = sourced_bash(
                f"ROS_DOMAIN_ID={domain_id} RMW_LOCALHOST_ONLY=1 "
                "ros2 control list_controllers --controller-manager /controller_manager",
                timeout=10.0)
            with probe.lock:
                history_start = len(probe.joint_history)
                stamp_start = len(probe.joint_stamps)
                command_start = probe.command_count
            started = time.monotonic()
            while time.monotonic() - started < duration_s:
                if launch.process.poll() is not None:
                    break
                time.sleep(0.05)
            observed_s = time.monotonic() - started
            with probe.lock:
                rows = list(probe.joint_history[history_start:])
                stamps = list(probe.joint_stamps[stamp_start:])
                command_count = probe.command_count - command_start
                status = probe.latest_status

            ranges = [
                max(row[1][joint] for row in rows) -
                min(row[1][joint] for row in rows)
                for joint in range(6)
            ] if rows else []
            timing = rate_metrics(stamps)
            final_joint = rows[-1][1] if rows else None
            controllers_ok = (
                controllers["returncode"] == 0 and
                "joint_state_broadcaster" in controllers["output"] and
                "fairino10_controller" in controllers["output"] and
                controllers["output"].count("active") >= 2)
            runtime_text = launch.log_path.read_text(errors="replace")
            activation_ok = all(marker in runtime_text for marker in (
                "ServoMoveStart and initial hold ServoJ succeeded; hardware ACTIVE",
                "First write ROS rad:",
                "First write SDK deg:",
            )) and "ServoJ failed" not in runtime_text
            passed = (
                observed_s >= duration_s * 0.99 and
                len(rows) >= duration_s * 100.0 and
                115.0 <= timing["mean_hz"] <= 135.0 and
                max(ranges, default=math.inf) <= 1.0e-4 and
                command_count == 0 and controllers_ok and activation_ok and
                "backend_ready=true" in status and "sdk_errors=0" in status and
                launch.process.poll() is None)
            outcome.update({
                "result": "PASS" if passed else "FAIL",
                "duration_observed_s": observed_s,
                "sample_count": len(rows),
                "timing": timing,
                "joint_range_rad": ranges,
                "initial_joint_rad": rows[0][1] if rows else None,
                "final_joint_rad": final_joint,
                "external_target_messages_observed": command_count,
                "activation_log_verified": activation_ok,
                "controller_list": controllers,
                "status": status,
            })
            result_code = 0 if passed else 1
    finally:
        if context.node is not None:
            context.__exit__(None, None, None)
        shutdown = launch.stop()
        outcome["shutdown"] = shutdown
        if shutdown["returncode"] != 0 or shutdown["forced"] or shutdown["segfault"]:
            outcome["result"] = "FAIL"
            result_code = 1
        write_json(results_dir / "activation_hold.json", outcome)
    return result_code


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=194)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--results-dir", default=None)
    arguments = parser.parse_args()
    results_dir = (
        pathlib.Path(arguments.results_dir) if arguments.results_dir
        else make_results_dir())
    sys.exit(run_test(
        arguments.robot_ip, arguments.domain_id,
        arguments.duration, results_dir))


if __name__ == "__main__":
    main()
