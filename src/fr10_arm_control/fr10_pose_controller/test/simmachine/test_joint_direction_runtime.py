#!/usr/bin/env python3
"""Run isolated positive/negative small motions for all six joints."""

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
    check_preflight,
    make_results_dir,
    max_abs_error,
    rate_metrics,
    write_json,
)


def _case(probe, results_dir: pathlib.Path, axis: int, sign: int,
          delta_deg: float) -> dict:
    test_id = f"JNT-{axis * 2 + (1 if sign > 0 else 2):02d}"
    start = probe.joint_position()
    target = list(start)
    target[axis] += sign * math.radians(delta_deg)
    margin = math.radians(10.0)
    if not (MIN_JOINT[axis] + margin <= target[axis] <= MAX_JOINT[axis] - margin):
        return {
            "result": "BLOCKED", "reason": "joint_margin",
            "axis": axis + 1, "sign": sign, "start_rad": start,
            "target_rad": target,
        }

    with probe.lock:
        history_start = len(probe.joint_history)
        stamp_start = len(probe.joint_stamps)
    begun = time.monotonic()
    probe.publish_joint(target)
    reached = probe.wait_joint(target, tolerance=0.002, timeout=15.0)
    if reached:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and "state=HOLDING" not in probe.status():
            time.sleep(0.01)
    with probe.lock:
        history = list(probe.joint_history[history_start:])
        stamps = list(probe.joint_stamps[stamp_start:])
        velocity_source = probe.joint_velocity_source
    final = probe.joint_position()
    status = probe.status()
    final_error = max_abs_error(final, target) if final else math.inf
    actual_delta = final[axis] - start[axis] if final else math.nan
    other_joint_delta = max(
        (abs(final[index] - start[index])
         for index in range(6) if index != axis), default=0.0) if final else math.inf
    limit_violations = sum(
        any(position[index] < MIN_JOINT[index] or
            position[index] > MAX_JOINT[index] for index in range(6))
        for _, position, _ in history)
    peak_velocity = max(
        (abs(row[2][axis]) for row in history), default=0.0)
    passed = (
        reached and sign * actual_delta > 0.0 and
        abs(actual_delta - sign * math.radians(delta_deg)) <= 0.002 and
        final_error <= 0.002 and other_joint_delta <= 0.002 and
        limit_violations == 0 and "state=HOLDING" in status and
        "sdk_errors=0" in status)
    probe.write_joint_csv(
        results_dir / f"{test_id.lower()}_j{axis + 1}_{'pos' if sign > 0 else 'neg'}.csv",
        history_start)

    probe.publish_joint(start)
    returned = probe.wait_joint(start, tolerance=0.002, timeout=15.0)
    return_error = max_abs_error(probe.joint_position(), start)
    if not returned or return_error > 0.002:
        passed = False
    return {
        "result": "PASS" if passed else "FAIL",
        "axis": axis + 1,
        "sign": sign,
        "requested_delta_deg": delta_deg,
        "start_rad": start,
        "target_rad": target,
        "final_rad": final,
        "actual_delta_deg": math.degrees(actual_delta),
        "final_error_rad": final_error,
        "max_other_joint_final_delta_rad": other_joint_delta,
        "joint_limit_violations": limit_violations,
        "peak_observed_velocity_rad_s": peak_velocity,
        "velocity_source": velocity_source,
        "duration_s": time.monotonic() - begun,
        "timing": rate_metrics(stamps),
        "returned_to_start": returned,
        "return_error_rad": return_error,
        "status": status,
    }


def run_test(robot_ip: str, domain_id: int, delta_deg: float,
             results_dir: pathlib.Path) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight_joint_direction.json", preflight)
    if not preflight["all_ok"]:
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    context = ProbeContext(domain_id)
    outcomes = {}
    result_code = 1
    try:
        probe = context.__enter__()
        if not probe.wait_ready(timeout=25.0):
            outcomes["STARTUP"] = {"result": "FAIL"}
        else:
            for axis in range(6):
                for sign in (1, -1):
                    result = _case(probe, results_dir, axis, sign, delta_deg)
                    test_id = f"JNT-{axis * 2 + (1 if sign > 0 else 2):02d}"
                    outcomes[test_id] = result
                    print(f"{result['result']:>7} {test_id} J{axis + 1} "
                          f"{'+' if sign > 0 else '-'}{delta_deg:g} deg")
                    if result["result"] == "FAIL" and not result.get(
                            "returned_to_start", True):
                        outcomes["ABORT"] = {
                            "result": "FAIL",
                            "reason": f"{test_id} could not return to start",
                        }
                        break
                if "ABORT" in outcomes:
                    break
            result_code = 0 if all(
                item["result"] in ("PASS", "BLOCKED")
                for item in outcomes.values()) else 1
    finally:
        if context.node is not None:
            context.__exit__(None, None, None)
        shutdown = launch.stop()
        outcomes["SHUTDOWN"] = {
            **shutdown,
            "result": "PASS" if shutdown["returncode"] == 0 and
            not shutdown["forced"] and not shutdown["segfault"] else "FAIL",
        }
        if outcomes["SHUTDOWN"]["result"] == "FAIL":
            result_code = 1
        write_json(results_dir / "joint_direction_summary.json", outcomes)
    return result_code


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=199)
    parser.add_argument("--delta-deg", type=float, default=5.0)
    parser.add_argument("--results-dir", default=None)
    arguments = parser.parse_args()
    results_dir = pathlib.Path(arguments.results_dir) if arguments.results_dir else make_results_dir()
    sys.exit(run_test(
        arguments.robot_ip, arguments.domain_id,
        arguments.delta_deg, results_dir))


if __name__ == "__main__":
    main()
