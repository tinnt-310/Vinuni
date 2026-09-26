#!/usr/bin/env python3
"""Run small coordinated multi-joint motions and reversals on SimMachine."""

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


def _safe_target(start, deltas_deg):
    margin = math.radians(10.0)
    target = [
        start[index] + math.radians(deltas_deg[index])
        for index in range(6)
    ]
    safe = all(
        MIN_JOINT[index] + margin <= target[index] <= MAX_JOINT[index] - margin
        for index in range(6))
    return target if safe else None


def _move(probe, results_dir, test_id, deltas_deg, return_to_start=True):
    start = probe.joint_position()
    target = _safe_target(start, deltas_deg)
    if target is None:
        return {"result": "BLOCKED", "reason": "joint_margin"}
    with probe.lock:
        history_start = len(probe.joint_history)
        stamp_start = len(probe.joint_stamps)
    begun = time.monotonic()
    probe.publish_joint(target)
    reached = probe.wait_joint(target, tolerance=0.002, timeout=20.0)
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
    sign_ok = all(
        delta == 0.0 or (final[index] - start[index]) * delta > 0.0
        for index, delta in enumerate(deltas_deg)) if final else False
    limit_violations = sum(
        any(q[index] < MIN_JOINT[index] or q[index] > MAX_JOINT[index]
            for index in range(6))
        for _, q, _ in history)
    peak_velocity = [
        max((abs(row[2][index]) for row in history), default=0.0)
        for index in range(6)
    ]
    passed = (
        reached and sign_ok and final_error <= 0.002 and
        limit_violations == 0 and "state=HOLDING" in status and
        "sdk_errors=0" in status)
    probe.write_joint_csv(results_dir / f"{test_id.lower()}.csv", history_start)
    returned = None
    return_error = None
    if return_to_start:
        probe.publish_joint(start)
        returned = probe.wait_joint(start, tolerance=0.002, timeout=20.0)
        return_error = max_abs_error(probe.joint_position(), start)
        passed = passed and returned and return_error <= 0.002
    return {
        "result": "PASS" if passed else "FAIL",
        "deltas_deg": list(deltas_deg),
        "start_rad": start,
        "target_rad": target,
        "final_rad": final,
        "final_error_rad": final_error,
        "signs_correct": sign_ok,
        "joint_limit_violations": limit_violations,
        "peak_observed_velocity_rad_s": peak_velocity,
        "velocity_source": velocity_source,
        "duration_s": time.monotonic() - begun,
        "timing": rate_metrics(stamps),
        "returned_to_start": returned,
        "return_error_rad": return_error,
        "status": status,
    }


def _repeat(probe, results_dir, cycles):
    origin = probe.joint_position()
    positive = _safe_target(origin, [3.0, -3.0, 0.0, 0.0, 0.0, 0.0])
    negative = _safe_target(origin, [-3.0, 3.0, 0.0, 0.0, 0.0, 0.0])
    if positive is None or negative is None:
        return {"result": "BLOCKED", "reason": "joint_margin"}
    history_start = len(probe.joint_history)
    completed = 0
    failures = []
    begun = time.monotonic()
    for cycle in range(cycles):
        for label, target in (("positive", positive), ("negative", negative)):
            probe.publish_joint(target)
            if not probe.wait_joint(target, tolerance=0.002, timeout=15.0):
                failures.append({"cycle": cycle + 1, "leg": label})
                break
        if failures:
            break
        completed += 1
    probe.publish_joint(origin)
    returned = probe.wait_joint(origin, tolerance=0.002, timeout=15.0)
    final_error = max_abs_error(probe.joint_position(), origin)
    status = probe.status()
    probe.write_joint_csv(results_dir / "jnt-25.csv", history_start)
    passed = (
        completed == cycles and not failures and returned and
        final_error <= 0.002 and "sdk_errors=0" in status)
    return {
        "result": "PASS" if passed else "FAIL",
        "requested_cycles": cycles,
        "completed_cycles": completed,
        "failures": failures,
        "duration_s": time.monotonic() - begun,
        "returned_to_start": returned,
        "return_error_rad": final_error,
        "status": status,
    }


def run_test(robot_ip, domain_id, results_dir, cycles):
    results_dir.mkdir(parents=True, exist_ok=True)
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight_joint_multi.json", preflight)
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
            cases = (
                ("JNT-20", [5.0, -5.0, 0.0, 0.0, 0.0, 0.0]),
                ("JNT-21", [5.0, 5.0, -5.0, 0.0, 0.0, 0.0]),
                ("JNT-22", [3.0, -3.0, 3.0, -3.0, 3.0, -3.0]),
                ("JNT-23", [-5.0, 5.0, -5.0, 5.0, 0.0, 0.0]),
            )
            for test_id, deltas in cases:
                outcomes[test_id] = _move(
                    probe, results_dir, test_id, deltas, return_to_start=True)
                print(f"{outcomes[test_id]['result']:>7} {test_id}")
                if outcomes[test_id]["result"] == "FAIL":
                    break
            if all(outcomes.get(test_id, {}).get("result") == "PASS"
                   for test_id, _ in cases):
                return_error = max_abs_error(
                    probe.joint_position(), outcomes["JNT-20"]["start_rad"])
                outcomes["JNT-24"] = {
                    "result": "PASS" if return_error <= 0.002 else "FAIL",
                    "return_error_rad": return_error,
                }
                outcomes["JNT-25"] = _repeat(
                    probe, results_dir, cycles)
                print(f"{outcomes['JNT-24']['result']:>7} JNT-24")
                print(f"{outcomes['JNT-25']['result']:>7} JNT-25")
            result_code = 0 if outcomes and all(
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
        write_json(results_dir / "joint_multi_summary.json", outcomes)
    return result_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=200)
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--results-dir", default=None)
    arguments = parser.parse_args()
    results = pathlib.Path(arguments.results_dir) if arguments.results_dir else make_results_dir()
    sys.exit(run_test(
        arguments.robot_ip, arguments.domain_id, results, arguments.cycles))


if __name__ == "__main__":
    main()
