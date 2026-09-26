#!/usr/bin/env python3
"""Bounded runtime matrix for RET-J01..RET-J10 on Docker SimMachine."""

from __future__ import annotations

import argparse
import math
import pathlib
import re
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
    write_json,
)


def _revision(status: str) -> int:
    match = re.search(r"(?:^| )revision=(\d+)", status)
    return int(match.group(1)) if match else 0


def _offset(start, axis, delta_deg):
    target = list(start)
    target[axis] += math.radians(delta_deg)
    margin = math.radians(10.0)
    if not (MIN_JOINT[axis] + margin <= target[axis] <=
            MAX_JOINT[axis] - margin):
        return None
    return target


def _wait_revision(probe, before, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _revision(probe.status()) > before:
            return True
        time.sleep(0.005)
    return False


def _wait_holding(probe, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if "state=HOLDING" in probe.status():
            return True
        time.sleep(0.01)
    return False


def _return(probe, origin):
    if max_abs_error(probe.joint_position(), origin) <= 0.0005:
        return True, max_abs_error(probe.joint_position(), origin)
    before = _revision(probe.status())
    probe.publish_joint(origin)
    accepted = _wait_revision(probe, before)
    reached = probe.wait_joint(origin, tolerance=0.002, timeout=15.0)
    holding = reached and _wait_holding(probe)
    error = max_abs_error(probe.joint_position(), origin)
    return accepted and reached and holding and error <= 0.002, error


def _continuity(history):
    max_step = 0.0
    violations = 0
    for previous, current in zip(history, history[1:]):
        dt = max(0.008, current[0] - previous[0])
        allowed = 0.30 * dt + 0.0003
        step = max(abs(current[1][axis] - previous[1][axis])
                   for axis in range(6))
        max_step = max(max_step, step)
        if step > allowed:
            violations += 1
    return {
        "max_feedback_position_step_rad": max_step,
        "configured_velocity_limit_rad_s": 0.30,
        "position_step_violations": violations,
        "criterion": "dq <= 0.30 * max(observed_dt, 0.008) + 0.0003 rad",
    }


def _mid_motion_case(probe, results_dir, test_id, first_deg, second_deg,
                     immediate=False, near_end=False):
    origin = probe.joint_position()
    first = _offset(origin, 0, first_deg)
    second = _offset(origin, 0, second_deg)
    if first is None or second is None:
        return {"result": "BLOCKED", "reason": "joint_margin"}
    with probe.lock:
        history_start = len(probe.joint_history)
    before = _revision(probe.status())
    probe.publish_joint(first)
    first_accepted = _wait_revision(probe, before)
    trigger_met = False
    trigger_deadline = time.monotonic() + 8.0
    if immediate:
        time.sleep(0.02)
        trigger_met = True
    else:
        while time.monotonic() < trigger_deadline:
            actual = probe.joint_position()
            if near_end:
                trigger_met = abs(actual[0] - first[0]) <= math.radians(1.0)
            else:
                trigger_met = abs(actual[0] - origin[0]) >= math.radians(2.0)
            if trigger_met:
                break
            time.sleep(0.005)
    state_at_retarget = {
        "joint_rad": probe.joint_position(),
        "velocity_rad_s": probe.joint_velocity(),
    }
    before_second = _revision(probe.status())
    retarget_sent = time.monotonic()
    probe.publish_joint(second)
    second_accepted = _wait_revision(probe, before_second)
    accepted_at = time.monotonic()
    reached = probe.wait_joint(second, tolerance=0.002, timeout=20.0)
    holding = reached and _wait_holding(probe)
    final = probe.joint_position()
    status = probe.status()
    with probe.lock:
        history = list(probe.joint_history[history_start:])
    continuity = _continuity(history)
    endpoint_error = max_abs_error(final, second)
    returned, return_error = _return(probe, origin)
    passed = (
        first_accepted and trigger_met and second_accepted and reached and holding and
        endpoint_error <= 0.002 and continuity["position_step_violations"] == 0 and
        returned and "sdk_errors=0" in status)
    probe.write_joint_csv(results_dir / f"{test_id.lower()}.csv", history_start)
    return {
        "result": "PASS" if passed else "FAIL",
        "first_target_deg": first_deg,
        "second_target_deg": second_deg,
        "first_accepted": first_accepted,
        "trigger_met": trigger_met,
        "second_accepted": second_accepted,
        "state_at_retarget": state_at_retarget,
        "retarget_accept_latency_ms": (accepted_at - retarget_sent) * 1000.0,
        "reached": reached,
        "holding": holding,
        "endpoint_error_rad": endpoint_error,
        **continuity,
        "returned_to_origin": returned,
        "return_error_rad": return_error,
        "status": status,
    }


def _alternating_case(probe, results_dir):
    origin = probe.joint_position()
    positive = _offset(origin, 0, 3.0)
    negative = _offset(origin, 0, -3.0)
    final_target = _offset(origin, 0, 2.0)
    history_start = len(probe.joint_history)
    accepted = 0
    begun = time.monotonic()
    for index in range(10):
        before = _revision(probe.status())
        probe.publish_joint(positive if index % 2 == 0 else negative)
        accepted += int(_wait_revision(probe, before, timeout=0.08))
        deadline = begun + (index + 1) * 0.05
        time.sleep(max(0.0, deadline - time.monotonic()))
    before = _revision(probe.status())
    probe.publish_joint(final_target)
    final_accepted = _wait_revision(probe, before)
    reached = probe.wait_joint(final_target, tolerance=0.002, timeout=15.0)
    holding = reached and _wait_holding(probe)
    with probe.lock:
        history = list(probe.joint_history[history_start:])
    continuity = _continuity(history)
    endpoint_error = max_abs_error(probe.joint_position(), final_target)
    status = probe.status()
    returned, return_error = _return(probe, origin)
    passed = (
        accepted >= 8 and final_accepted and reached and holding and
        endpoint_error <= 0.002 and continuity["position_step_violations"] == 0 and
        returned and "sdk_errors=0" in status)
    probe.write_joint_csv(results_dir / "ret-j05.csv", history_start)
    return {
        "result": "PASS" if passed else "FAIL",
        "submissions": 10,
        "accepted_before_final": accepted,
        "coalesced_or_not_observed": 10 - accepted,
        "final_accepted": final_accepted,
        "input_rate_hz": 20.0,
        "endpoint_error_rad": endpoint_error,
        **continuity,
        "returned_to_origin": returned,
        "return_error_rad": return_error,
        "status": status,
    }


def _deadband_case(probe, test_id, delta_rad, repetitions, expect_accept):
    origin = probe.joint_position()
    target = list(origin)
    target[0] += delta_rad
    before = _revision(probe.status())
    start_position = probe.joint_position()
    for _ in range(repetitions):
        probe.publish_joint(target)
        time.sleep(0.02)
    if expect_accept:
        accepted = _wait_revision(probe, before)
        reached = _wait_holding(probe, timeout=8.0)
    else:
        time.sleep(0.3)
        accepted = _revision(probe.status()) > before
        reached = True
    final = probe.joint_position()
    motion = max_abs_error(final, start_position)
    revision_delta = _revision(probe.status()) - before
    status = probe.status()
    if expect_accept:
        passed = accepted and reached and revision_delta == 1 and \
            max_abs_error(final, target) <= 0.002
        returned, return_error = _return(probe, origin)
        passed = passed and returned
    else:
        passed = not accepted and revision_delta == 0 and motion <= 0.0005
        returned, return_error = True, max_abs_error(final, origin)
    return {
        "result": "PASS" if passed and "sdk_errors=0" in status else "FAIL",
        "delta_rad": delta_rad,
        "submissions": repetitions,
        "accepted": accepted,
        "revision_delta": revision_delta,
        "observed_motion_rad": motion,
        "returned_to_origin": returned,
        "return_error_rad": return_error,
        "status": status,
    }


def run_test(robot_ip, domain_id, results_dir):
    results_dir.mkdir(parents=True, exist_ok=True)
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight_joint_retarget.json", preflight)
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
            specifications = (
                ("RET-J01", 10.0, 12.0, False, False),
                ("RET-J02", 10.0, 20.0, False, False),
                ("RET-J03", 15.0, 5.0, False, False),
                ("RET-J04", 15.0, -5.0, False, False),
            )
            for test_id, first, second, immediate, near_end in specifications:
                outcomes[test_id] = _mid_motion_case(
                    probe, results_dir, test_id, first, second,
                    immediate=immediate, near_end=near_end)
                print(f"{outcomes[test_id]['result']:>7} {test_id}")
            outcomes["RET-J05"] = _alternating_case(probe, results_dir)
            print(f"{outcomes['RET-J05']['result']:>7} RET-J05")
            outcomes["RET-J06"] = _deadband_case(
                probe, "RET-J06", math.radians(5.0), 10, True)
            outcomes["RET-J07"] = _deadband_case(
                probe, "RET-J07", 0.0002, 10, False)
            outcomes["RET-J08"] = _deadband_case(
                probe, "RET-J08", 0.0010, 1, True)
            print(f"{outcomes['RET-J06']['result']:>7} RET-J06")
            print(f"{outcomes['RET-J07']['result']:>7} RET-J07")
            print(f"{outcomes['RET-J08']['result']:>7} RET-J08")
            outcomes["RET-J09"] = _mid_motion_case(
                probe, results_dir, "RET-J09", 10.0, 12.0, near_end=True)
            outcomes["RET-J10"] = _mid_motion_case(
                probe, results_dir, "RET-J10", 10.0, -5.0, immediate=True)
            print(f"{outcomes['RET-J09']['result']:>7} RET-J09")
            print(f"{outcomes['RET-J10']['result']:>7} RET-J10")
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
        write_json(results_dir / "joint_retarget_matrix.json", outcomes)
    return result_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=204)
    parser.add_argument("--results-dir", default=None)
    arguments = parser.parse_args()
    results = pathlib.Path(arguments.results_dir) if arguments.results_dir else make_results_dir()
    sys.exit(run_test(arguments.robot_ip, arguments.domain_id, results))


if __name__ == "__main__":
    main()
