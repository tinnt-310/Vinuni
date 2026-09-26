#!/usr/bin/env python3
"""Online Dynamic Joint Retargeting runtime tests for FAIRINO FR10.

Tests:
1. Single-Joint Large A -> B Reverse Retarget (Mid-flight redirection).
2. Multi-Joint Retargeting:
   - MJ_005: Same-direction retarget
   - MJ_006: Multi-joint reverse retarget
   - MJ_007: Multi-joint A -> B -> C (Only C must be final, old targets must not replay)
3. Saves joint_retarget.csv and detailed timestamps.
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
    check_preflight,
    generate,
    make_results_dir,
    max_abs_error,
    write_json,
)


def run_single_joint_retarget(probe, results_dir: pathlib.Path, max_risk: int = 4) -> dict:
    label = "JOINT_LARGE_A_B_C"
    if max_risk < 4:
        return {"result": "NOT_TESTED", "reason": "risk ceiling"}

    q0 = probe.joint_position()
    a = generate(q0, 0, 70.0)
    if not a.allowed:
        return {"result": a.blocked_reason}

    direction = a.direction
    b = list(q0)
    b[0] = q0[0] - direction * math.radians(40.0)
    if not (MIN_JOINT[0] + math.radians(10) <= b[0] <= MAX_JOINT[0] - math.radians(10)):
        return {"result": "BLOCKED_BY_JOINT_MARGIN"}

    c = list(q0)
    c[0] = q0[0] + direction * math.radians(15.0)

    history_start = len(probe.joint_history)
    t_a = time.monotonic()
    probe.publish_joint(a.target)

    deadline = t_a + 8.0
    state_b = None
    while time.monotonic() < deadline:
        q = probe.joint_position()
        qd = probe.joint_velocity()
        if q and qd and abs(q[0] - q0[0]) >= math.radians(10) and abs(qd[0]) > 0.08:
            state_b = {"q": q, "qd": qd}
            break
        time.sleep(0.01)

    if state_b is None:
        return {"result": "FAIL", "reason": "A was not observably moving"}

    t_b = time.monotonic()
    probe.publish_joint(b)

    reverse_deadline = time.monotonic() + 5.0
    reversed_motion = False
    while time.monotonic() < reverse_deadline:
        velocity = probe.joint_velocity()
        if velocity and direction * velocity[0] < -0.05:
            reversed_motion = True
            break
        time.sleep(0.01)

    state_c = {"q": probe.joint_position(), "qd": probe.joint_velocity()}
    t_c = time.monotonic()
    probe.publish_joint(c)

    reached = probe.wait_joint(c, timeout=30.0)
    if reached:
        hold_deadline = time.monotonic() + 2.0
        while time.monotonic() < hold_deadline and "state=HOLDING" not in probe.status():
            time.sleep(0.01)

    final = probe.joint_position()
    status = probe.status()
    error = max_abs_error(final, c) if final else math.inf
    passed = (reversed_motion and reached and error <= 0.002 and
              "state=HOLDING" in status and "sdk_errors=0" in status)

    probe.write_joint_csv(results_dir / "joint_retarget.csv", history_start)

    if reached:
        probe.publish_joint(q0)
        probe.wait_joint(q0, timeout=30.0)

    return {
        "result": "PASS" if passed else "FAIL",
        "target_a": a.target,
        "state_at_b": state_b,
        "target_b": b,
        "state_at_c": state_c,
        "target_c": c,
        "t_a": t_a,
        "t_b": t_b,
        "t_c": t_c,
        "reversed_motion": reversed_motion,
        "remaining_a_at_b": abs(a.target[0] - state_b["q"][0]),
        "final": final,
        "final_error_rad": error,
        "status": status,
    }


def run_multi_joint_retarget(probe, results_dir: pathlib.Path, label: str,
                             reverse: bool, three_targets: bool = False,
                             max_risk: int = 4) -> dict:
    if max_risk < 4:
        return {"result": "NOT_TESTED", "reason": "risk ceiling"}

    start = probe.joint_position()
    # Coordinated multi-axis deltas
    target_a = list(start)
    target_b = list(start)
    target_c = list(start)
    directions = [1, 1, 1, 0, 0, 0]

    for axis, magnitude in enumerate((60, 50, 40)):
        avail_pos = MAX_JOINT[axis] - math.radians(10) - start[axis]
        avail_neg = start[axis] - (MIN_JOINT[axis] + math.radians(10))
        directions[axis] = 1 if avail_pos >= avail_neg else -1
        target_a[axis] += directions[axis] * math.radians(magnitude)
        sign_b = -directions[axis] if reverse else directions[axis]
        target_b[axis] += sign_b * math.radians(magnitude * 0.5)
        target_c[axis] += directions[axis] * math.radians(15 + 5 * axis)

    history_start = len(probe.joint_history)
    t_a = time.monotonic()
    probe.publish_joint(target_a)

    moving_state = None
    deadline = t_a + 8.0
    while time.monotonic() < deadline:
        q = probe.joint_position()
        qd = probe.joint_velocity()
        moving = sum(abs(qd[i]) > 0.05 for i in range(3)) if qd else 0
        if q and qd and moving >= 2 and max(abs(q[i] - start[i]) for i in range(3)) >= math.radians(8):
            moving_state = {"q": q, "qd": qd}
            break
        time.sleep(0.01)

    if moving_state is None:
        return {"result": "FAIL", "reason": "Target A was not moving on multiple axes"}

    t_b = time.monotonic()
    probe.publish_joint(target_b)

    reversed_axes = 0
    if reverse:
        reverse_deadline = time.monotonic() + 7.0
        while time.monotonic() < reverse_deadline:
            qd = probe.joint_velocity()
            reversed_axes = sum(directions[i] * qd[i] < -0.04 for i in range(3)) if qd else 0
            if reversed_axes >= 2:
                break
            time.sleep(0.01)
    else:
        reversed_axes = 2

    final_target = target_b
    state_at_c = None
    t_c = None
    if three_targets:
        state_at_c = {"q": probe.joint_position(), "qd": probe.joint_velocity()}
        t_c = time.monotonic()
        probe.publish_joint(target_c)
        final_target = target_c

    reached = probe.wait_joint(final_target, timeout=40.0)
    if reached:
        hold_deadline = time.monotonic() + 2.0
        while time.monotonic() < hold_deadline and "state=HOLDING" not in probe.status():
            time.sleep(0.01)

    final = probe.joint_position()
    status = probe.status()
    error = max_abs_error(final, final_target) if final else math.inf
    passed = (reached and reversed_axes >= 2 and error <= 0.002 and
              "state=HOLDING" in status and "sdk_errors=0" in status)

    probe.write_joint_csv(results_dir / f"{label.lower()}.csv", history_start)

    if reached:
        probe.publish_joint(start)
        probe.wait_joint(start, timeout=40.0)

    return {
        "result": "PASS" if passed else "FAIL",
        "start": start,
        "target_a": target_a,
        "state_at_b": moving_state,
        "target_b": target_b,
        "state_at_c": state_at_c,
        "target_c": target_c if three_targets else None,
        "t_a": t_a,
        "t_b": t_b,
        "t_c": t_c,
        "reversed_axes": reversed_axes,
        "final": final,
        "final_error_rad": error,
        "status": status,
    }


def run_all_joint_retarget_tests(robot_ip: str = "192.168.58.2", domain_id: int = 171,
                                 results_dir: pathlib.Path = None, max_risk: int = 4) -> int:
    results_dir = results_dir or make_results_dir()
    preflight = check_preflight(robot_ip)
    if not preflight["all_ok"]:
        print("FAIL: Preflight failed before running retarget tests.")
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    probe_context = ProbeContext(domain_id)
    try:
        probe = probe_context.__enter__()
        if not probe.wait_ready(timeout=20.0):
            print("FAIL: ROS runtime not ready.")
            return 1

        print("=== [PHASE 1] Single-Joint Large A->B->C Retarget ===")
        res1 = run_single_joint_retarget(probe, results_dir, max_risk)
        print(f"  JOINT_LARGE_A_B_C -> {res1['result']}")

        print("\n=== [PHASE 2] Multi-Joint Online Retargeting ===")
        res2 = run_multi_joint_retarget(probe, results_dir, "MJ_005_SAME_DIRECTION_RETARGET", False, False, max_risk)
        print(f"  MJ_005_SAME_DIRECTION_RETARGET -> {res2['result']}")

        res3 = run_multi_joint_retarget(probe, results_dir, "MJ_006_REVERSE_RETARGET", True, False, max_risk)
        print(f"  MJ_006_REVERSE_RETARGET        -> {res3['result']}")

        res4 = run_multi_joint_retarget(probe, results_dir, "MJ_007_A_B_C", True, True, max_risk)
        print(f"  MJ_007_A_B_C                   -> {res4['result']}")

        write_json(results_dir / "joint_retarget_summary.json", {
            "JOINT_LARGE_A_B_C": res1,
            "MJ_005_SAME_DIRECTION": res2,
            "MJ_006_REVERSE": res3,
            "MJ_007_A_B_C": res4,
        })

        all_ok = all(r["result"] in ("PASS", "NOT_TESTED", "BLOCKED_BY_JOINT_MARGIN")
                     for r in (res1, res2, res3, res4))
        return 0 if all_ok else 1

    finally:
        probe_context.__exit__(None, None, None)
        launch.stop()


def main():
    parser = argparse.ArgumentParser(description="Run Joint Retargeting Runtime Tests")
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir", default=None)
    parser.add_argument("--max-risk", type=int, default=4)
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    sys.exit(run_all_joint_retarget_tests(args.robot_ip, args.domain_id, results_dir, args.max_risk))


if __name__ == "__main__":
    main()
