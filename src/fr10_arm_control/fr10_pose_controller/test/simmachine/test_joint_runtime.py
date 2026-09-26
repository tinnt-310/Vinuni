#!/usr/bin/env python3
"""Single-Joint and Multi-Joint runtime motion tests for FAIRINO FR10.

Tests:
1. Single-Joint Matrix: J1..J6 across 15°, 40°, 65–80°, >90° (if safe).
2. Multi-Joint Motions:
   - MJ_001: 3 joints, 15°
   - MJ_002: 6 joints, 15°
   - MJ_003: Large multi-joint (1-2 joints >60°, others 20-45°)
   - MJ_004: Sign crossing across zero
3. Saves CSV trajectories and joint_matrix.json.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time
from typing import Optional, Sequence

from common import (
    MAX_JOINT,
    MIN_JOINT,
    ManagedLaunch,
    ProbeContext,
    SafeTarget,
    check_preflight,
    generate,
    make_results_dir,
    max_abs_error,
    write_json,
)


def run_joint_case(probe, results_dir: pathlib.Path, joint_index: int, delta_deg: float,
                   label: str, max_risk: int = 4) -> dict:
    actual = probe.joint_position()
    safe = generate(actual, joint_index, delta_deg)
    if not safe.allowed:
        return {"result": safe.blocked_reason, **safe.json()}
    if delta_deg >= 40 and max_risk < 2:
        return {"result": "NOT_TESTED", "reason": "risk ceiling"}
    if delta_deg >= 60 and max_risk < 3:
        return {"result": "NOT_TESTED", "reason": "risk ceiling"}
    if delta_deg > 90 and max_risk < 3:
        return {"result": "NOT_TESTED", "reason": "risk ceiling"}

    history_start = len(probe.joint_history)
    started = time.monotonic()
    probe.publish_joint(safe.target)
    timeout = max(15.0, math.radians(delta_deg) / 0.25 + 12.0)
    reached = probe.wait_joint(safe.target, timeout=timeout)
    if reached:
        hold_deadline = time.monotonic() + 2.0
        while time.monotonic() < hold_deadline and "state=HOLDING" not in probe.status():
            time.sleep(0.01)

    final = probe.joint_position()
    status = probe.status()
    error = max_abs_error(final, safe.target) if final else math.inf
    holding = "state=HOLDING" in status
    sdk_ok = "sdk_errors=0" in status and "sdk_last_error=0" in status
    passed = reached and holding and sdk_ok and error <= 0.002

    csv_path = results_dir / f"{label.lower()}.csv"
    probe.write_joint_csv(csv_path, history_start)

    # If this is 15 deg or 70 deg, save canonical csv names too
    if delta_deg == 15 and joint_index == 0:
        probe.write_joint_csv(results_dir / "joint_small.csv", history_start)
    elif delta_deg >= 65 and joint_index == 0:
        probe.write_joint_csv(results_dir / "joint_large.csv", history_start)

    result = {
        "result": "PASS" if passed else "FAIL",
        **safe.json(),
        "start": actual,
        "final": final,
        "final_error_rad": error,
        "duration_s": time.monotonic() - started,
        "status": status,
    }

    # Clean return to initial position
    if reached:
        probe.publish_joint(actual)
        probe.wait_joint(actual, timeout=timeout)

    return result


def multi_joint_target(actual: Sequence[float], deltas_deg: Sequence[float],
                       margin_deg: float = 10.0):
    target = list(actual)
    directions = []
    margin = math.radians(margin_deg)
    for axis, delta_deg in enumerate(deltas_deg):
        delta = math.radians(abs(delta_deg))
        if delta == 0.0:
            directions.append(0)
            continue
        positive = MAX_JOINT[axis] - margin - actual[axis]
        negative = actual[axis] - (MIN_JOINT[axis] + margin)
        direction = 1 if positive >= negative else -1
        if max(positive, negative) + 1e-12 < delta:
            return None, None
        target[axis] += direction * delta
        directions.append(direction)
    return target, directions


def run_multi_joint_case(probe, results_dir: pathlib.Path, label: str,
                         deltas_deg: Sequence[float], risk: int, max_risk: int = 4) -> dict:
    if max_risk < risk:
        return {"result": "NOT_TESTED", "reason": "risk ceiling"}
    start = probe.joint_position()
    target, directions = multi_joint_target(start, deltas_deg)
    if target is None:
        return {"result": "BLOCKED_BY_JOINT_MARGIN", "start": start, "deltas_deg": deltas_deg}

    history_start = len(probe.joint_history)
    begun = time.monotonic()
    probe.publish_joint(target)
    timeout = max(20.0, math.radians(max(deltas_deg)) / 0.25 + 15.0)
    reached = probe.wait_joint(target, timeout=timeout)
    if reached:
        hold_deadline = time.monotonic() + 2.0
        while time.monotonic() < hold_deadline and "state=HOLDING" not in probe.status():
            time.sleep(0.01)

    final = probe.joint_position()
    status = probe.status()
    error = max_abs_error(final, target) if final else math.inf
    passed = reached and error <= 0.002 and "state=HOLDING" in status and "sdk_errors=0" in status

    probe.write_joint_csv(results_dir / f"{label.lower()}.csv", history_start)
    result = {
        "result": "PASS" if passed else "FAIL",
        "start": start,
        "target": target,
        "directions": directions,
        "deltas_deg": list(deltas_deg),
        "final": final,
        "final_error_rad": error,
        "duration_s": time.monotonic() - begun,
        "status": status,
    }

    if reached:
        probe.publish_joint(start)
        probe.wait_joint(start, timeout=timeout)

    return result


def run_all_joint_tests(robot_ip: str = "192.168.58.2", domain_id: int = 171,
                        results_dir: Optional[pathlib.Path] = None, max_risk: int = 4) -> int:
    results_dir = results_dir or make_results_dir()
    preflight = check_preflight(robot_ip)
    if not preflight["all_ok"]:
        print("FAIL: Preflight failed before running joint tests.")
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    probe_context = ProbeContext(domain_id)
    try:
        probe = probe_context.__enter__()
        if not probe.wait_ready(timeout=20.0):
            print("FAIL: ROS runtime not ready.")
            return 1

        print("=== [PHASE 1] Single-Joint Matrix (J1..J6) ===")
        matrix_table = {}
        all_passed = True
        for joint in range(6):
            matrix_table[f"J{joint+1}"] = {}
            for delta, column in ((15, "15deg"), (40, "40deg"), (70, "65_80deg"), (100, "gt90deg")):
                label = f"JOINT_J{joint+1}_{column.upper()}"
                res = run_joint_case(probe, results_dir, joint, delta, label, max_risk)
                matrix_table[f"J{joint+1}"][column] = res["result"]
                print(f"  {label:<25} -> {res['result']}")
                if res["result"] not in ("PASS", "BLOCKED_BY_JOINT_MARGIN", "NOT_TESTED"):
                    all_passed = False

        write_json(results_dir / "joint_matrix.json", matrix_table)

        print("\n=== [PHASE 2] Multi-Joint Coordinated Motions ===")
        for label, deltas, risk in [
            ("MJ_001_3JOINT_15DEG", [15, 15, 15, 0, 0, 0], 2),
            ("MJ_002_6JOINT_15DEG", [15, 15, 15, 15, 15, 15], 2),
            ("MJ_003_LARGE", [70, 65, 35, 30, 25, 20], 4),
        ]:
            res = run_multi_joint_case(probe, results_dir, label, deltas, risk, max_risk)
            print(f"  {label:<25} -> {res['result']}")
            if res["result"] not in ("PASS", "BLOCKED_BY_JOINT_MARGIN", "NOT_TESTED"):
                all_passed = False

        # MJ_004 Sign crossing
        start = probe.joint_position()
        crossing = list(start)
        crossing_axes = []
        for axis in range(6):
            candidate = math.copysign(math.radians(10.0), -start[axis])
            if (MIN_JOINT[axis] + math.radians(10) <= candidate <=
                    MAX_JOINT[axis] - math.radians(10) and
                    abs(candidate - start[axis]) <= math.radians(110)):
                crossing[axis] = candidate
                crossing_axes.append(axis)
            if len(crossing_axes) == 3:
                break

        if len(crossing_axes) >= 2:
            deltas = [math.degrees(abs(crossing[i] - start[i])) for i in range(6)]
            res = run_multi_joint_case(probe, results_dir, "MJ_004_SIGN_CROSSING", deltas, 4, max_risk)
            print(f"  MJ_004_SIGN_CROSSING     -> {res['result']}")
            if res["result"] not in ("PASS", "BLOCKED_BY_JOINT_MARGIN", "NOT_TESTED"):
                all_passed = False

        return 0 if all_passed else 1

    finally:
        probe_context.__exit__(None, None, None)
        launch.stop()


def main():
    parser = argparse.ArgumentParser(description="Run Joint Runtime Tests")
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir", default=None)
    parser.add_argument("--max-risk", type=int, default=4)
    args = parser.parse_args()
    results_dir = pathlib.Path(args.results_dir) if args.results_dir else None
    sys.exit(run_all_joint_tests(args.robot_ip, args.domain_id, results_dir, args.max_risk))


if __name__ == "__main__":
    main()
