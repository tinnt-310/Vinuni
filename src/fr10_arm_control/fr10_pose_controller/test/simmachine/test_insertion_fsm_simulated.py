#!/usr/bin/env python3
"""Canonical insertion/controlled-stop/retract FSM test on SimMachine."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import time
from functools import lru_cache

from common import (
    ManagedLaunch,
    ProbeContext,
    analyze_log,
    check_preflight,
    make_results_dir,
    position_error,
    quaternion_error,
    write_json,
)


def matmul(left, right):
    return [[sum(left[row][k] * right[k][col] for k in range(3))
             for col in range(3)] for row in range(3)]


def matvec(matrix, vector):
    return [sum(matrix[row][col] * vector[col] for col in range(3))
            for row in range(3)]


def rotation_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


@lru_cache(maxsize=1)
def model_tool_translation():
    import importlib.util
    import xacro
    from ament_index_python.packages import get_package_share_directory
    share = pathlib.Path(get_package_share_directory('vehicle_description'))
    spec = importlib.util.spec_from_file_location('tool_geometry', share/'launch/tool_geometry.py')
    geometry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(geometry)
    description = xacro.process_file(str(share/'urdf/vehicle.urdf.xacro'), mappings={
        'enable_vehicle': 'false', 'enable_arm': 'true'}).toxml()
    transform = geometry.fixed_transform(description, 'wrist3_link', 'charging_tcp')
    return tuple(row[3] for row in transform[:3])


def fk_tcp_position(joints):
    """Independent position-only port of the configured analytical FK."""
    rotation = [[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]]
    position = [0.0, 0.0, 0.0]

    def rotate(matrix):
        nonlocal rotation
        rotation = matmul(rotation, matrix)

    def translate(vector):
        delta = matvec(rotation, vector)
        for axis in range(3):
            position[axis] += delta[axis]

    rotate(rotation_z(joints[0]))
    translate([0.0, 0.0, 0.180])
    rotate(rotation_x(math.pi / 2.0))
    rotate(rotation_z(joints[1]))
    translate([-0.700, 0.0, 0.0])
    rotate(rotation_z(joints[2]))
    translate([-0.586, 0.0, 0.0])
    rotate(rotation_z(joints[3]))
    translate([0.0, 0.0, 0.159])
    rotate(rotation_x(math.pi / 2.0))
    rotate(rotation_z(joints[4]))
    translate([0.0, 0.0, 0.114])
    rotate(rotation_x(-math.pi / 2.0))
    rotate(rotation_z(joints[5]))
    translate(model_tool_translation())
    return position


def rotate_local_z(quaternion):
    x, y, z, w = quaternion
    return [2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y)]


def wait_status(probe, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = probe.status()
        if predicate(status):
            return status
        time.sleep(0.01)
    return None


def line_deviation(point, start, goal):
    direction = [goal[i] - start[i] for i in range(3)]
    offset = [point[i] - start[i] for i in range(3)]
    length_squared = sum(value * value for value in direction)
    if length_squared < 1e-12:
        return math.inf
    scale = sum(offset[i] * direction[i] for i in range(3)) / length_squared
    residual = [offset[i] - scale * direction[i] for i in range(3)]
    return math.sqrt(sum(value * value for value in residual))


def write_tcp_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["monotonic_s", "x", "y", "z",
                         "qx", "qy", "qz", "qw"])
        for stamp, position, quaternion in rows:
            writer.writerow([stamp, *position, *quaternion])


def run(robot_ip: str, domain_id: int, results_dir: pathlib.Path) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight.json", preflight)
    if not preflight["all_ok"]:
        write_json(results_dir / "insertion_fsm_summary.json", {
            "result": "FAIL", "reason": "preflight failed"})
        return 2

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    context = ProbeContext(domain_id)
    probe = None
    result = {"test": "INSERTION_FSM_CONTROLLED_STOP"}
    try:
        probe = context.__enter__()
        if not probe.wait_ready(timeout=30.0):
            result.update(result="FAIL", reason="runtime not ready")
            return 1

        # Deterministic charging-socket pose shared with the validated LD-03
        # endpoint.  Reusing the current TCP would shift the synthetic socket
        # by another 200 mm after every run and eventually make the staging
        # pose unreachable, so it is not a repeatable FSM test.
        target = ([0.450, 0.450, 0.750],
                  [-0.75, 0.4330127019, -0.4330127019, -0.25])
        insertion_axis = rotate_local_z(target[1])
        pre_insert = [target[0][axis] - 0.200 * insertion_axis[axis]
                      for axis in range(3)]
        with probe.lock:
            campaign_tcp_start = len(probe.tcp_history)
        probe.publish_pose(target[0], target[1])

        insertion_status = wait_status(
            probe,
            lambda status: ("state=TRACKING" in status and
                            "pose_policy=INSERTION" in status),
            timeout=90.0,
        )
        if insertion_status is None:
            result.update(result="FAIL", stage="staging",
                          status=probe.status())
            return 1
        insertion_start = probe.tcp_pose()[0]
        insertion_length = position_error(insertion_start, target[0])
        if insertion_length < 0.050:
            result.update(result="FAIL", stage="insertion",
                          reason="insertion segment unexpectedly short",
                          insertion_length_m=insertion_length)
            return 1

        midway_deadline = time.monotonic() + 30.0
        while time.monotonic() < midway_deadline:
            actual = probe.tcp_pose()
            if actual and position_error(actual[0], insertion_start) >= (
                    0.45 * insertion_length):
                break
            if "state=FAULT" in probe.status():
                break
            time.sleep(0.01)
        else:
            result.update(result="FAIL", stage="insertion",
                          reason="midpoint not reached", status=probe.status())
            return 1

        with probe.lock:
            stop_tcp_start = len(probe.tcp_history)
            stop_joint_start = len(probe.joint_history)
        stop_trigger_s = time.monotonic()
        probe.publish_controlled_stop()
        holding_status = wait_status(
            probe,
            lambda status: ("state=HOLDING" in status and
                            "reason=CONTROLLED_STOP" in status),
            timeout=10.0,
        )
        if holding_status is None:
            result.update(result="FAIL", stage="controlled_stop",
                          reason="HOLDING not reached", status=probe.status())
            return 1

        with probe.lock:
            stop_joint_rows = list(probe.joint_history[stop_joint_start:])
            stop_tcp_rows = list(probe.tcp_history[stop_tcp_start:])
        direction = [target[0][axis] - insertion_start[axis]
                     for axis in range(3)]
        direction_norm = math.sqrt(sum(value * value for value in direction))
        direction = [value / direction_norm for value in direction]
        actual_fk_rows = [
            (stamp, fk_tcp_position(joints))
            for stamp, joints, _ in stop_joint_rows if stamp >= stop_trigger_s
        ]
        if actual_fk_rows:
            trigger_position = actual_fk_rows[0][1]
            stop_overshoot_m = max(
                sum((position[axis] - trigger_position[axis]) * direction[axis]
                    for axis in range(3))
                for _, position in actual_fk_rows)
        else:
            stop_overshoot_m = math.inf

        stopped_pose = probe.tcp_pose()
        with probe.lock:
            retract_tcp_start = len(probe.tcp_history)
        probe.publish_retract()
        retracted = probe.wait_pose(
            pre_insert, target[1], position_tolerance=0.002,
            angular_tolerance=math.radians(1.0), timeout=60.0)
        retract_status = probe.status()
        with probe.lock:
            retract_rows = list(probe.tcp_history[retract_tcp_start:])
            campaign_rows = list(probe.tcp_history[campaign_tcp_start:])
        max_retract_deviation_m = max(
            (line_deviation(row[1], stopped_pose[0], pre_insert)
             for row in retract_rows), default=math.inf)
        final = probe.tcp_pose()
        final_position_error_m = (
            position_error(final[0], pre_insert) if final else math.inf)
        final_orientation_error_rad = (
            quaternion_error(final[1], target[1]) if final else math.inf)
        passed = (
            retracted and stop_overshoot_m < 0.001
            and max_retract_deviation_m <= 0.002
            and final_position_error_m <= 0.002
            and final_orientation_error_rad <= math.radians(1.0)
            and "state=HOLDING" in retract_status
            and "sdk_errors=0" in retract_status
        )
        result.update(
            result="PASS" if passed else "FAIL",
            staging="PASS", insertion="PASS",
            controlled_stop="PASS" if stop_overshoot_m < 0.001 else "FAIL",
            retract="PASS" if retracted else "FAIL",
            target=target, pre_insert=pre_insert,
            insertion_start=insertion_start,
            insertion_length_m=insertion_length,
            controlled_stop_status=holding_status,
            stop_overshoot_m=stop_overshoot_m,
            stop_joint_samples=len(actual_fk_rows),
            stop_tcp_samples=len(stop_tcp_rows),
            retract_max_perpendicular_deviation_m=max_retract_deviation_m,
            final_position_error_m=final_position_error_m,
            final_orientation_error_rad=final_orientation_error_rad,
            final_status=retract_status,
        )
        write_tcp_csv(results_dir / "insertion_fsm_tcp.csv", campaign_rows)
        return 0 if passed else 1
    finally:
        if probe is not None:
            try:
                probe.publish_controlled_stop()
                time.sleep(0.5)
            except Exception:
                pass
        try:
            context.__exit__(None, None, None)
        finally:
            shutdown = launch.stop(timeout=20.0)
        sdk = analyze_log(launch.log_path)
        result["sdk_audit"] = sdk
        result["shutdown"] = shutdown
        if result.get("result") == "PASS" and sum(sdk.values()) != 0:
            result["result"] = "FAIL"
            result["reason"] = "SDK/runtime log audit failed"
        write_json(results_dir / "insertion_fsm_summary.json", result)
        (results_dir / "insertion_fsm_report.md").write_text(
            "# Insertion FSM SimMachine report\n\n" +
            "\n".join(f"- {key}: {value}" for key, value in result.items()) +
            "\n")
        print(json.dumps(result, indent=2, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=171)
    parser.add_argument("--results-dir")
    args = parser.parse_args()
    results_dir = (pathlib.Path(args.results_dir) if args.results_dir
                   else make_results_dir())
    return run(args.robot_ip, args.domain_id, results_dir)


if __name__ == "__main__":
    raise SystemExit(main())
