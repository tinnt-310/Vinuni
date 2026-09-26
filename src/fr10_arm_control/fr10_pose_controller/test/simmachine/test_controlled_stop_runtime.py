#!/usr/bin/env python3
"""Quantitative controlled-stop proof on Docker FAIRINO SimMachine only."""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys
import time

from common import (
    ManagedLaunch,
    ProbeContext,
    analyze_log,
    check_preflight,
    generate,
    write_json,
)


DT = 0.008


def derivatives(position_rows):
    velocities = []
    accelerations = []
    jerks = []
    for index in range(1, len(position_rows)):
        velocities.append([
            (position_rows[index][joint] - position_rows[index - 1][joint]) / DT
            for joint in range(6)
        ])
    for index in range(1, len(velocities)):
        accelerations.append([
            (velocities[index][joint] - velocities[index - 1][joint]) / DT
            for joint in range(6)
        ])
    for index in range(1, len(accelerations)):
        jerks.append([
            (accelerations[index][joint] - accelerations[index - 1][joint]) / DT
            for joint in range(6)
        ])
    return velocities, accelerations, jerks


def peak(rows):
    return max((abs(value) for row in rows for value in row), default=0.0)


def run(robot_ip: str, domain_id: int, results_dir: pathlib.Path) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    preflight = check_preflight(robot_ip)
    write_json(results_dir / "preflight_controlled_stop.json", preflight)
    if not preflight["all_ok"]:
        return 1

    launch = ManagedLaunch.start(results_dir, robot_ip, domain_id)
    context = ProbeContext(domain_id)
    result = {"test": "SIMMACHINE_CONTROLLED_STOP", "robot_ip": robot_ip}
    runtime_passed = False
    try:
        probe = context.__enter__()
        if not probe.wait_ready(timeout=20.0):
            raise RuntimeError("runtime not ready")

        start = probe.joint_position()
        safe = generate(start, 0, 15.0)
        with probe.lock:
            command_start = len(probe.command_history)
        probe.publish_joint(safe.target)

        moving_deadline = time.monotonic() + 5.0
        moving = False
        while time.monotonic() < moving_deadline:
            velocity = probe.joint_velocity()
            if velocity and abs(velocity[0]) > 0.10:
                moving = True
                break
            time.sleep(0.008)
        if not moving:
            raise RuntimeError("J1 did not reach trigger velocity")

        stop_requested = time.monotonic()
        probe.publish_controlled_stop()
        observed_stopping = False
        observed_holding = False
        holding_time = None
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            status = probe.status()
            observed_stopping = observed_stopping or "state=STOPPING" in status
            if observed_stopping and "state=HOLDING" in status:
                observed_holding = True
                holding_time = time.monotonic()
                break
            if "state=FAULT" in status:
                break
            time.sleep(0.008)

        time.sleep(0.20)
        with probe.lock:
            commands = list(probe.command_history[command_start:])
            status_history = list(probe.status_history)
            final_joint = list(probe.latest_joint.position)
            final_velocity = list(probe.latest_joint.velocity)

        # Select exact command samples spanning a short pre-trigger window and
        # the complete braking profile. The command stream is generated at the
        # fixed 8 ms executor period; callback wall time is retained separately.
        selected = [
            row for row in commands
            if row[0] >= stop_requested - 0.032 and
            (holding_time is None or row[0] <= holding_time + 0.032)
        ]
        positions = [row[1] for row in selected]
        velocities, accelerations, jerks = derivatives(positions)
        peak_velocity = peak(velocities)
        peak_acceleration = peak(accelerations)
        peak_jerk = peak(jerks)
        final_reference_velocity = peak(velocities[-3:]) if velocities else math.inf
        stop_duration = (holding_time - stop_requested) if holding_time else math.inf

        # Finite differences of exact position references introduce one-sample
        # numerical error. Core CTest is the authoritative exact qdot/qddot/jerk
        # bound; runtime thresholds below include 5%/10% measurement margins.
        dynamics_ok = (
            peak_velocity <= 0.30 * 1.05 and
            peak_acceleration <= 0.80 * 1.05 and
            peak_jerk <= 4.0 * 1.10 and
            final_reference_velocity <= 0.005
        )
        runtime_passed = (
            observed_stopping and observed_holding and
            0.0 < stop_duration < 2.0 and len(selected) >= 10 and
            dynamics_ok and
            "CONTROLLED STOP COMPLETE" in
            (results_dir / "runtime.log").read_text(errors="replace")
        )
        result.update(
            observed_stopping=observed_stopping,
            observed_holding=observed_holding,
            stop_duration_s=stop_duration,
            command_samples=len(selected),
            configured_limits={
                "velocity_rad_s": 0.30,
                "acceleration_rad_s2": 0.80,
                "jerk_rad_s3": 4.0,
            },
            measured_from_position_reference={
                "peak_velocity_rad_s": peak_velocity,
                "peak_acceleration_rad_s2": peak_acceleration,
                "peak_jerk_rad_s3": peak_jerk,
                "final_velocity_rad_s": final_reference_velocity,
                "method": "finite_difference_at_nominal_8ms",
            },
            final_joint_rad=final_joint,
            final_feedback_velocity_rad_s=final_velocity,
            status_samples=len(status_history),
        )

        with (results_dir / "controlled_stop_reference.csv").open(
                "w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["monotonic_s"] + [f"q{i + 1}_rad" for i in range(6)])
            writer.writerows(selected)
    except Exception as error:  # Preserve evidence and still shut down cleanly.
        result["reason"] = str(error)
    finally:
        context.__exit__(None, None, None)
        shutdown = launch.stop(timeout=20.0)
        result["shutdown"] = shutdown
    errors = analyze_log(results_dir / "runtime.log")
    sdk_ok = all(errors[key] == 0 for key in (
        "servo_j_code14", "joint_read_minus2", "rpc_errors",
        "sdk_command_errors", "segfaults"))
    shutdown_ok = (
        shutdown["backend_clean"] and not shutdown["forced"] and
        not shutdown["segfault"] and shutdown["returncode"] == 0
    )
    passed = runtime_passed and sdk_ok and shutdown_ok
    result["sdk_errors"] = errors
    result["result"] = "PASS" if passed else "FAIL"
    write_json(results_dir / "controlled_stop.json", result)
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="192.168.58.2")
    parser.add_argument("--domain-id", type=int, default=207)
    parser.add_argument("--results-dir", required=True)
    args = parser.parse_args()
    sys.exit(run(args.robot_ip, args.domain_id, pathlib.Path(args.results_dir)))


if __name__ == "__main__":
    main()
