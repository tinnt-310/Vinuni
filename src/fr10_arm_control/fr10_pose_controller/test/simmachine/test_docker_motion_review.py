#!/usr/bin/env python3
"""Docker-only regression campaign. Never changes cabinet mode, enable or alarms.

Run in fr10-test-runner after the host verifies fairino-container's identity and
exclusive ownership. The only robot endpoint accepted is 192.168.58.2.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time

from ros_health import ProbeContext

WORKSPACE = Path(__file__).resolve().parents[5]


def wait_until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1,
                max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


def stream_metrics(samples):
    intervals = [samples[index][0] - samples[index - 1][0]
                 for index in range(1, len(samples))
                 if samples[index][0] > samples[index - 1][0]]
    max_step = 0.0
    velocities = []
    for index, dt in enumerate(intervals, start=1):
        previous = samples[index - 1][1]
        current = samples[index][1]
        delta = [current[joint] - previous[joint]
                 for joint in range(min(len(previous), len(current)))]
        max_step = max(max_step, max((abs(value) for value in delta), default=0.0))
        velocities.append((samples[index][0],
                           [value / dt for value in delta]))
    max_delta_velocity = 0.0
    for index in range(1, len(velocities)):
        previous = velocities[index - 1][1]
        current = velocities[index][1]
        max_delta_velocity = max(
            max_delta_velocity,
            max((abs(current[joint] - previous[joint])
                 for joint in range(min(len(previous), len(current)))),
                default=0.0))
    return {
        "samples": len(samples),
        "mean_interval_ms": (sum(intervals) / len(intervals) * 1000.0
                             if intervals else 0.0),
        "p95_interval_ms": percentile(intervals, 0.95) * 1000.0,
        "p99_interval_ms": percentile(intervals, 0.99) * 1000.0,
        "max_interval_ms": max(intervals, default=0.0) * 1000.0,
        "max_position_step_rad": max_step,
        "max_delta_velocity_rad_s": max_delta_velocity,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion", action="store_true")
    parser.add_argument("--pose-motion", action="store_true")
    args = parser.parse_args()
    if not Path("/.dockerenv").exists() or os.environ.get("FR10_DOCKER_SIM_VERIFIED") != "192.168.58.2":
        raise SystemExit("Host Docker identity verification required; Docker-only test")
    with socket.create_connection(("192.168.58.2", 8080), timeout=2):
        pass
    output = WORKSPACE / "test_results" / ("docker_motion_review_" + time.strftime("%Y%m%d_%H%M%S"))
    output.mkdir(parents=True, exist_ok=False)
    report = {"simulator": "fairino-container", "robot_ip": "192.168.58.2", "tests": {}}
    logfile = (output / "runtime.log").open("w")
    launch = subprocess.Popen([
        "ros2", "launch", "fr10_pose_controller", "fr10_pose_controller.launch.py",
        "use_fake_hardware:=false", "use_rviz:=false", "robot_ip:=192.168.58.2",
        "controller_overlay:=" + str(WORKSPACE / "src/fr10_arm_control/fr10_pose_controller/config/simmachine_smooth.yaml"),
    ], stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
    context = ProbeContext(int(os.environ.get("ROS_DOMAIN_ID", "183")))
    try:
        probe = context.__enter__()
        report["tests"]["ready"] = probe.wait_ready(timeout=20)
        if not report["tests"]["ready"]:
            raise RuntimeError("Runtime not ready; inspect activation/health log. No target sent.")
        report["tests"]["joint_states_single_publisher"] = wait_until(
            lambda: probe.count_publishers("/joint_states") == 1, 3)
        report["joint_states_publishers"] = probe.count_publishers(
            "/joint_states")
        report["initial_joints_rad"] = probe.joint_position()
        report["initial_tcp_pose"] = probe.tcp_pose()
        report["initial_status"] = probe.status()
        if args.motion:
            initial = probe.joint_position()
            goal = list(initial)
            goal[0] += math.radians(-25 if initial[0] > 0.5 else 25)
            probe.publish_joint(goal)
            moving = wait_until(lambda: "state=TRACKING" in probe.status(), 8)
            report["tests"]["joint_activated"] = moving
            if not moving:
                raise RuntimeError("JOINT target was not activated")
            time.sleep(0.3)
            probe.publish_controlled_stop()
            held = wait_until(lambda: "state=HOLDING" in probe.status(), 8)
            report["tests"]["controlled_stop_holding"] = held
            if not held:
                raise RuntimeError("Controlled stop did not complete")
            time.sleep(0.3)
            held_q = probe.joint_position()
            time.sleep(0.6)
            report["tests"]["no_auto_retry"] = max(abs(a-b) for a,b in zip(held_q, probe.joint_position())) < 0.002
            probe.publish_joint(goal)
            report["tests"]["same_goal_resubmit"] = wait_until(lambda: "state=TRACKING" in probe.status(), 8)
            report["tests"]["joint_goal_reached"] = probe.wait_joint(goal, timeout=20)
            report["tests"]["joint_goal_holding"] = wait_until(
                lambda: "state=HOLDING" in probe.status()
                and "outcome=GOAL_REACHED" in probe.status(), 5)
            report["final_joints_rad"] = probe.joint_position()
            report["final_status"] = probe.status()
        if args.pose_motion:
            # Reuse the measured TCP as the socket goal. Production therefore
            # makes a 250 mm local-axis staging move and a strict 250 mm
            # insertion back to the measured endpoint. This exercises a long
            # POSE path without inventing an unverified world-frame target.
            pose_history_start = len(probe.tcp_history)
            target_position, target_quaternion = probe.tcp_pose()
            probe.publish_pose(target_position, target_quaternion)
            free_space = wait_until(
                lambda: "state=TRACKING" in probe.status()
                and "mode=POSE" in probe.status()
                and "pose_policy=FREE_SPACE" in probe.status(), 30)
            report["tests"]["pose_free_space_activated"] = free_space
            if not free_space:
                raise RuntimeError("POSE FREE_SPACE staging was not activated")
            insertion = wait_until(
                lambda: "state=TRACKING" in probe.status()
                and "mode=POSE" in probe.status()
                and "pose_policy=INSERTION" in probe.status(), 45)
            report["tests"]["pose_insertion_activated"] = insertion
            if not insertion:
                raise RuntimeError("POSE did not transition from PRE_INSERT to INSERTION")
            report["tests"]["pose_goal_reached"] = probe.wait_pose(
                target_position, target_quaternion, timeout=45)
            report["tests"]["pose_goal_holding"] = wait_until(
                lambda: "state=HOLDING" in probe.status()
                and "outcome=GOAL_REACHED" in probe.status()
                and "mode=POSE" in probe.status(), 8)
            with probe.lock:
                pose_samples = list(probe.tcp_history[pose_history_start:])
            report["pose_target"] = [target_position, target_quaternion]
            report["pose_tcp_samples"] = len(pose_samples)
            report["pose_max_excursion_m"] = max(
                (math.sqrt(sum((sample[1][axis] - target_position[axis]) ** 2
                               for axis in range(3)))
                 for sample in pose_samples), default=0.0)
            report["final_tcp_pose"] = probe.tcp_pose()
            report["final_status"] = probe.status()
        with probe.lock:
            report["command_samples"] = len(probe.command_history)
            command_history = list(probe.command_history)
            joint_history = list(probe.joint_history)
            status_history = list(probe.status_history)
            (output / "commands.json").write_text(json.dumps(command_history))
            (output / "status.json").write_text(json.dumps(status_history))
            (output / "feedback.json").write_text(json.dumps(joint_history))
        report["stream_metrics"] = {
            "command": stream_metrics(command_history),
            "feedback": stream_metrics(
                [(sample[0], sample[1]) for sample in joint_history]),
        }
        sampled_reference_errors = []
        for _, status in status_history:
            match = re.search(r"reference_actual_rad=([0-9.eE+-]+)", status)
            if match:
                sampled_reference_errors.append(float(match.group(1)))
        report["sampled_max_reference_actual_rad"] = max(
            sampled_reference_errors, default=0.0)
    except Exception as error:
        report["error"] = str(error)
    finally:
        # Shut down the motion node first, preserving the hardware writer while
        # its controlled braking drains. Signal only children of this launch.
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            try:
                argv = (entry / "cmdline").read_bytes().split(b"\0")
                if argv and Path(os.fsdecode(argv[0])).name == "fr10_pose_controller_node" and os.getpgid(pid) == launch.pid:
                    os.kill(pid, signal.SIGINT)
            except (OSError, ProcessLookupError):
                continue
        time.sleep(3)
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
        try:
            report["launch_exit"] = launch.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
            report["forced_shutdown"] = True
        context.__exit__(None, None, None)
        logfile.close()
        text = (output / "runtime.log").read_text()
        report["tests"]["no_auto_reset"] = "Automatically calling ResetAllError" not in text
        report["tests"]["no_segfault"] = "exit code -11" not in text and "Segmentation fault" not in text
        report["tests"]["no_runtime_abort"] = "exit code -6" not in text and "terminate called" not in text
        report["passed"] = bool(report["tests"]) and all(report["tests"].values()) and "error" not in report and not report.get("forced_shutdown", False)
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        print("Evidence:", output)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
