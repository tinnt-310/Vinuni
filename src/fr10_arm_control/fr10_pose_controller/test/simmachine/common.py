#!/usr/bin/env python3
"""Shared process/result helpers for FR10 SimMachine tests."""

from __future__ import annotations

import json
import math
import os
import pathlib
import shlex
import signal
import socket
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, Iterable, Optional, Sequence

from metrics import (
    max_abs_error,
    position_error,
    quaternion_error,
    rate_metrics,
)
from ros_health import ProbeContext, RuntimeProbe
from safe_joint_target import MAX_JOINT, MIN_JOINT, SafeTarget, generate
from sdk_log_monitor import analyze as analyze_log


WORKSPACE = pathlib.Path(__file__).resolve().parents[5]


def select_install_setup() -> pathlib.Path:
    """Select a verified overlay; default preserves existing test behavior."""
    candidate = pathlib.Path(os.environ.get(
        'FR10_TEST_SETUP', str(WORKSPACE / 'install/setup.bash'))).resolve()
    if not candidate.is_file():
        raise FileNotFoundError(f'ROS test setup not found: {candidate}')
    return candidate


PACKAGE = WORKSPACE / "src" / "fr10_arm_control" / "fr10_pose_controller"


def timestamp_slug() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def write_json(path: pathlib.Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def run(command: Iterable[str], timeout: float = 30.0,
        env: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
    started = time.monotonic()
    try:
        result = subprocess.run(
            list(command), cwd=WORKSPACE, env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=timeout, check=False)
        return {
            "command": shlex.join(command), "returncode": result.returncode,
            "duration_s": time.monotonic() - started, "output": result.stdout,
        }
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        return {
            "command": shlex.join(command), "returncode": 124,
            "duration_s": time.monotonic() - started,
            "output": output + "\nTIMEOUT\n",
        }


def sourced_bash(command: str, timeout: float = 120.0) -> Dict[str, Any]:
    setup = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(select_install_setup()))} && "
    )
    return run(["/bin/bash", "-lc", setup + command], timeout=timeout)


def find_motion_owners() -> Dict[str, Any]:
    patterns = (
        "fr10_pose_controller_node", "ros2_control_node", "controller_manager",
        "test_simmachine_integration", "read_current_pose", "FRRobot",
    )
    result = run(["ps", "-eo", "pid=,args="], timeout=5.0)
    own_pid = os.getpid()
    owners = []
    for line in result["output"].splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        fields = stripped.split(maxsplit=1)
        try:
            pid = int(fields[0])
        except ValueError:
            continue
        if pid in (own_pid, os.getppid()):
            continue
        command = fields[1] if len(fields) > 1 else ""
        if any(pattern in command for pattern in patterns):
            owners.append({"pid": pid, "command": command})
    return {"pass": not owners, "owners": owners}


def quaternion_multiply(lhs: Sequence[float],
                        rhs: Sequence[float]) -> list[float]:
    """Hamilton product for ROS quaternions in [x, y, z, w] order."""
    x1, y1, z1, w1 = lhs
    x2, y2, z2, w2 = rhs
    result = [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]
    norm = math.sqrt(sum(value * value for value in result))
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in result]


def axis_angle_quaternion(axis: int, angle_rad: float) -> list[float]:
    """Create a unit quaternion for axis index 0=X, 1=Y, or 2=Z."""
    quaternion = [0.0, 0.0, 0.0, math.cos(angle_rad / 2.0)]
    quaternion[axis] = math.sin(angle_rad / 2.0)
    return quaternion


def make_results_dir(custom_path: Optional[str] = None) -> pathlib.Path:
    """Create and return the requested or timestamped results directory."""
    path = (pathlib.Path(custom_path) if custom_path else
            WORKSPACE / "test_results" / timestamp_slug())
    path.mkdir(parents=True, exist_ok=True)
    return path


def check_preflight(robot_ip: str) -> Dict[str, Any]:
    """Check SimMachine, network endpoints, and exclusive motion ownership."""
    docker = run(
        ["docker", "ps", "--format", "{{.ID}} {{.Names}} {{.Status}}"],
        timeout=10.0)
    ping = run(["ping", "-c", "3", "-W", "1", robot_ip], timeout=10.0)
    ports: Dict[str, Any] = {}
    for port in (8080, 20005):
        try:
            with socket.create_connection((robot_ip, port), timeout=2.0):
                ports[str(port)] = True
        except OSError as error:
            ports[str(port)] = False
            ports[f"{port}_error"] = str(error)
    owners = find_motion_owners()
    docker_ok = (docker["returncode"] == 0 and
                 "fairino" in docker["output"].lower())
    network_ok = ping["returncode"] == 0 and ports.get("8080", False)
    owner_ok = owners["pass"]
    return {
        "docker": docker,
        "docker_ok": docker_ok,
        "ping": ping,
        "tcp_ports": ports,
        "network_ok": network_ok,
        "duplicate_owner": owners,
        "owner_ok": owner_ok,
        "all_ok": docker_ok and network_ok and owner_ok,
    }


@dataclass
class ManagedLaunch:
    process: subprocess.Popen
    log_handle: Any
    log_path: pathlib.Path

    @classmethod
    def start(cls, results_dir: pathlib.Path, robot_ip: str,
              domain_id: int,
              use_fake_hardware: bool = False,
              controller_overlay: str = "") -> "ManagedLaunch":
        results_dir.mkdir(parents=True, exist_ok=True)
        log_path = results_dir / "runtime.log"
        log_handle = log_path.open("w")
        hardware_value = "true" if use_fake_hardware else "false"
        overlay_arg = (f" controller_overlay:={shlex.quote(controller_overlay)}"
                       if controller_overlay else "")
        command = (
            "source /opt/ros/humble/setup.bash && "
            f"source {shlex.quote(str(select_install_setup()))} && "
            f"export ROS_DOMAIN_ID={domain_id} && "
            "export RMW_LOCALHOST_ONLY=1 && "
            f"export ROS_LOG_DIR={shlex.quote(str(results_dir / 'ros_log'))} && "
            "exec ros2 launch fr10_pose_controller fr10_pose_controller.launch.py "
            f"use_fake_hardware:={hardware_value} use_gazebo:=false use_rviz:=false "
            f"backend:=ros_control robot_ip:={shlex.quote(robot_ip)} "
            "start_robot_stack:=true"
            f"{overlay_arg}"
        )
        process = subprocess.Popen(
            ["/bin/bash", "-lc", command], cwd=WORKSPACE,
            stdout=log_handle, stderr=subprocess.STDOUT,
            start_new_session=True, text=True)
        return cls(process=process, log_handle=log_handle, log_path=log_path)

    def stop(self, timeout: float = 15.0) -> Dict[str, Any]:
        started = time.monotonic()
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
        try:
            returncode = self.process.wait(timeout=timeout)
            forced = False
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                returncode = self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                returncode = self.process.wait(timeout=5.0)
            forced = True
        self.log_handle.close()
        text = self.log_path.read_text(errors="replace")
        return {
            "returncode": returncode, "forced": forced,
            "duration_s": time.monotonic() - started,
            "segfault": "Segmentation fault" in text or returncode == -11,
            "backend_clean":
                "RosControlBackend shutdown completed" in text,
        }
