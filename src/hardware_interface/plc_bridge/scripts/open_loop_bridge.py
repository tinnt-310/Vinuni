#!/usr/bin/env python3
"""ROS 2 keyboard/joystick to FX5U open-loop commissioning bridge."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence, Tuple

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
for import_root in (PROJECT_ROOT, PROJECT_ROOT / "src"):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from scripts.common import DEFAULT_CONFIG, build_runtime, safe_shutdown  # noqa: E402
from fx5u_controller.open_loop import (  # noqa: E402
    ManualCommandSelector,
    OpenLoopSwerveKinematics,
    VelocityCommand,
    clamp_velocity,
    logical_to_plc,
    slew_velocity,
    slew_heading_deg,
    speed_to_raw,
    steering_distance_deg,
    steering_to_plc_degree,
)

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String
except ImportError as exc:  # pragma: no cover - depends on the sourced ROS environment
    raise SystemExit("ROS 2 Python modules are unavailable; source /opt/ros/humble/setup.bash") from exc


DEFAULT_BRIDGE_CONFIG = PROJECT_ROOT / "config" / "open_loop_bridge.yaml"


@dataclass(frozen=True)
class BridgeSettings:
    keyboard_topic: str
    joy_topic: str
    status_topic: str
    write_rate_hz: float
    command_timeout_s: float
    steering_settle_s: float
    allow_steering_while_driving: bool
    max_linear_m_s: float
    max_angular_rad_s: float
    max_linear_accel_m_s2: float
    max_angular_accel_rad_s2: float
    max_wheel_rate_rad_s: float
    raw_at_one_m_s: int
    minimum_angle_change_deg: float
    steering_rate_limit_deg_s: float
    minimum_raw_change: int
    max_raw_step_per_tick: int
    wheelbase_m: float
    track_width_m: float
    wheel_radius_m: float
    deadman_button: int
    axis_x: int
    axis_y: int
    axis_yaw: int
    invert_x: bool
    invert_y: bool
    invert_yaw: bool
    logical_to_plc_ids: Tuple[int, int, int, int]
    steering_sign: Tuple[int, int, int, int]
    steering_zero_offset_deg: Tuple[float, float, float, float]
    drive_direction_inverted: Tuple[bool, bool, bool, bool]


def _section(data: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = data.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a YAML mapping")
    return value


def _positive(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _index(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _four(values: Any, name: str) -> Sequence[Any]:
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence) or len(values) != 4:
        raise ValueError(f"{name} must contain four values")
    return values


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a boolean")
    return value


def load_bridge_settings(path: Path) -> BridgeSettings:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise ValueError("bridge config root must be a YAML mapping")
    topics = _section(raw, "topics")
    timing = _section(raw, "timing")
    limits = _section(raw, "limits")
    geometry = _section(raw, "geometry")
    joystick = _section(raw, "joystick")
    wheels = _section(raw, "wheels")
    mapping = tuple(int(v) for v in _four(wheels.get("logical_to_plc_ids"), "logical_to_plc_ids"))
    if set(mapping) != {1, 2, 3, 4}:
        raise ValueError("logical_to_plc_ids must be a permutation of 1,2,3,4")
    signs = tuple(int(v) for v in _four(wheels.get("steering_sign"), "steering_sign"))
    if any(value not in (-1, 1) for value in signs):
        raise ValueError("steering_sign values must be -1 or 1")
    offsets = tuple(float(v) for v in
                    _four(wheels.get("steering_zero_offset_deg"), "steering_zero_offset_deg"))
    inversions_raw = _four(wheels.get("drive_direction_inverted"), "drive_direction_inverted")
    if not all(math.isfinite(value) for value in offsets):
        raise ValueError("steering_zero_offset_deg values must be finite")
    if any(not isinstance(v, bool) for v in inversions_raw):
        raise ValueError("drive_direction_inverted values must be booleans")
    inversions = tuple(inversions_raw)
    result = BridgeSettings(
        keyboard_topic=str(topics["keyboard_cmd_vel"]),
        joy_topic=str(topics["joy"]),
        status_topic=str(topics["status"]),
        write_rate_hz=_positive(timing["write_rate_hz"], "write_rate_hz"),
        command_timeout_s=_positive(timing["command_timeout_s"], "command_timeout_s"),
        steering_settle_s=_positive(timing["steering_settle_s"], "steering_settle_s"),
        allow_steering_while_driving=_boolean(
            timing["allow_steering_while_driving"], "allow_steering_while_driving"
        ),
        max_linear_m_s=_positive(limits["max_linear_m_s"], "max_linear_m_s"),
        max_angular_rad_s=_positive(limits["max_angular_rad_s"], "max_angular_rad_s"),
        max_linear_accel_m_s2=_positive(
            limits["max_linear_accel_m_s2"], "max_linear_accel_m_s2"
        ),
        max_angular_accel_rad_s2=_positive(
            limits["max_angular_accel_rad_s2"], "max_angular_accel_rad_s2"
        ),
        max_wheel_rate_rad_s=_positive(
            limits["max_wheel_rate_rad_s"], "max_wheel_rate_rad_s"
        ),
        raw_at_one_m_s=_index(limits["raw_at_one_m_s"], "raw_at_one_m_s"),
        minimum_angle_change_deg=_positive(
            limits["minimum_angle_change_deg"], "minimum_angle_change_deg"
        ),
        steering_rate_limit_deg_s=_positive(
            limits["steering_rate_limit_deg_s"], "steering_rate_limit_deg_s"
        ),
        minimum_raw_change=int(limits["minimum_raw_change"]),
        max_raw_step_per_tick=int(limits["max_raw_step_per_tick"]),
        wheelbase_m=_positive(geometry["wheelbase_m"], "wheelbase_m"),
        track_width_m=_positive(geometry["track_width_m"], "track_width_m"),
        wheel_radius_m=_positive(geometry["wheel_radius_m"], "wheel_radius_m"),
        deadman_button=_index(joystick["deadman_button"], "deadman_button"),
        axis_x=_index(joystick["axis_x"], "axis_x"),
        axis_y=_index(joystick["axis_y"], "axis_y"),
        axis_yaw=_index(joystick["axis_yaw"], "axis_yaw"),
        invert_x=_boolean(joystick["invert_x"], "invert_x"),
        invert_y=_boolean(joystick["invert_y"], "invert_y"),
        invert_yaw=_boolean(joystick["invert_yaw"], "invert_yaw"),
        logical_to_plc_ids=mapping,  # type: ignore[arg-type]
        steering_sign=signs,  # type: ignore[arg-type]
        steering_zero_offset_deg=offsets,  # type: ignore[arg-type]
        drive_direction_inverted=inversions,  # type: ignore[arg-type]
    )
    if result.minimum_raw_change < 0:
        raise ValueError("minimum_raw_change must be nonnegative")
    if result.max_raw_step_per_tick <= 0:
        raise ValueError("max_raw_step_per_tick must be positive")
    return result


class Fx5uOpenLoopBridge(Node):
    def __init__(self, controller: Any, settings: BridgeSettings, stop_raw: int) -> None:
        super().__init__("fx5u_open_loop_bridge")
        self.controller = controller
        self.settings = settings
        self.stop_raw = stop_raw
        self.selector = ManualCommandSelector(settings.command_timeout_s)
        self.kinematics = OpenLoopSwerveKinematics(
            settings.wheelbase_m, settings.track_width_m,
            settings.wheel_radius_m, settings.max_wheel_rate_rad_s,
        )
        self.previous_steering = (0.0, 0.0, 0.0, 0.0)
        self.last_angles: Tuple[int, ...] | None = None
        self.last_angle_float: Tuple[float, ...] | None = None
        self.last_steering_s: float | None = None
        self.last_raw: Tuple[int, ...] = (stop_raw,) * 4
        self.last_directions: Tuple[bool, ...] | None = None
        self.drive_stopped = True
        self.faulted = False
        self.settle_until_s = 0.0
        self.smoothed_velocity = VelocityCommand(0.0, 0.0, 0.0)
        self.last_slew_s: float | None = None
        self.last_source: str | None = None
        self.status_pub = self.create_publisher(String, settings.status_topic, 10)
        self.keyboard_sub = self.create_subscription(
            Twist, settings.keyboard_topic, self.on_keyboard, 10
        )
        self.joy_sub = self.create_subscription(Joy, settings.joy_topic, self.on_joy, 10)
        self.reset_sub = self.create_subscription(
            String, "/plc_bridge/reset_drive", self.on_reset_drive, 1
        )
        self.timer = self.create_timer(1.0 / settings.write_rate_hz, self.on_timer)
        self.publish_status("ready", source=None)
        self.get_logger().warning(
            "OPEN LOOP: no steering/speed feedback; "
            f"keyboard={settings.keyboard_topic} joy={settings.joy_topic} "
            f"deadman_button={settings.deadman_button}"
        )

    def on_keyboard(self, message: Twist) -> None:
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("Rejected nonfinite keyboard Twist")
            return
        self.selector.update_keyboard(
            VelocityCommand(message.linear.x, message.linear.y, message.angular.z),
            time.monotonic(),
        )

    def on_joy(self, message: Joy) -> None:
        required_axis = max(self.settings.axis_x, self.settings.axis_y, self.settings.axis_yaw)
        if required_axis >= len(message.axes) or self.settings.deadman_button >= len(message.buttons):
            self.get_logger().error("Joystick message does not contain configured axes/buttons")
            self.selector.update_joystick(
                VelocityCommand(0.0, 0.0, 0.0), time.monotonic(), enabled=False
            )
            return
        enabled = bool(message.buttons[self.settings.deadman_button])
        signs = (
            -1.0 if self.settings.invert_x else 1.0,
            -1.0 if self.settings.invert_y else 1.0,
            -1.0 if self.settings.invert_yaw else 1.0,
        )
        axes = (
            message.axes[self.settings.axis_x],
            message.axes[self.settings.axis_y],
            message.axes[self.settings.axis_yaw],
        )
        if not all(math.isfinite(value) for value in axes):
            self.get_logger().error("Rejected nonfinite joystick axes")
            enabled = False
        axes = tuple(max(-1.0, min(1.0, value)) for value in axes)
        command = VelocityCommand(
            signs[0] * axes[0] * self.settings.max_linear_m_s,
            signs[1] * axes[1] * self.settings.max_linear_m_s,
            signs[2] * axes[2] * self.settings.max_angular_rad_s,
        )
        self.selector.update_joystick(command, time.monotonic(), enabled=enabled)

    def on_reset_drive(self, message: String) -> None:
        if message.data != "reset":
            return
        if self.faulted:
            self.get_logger().error("Reset rejected: bridge communication fault; reconnect first")
            return
        try:
            self.controller.stop_drive()
            self.drive_stopped = True
            self.last_raw = (self.stop_raw,) * 4
            self.smoothed_velocity = VelocityCommand(0.0, 0.0, 0.0)
            self.last_slew_s = time.monotonic()
            self.selector = ManualCommandSelector(self.settings.command_timeout_s)
            self.controller.reset_drive_on_start()
            self.controller.stop_drive()
            self.get_logger().info("Manual reset acknowledged: M222 ON then OFF; drive stopped")
            self.publish_status("drive_reset", source="keyboard")
        except Exception as exc:
            self.enter_fault(exc)

    def on_timer(self) -> None:
        if self.faulted:
            return
        now_s = time.monotonic()
        source, command = self.selector.select(now_s)
        command = clamp_velocity(
            command, self.settings.max_linear_m_s, self.settings.max_angular_rad_s
        )
        if source is None or command.stopped:
            self.smoothed_velocity = VelocityCommand(0.0, 0.0, 0.0)
            self.last_slew_s = now_s
            self.stop_drive("timeout" if source is None else f"{source}_zero")
            return
        if now_s < self.settle_until_s:
            return
        elapsed_s = (
            1.0 / self.settings.write_rate_hz if self.last_slew_s is None
            else max(0.0, min(now_s - self.last_slew_s, 0.25))
        )
        smoothed = slew_velocity(
            self.smoothed_velocity, command, elapsed_s,
            self.settings.max_linear_accel_m_s2,
            self.settings.max_angular_accel_rad_s2,
        )
        try:
            self.send_motion(source, smoothed, now_s)
        except Exception as exc:
            self.enter_fault(exc)
        else:
            self.smoothed_velocity = smoothed
            self.last_slew_s = now_s

    def send_motion(self, source: str, command: VelocityCommand, now_s: float) -> None:
        wheel_command = self.kinematics.calculate(command, self.previous_steering)
        target_logical_angles = tuple(
            steering_to_plc_degree(angle, sign, offset)
            for angle, sign, offset in zip(
                wheel_command.steering_rad,
                self.settings.steering_sign,
                self.settings.steering_zero_offset_deg,
            )
        )
        target_angles = tuple(int(value) for value in logical_to_plc(
            target_logical_angles, self.settings.logical_to_plc_ids
        ))
        target_raw = tuple(int(value) for value in logical_to_plc(
            tuple(
                speed_to_raw(
                    rate * self.settings.wheel_radius_m,
                    self.settings.raw_at_one_m_s, self.stop_raw,
                )
                for rate in wheel_command.wheel_rate_rad_s
            ),
            self.settings.logical_to_plc_ids,
        ))
        plc_rates = logical_to_plc(
            wheel_command.wheel_rate_rad_s, self.settings.logical_to_plc_ids
        )
        plc_inversions = logical_to_plc(
            self.settings.drive_direction_inverted, self.settings.logical_to_plc_ids
        )
        directions = tuple(
            self.last_directions[index]
            if abs(float(rate)) < 1.0e-9 and self.last_directions is not None
            else (float(rate) >= 0.0) != bool(inverted)
            for index, (rate, inverted) in enumerate(zip(plc_rates, plc_inversions))
        )

        first_steering = self.last_angles is None
        steering_dt = (
            1.0 / self.settings.write_rate_hz if self.last_steering_s is None
            else max(0.0, min(now_s - self.last_steering_s, 0.25))
        )
        if self.last_angle_float is None:
            stepped_angle_float = tuple(float(angle) for angle in target_angles)
        else:
            max_step = self.settings.steering_rate_limit_deg_s * steering_dt
            stepped_angle_float = tuple(
                slew_heading_deg(previous, float(target), max_step)
                for previous, target in zip(self.last_angle_float, target_angles)
            )
        self.last_angle_float = stepped_angle_float
        self.last_steering_s = now_s
        plc_angles = tuple(int(round(angle)) % 360 for angle in stepped_angle_float)
        angle_changed = first_steering or any(
            steering_distance_deg(current, previous) >= self.settings.minimum_angle_change_deg
            for current, previous in zip(plc_angles, self.last_angles or plc_angles)
        )
        if angle_changed:
            if not self.settings.allow_steering_while_driving and not self.drive_stopped:
                self.controller.stop_drive()
                self.drive_stopped = True
                self.last_raw = (self.stop_raw,) * 4
            self.controller.move_angles(plc_angles)
            self.last_angles = plc_angles
            logical_sent = tuple(
                stepped_angle_float[plc_id - 1]
                for plc_id in self.settings.logical_to_plc_ids
            )
            self.previous_steering = tuple(
                math.radians(math.remainder((angle - offset) * sign, 360.0))
                for angle, sign, offset in zip(
                    logical_sent,
                    self.settings.steering_sign,
                    self.settings.steering_zero_offset_deg,
                )
            )

        if first_steering:
            self.controller.set_directions(directions)
            self.last_directions = directions
            self.settle_until_s = now_s + self.settings.steering_settle_s
            self.publish_status("initial_steering_settle", source=source, angles=plc_angles)
            return

        direction_changed = directions != self.last_directions
        stop_raw = (self.stop_raw,) * 4
        if direction_changed and self.last_raw != stop_raw and not self.drive_stopped:
            # Reverse only after the wheel speed command has reached zero.
            desired_raw = stop_raw
        else:
            if direction_changed:
                self.controller.set_directions(directions)
                self.last_directions = directions
            desired_raw = target_raw

        previous_raw = stop_raw if self.drive_stopped else self.last_raw
        largest_change = max(
            abs(target - previous)
            for target, previous in zip(desired_raw, previous_raw)
        )
        scale = (
            min(1.0, self.settings.max_raw_step_per_tick / largest_change)
            if largest_change else 1.0
        )
        stepped_raw = tuple(
            int(round(previous + (target - previous) * scale))
            for target, previous in zip(desired_raw, previous_raw)
        )
        raw_changed = self.drive_stopped or any(
            abs(current - previous) >= self.settings.minimum_raw_change
            for current, previous in zip(stepped_raw, previous_raw)
        )
        if raw_changed:
            self.controller.move_speeds(stepped_raw)
            self.last_raw = stepped_raw
            self.drive_stopped = False
        self.last_source = source
        self.publish_status(
            "driving", source=source, angles=plc_angles, raw=self.last_raw,
            directions=self.last_directions,
        )

    def stop_drive(self, reason: str) -> None:
        if self.drive_stopped:
            return
        try:
            self.controller.stop_drive()
        except Exception as exc:
            self.enter_fault(exc)
            return
        self.drive_stopped = True
        self.last_raw = (self.stop_raw,) * 4
        self.publish_status("stopped", source=None, reason=reason)

    def enter_fault(self, error: BaseException) -> None:
        self.faulted = True
        self.get_logger().fatal(f"PLC bridge fault: {error}")
        try:
            self.controller.emergency_stop()
        except Exception as stop_error:
            self.get_logger().fatal(f"Emergency-stop writes also failed: {stop_error}")
        self.publish_status("fault", source=None, error=str(error))

    def publish_status(self, state: str, source: str | None, **values: Any) -> None:
        payload = {"state": state, "source": source, "open_loop": True, **values}
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(message)


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--bridge-config", type=Path, default=DEFAULT_BRIDGE_CONFIG)
    parser.add_argument("--ip")
    parser.add_argument("--port", type=int)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_known_args()


def main() -> int:
    args, ros_args = parse_args()
    if not args.execute:
        print("Refusing real PLC access without --execute", file=sys.stderr)
        return 2
    try:
        settings = load_bridge_settings(args.bridge_config)
        config, controller, runtime, logger = build_runtime(
            args.config, ip_override=args.ip, port_override=args.port, pulse_override=None
        )
        speed_to_raw(0.0, settings.raw_at_one_m_s, config.speed_control.stop_value)
    except Exception as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2
    node = None
    exit_code = 0
    try:
        controller.connect()
        controller.stop_drive()
        controller.reset_drive_on_start()
        controller.stop_drive()
        runtime.start()
        rclpy.init(args=ros_args)
        node = Fx5uOpenLoopBridge(controller, settings, config.speed_control.stop_value)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        exit_code = 1
        logger.exception("open-loop bridge failed error=%s", exc)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not safe_shutdown(controller, runtime, logger):
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
