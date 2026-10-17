"""Pure, testable open-loop swerve kinematics and command arbitration."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    wz: float

    @property
    def stopped(self) -> bool:
        return max(abs(self.vx), abs(self.vy), abs(self.wz)) < 1.0e-9


@dataclass(frozen=True)
class WheelCommand:
    steering_rad: Tuple[float, float, float, float]
    wheel_rate_rad_s: Tuple[float, float, float, float]


class ManualCommandSelector:
    """Select joystick while deadman is held, otherwise fresh keyboard input."""

    def __init__(self, timeout_s: float) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        self.timeout_s = timeout_s
        self._keyboard: Optional[VelocityCommand] = None
        self._keyboard_time = float("-inf")
        self._joystick: Optional[VelocityCommand] = None
        self._joystick_time = float("-inf")
        self._joystick_enabled = False

    def update_keyboard(self, command: VelocityCommand, received_s: float) -> None:
        self._validate(command, received_s)
        self._keyboard, self._keyboard_time = command, received_s

    def update_joystick(
        self, command: VelocityCommand, received_s: float, *, enabled: bool
    ) -> None:
        self._validate(command, received_s)
        self._joystick, self._joystick_time = command, received_s
        self._joystick_enabled = bool(enabled)

    def select(self, now_s: float) -> Tuple[Optional[str], VelocityCommand]:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        joy_age = now_s - self._joystick_time
        key_age = now_s - self._keyboard_time
        if self._joystick_enabled and self._joystick is not None and 0 <= joy_age <= self.timeout_s:
            return "joystick", self._joystick
        if self._keyboard is not None and 0 <= key_age <= self.timeout_s:
            return "keyboard", self._keyboard
        return None, VelocityCommand(0.0, 0.0, 0.0)

    @staticmethod
    def _validate(command: VelocityCommand, received_s: float) -> None:
        if not all(math.isfinite(v) for v in (command.vx, command.vy, command.wz, received_s)):
            raise ValueError("command and receipt time must be finite")


class OpenLoopSwerveKinematics:
    """Inverse kinematics seeded by last-commanded, not measured, steering."""

    def __init__(
        self, wheelbase_m: float, track_width_m: float,
        wheel_radius_m: float, max_wheel_rate_rad_s: float,
    ) -> None:
        values = (wheelbase_m, track_width_m, wheel_radius_m, max_wheel_rate_rad_s)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("geometry and max wheel rate must be finite and positive")
        self.wheel_radius_m = wheel_radius_m
        self.max_wheel_rate_rad_s = max_wheel_rate_rad_s
        hx, hy = wheelbase_m / 2.0, track_width_m / 2.0
        self._positions = ((hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy))

    def calculate(
        self, command: VelocityCommand, previous_steering_rad: Sequence[float]
    ) -> WheelCommand:
        if len(previous_steering_rad) != 4:
            raise ValueError("previous_steering_rad must contain four values")
        if not all(math.isfinite(v) for v in
                   (command.vx, command.vy, command.wz, *previous_steering_rad)):
            raise ValueError("kinematics inputs must be finite")
        steering, rates = [], []
        for (x_pos, y_pos), previous in zip(self._positions, previous_steering_rad):
            vx = command.vx - command.wz * y_pos
            vy = command.vy + command.wz * x_pos
            speed = math.hypot(vx, vy)
            if speed < 1.0e-9:
                steering.append(float(previous))
                rates.append(0.0)
                continue
            target = math.atan2(vy, vx)
            delta = math.remainder(target - previous, 2.0 * math.pi)
            if abs(delta) > math.pi / 2.0:
                target = math.remainder(target + math.pi, 2.0 * math.pi)
                speed = -speed
            else:
                target = math.remainder(target, 2.0 * math.pi)
            steering.append(target)
            rates.append(speed / self.wheel_radius_m)
        peak = max(abs(v) for v in rates)
        if peak > self.max_wheel_rate_rad_s:
            scale = self.max_wheel_rate_rad_s / peak
            rates = [v * scale for v in rates]
        return WheelCommand(tuple(steering), tuple(rates))  # type: ignore[arg-type]


def clamp_velocity(
    command: VelocityCommand, max_linear_m_s: float, max_angular_rad_s: float
) -> VelocityCommand:
    if not all(math.isfinite(v) for v in
               (command.vx, command.vy, command.wz, max_linear_m_s, max_angular_rad_s)):
        raise ValueError("velocity command and limits must be finite")
    if max_linear_m_s <= 0 or max_angular_rad_s <= 0:
        raise ValueError("velocity limits must be positive")
    magnitude = math.hypot(command.vx, command.vy)
    scale = min(1.0, max_linear_m_s / magnitude) if magnitude else 1.0
    return VelocityCommand(
        command.vx * scale, command.vy * scale,
        max(-max_angular_rad_s, min(max_angular_rad_s, command.wz)),
    )


def slew_velocity(
    current: VelocityCommand, target: VelocityCommand, dt_s: float,
    max_linear_accel_m_s2: float, max_angular_accel_rad_s2: float,
) -> VelocityCommand:
    """Approach a live Twist target without exceeding configured acceleration."""
    values = (
        current.vx, current.vy, current.wz, target.vx, target.vy, target.wz,
        dt_s, max_linear_accel_m_s2, max_angular_accel_rad_s2,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocity slew inputs must be finite")
    if dt_s < 0 or max_linear_accel_m_s2 <= 0 or max_angular_accel_rad_s2 <= 0:
        raise ValueError("velocity slew requires nonnegative time and positive limits")
    delta_x = target.vx - current.vx
    delta_y = target.vy - current.vy
    linear_delta = math.hypot(delta_x, delta_y)
    linear_step = max_linear_accel_m_s2 * dt_s
    scale = min(1.0, linear_step / linear_delta) if linear_delta else 1.0
    yaw_delta = target.wz - current.wz
    yaw_step = max_angular_accel_rad_s2 * dt_s
    return VelocityCommand(
        current.vx + delta_x * scale,
        current.vy + delta_y * scale,
        current.wz + max(-yaw_step, min(yaw_step, yaw_delta)),
    )


def speed_to_raw(speed_m_s: float, raw_at_one_m_s: int, stop_raw: int = 660) -> int:
    """Provisional linear wheel-speed calibration; direction uses separate bits."""
    if not math.isfinite(speed_m_s):
        raise ValueError("speed must be finite")
    if any(isinstance(v, bool) or not isinstance(v, int)
           for v in (raw_at_one_m_s, stop_raw)):
        raise ValueError("raw calibration values must be integers")
    if not 0 <= raw_at_one_m_s < stop_raw:
        raise ValueError("raw_at_one_m_s must be below positive stop_raw")
    return max(0, min(stop_raw, int(round(
        stop_raw - abs(speed_m_s) * (stop_raw - raw_at_one_m_s)
    ))))


def rate_to_raw(rate_rad_s: float, max_rate_rad_s: float, stop_raw: int = 660) -> int:
    """Map zero rate to 660/0.8 V and maximum magnitude to 0/3.8 V."""
    if not math.isfinite(rate_rad_s) or not math.isfinite(max_rate_rad_s):
        raise ValueError("speed mapping values must be finite")
    if max_rate_rad_s <= 0 or isinstance(stop_raw, bool) or stop_raw <= 0:
        raise ValueError("max_rate_rad_s and stop_raw must be positive")
    ratio = min(abs(rate_rad_s) / max_rate_rad_s, 1.0)
    return int(round(stop_raw * (1.0 - ratio)))


def steering_to_plc_degree(angle_rad: float, sign: int, zero_offset_deg: float) -> int:
    """Encode a logical steering angle as one PLC word in the 0..359 degree range."""
    if not math.isfinite(angle_rad) or not math.isfinite(zero_offset_deg):
        raise ValueError("steering angle and zero offset must be finite")
    if sign not in (-1, 1):
        raise ValueError("steering sign must be -1 or 1")
    return int(round(math.degrees(angle_rad) * sign + zero_offset_deg)) % 360


def slew_heading_deg(current_deg: float, target_deg: float, max_step_deg: float) -> float:
    """Move a wrapped steering angle by at most max_step_deg."""
    if not all(math.isfinite(value) for value in
               (current_deg, target_deg, max_step_deg)):
        raise ValueError("steering slew inputs must be finite")
    if max_step_deg < 0:
        raise ValueError("max_step_deg must be nonnegative")
    delta = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    if abs(delta) <= max_step_deg:
        return target_deg % 360.0
    return (current_deg + math.copysign(max_step_deg, delta)) % 360.0


def steering_distance_deg(first: int, second: int) -> int:
    """Shortest difference between two whole-degree headings."""
    if not all(isinstance(value, int) and not isinstance(value, bool)
               and 0 <= value < 360 for value in (first, second)):
        raise ValueError("PLC steering angles must be integers in 0..359")
    return abs((first - second + 180) % 360 - 180)


def logical_to_plc(values: Sequence[object], logical_to_plc_ids: Sequence[int]) -> Tuple[object, ...]:
    """Reorder FL/FR/RL/RR values into PLC wheel IDs 1/2/3/4."""
    if len(values) != 4 or len(logical_to_plc_ids) != 4 or set(logical_to_plc_ids) != {1, 2, 3, 4}:
        raise ValueError("wheel values need a four-ID permutation")
    result = [None, None, None, None]
    for value, plc_id in zip(values, logical_to_plc_ids):
        result[plc_id - 1] = value
    return tuple(result)
