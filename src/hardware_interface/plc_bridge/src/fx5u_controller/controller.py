"""Serialized high-level wheel control and optional feedback access."""

from __future__ import annotations

import logging
import math
import re
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .driver import PymcProtocolSLMPDriver, SLMPDriver
from .exceptions import (
    ControllerStateError,
    PulseCleanupError,
    SLMPCommunicationError,
)
from .models import FeedbackConfig, SpeedControlConfig, TelemetrySnapshot, WheelMapping


LOGGER = logging.getLogger("fx5u_controller.controller")
SIGNED_WORD_MIN = -32_768
SIGNED_WORD_MAX = 32_767
BIT_DEVICE_RE = re.compile(r"^M\d+$")
OUTPUT_DEVICE_RE = re.compile(r"^Y[0-9A-F]+$")


class MitsubishiWheelController:
    """Controller for independent steering servos and wheel-drive motors.

    ``servo_enabled`` is software state only: True means an SLMP Servo ON write
    for a steering axis was acknowledged. It is not Servo Ready feedback.
    """

    def __init__(
        self,
        ip: str,
        port: int,
        trigger_pulse_s: float,
        *,
        wheels: WheelMapping,
        speed_control: SpeedControlConfig,
        plc_type: str = "Q",
        socket_timeout_s: float = 2.0,
        driver: Optional[SLMPDriver] = None,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> None:
        if set(wheels) != {1, 2, 3, 4}:
            raise ValueError("wheels must contain exactly wheel IDs 1, 2, 3, and 4")
        if BIT_DEVICE_RE.fullmatch(speed_control.enable_device) is None:
            raise ValueError("speed_control.enable_device must have format M<number>")
        if BIT_DEVICE_RE.fullmatch(speed_control.startup_reset_device) is None:
            raise ValueError("speed_control.startup_reset_device must have format M<number>")
        if OUTPUT_DEVICE_RE.fullmatch(speed_control.output_enable_device) is None:
            raise ValueError("speed_control.output_enable_device must have format Y<hex>")
        for name, raw_value in (
            ("stop_value", speed_control.stop_value),
            ("minimum_value", speed_control.minimum_value),
            ("maximum_value", speed_control.maximum_value),
        ):
            self._validate_word(raw_value, f"speed_control.{name}")
        if speed_control.minimum_value > speed_control.maximum_value:
            raise ValueError("speed_control.minimum_value must not exceed maximum_value")
        if not (
            speed_control.minimum_value
            <= speed_control.stop_value
            <= speed_control.maximum_value
        ):
            raise ValueError("speed_control.stop_value must be within the configured range")
        if (
            isinstance(trigger_pulse_s, bool)
            or not isinstance(trigger_pulse_s, (int, float))
            or not math.isfinite(float(trigger_pulse_s))
            or trigger_pulse_s <= 0
        ):
            raise ValueError("trigger_pulse_s must be a finite number greater than zero")

        self.wheels = dict(wheels)
        self.speed_control = speed_control
        self.trigger_pulse_s = float(trigger_pulse_s)
        self._driver: SLMPDriver = driver or PymcProtocolSLMPDriver(
            ip,
            port,
            plc_type=plc_type,
            socket_timeout_s=socket_timeout_s,
        )
        self._sleep = sleep_fn
        self._lock = threading.RLock()
        self._communication_fault = False
        self.servo_enabled: Dict[int, bool] = {wheel: False for wheel in self.wheels}
        self.direction_clockwise: Dict[int, Optional[bool]] = {
            wheel: None for wheel in self.wheels
        }
        self.speed_control_enabled = False

    @property
    def connected(self) -> bool:
        return self._driver.connected

    @property
    def communication_fault(self) -> bool:
        return self._communication_fault

    def connect(self) -> None:
        with self._lock:
            if self.connected:
                if self._communication_fault:
                    raise ControllerStateError(
                        "A communication fault is latched; disconnect and reconnect before commands"
                    )
                return
            try:
                self._driver.connect()
            except Exception as exc:
                self._communication_fault = True
                self._raise_as_communication_error("connect", exc)
            self._communication_fault = False
            self._clear_servo_state()
            self._clear_direction_state()
            self.speed_control_enabled = False
            LOGGER.info("controller connected; software command state reset")

    def disconnect(self) -> None:
        """Command the stop value, disable speed/servo control, and close the socket."""

        with self._lock:
            errors = []
            if self.connected:
                self._stop_speed_control_locked(errors)
                try:
                    self._write_bits_locked(self._head("trigger"), [0, 0, 0, 0])
                except Exception as exc:
                    errors.append(("clear all triggers", exc))
                try:
                    self._write_bits_locked(self._head("servo"), [0, 0, 0, 0])
                except Exception as exc:
                    errors.append(("turn all servos OFF", exc))
                else:
                    self._clear_servo_state()
                try:
                    self._driver.disconnect()
                except Exception as exc:
                    errors.append(("close connection", exc))
                else:
                    self._clear_direction_state()
            LOGGER.info("controlled disconnect complete errors=%s", len(errors))
            if errors:
                details = "; ".join(f"{operation}: {error}" for operation, error in errors)
                raise SLMPCommunicationError(f"Controlled disconnect was incomplete: {details}")

    def servo_on(self, wheel: int) -> None:
        with self._lock:
            self._require_operational_locked()
            wheel = self._validate_wheel(wheel)
            self._servo_on_locked(wheel)
            LOGGER.info("servo command wheel=%s state=ON", wheel)

    def servo_off(self, wheel: int) -> None:
        with self._lock:
            self._require_operational_locked()
            wheel = self._validate_wheel(wheel)
            self._write_bits_locked(self.wheels[wheel].servo, [0])
            self.servo_enabled[wheel] = False
            LOGGER.info("servo command wheel=%s state=OFF", wheel)

    def servo_on_all(self) -> None:
        with self._lock:
            self._require_operational_locked()
            self._servo_on_all_locked()
            LOGGER.info("servo command wheels=all state=ON")

    def servo_off_all(self) -> None:
        with self._lock:
            self._require_operational_locked()
            self._write_bits_locked(self._head("servo"), [0, 0, 0, 0])
            self._clear_servo_state()
            LOGGER.info("steering servo command wheels=all state=OFF")

    def reset_error(self, wheel: int) -> None:
        with self._lock:
            self._require_operational_locked()
            wheel = self._validate_wheel(wheel)
            self._pulse_locked(self.wheels[wheel].reset, 1, "fault-reset")
            LOGGER.info("fault reset pulse wheel=%s duration_s=%s", wheel, self.trigger_pulse_s)

    def reset_error_all(self) -> None:
        with self._lock:
            self._require_operational_locked()
            self._pulse_locked(self._head("reset"), 4, "fault-reset")
            LOGGER.info("fault reset pulse wheels=all duration_s=%s", self.trigger_pulse_s)

    def reset_drive_on_start(self) -> None:
        """Pulse the configured drive reset once after connecting the bridge."""

        with self._lock:
            self._require_operational_locked()
            self._pulse_locked(
                self.speed_control.startup_reset_device, 1, "drive startup reset"
            )
            LOGGER.info(
                "drive startup reset pulse device=%s duration_s=%s",
                self.speed_control.startup_reset_device, self.trigger_pulse_s,
            )

    def set_angle(self, wheel: int, angle_deg: int) -> None:
        with self._lock:
            self._require_operational_locked()
            wheel = self._validate_wheel(wheel)
            angle_deg = self._validate_angle(angle_deg, "angle_deg")
            LOGGER.info("angle stage requested wheel=%s degrees=%s", wheel, angle_deg)
            self._ensure_servo_locked(wheel)
            self._write_words_locked(self.wheels[wheel].angle, [angle_deg])
            LOGGER.info("angle staged wheel=%s degrees=%s", wheel, angle_deg)

    def set_angles(self, angle_deg_list: Sequence[int]) -> None:
        with self._lock:
            self._require_operational_locked()
            angles = self._validate_four_angles(angle_deg_list, "angle_deg_list")
            LOGGER.info("angle stage requested wheels=all degrees=%s", angles)
            self._ensure_all_servos_locked()
            self._write_words_locked(self._head("angle"), angles)
            LOGGER.info("angles staged wheels=all degrees=%s", angles)

    def set_speed(self, wheel: int, speed_value: int) -> None:
        with self._lock:
            self._require_operational_locked()
            wheel = self._validate_wheel(wheel)
            value = self._validate_speed_value(speed_value, "speed_value")
            LOGGER.info("speed requested wheel=%s value=%s", wheel, value)
            self._ensure_speed_control_locked()
            self._write_words_locked(self.wheels[wheel].speed, [value])
            LOGGER.info("speed command wheel=%s value=%s", wheel, value)

    def set_speeds(self, speed_values: Sequence[int]) -> None:
        with self._lock:
            self._require_operational_locked()
            values = self._validate_four_speed_values(speed_values, "speed_values")
            LOGGER.info("speed requested wheels=all values=%s", values)
            self._ensure_speed_control_locked()
            self._write_words_locked(self._head("speed"), values)
            LOGGER.info("speed command wheels=all values=%s", values)

    def set_direction(self, wheel: int, clockwise: bool) -> None:
        """Set one wheel-drive direction: True=CW, False=CCW."""

        with self._lock:
            self._require_operational_locked()
            wheel = self._validate_wheel(wheel)
            clockwise = self._validate_direction(clockwise, "clockwise")
            self._write_bits_locked(
                self.wheels[wheel].direction, [int(clockwise)]
            )
            self.direction_clockwise[wheel] = clockwise
            LOGGER.info(
                "drive direction command wheel=%s device=%s direction=%s",
                wheel,
                self.wheels[wheel].direction,
                "CW" if clockwise else "CCW",
            )

    def set_directions(self, clockwise_values: Sequence[bool]) -> None:
        """Set four wheel-drive directions using their nonconsecutive M devices."""

        with self._lock:
            self._require_operational_locked()
            values = self._validate_four_directions(clockwise_values)
            for wheel, clockwise in zip((1, 2, 3, 4), values):
                self._write_bits_locked(
                    self.wheels[wheel].direction, [int(clockwise)]
                )
                self.direction_clockwise[wheel] = clockwise
            LOGGER.info(
                "drive direction command wheels=all directions=%s",
                ["CW" if value else "CCW" for value in values],
            )

    def trigger(self, wheel: int) -> None:
        with self._lock:
            self._require_operational_locked()
            wheel = self._validate_wheel(wheel)
            self._ensure_servo_locked(wheel)
            self._pulse_locked(self.wheels[wheel].trigger, 1, "steering trigger")
            LOGGER.info("steering trigger wheel=%s duration_s=%s", wheel, self.trigger_pulse_s)

    def trigger_all(self) -> None:
        with self._lock:
            self._require_operational_locked()
            self._ensure_all_servos_locked()
            self._pulse_locked(self._head("trigger"), 4, "steering trigger")
            LOGGER.info("steering trigger wheels=all duration_s=%s", self.trigger_pulse_s)

    def move_angle(self, wheel: int, angle_deg: int) -> None:
        with self._lock:
            self._require_operational_locked()
            wheel = self._validate_wheel(wheel)
            angle_deg = self._validate_angle(angle_deg, "angle_deg")
            LOGGER.info("angle command requested wheel=%s degrees=%s", wheel, angle_deg)
            self._ensure_servo_locked(wheel)
            self._write_words_locked(self.wheels[wheel].angle, [angle_deg])
            self._pulse_locked(self.wheels[wheel].trigger, 1, "steering trigger")
            LOGGER.info("angle command wheel=%s degrees=%s", wheel, angle_deg)

    def move_angles(self, angle_deg_list: Sequence[int]) -> None:
        with self._lock:
            self._require_operational_locked()
            angles = self._validate_four_angles(angle_deg_list, "angle_deg_list")
            LOGGER.info("angle command requested wheels=all degrees=%s", angles)
            self._ensure_all_servos_locked()
            self._write_words_locked(self._head("angle"), angles)
            self._pulse_locked(self._head("trigger"), 4, "steering trigger")
            LOGGER.info("angle command wheels=all degrees=%s", angles)

    def move_speed(self, wheel: int, speed_value: int) -> None:
        """Compatibility alias for direct-value speed control."""

        self.set_speed(wheel, speed_value)

    def move_speeds(self, speed_values: Sequence[int]) -> None:
        """Compatibility alias for direct-value speed control."""

        self.set_speeds(speed_values)

    def stop_drive(self) -> None:
        """Stop wheel drive and disable M58 without disabling steering servos."""

        with self._lock:
            self._require_operational_locked()
            errors: List[Tuple[str, BaseException]] = []
            self._stop_speed_control_locked(errors)
            if errors:
                details = "; ".join(
                    f"{operation}: {error}" for operation, error in errors
                )
                raise SLMPCommunicationError(f"Drive stop was incomplete: {details}")
            LOGGER.info("wheel drive stopped and speed control disabled")

    def read_feedback(self, config: FeedbackConfig) -> TelemetrySnapshot:
        """Read only explicitly configured feedback devices."""

        if not config.enabled:
            raise ControllerStateError("feedback is disabled in configuration")
        with self._lock:
            self._require_operational_locked()
            servo_ready = self._read_optional_bits_locked(config.servo_ready)
            actual_angle = self._read_optional_words_locked(config.actual_angle)
            actual_speed = self._read_optional_words_locked(config.actual_speed)
            alarm_code = self._read_optional_words_locked(config.alarm_code)
            snapshot = TelemetrySnapshot(
                captured_at=datetime.now(timezone.utc),
                servo_ready=None
                if servo_ready is None
                else tuple(bool(value) for value in servo_ready),  # type: ignore[arg-type]
                actual_angle=self._as_four_tuple(actual_angle),
                actual_speed=self._as_four_tuple(actual_speed),
                alarm_code=self._as_four_tuple(alarm_code),
            )
            LOGGER.debug("feedback snapshot=%s", snapshot)
            if snapshot.alarm_code is not None and any(snapshot.alarm_code):
                LOGGER.warning("nonzero PLC alarm feedback codes=%s", snapshot.alarm_code)
            return snapshot

    def update_heartbeat(self, device: str, state: bool) -> None:
        """Write an explicitly configured watchdog heartbeat bit."""

        self._validate_bit_device(device)
        if not isinstance(state, bool):
            raise TypeError("heartbeat state must be bool")
        with self._lock:
            self._require_operational_locked()
            self._write_bits_locked(device, [int(state)])
            LOGGER.debug("heartbeat device=%s state=%s", device, int(state))

    def clear_heartbeat(self, device: str) -> None:
        """Best-effort heartbeat clear, permitted after a latched fault."""

        self._validate_bit_device(device)
        with self._lock:
            if not self.connected:
                return
            self._write_bits_locked(device, [0])
            LOGGER.info("heartbeat cleared device=%s", device)

    def emergency_stop(self) -> None:
        """Command the stop value, disable M58, clear triggers, and disable servos."""

        with self._lock:
            LOGGER.critical("software emergency stop requested connected=%s", self.connected)
            if not self.connected:
                return
            errors = []
            self._stop_speed_control_locked(errors)
            try:
                self._write_bits_locked(self._head("trigger"), [0, 0, 0, 0])
            except Exception as exc:
                errors.append(("clear all triggers", exc))
            try:
                self._write_bits_locked(self._head("servo"), [0, 0, 0, 0])
            except Exception as exc:
                errors.append(("turn all servos OFF", exc))
            else:
                self._clear_servo_state()
            if errors:
                details = "; ".join(f"{operation}: {error}" for operation, error in errors)
                LOGGER.critical("software emergency stop incomplete details=%s", details)
                raise SLMPCommunicationError(f"Software emergency stop was incomplete: {details}")
            LOGGER.critical("software emergency stop writes acknowledged")

    def _servo_on_locked(self, wheel: int) -> None:
        self._write_bits_locked(self.wheels[wheel].servo, [1])
        self.servo_enabled[wheel] = True

    def _servo_on_all_locked(self) -> None:
        self._write_bits_locked(self._head("servo"), [1, 1, 1, 1])
        for wheel in self.wheels:
            self.servo_enabled[wheel] = True

    def _ensure_servo_locked(self, wheel: int) -> None:
        if not self.servo_enabled[wheel]:
            self._servo_on_locked(wheel)

    def _ensure_all_servos_locked(self) -> None:
        if not all(self.servo_enabled.values()):
            self._servo_on_all_locked()

    def _ensure_speed_control_locked(self) -> None:
        if not self.speed_control_enabled:
            # Preload the confirmed stop value so stale D data cannot cause a
            # transient during the first interval between M58 ON and target.
            self._write_words_locked(self._head("speed"), self._stop_speed_values())
        # Reassert M58 before every target write because software state is not
        # feedback and PLC logic may have cleared the device independently.
        self._write_bits_locked(self.speed_control.enable_device, [1])
        self._write_bits_locked(self.speed_control.output_enable_device, [1])
        self.speed_control_enabled = True
        LOGGER.info(
            "speed control enable acknowledged devices=%s,%s safe_raw=%s",
            self.speed_control.enable_device,
            self.speed_control.output_enable_device,
            self.speed_control.stop_value,
        )

    def _stop_speed_control_locked(self, errors: list) -> None:
        try:
            self._write_words_locked(self._head("speed"), self._stop_speed_values())
        except Exception as exc:
            errors.append(("command all speeds to the stop value", exc))
        output_off = False
        control_off = False
        try:
            self._write_bits_locked(self.speed_control.output_enable_device, [0])
        except Exception as exc:
            errors.append((f"turn {self.speed_control.output_enable_device} OFF", exc))
        else:
            output_off = True
        try:
            self._write_bits_locked(self.speed_control.enable_device, [0])
        except Exception as exc:
            errors.append((f"turn {self.speed_control.enable_device} OFF", exc))
        else:
            control_off = True
        if output_off and control_off:
            self.speed_control_enabled = False

    def _pulse_locked(self, head_device: str, count: int, pulse_name: str) -> None:
        primary_error: Optional[BaseException] = None
        try:
            self._write_bits_locked(head_device, [1] * count)
            self._sleep(self.trigger_pulse_s)
        except BaseException as exc:
            primary_error = exc
        try:
            self._write_bits_locked(head_device, [0] * count)
        except BaseException as cleanup_error:
            if primary_error is not None:
                raise PulseCleanupError(
                    f"{pulse_name} at {head_device} failed and its OFF cleanup also failed",
                    primary_error,
                    cleanup_error,
                ) from cleanup_error
            raise
        if primary_error is not None:
            raise primary_error

    def _write_bits_locked(self, head_device: str, values: Sequence[int]) -> None:
        try:
            self._driver.write_bits(head_device, values)
        except BaseException as exc:
            self._latch_communication_fault("bit write", head_device, exc)

    def _write_words_locked(self, head_device: str, values: Sequence[int]) -> None:
        try:
            self._driver.write_words(head_device, values)
        except BaseException as exc:
            self._latch_communication_fault("word write", head_device, exc)

    def _read_bits_locked(self, head_device: str, count: int) -> Sequence[int]:
        try:
            return self._driver.read_bits(head_device, count)
        except BaseException as exc:
            self._latch_communication_fault("bit read", head_device, exc)

    def _read_words_locked(self, head_device: str, count: int) -> Sequence[int]:
        try:
            return self._driver.read_words(head_device, count)
        except BaseException as exc:
            self._latch_communication_fault("word read", head_device, exc)

    def _read_optional_bits_locked(
        self, devices: Optional[Tuple[str, str, str, str]]
    ) -> Optional[Sequence[int]]:
        if devices is None:
            return None
        return self._read_devices_locked(devices, self._read_bits_locked)

    def _read_optional_words_locked(
        self, devices: Optional[Tuple[str, str, str, str]]
    ) -> Optional[Sequence[int]]:
        if devices is None:
            return None
        return self._read_devices_locked(devices, self._read_words_locked)

    def _read_devices_locked(
        self,
        devices: Tuple[str, str, str, str],
        reader: Callable[[str, int], Sequence[int]],
    ) -> Sequence[int]:
        if self._are_consecutive(devices):
            return reader(devices[0], 4)
        values: List[int] = []
        for device in devices:
            values.extend(reader(device, 1))
        return values

    def _latch_communication_fault(
        self, operation: str, device: str, exc: BaseException
    ) -> None:
        self._communication_fault = True
        LOGGER.error(
            "communication fault latched operation=%s device=%s error=%s",
            operation,
            device,
            exc,
        )
        if not isinstance(exc, Exception):
            raise exc
        self._raise_as_communication_error(f"{operation} at {device}", exc)

    def _require_operational_locked(self) -> None:
        if not self.connected:
            raise ControllerStateError("SLMP is not connected; call connect() first")
        if self._communication_fault:
            raise ControllerStateError(
                "A communication fault is latched; run emergency_stop(), then disconnect "
                "and reconnect before issuing more commands"
            )

    def _head(self, field: str) -> str:
        return getattr(self.wheels[1], field)

    def _validate_wheel(self, wheel: int) -> int:
        if isinstance(wheel, bool) or not isinstance(wheel, int) or wheel not in (1, 2, 3, 4):
            raise ValueError("wheel must be exactly one of (1, 2, 3, 4)")
        return wheel

    @staticmethod
    def _validate_word(value: int, parameter_name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(
                f"{parameter_name} must be an integer; no fractional value or scaling is applied"
            )
        if not SIGNED_WORD_MIN <= value <= SIGNED_WORD_MAX:
            raise ValueError(
                f"{parameter_name}={value} is outside signed 16-bit range "
                f"{SIGNED_WORD_MIN}..{SIGNED_WORD_MAX}"
            )
        return value

    @classmethod
    def _validate_four_words(cls, values: Sequence[int], name: str) -> Sequence[int]:
        if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
            raise TypeError(f"{name} must be a sequence of four integers")
        if len(values) != 4:
            raise ValueError(f"{name} must contain exactly four values")
        return [
            cls._validate_word(value, f"{name}[{index}]")
            for index, value in enumerate(values)
        ]

    @classmethod
    def _validate_angle(cls, value: int, name: str) -> int:
        value = cls._validate_word(value, name)
        if not 0 <= value < 360:
            raise ValueError(f"{name}={value} is outside PLC steering range 0..359 degrees")
        return value

    @classmethod
    def _validate_four_angles(cls, values: Sequence[int], name: str) -> Sequence[int]:
        if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
            raise TypeError(f"{name} must be a sequence of four integers")
        if len(values) != 4:
            raise ValueError(f"{name} must contain exactly four values")
        return [
            cls._validate_angle(value, f"{name}[{index}]")
            for index, value in enumerate(values)
        ]

    def _validate_speed_value(self, value: int, parameter_name: str) -> int:
        value = self._validate_word(value, parameter_name)
        if not (
            self.speed_control.minimum_value
            <= value
            <= self.speed_control.maximum_value
        ):
            raise ValueError(
                f"{parameter_name}={value} is outside configured direct-speed range "
                f"{self.speed_control.minimum_value}..{self.speed_control.maximum_value}"
            )
        return value

    def _validate_four_speed_values(
        self, values: Sequence[int], name: str
    ) -> Sequence[int]:
        if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
            raise TypeError(f"{name} must be a sequence of four integers")
        if len(values) != 4:
            raise ValueError(f"{name} must contain exactly four values")
        return [
            self._validate_speed_value(value, f"{name}[{index}]")
            for index, value in enumerate(values)
        ]

    @staticmethod
    def _validate_direction(value: bool, parameter_name: str) -> bool:
        if not isinstance(value, bool):
            raise TypeError(f"{parameter_name} must be bool")
        return value

    @classmethod
    def _validate_four_directions(cls, values: Sequence[bool]) -> Sequence[bool]:
        if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
            raise TypeError("clockwise_values must be a sequence of four booleans")
        if len(values) != 4:
            raise ValueError("clockwise_values must contain exactly four values")
        return [
            cls._validate_direction(value, f"clockwise_values[{index}]")
            for index, value in enumerate(values)
        ]

    def _stop_speed_values(self) -> List[int]:
        return [self.speed_control.stop_value] * 4

    @staticmethod
    def _validate_bit_device(device: str) -> None:
        if not isinstance(device, str) or BIT_DEVICE_RE.fullmatch(device) is None:
            raise ValueError("heartbeat device must have format M<number>")

    @staticmethod
    def _are_consecutive(devices: Sequence[str]) -> bool:
        prefix = devices[0][0]
        first = int(devices[0][1:])
        return list(devices) == [f"{prefix}{first + offset}" for offset in range(4)]

    @staticmethod
    def _as_four_tuple(
        values: Optional[Sequence[int]],
    ) -> Optional[Tuple[int, int, int, int]]:
        if values is None:
            return None
        return tuple(values)  # type: ignore[return-value]

    def _clear_servo_state(self) -> None:
        for wheel in self.wheels:
            self.servo_enabled[wheel] = False

    def _clear_direction_state(self) -> None:
        for wheel in self.wheels:
            self.direction_clockwise[wheel] = None

    @staticmethod
    def _raise_as_communication_error(operation: str, exc: Exception) -> None:
        if isinstance(exc, SLMPCommunicationError):
            raise exc
        raise SLMPCommunicationError(f"SLMP {operation} failed: {exc}") from exc
