"""Strict YAML configuration loading for site deployment."""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Set, Tuple, Union

import yaml

from .exceptions import ConfigurationError
from .models import (
    ApplicationConfig,
    DeviceGroup,
    FeedbackConfig,
    HeartbeatConfig,
    LoggingConfig,
    PLCConfig,
    RegisterMappingConfig,
    SpeedControlConfig,
    TimingConfig,
    WheelDevices,
    WheelMapping,
)


EXPECTED_WHEEL_NAMES = {
    1: "right_top",
    2: "right_bottom",
    3: "left_top",
    4: "left_bottom",
}
BIT_DEVICE_RE = re.compile(r"^M\d+$")
WORD_DEVICE_RE = re.compile(r"^D\d+$")
VALID_LOG_LEVELS = {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}


def load_application_config(path: Union[Path, str]) -> ApplicationConfig:
    config_path = Path(path).expanduser().resolve()
    data = _load_yaml_mapping(config_path)
    _reject_unknown(
        data,
        {"plc", "timing", "logging", "feedback", "heartbeat", "register_mapping_file"},
        "application config",
    )

    mapping_name = _required(data, "register_mapping_file", "application config")
    if not isinstance(mapping_name, str) or not mapping_name.strip():
        raise ConfigurationError("register_mapping_file must be a non-empty path")
    mapping_path = _resolve_relative(config_path.parent, mapping_name)

    register_mapping = _load_register_configuration(mapping_path)
    application = ApplicationConfig(
        plc=_parse_plc(_section(data, "plc")),
        timing=_parse_timing(_section(data, "timing")),
        logging=_parse_logging(_section(data, "logging"), config_path.parent),
        feedback=_parse_feedback(_section(data, "feedback")),
        heartbeat=_parse_heartbeat(_section(data, "heartbeat")),
        wheels=register_mapping.wheels,
        speed_control=register_mapping.speed_control,
    )
    _validate_device_separation(application)
    return application


def load_register_mapping(path: Union[Path, str]) -> WheelMapping:
    """Load the four wheel devices for callers that only need wheel mapping."""

    return _load_register_configuration(path).wheels


def _load_register_configuration(path: Union[Path, str]) -> RegisterMappingConfig:
    mapping_path = Path(path).expanduser().resolve()
    data = _load_yaml_mapping(mapping_path)
    _reject_unknown(data, {"wheels", "speed_control"}, "register mapping")
    raw_wheels = _required(data, "wheels", "register mapping")
    if not isinstance(raw_wheels, Mapping):
        raise ConfigurationError("register mapping 'wheels' must be a mapping")

    normalized: Dict[int, Any] = {}
    for raw_key, raw_value in raw_wheels.items():
        if isinstance(raw_key, bool):
            raise ConfigurationError("wheel keys must be integers 1..4")
        try:
            wheel = int(raw_key)
        except (TypeError, ValueError) as exc:
            raise ConfigurationError(f"invalid wheel key: {raw_key!r}") from exc
        if str(wheel) != str(raw_key):
            raise ConfigurationError(f"invalid wheel key: {raw_key!r}")
        normalized[wheel] = raw_value

    if set(normalized) != {1, 2, 3, 4}:
        raise ConfigurationError("register mapping must define exactly wheels 1, 2, 3, and 4")

    wheels: Dict[int, WheelDevices] = {}
    used_devices: Set[str] = set()
    fields = {"name", "trigger", "servo", "reset", "direction", "angle", "speed"}
    for wheel in (1, 2, 3, 4):
        raw = normalized[wheel]
        if not isinstance(raw, Mapping):
            raise ConfigurationError(f"wheel {wheel} must be a mapping")
        _reject_unknown(raw, fields, f"wheel {wheel}")
        missing = fields - set(raw)
        if missing:
            raise ConfigurationError(f"wheel {wheel} is missing: {', '.join(sorted(missing))}")

        name = raw["name"]
        if name != EXPECTED_WHEEL_NAMES[wheel]:
            raise ConfigurationError(
                f"wheel {wheel} name must remain {EXPECTED_WHEEL_NAMES[wheel]!r}, got {name!r}"
            )
        trigger = _device(raw["trigger"], BIT_DEVICE_RE, f"wheel {wheel}.trigger")
        servo = _device(raw["servo"], BIT_DEVICE_RE, f"wheel {wheel}.servo")
        reset = _device(raw["reset"], BIT_DEVICE_RE, f"wheel {wheel}.reset")
        direction = _device(raw["direction"], BIT_DEVICE_RE, f"wheel {wheel}.direction")
        angle = _device(raw["angle"], WORD_DEVICE_RE, f"wheel {wheel}.angle")
        speed = _device(raw["speed"], WORD_DEVICE_RE, f"wheel {wheel}.speed")
        for device in (trigger, servo, reset, direction, angle, speed):
            if device in used_devices:
                raise ConfigurationError(f"PLC device {device} is assigned more than once")
            used_devices.add(device)
        wheels[wheel] = WheelDevices(
            name, trigger, servo, reset, direction, angle, speed
        )

    for field in ("trigger", "servo", "reset", "angle", "speed"):
        devices = [getattr(wheels[wheel], field) for wheel in (1, 2, 3, 4)]
        _require_consecutive(devices, field)
    speed_control = _parse_speed_control(_section(data, "speed_control"))
    if speed_control.enable_device in used_devices:
        raise ConfigurationError(
            f"PLC device {speed_control.enable_device} is assigned more than once"
        )
    return RegisterMappingConfig(wheels=wheels, speed_control=speed_control)


def _parse_speed_control(raw: Mapping[str, Any]) -> SpeedControlConfig:
    fields = {"enable_device", "stop_value", "minimum_value", "maximum_value"}
    _reject_unknown(raw, fields, "speed_control")
    enable_device = _device(
        _required(raw, "enable_device", "speed_control"),
        BIT_DEVICE_RE,
        "speed_control.enable_device",
    )
    stop_value = _signed_word(
        _required(raw, "stop_value", "speed_control"),
        "speed_control.stop_value",
    )
    minimum_value = _signed_word(
        _required(raw, "minimum_value", "speed_control"),
        "speed_control.minimum_value",
    )
    maximum_value = _signed_word(
        _required(raw, "maximum_value", "speed_control"),
        "speed_control.maximum_value",
    )
    if minimum_value > maximum_value:
        raise ConfigurationError("speed_control.minimum_value must not exceed maximum_value")
    if not minimum_value <= stop_value <= maximum_value:
        raise ConfigurationError("speed_control.stop_value must be within the configured range")
    return SpeedControlConfig(enable_device, stop_value, minimum_value, maximum_value)


def require_plc_endpoint(config: PLCConfig) -> Tuple[str, int]:
    if config.ip is None or config.port is None:
        raise ConfigurationError(
            "PLC endpoint is not configured; set plc.ip and plc.port or pass CLI overrides"
        )
    return config.ip, config.port


def _parse_plc(raw: Mapping[str, Any]) -> PLCConfig:
    _reject_unknown(raw, {"ip", "port", "plc_type", "socket_timeout_s"}, "plc")
    ip_value = raw.get("ip")
    if ip_value is not None and (not isinstance(ip_value, str) or not ip_value.strip()):
        raise ConfigurationError("plc.ip must be null or a non-empty string")
    ip = ip_value.strip() if isinstance(ip_value, str) else None

    port = raw.get("port")
    if port is not None:
        port = _integer(port, "plc.port", minimum=1, maximum=65_535)

    plc_type = raw.get("plc_type", "Q")
    if not isinstance(plc_type, str) or plc_type not in {"Q", "L", "QnA", "iQ-L", "iQ-R"}:
        raise ConfigurationError("plc.plc_type is not accepted by pymcprotocol.Type3E")
    timeout = _positive_number(raw.get("socket_timeout_s", 2.0), "plc.socket_timeout_s")
    return PLCConfig(ip, port, plc_type, timeout)


def _parse_timing(raw: Mapping[str, Any]) -> TimingConfig:
    _reject_unknown(raw, {"trigger_pulse_s"}, "timing")
    pulse = _positive_number(
        _required(raw, "trigger_pulse_s", "timing"), "timing.trigger_pulse_s"
    )
    return TimingConfig(pulse)


def _parse_logging(raw: Mapping[str, Any], base_dir: Path) -> LoggingConfig:
    _reject_unknown(raw, {"level", "file", "max_bytes", "backup_count", "console"}, "logging")
    level = str(raw.get("level", "INFO")).upper()
    if level not in VALID_LOG_LEVELS:
        raise ConfigurationError(f"logging.level must be one of {sorted(VALID_LOG_LEVELS)}")
    file_value = _required(raw, "file", "logging")
    if not isinstance(file_value, str) or not file_value.strip():
        raise ConfigurationError("logging.file must be a non-empty path")
    return LoggingConfig(
        level=level,
        file=_resolve_relative(base_dir, file_value),
        max_bytes=_integer(raw.get("max_bytes", 10_485_760), "logging.max_bytes", minimum=1),
        backup_count=_integer(raw.get("backup_count", 10), "logging.backup_count", minimum=1),
        console=_boolean(raw.get("console", True), "logging.console"),
    )


def _parse_feedback(raw: Mapping[str, Any]) -> FeedbackConfig:
    _reject_unknown(raw, {"enabled", "poll_interval_s", "devices"}, "feedback")
    enabled = _boolean(raw.get("enabled", False), "feedback.enabled")
    interval = _positive_number(raw.get("poll_interval_s", 0.1), "feedback.poll_interval_s")
    devices = raw.get("devices", {})
    if not isinstance(devices, Mapping):
        raise ConfigurationError("feedback.devices must be a mapping")
    fields = {"servo_ready", "actual_angle", "actual_speed", "alarm_code"}
    _reject_unknown(devices, fields, "feedback.devices")
    result = {
        "servo_ready": _device_group(devices.get("servo_ready"), BIT_DEVICE_RE, "servo_ready"),
        "actual_angle": _device_group(devices.get("actual_angle"), WORD_DEVICE_RE, "actual_angle"),
        "actual_speed": _device_group(devices.get("actual_speed"), WORD_DEVICE_RE, "actual_speed"),
        "alarm_code": _device_group(devices.get("alarm_code"), WORD_DEVICE_RE, "alarm_code"),
    }
    if enabled and all(value is None for value in result.values()):
        raise ConfigurationError(
            "feedback.enabled is true but no confirmed feedback devices are configured"
        )
    return FeedbackConfig(enabled=enabled, poll_interval_s=interval, **result)


def _parse_heartbeat(raw: Mapping[str, Any]) -> HeartbeatConfig:
    fields = {"enabled", "device", "interval_s", "plc_watchdog_timeout_s"}
    _reject_unknown(raw, fields, "heartbeat")
    enabled = _boolean(raw.get("enabled", False), "heartbeat.enabled")
    device_value = raw.get("device")
    device = None
    if device_value is not None:
        device = _device(device_value, BIT_DEVICE_RE, "heartbeat.device")
    interval = _positive_number(raw.get("interval_s", 0.2), "heartbeat.interval_s")
    timeout = _positive_number(
        raw.get("plc_watchdog_timeout_s", 0.5), "heartbeat.plc_watchdog_timeout_s"
    )
    if enabled and device is None:
        raise ConfigurationError(
            "heartbeat.enabled is true but no PLC-approved heartbeat.device is configured"
        )
    if enabled and interval >= timeout:
        raise ConfigurationError(
            "heartbeat.interval_s must be less than heartbeat.plc_watchdog_timeout_s"
        )
    return HeartbeatConfig(enabled, device, interval, timeout)


def _device_group(value: Any, pattern: re.Pattern[str], name: str) -> DeviceGroup:
    if value is None:
        return None
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != 4:
        raise ConfigurationError(f"feedback.devices.{name} must be null or four devices")
    parsed = tuple(
        _device(device, pattern, f"feedback.devices.{name}[{index}]")
        for index, device in enumerate(value)
    )
    return parsed  # type: ignore[return-value]


def _load_yaml_mapping(path: Path) -> Mapping[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except OSError as exc:
        raise ConfigurationError(f"cannot read configuration {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, Mapping):
        raise ConfigurationError(f"configuration {path} must contain a YAML mapping")
    return data


def _section(data: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = _required(data, name, "application config")
    if not isinstance(value, Mapping):
        raise ConfigurationError(f"{name} must be a mapping")
    return value


def _required(data: Mapping[str, Any], key: str, location: str) -> Any:
    if key not in data:
        raise ConfigurationError(f"{location} is missing required key {key!r}")
    return data[key]


def _reject_unknown(data: Mapping[Any, Any], allowed: Set[str], location: str) -> None:
    unknown = {str(key) for key in data if key not in allowed}
    if unknown:
        raise ConfigurationError(f"unknown keys in {location}: {', '.join(sorted(unknown))}")


def _resolve_relative(base_dir: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return (base_dir / path).resolve() if not path.is_absolute() else path.resolve()


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigurationError(f"{name} must be true or false")
    return value


def _integer(value: Any, name: str, *, minimum: int, maximum: Optional[int] = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError(f"{name} must be an integer")
    if value < minimum or (maximum is not None and value > maximum):
        upper = f"..{maximum}" if maximum is not None else " or greater"
        raise ConfigurationError(f"{name} must be {minimum}{upper}")
    return value


def _positive_number(value: Any, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or value <= 0
    ):
        raise ConfigurationError(f"{name} must be a finite number greater than zero")
    return float(value)


def _signed_word(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError(f"{name} must be an integer")
    if not -32_768 <= value <= 32_767:
        raise ConfigurationError(f"{name} must be within signed 16-bit range")
    return value


def _device(value: Any, pattern: re.Pattern[str], name: str) -> str:
    if not isinstance(value, str) or pattern.fullmatch(value) is None:
        expected = "M<number>" if pattern is BIT_DEVICE_RE else "D<number>"
        raise ConfigurationError(f"{name} must have format {expected}")
    return value


def _require_consecutive(devices: Sequence[str], field: str) -> None:
    prefix = devices[0][0]
    first = int(devices[0][1:])
    expected = [f"{prefix}{first + offset}" for offset in range(4)]
    if list(devices) != expected:
        raise ConfigurationError(
            f"wheel {field} devices must be consecutive for safe batch access: {expected}"
        )


def _validate_device_separation(config: ApplicationConfig) -> None:
    command_devices = {
        getattr(config.wheels[wheel], field)
        for wheel in (1, 2, 3, 4)
        for field in ("trigger", "servo", "reset", "direction", "angle", "speed")
    }
    command_devices.add(config.speed_control.enable_device)
    feedback_devices = {
        device
        for group in (
            config.feedback.servo_ready,
            config.feedback.actual_angle,
            config.feedback.actual_speed,
            config.feedback.alarm_code,
        )
        if group is not None
        for device in group
    }
    overlap = command_devices & feedback_devices
    if overlap:
        raise ConfigurationError(
            "feedback devices overlap command devices: " + ", ".join(sorted(overlap))
        )
    heartbeat = config.heartbeat.device
    if heartbeat is not None and heartbeat in command_devices | feedback_devices:
        raise ConfigurationError(
            f"heartbeat device {heartbeat} overlaps another configured device"
        )
