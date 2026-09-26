"""Validated, immutable application models."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Mapping, Optional, Tuple


@dataclass(frozen=True)
class WheelDevices:
    name: str
    trigger: str
    servo: str
    reset: str
    direction: str
    angle: str
    speed: str


WheelMapping = Mapping[int, WheelDevices]
DeviceGroup = Optional[Tuple[str, str, str, str]]


@dataclass(frozen=True)
class SpeedControlConfig:
    enable_device: str
    stop_value: int
    minimum_value: int
    maximum_value: int


@dataclass(frozen=True)
class RegisterMappingConfig:
    wheels: WheelMapping
    speed_control: SpeedControlConfig


@dataclass(frozen=True)
class PLCConfig:
    ip: Optional[str]
    port: Optional[int]
    plc_type: str
    socket_timeout_s: float


@dataclass(frozen=True)
class TimingConfig:
    trigger_pulse_s: float


@dataclass(frozen=True)
class LoggingConfig:
    level: str
    file: Path
    max_bytes: int
    backup_count: int
    console: bool


@dataclass(frozen=True)
class FeedbackConfig:
    enabled: bool
    poll_interval_s: float
    servo_ready: DeviceGroup
    actual_angle: DeviceGroup
    actual_speed: DeviceGroup
    alarm_code: DeviceGroup


@dataclass(frozen=True)
class HeartbeatConfig:
    enabled: bool
    device: Optional[str]
    interval_s: float
    plc_watchdog_timeout_s: float


@dataclass(frozen=True)
class ApplicationConfig:
    plc: PLCConfig
    timing: TimingConfig
    logging: LoggingConfig
    feedback: FeedbackConfig
    heartbeat: HeartbeatConfig
    wheels: WheelMapping
    speed_control: SpeedControlConfig


@dataclass(frozen=True)
class TelemetrySnapshot:
    captured_at: datetime
    servo_ready: Optional[Tuple[bool, bool, bool, bool]]
    actual_angle: Optional[Tuple[int, int, int, int]]
    actual_speed: Optional[Tuple[int, int, int, int]]
    alarm_code: Optional[Tuple[int, int, int, int]]
