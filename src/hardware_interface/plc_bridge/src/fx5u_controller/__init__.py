"""Production-oriented Mitsubishi FX5U four-wheel controller."""

import logging


logging.getLogger("fx5u_controller").addHandler(logging.NullHandler())

from .config import load_application_config, load_register_mapping, require_plc_endpoint
from .controller import MitsubishiWheelController
from .driver import PymcProtocolSLMPDriver, SLMPDriver
from .exceptions import (
    ConfigurationError,
    ControllerStateError,
    PulseCleanupError,
    SLMPCommunicationError,
    WheelControllerError,
)
from .logging_setup import configure_logging, shutdown_logging
from .models import (
    ApplicationConfig,
    FeedbackConfig,
    HeartbeatConfig,
    SpeedControlConfig,
    TelemetrySnapshot,
    WheelDevices,
    WheelMapping,
)
from .runtime import RuntimeSupervisor

__all__ = [
    "ApplicationConfig",
    "ConfigurationError",
    "ControllerStateError",
    "FeedbackConfig",
    "HeartbeatConfig",
    "MitsubishiWheelController",
    "PulseCleanupError",
    "PymcProtocolSLMPDriver",
    "RuntimeSupervisor",
    "SLMPCommunicationError",
    "SLMPDriver",
    "SpeedControlConfig",
    "TelemetrySnapshot",
    "WheelControllerError",
    "WheelDevices",
    "WheelMapping",
    "configure_logging",
    "load_application_config",
    "load_register_mapping",
    "require_plc_endpoint",
    "shutdown_logging",
]
