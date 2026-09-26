"""Shared configuration and shutdown helpers for executable scripts."""

from __future__ import annotations

import logging
import sys
from dataclasses import replace
from pathlib import Path
from typing import Optional, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
for import_root in (PROJECT_ROOT, SRC_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from fx5u_controller import (  # noqa: E402
    ApplicationConfig,
    MitsubishiWheelController,
    RuntimeSupervisor,
    configure_logging,
    load_application_config,
    require_plc_endpoint,
)


DEFAULT_CONFIG = PROJECT_ROOT / "config" / "default.yaml"


def build_runtime(
    config_path: Path,
    *,
    ip_override: Optional[str],
    port_override: Optional[int],
    pulse_override: Optional[float],
) -> Tuple[ApplicationConfig, MitsubishiWheelController, RuntimeSupervisor, logging.Logger]:
    config = load_application_config(config_path)
    if ip_override is not None or port_override is not None:
        config = replace(
            config,
            plc=replace(
                config.plc,
                ip=ip_override if ip_override is not None else config.plc.ip,
                port=port_override if port_override is not None else config.plc.port,
            ),
        )
    if pulse_override is not None:
        config = replace(config, timing=replace(config.timing, trigger_pulse_s=pulse_override))

    logger = configure_logging(config.logging)
    ip, port = require_plc_endpoint(config.plc)
    controller = MitsubishiWheelController(
        ip=ip,
        port=port,
        trigger_pulse_s=config.timing.trigger_pulse_s,
        wheels=config.wheels,
        speed_control=config.speed_control,
        plc_type=config.plc.plc_type,
        socket_timeout_s=config.plc.socket_timeout_s,
    )
    runtime = RuntimeSupervisor(controller, config.feedback, config.heartbeat)
    return config, controller, runtime, logger


def safe_shutdown(
    controller: MitsubishiWheelController,
    runtime: RuntimeSupervisor,
    logger: logging.Logger,
) -> bool:
    """Attempt every shutdown stage independently and report every failure."""

    ok = True
    for name, operation in (
        ("runtime stop", runtime.stop),
        ("software emergency stop", controller.emergency_stop),
        ("controlled disconnect", controller.disconnect),
    ):
        try:
            operation()
        except Exception as exc:
            ok = False
            logger.critical("shutdown stage failed stage=%s error=%s", name, exc)
    return ok
