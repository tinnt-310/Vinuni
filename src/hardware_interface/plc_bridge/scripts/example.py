"""Explicit, hardware-gated commissioning example."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.common import DEFAULT_CONFIG, build_runtime, safe_shutdown  # noqa: E402
from fx5u_controller import ConfigurationError  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send the documented four-wheel test commands to an FX5U PLC"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--ip", help="override plc.ip from YAML")
    parser.add_argument("--port", type=int, help="override plc.port from YAML")
    parser.add_argument("--pulse", type=float, help="override trigger pulse duration")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="required acknowledgement that this will command real hardware",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.execute:
        print("Refusing to command hardware without --execute", file=sys.stderr)
        return 2
    try:
        _, controller, runtime, logger = build_runtime(
            args.config,
            ip_override=args.ip,
            port_override=args.port,
            pulse_override=args.pulse,
        )
    except (ConfigurationError, ValueError) as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2

    exit_code = 0
    try:
        controller.connect()
        runtime.start()
        controller.reset_error_all()
        controller.servo_on_all()
        # Angle-only commissioning example. It deliberately leaves direct
        # speed control and M58 disabled.
        controller.move_angles([10, 10, -10, -10])
    except Exception as exc:
        exit_code = 1
        logger.exception("commissioning example failed error=%s", exc)
    finally:
        if not safe_shutdown(controller, runtime, logger):
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
