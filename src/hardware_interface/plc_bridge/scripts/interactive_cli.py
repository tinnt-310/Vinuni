#!/usr/bin/env python3
"""Interactive terminal controller using validated YAML configuration."""

from __future__ import annotations

import argparse
import cmd
import logging
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
for import_root in (PROJECT_ROOT, PROJECT_ROOT / "src"):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from scripts.common import DEFAULT_CONFIG, build_runtime, safe_shutdown  # noqa: E402
from fx5u_controller import (  # noqa: E402
    ConfigurationError,
    MitsubishiWheelController,
    RuntimeSupervisor,
)


class WheelControllerShell(cmd.Cmd):
    intro = (
        "\nFX5U 4-WHEEL CONTROLLER\n"
        "Type 'help' or '?' for commands. Ctrl+C performs shutdown.\n"
    )
    prompt = "FX5U> "

    def __init__(
        self,
        controller: MitsubishiWheelController,
        runtime: RuntimeSupervisor,
        logger: logging.Logger,
    ) -> None:
        super().__init__()
        self.controller = controller
        self.runtime = runtime
        self.logger = logger
        self.halted = False

    def do_status(self, arg: str) -> None:
        """status: show connection, software servo state, and latest telemetry."""

        del arg
        print(f"Connected: {self.controller.connected}")
        print(f"Communication fault: {self.controller.communication_fault}")
        print(
            f"Speed control {self.controller.speed_control.enable_device}/"
            f"{self.controller.speed_control.output_enable_device}: "
            f"{'ON-command acknowledged' if self.controller.speed_control_enabled else 'OFF'}"
        )
        print(f"Runtime services: {'RUNNING' if self.runtime.running else 'STOPPED'}")
        print(f"Runtime error: {self.runtime.last_error}")
        for wheel in (1, 2, 3, 4):
            state = (
                "ON-command acknowledged"
                if self.controller.servo_enabled[wheel]
                else "OFF/not commanded"
            )
            print(
                f"Steering servo {wheel} ({self.controller.wheels[wheel].name}): {state}"
            )
            direction = self.controller.direction_clockwise[wheel]
            direction_state = (
                "not commanded"
                if direction is None
                else (
                    "CW-command acknowledged"
                    if direction
                    else "CCW-command acknowledged"
                )
            )
            print(
                f"Drive direction {wheel} ({self.controller.wheels[wheel].direction}): "
                f"{direction_state}"
            )
        print(f"Latest feedback: {self.runtime.latest_feedback}")

    def do_reset(self, arg: str) -> None:
        """reset [all|1|2|3|4]: pulse configured fault-reset bit(s)."""

        if not self._command_allowed():
            return
        target = arg.strip() or "all"
        try:
            if target == "all":
                self.controller.reset_error_all()
            elif target in ("1", "2", "3", "4"):
                self.controller.reset_error(int(target))
            else:
                raise ValueError("target must be all, 1, 2, 3, or 4")
            print("OK")
        except Exception as exc:
            self._report_error("reset", exc)

    def do_servo(self, arg: str) -> None:
        """servo on|off [all|1|2|3|4]: command steering-servo state only."""

        parts = arg.strip().lower().split()
        if (
            not parts
            or len(parts) > 2
            or parts[0] not in ("on", "off")
        ):
            print("Usage: servo on|off [all|1|2|3|4]")
            return
        action = parts[0]
        target = parts[1] if len(parts) == 2 else "all"
        if self.halted and action == "on":
            print("Servo ON is locked after stop. Exit and reconnect.")
            return
        try:
            if target == "all":
                if action == "on":
                    self.controller.servo_on_all()
                else:
                    self.controller.servo_off_all()
            elif target in ("1", "2", "3", "4"):
                method = self.controller.servo_on if action == "on" else self.controller.servo_off
                method(int(target))
            else:
                raise ValueError("target must be all, 1, 2, 3, or 4")
            print("OK")
        except Exception as exc:
            self._report_error("servo", exc)

    def do_angle(self, arg: str) -> None:
        """angle <a1> <a2> <a3> <a4> OR angle <wheel> <degrees>."""

        if not self._command_allowed():
            return
        try:
            values = [int(value) for value in arg.split()]
            if len(values) == 4:
                self.controller.move_angles(values)
            elif len(values) == 2:
                self.controller.move_angle(values[0], values[1])
            else:
                raise ValueError("expected four angles or <wheel> <degrees>")
            print("OK")
        except Exception as exc:
            self._report_error("angle", exc)

    def do_direction(self, arg: str) -> None:
        """direction <all|1|2|3|4> <cw|ccw>: set wheel-drive direction."""

        if not self._command_allowed():
            return
        parts = arg.strip().lower().split()
        if len(parts) != 2:
            print("Usage: direction <all|1|2|3|4> <cw|ccw>")
            return
        target, direction_name = parts
        direction_values = {
            "cw": True,
            "clockwise": True,
            "on": True,
            "ccw": False,
            "counterclockwise": False,
            "off": False,
        }
        if direction_name not in direction_values:
            print("Usage: direction <all|1|2|3|4> <cw|ccw>")
            return
        clockwise = direction_values[direction_name]
        try:
            if target == "all":
                self.controller.set_directions([clockwise] * 4)
            elif target in ("1", "2", "3", "4"):
                self.controller.set_direction(int(target), clockwise)
            else:
                raise ValueError("target must be all, 1, 2, 3, or 4")
            print("OK")
        except Exception as exc:
            self._report_error("direction", exc)

    def do_speed(self, arg: str) -> None:
        """speed <v1> <v2> <v3> <v4> OR speed <wheel> <value>; direct PLC value."""

        if not self._command_allowed():
            return
        try:
            values = [int(value) for value in arg.split()]
            if len(values) == 4:
                self.controller.move_speeds(values)
            elif len(values) == 2:
                self.controller.move_speed(values[0], values[1])
            else:
                raise ValueError("expected four direct values or <wheel> <value>")
            print("OK")
        except Exception as exc:
            self._report_error("speed", exc)

    def do_stop(self, arg: str) -> None:
        """stop: stop heartbeat and issue the software emergency stop."""

        del arg
        self.halted = True
        try:
            self.runtime.stop()
        except Exception as exc:
            self._report_error("runtime stop", exc)
        try:
            self.controller.emergency_stop()
            print("Software emergency stop sent. Exit and reconnect before more commands.")
        except Exception as exc:
            self._report_error("emergency stop", exc)

    def do_exit(self, arg: str) -> bool:
        """exit: leave the shell; safe shutdown runs immediately afterward."""

        del arg
        return True

    def do_quit(self, arg: str) -> bool:
        """quit: same as exit."""

        return self.do_exit(arg)

    def do_EOF(self, arg: str) -> bool:
        """Ctrl+D: same as exit."""

        print()
        return self.do_exit(arg)

    def emptyline(self) -> bool:
        return False

    def _command_allowed(self) -> bool:
        if self.halted:
            print("Commands are locked after stop. Exit and reconnect.")
            return False
        return True

    def _report_error(self, operation: str, exc: Exception) -> None:
        print(f"ERROR: {exc}", file=sys.stderr)
        self.logger.error("interactive command failed command=%s error=%s", operation, exc)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive FX5U four-wheel controller")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--ip", help="override plc.ip from YAML")
    parser.add_argument("--port", type=int, help="override plc.port from YAML")
    parser.add_argument("--pulse", type=float, help="override trigger pulse duration")
    parser.add_argument("--execute", action="store_true", help="required to connect to hardware")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.execute:
        print("Refusing to connect to hardware without --execute", file=sys.stderr)
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
        controller.stop_drive()
        controller.reset_drive_on_start()
        controller.stop_drive()
        runtime.start()
        WheelControllerShell(controller, runtime, logger).cmdloop()
    except KeyboardInterrupt:
        print("\nCtrl+C: initiating safe shutdown.")
        logger.warning("interactive session interrupted by operator")
    except Exception as exc:
        exit_code = 1
        logger.exception("interactive session failed error=%s", exc)
    finally:
        if not safe_shutdown(controller, runtime, logger):
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
