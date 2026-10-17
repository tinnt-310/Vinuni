#!/usr/bin/env python3
"""Periodic keyboard teleop for the open-loop FX5U bridge.

Unlike teleop_twist_keyboard 2.4.1, this node republishes the selected command
so that the bridge watchdog continues to receive a live command stream.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import select
import shutil
import sys
import termios
import time
import tty
from dataclasses import dataclass

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from rclpy.node import Node
except ImportError as exc:  # pragma: no cover - depends on sourced ROS environment
    raise SystemExit("ROS 2 Python modules are unavailable; source /opt/ros/humble/setup.bash") from exc


HELP = """
FX5U swerve keyboard (lenh duoc lap lai de dap ung watchdog)

Di chuyen tieu chuan:
   U    I    O
   J    K    L
   M    ,    .

Holonomic/truot ngang (giu Shift):
   U    I    O
   J    K    L
   M    <    >

Giu phim q/z: tang/giam toc do tinh tien va toc do quay lien tuc.
Giu phim w/x: tang/giam toc do tinh tien.
Giu phim e/c: tang/giam toc do quay.
Co the giu phim toc do cung luc voi phim di chuyen.

Nhan Shift cung voi phim di chuyen de truot ngang.
Nha phim di chuyen: dung sau 0.08 s; Space/K: DUNG NGAY.
Ctrl+C/Esc: DUNG va thoat.
Can terminal ho tro Kitty keyboard protocol (key press/release).
Chi chay khi ban da nang 4 banh va co nut dung khan cap trong tam tay.
"""


@dataclass(frozen=True)
class Command:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    @property
    def stopped(self) -> bool:
        return self.x == 0.0 and self.y == 0.0 and self.yaw == 0.0


def command_for_key(key: str, linear_speed: float, angular_speed: float) -> Command | None:
    bindings = {
        "i": Command(x=linear_speed),
        "o": Command(x=linear_speed, yaw=-angular_speed),
        "j": Command(yaw=angular_speed),
        "l": Command(yaw=-angular_speed),
        "u": Command(x=linear_speed, yaw=angular_speed),
        ",": Command(x=-linear_speed),
        ".": Command(x=-linear_speed, yaw=angular_speed),
        "m": Command(x=-linear_speed, yaw=-angular_speed),
        "I": Command(x=linear_speed),
        "O": Command(x=linear_speed, y=-linear_speed),
        "J": Command(y=linear_speed),
        "L": Command(y=-linear_speed),
        "U": Command(x=linear_speed, y=linear_speed),
        "<": Command(x=-linear_speed),
        ">": Command(x=-linear_speed, y=-linear_speed),
        "M": Command(x=-linear_speed, y=linear_speed),
        " ": Command(),
        "k": Command(),
        "K": Command(),
    }
    return bindings.get(key)


class KeyboardControl:
    """Keep the selected motion key while speed keys update a live Twist."""

    def __init__(
        self, linear_speed: float, angular_speed: float,
        max_linear_speed: float, max_angular_speed: float,
    ) -> None:
        values = (linear_speed, angular_speed, max_linear_speed, max_angular_speed)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError("keyboard speeds and limits must be finite and positive")
        if linear_speed > max_linear_speed or angular_speed > max_angular_speed:
            raise ValueError("initial keyboard speed exceeds a configured limit")
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.active_key: str | None = None
        self.command = Command()

    def handle_key(self, key: str) -> bool:
        speed_keys = {
            "q": (1.1, 1.1), "z": (1 / 1.1, 1 / 1.1),
            "w": (1.1, 1.0), "x": (1 / 1.1, 1.0),
            "e": (1.0, 1.1), "c": (1.0, 1 / 1.1),
        }
        if key in speed_keys:
            linear_factor, angular_factor = speed_keys[key]
            self.linear_speed = min(self.max_linear_speed, self.linear_speed * linear_factor)
            self.angular_speed = min(self.max_angular_speed, self.angular_speed * angular_factor)
            if self.active_key is not None:
                updated = command_for_key(
                    self.active_key, self.linear_speed, self.angular_speed
                )
                assert updated is not None
                self.command = updated
            return True
        command = command_for_key(key, self.linear_speed, self.angular_speed)
        if command is None:
            return False
        self.command = command
        self.active_key = None if command.stopped else key
        return True


@dataclass(frozen=True)
class KeyEvent:
    code: int
    modifiers: int
    action: str


class KittyKeyParser:
    """Parse CSI-u key press/repeat/release and terminal focus events."""

    _key_pattern = re.compile(
        rb"\x1b\[(\d+)(?::[\d:]*)?(?:;(\d+)(?::([123]))?)?u"
    )
    _flags_pattern = re.compile(rb"\x1b\[\?(\d+)u")

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[KeyEvent | tuple[str, int] | str]:
        self.buffer.extend(data)
        events: list[KeyEvent | tuple[str, int] | str] = []
        while self.buffer:
            if self.buffer[0] != 27:
                byte = self.buffer.pop(0)
                if byte == 3:
                    events.append(KeyEvent(ord("c"), 4, "press"))
                continue
            if len(self.buffer) < 2:
                break
            if self.buffer[1] != ord("["):
                del self.buffer[0]
                continue
            end = next(
                (index for index in range(2, len(self.buffer))
                 if 64 <= self.buffer[index] <= 126),
                None,
            )
            if end is None:
                if len(self.buffer) > 128:
                    self.buffer.clear()
                break
            sequence = bytes(self.buffer[:end + 1])
            del self.buffer[:end + 1]
            flags = self._flags_pattern.fullmatch(sequence)
            if flags:
                events.append(("flags", int(flags.group(1))))
                continue
            if sequence == b"\x1b[O":
                events.append("focus_out")
                continue
            if sequence == b"\x1b[I":
                events.append("focus_in")
                continue
            key = self._key_pattern.fullmatch(sequence)
            if key:
                action = {b"1": "press", b"2": "repeat", b"3": "release"}[
                    key.group(3) or b"1"
                ]
                events.append(KeyEvent(
                    int(key.group(1)), int(key.group(2) or b"1") - 1, action
                ))
        return events


class HeldKeyboardControl:
    """Translate true held/released key state into a periodic Twist target."""

    MOTION_CODES = {ord(key) for key in "uiojlm,."}
    SPEED_CODES = {ord(key) for key in "qzwxec"}
    SHIFT_CODES = {57441, 57447}

    def __init__(
        self, linear_speed: float, angular_speed: float,
        max_linear_speed: float, max_angular_speed: float,
        release_grace_s: float = 0.08,
    ) -> None:
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.release_grace_s = release_grace_s
        self.held: dict[int, None] = {}
        self.grace_key: int | None = None
        self.grace_shift = False
        self.grace_until_s = 0.0

    def clear(self) -> None:
        self.held.clear()
        self.grace_key = None
        self.grace_until_s = 0.0

    def handle(self, event: KeyEvent, now_s: float) -> str | None:
        code = event.code
        if code == ord("r") and event.action == "press":
            self.clear()
            return "reset"
        if code == ord("c") and event.modifiers & 4 and event.action == "press":
            self.clear()
            return "exit"
        if code in (32, ord("k")) and event.action == "press":
            self.clear()
            return "stop"
        if code == 27 and event.action == "press":
            self.clear()
            return "exit"
        if code not in self.MOTION_CODES | self.SPEED_CODES | self.SHIFT_CODES:
            return None
        if event.action == "press":
            if code not in self.held:
                self.held[code] = None
            self.grace_key = None
        elif event.action == "release":
            if code in self.held:
                if code in self.MOTION_CODES:
                    self.grace_key = code
                    self.grace_shift = self.shifted
                    self.grace_until_s = now_s + self.release_grace_s
                del self.held[code]
        return None

    @property
    def shifted(self) -> bool:
        return any(code in self.held for code in self.SHIFT_CODES)

    def tick(self, dt_s: float) -> None:
        dt_s = max(0.0, min(dt_s, 0.1))
        linear_direction = (
            int(ord("q") in self.held) + int(ord("w") in self.held)
            - int(ord("z") in self.held) - int(ord("x") in self.held)
        )
        angular_direction = (
            int(ord("q") in self.held) + int(ord("e") in self.held)
            - int(ord("z") in self.held) - int(ord("c") in self.held)
        )
        self.linear_speed = max(
            0.0, min(self.max_linear_speed,
                     self.linear_speed + linear_direction * 0.10 * dt_s)
        )
        self.angular_speed = max(
            0.0, min(self.max_angular_speed,
                     self.angular_speed + angular_direction * 0.15 * dt_s)
        )

    def target(self, now_s: float) -> Command:
        motion = next(
            (code for code in reversed(self.held) if code in self.MOTION_CODES),
            None,
        )
        shifted = self.shifted
        if motion is None and self.grace_key is not None and now_s < self.grace_until_s:
            motion = self.grace_key
            shifted = self.grace_shift
        if motion is None:
            return Command()
        key = chr(motion)
        if shifted:
            key = {"u": "U", "i": "I", "o": "O", "j": "J",
                   "l": "L", "m": "M", ",": "<", ".": ">"}.get(key, key)
        command = command_for_key(key, self.linear_speed, self.angular_speed)
        assert command is not None
        return command


class TerminalKeyboardControl(KeyboardControl):
    """Legacy TTY input: repeat refreshes a bounded command lease."""

    def __init__(self, *args, timeout_s: float = 0.75):
        super().__init__(*args)
        self.timeout_s = timeout_s
        self.deadline_s = 0.0
        self.escape = ""
        self.last_reset_input_s = float("-inf")

    def clear(self):
        self.command = Command()
        self.active_key = None
        self.deadline_s = 0.0

    def tick(self, dt_s):
        pass

    def target(self, now_s):
        if now_s >= self.deadline_s:
            self.clear()
        return self.command

    def feed(self, data: bytes, now_s: float):
        self.target(now_s)
        for byte in data:
            key = chr(byte)
            if byte == 3:
                self.clear()
                return "exit"
            if self.escape:
                self.escape += key
                if len(self.escape) == 2 and key in "[O":
                    continue
                if len(self.escape) >= 3 and 64 <= byte <= 126:
                    self.escape = ""
                elif len(self.escape) == 2 or len(self.escape) > 128:
                    self.escape = ""
                continue
            if byte == 27:
                self.escape = key
                continue
            if key in " kK":
                self.clear()
                return "stop"
            if key in "rR":
                fresh_press = now_s - self.last_reset_input_s > self.timeout_s
                self.last_reset_input_s = now_s
                self.clear()
                if fresh_press:
                    return "reset"
                continue
            if self.handle_key(key) and not self.command.stopped:
                self.deadline_s = now_s + self.timeout_s
        return None


class PeriodicKeyboard(Node):
    def __init__(self, topic: str, publish_rate_hz: float) -> None:
        super().__init__("fx5u_manual_keyboard")
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.reset_publisher = self.create_publisher(String, "/plc_bridge/reset_drive", 1)
        self.command = Command()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish)

    def set_command(self, command: Command) -> None:
        self.command = command
        self.publish()

    def reset_drive(self) -> None:
        self.set_command(Command())
        message = String()
        message.data = "reset"
        self.reset_publisher.publish(message)
        print("\r\nDa gui yeu cau reset M222; xem ket qua tai terminal bridge.\r\n")

    def publish(self) -> None:
        message = Twist()
        message.linear.x = self.command.x
        message.linear.y = self.command.y
        message.angular.z = self.command.yaw
        self.publisher.publish(message)


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--linear-speed", type=float, default=0.10)
    parser.add_argument("--angular-speed", type=float, default=0.15)
    parser.add_argument("--max-linear-speed", type=float, default=0.20)
    parser.add_argument("--max-angular-speed", type=float, default=0.30)
    parser.add_argument("--publish-rate", type=float, default=50.0)
    parser.add_argument("--input-mode", choices=("auto", "terminal", "kitty"), default="auto")
    parser.add_argument("--key-timeout", type=float, default=0.75)
    args, ros_args = parser.parse_known_args()
    for name in ("linear_speed", "angular_speed", "max_linear_speed",
                 "max_angular_speed", "publish_rate", "key_timeout"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if args.linear_speed > args.max_linear_speed:
        parser.error("--linear-speed must not exceed --max-linear-speed")
    if args.angular_speed > args.max_angular_speed:
        parser.error("--angular-speed must not exceed --max-angular-speed")
    return args, ros_args


class KeyboardProtocolError(RuntimeError):
    pass


def _query_keyboard_flags(
    parser: KittyKeyParser, input_fd: int, output_fd: int, timeout_s: float = 1.0
) -> int | None:
    os.write(output_fd, b"\x1b[?u")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        readable, _, _ = select.select(
            [input_fd], [], [], max(0.0, deadline - time.monotonic())
        )
        if not readable:
            break
        data = os.read(input_fd, 4096)
        if not data:
            break
        for event in parser.feed(data):
            if isinstance(event, tuple) and event[0] == "flags":
                return event[1]
    return None


def enable_key_release_mode(
    parser: KittyKeyParser, input_fd: int, output_fd: int
) -> None:
    if _query_keyboard_flags(parser, input_fd, output_fd) is None:
        raise KeyboardProtocolError(
            "Terminal SSH khong ho tro Kitty keyboard protocol / key release. "
            "Can terminal ho tro CSI-u; khong the dieu khien kieu giu/tha phim an toan."
        )
    os.write(output_fd, b"\x1b[>11u")
    try:
        flags = _query_keyboard_flags(parser, input_fd, output_fd)
        if flags is None or flags & 10 != 10:
            raise KeyboardProtocolError(
                "Terminal khong bao cao du key release va report-all-keys (flags 10)."
            )
    except BaseException:
        os.write(output_fd, b"\x1b[<u")
        raise
    os.write(output_fd, b"\x1b[?1004h")


def main() -> int:
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print("ERROR: run_keyboard.sh requires an interactive terminal", file=sys.stderr)
        return 2
    args, ros_args = parse_args()
    input_fd = sys.stdin.fileno()
    output_fd = sys.stdout.fileno()
    terminal_settings = termios.tcgetattr(input_fd)
    parser = KittyKeyParser()
    protocol_enabled = False
    node = None
    exit_code = 0
    try:
        tty.setraw(input_fd)
        if args.input_mode != "terminal":
            try:
                enable_key_release_mode(parser, input_fd, output_fd)
                protocol_enabled = True
            except KeyboardProtocolError:
                if args.input_mode == "kitty":
                    raise
        rclpy.init(args=ros_args)
        node = PeriodicKeyboard(args.topic, args.publish_rate)
        if protocol_enabled:
            control = HeldKeyboardControl(
                args.linear_speed, args.angular_speed,
                args.max_linear_speed, args.max_angular_speed,
            )
            print(HELP)
        else:
            control = TerminalKeyboardControl(
                args.linear_speed, args.angular_speed,
                args.max_linear_speed, args.max_angular_speed,
                timeout_s=args.key_timeout,
            )
            print("\r\nTERMINAL: u i o / j k l / m , .; Shift: truot ngang.\r\n"
                  "q/z: ca hai toc do; w/x: tinh tien; e/c: quay (moi lan x1.1).\r\n"
                  f"Khong co key-up: dung sau {args.key_timeout:.2f}s khong nhan phim.\r\n"
                  "Giu phim de lap; Space/K dung ngay; Ctrl+C thoat.\r\n")
        print(f"topic={args.topic} rate={args.publish_rate:.1f} Hz")
        print("\r\nr: dung truyen dong va nhap M222 reset (bam mot lan).\r\n")
        period_s = 1.0 / args.publish_rate
        last_tick_s = time.monotonic()
        next_tick_s = last_tick_s
        last_display_s = 0.0
        last_display_state = None
        running = True
        while rclpy.ok() and running:
            now_s = time.monotonic()
            if now_s >= next_tick_s:
                control.tick(now_s - last_tick_s)
                command = control.target(now_s)
                node.set_command(command)
                last_tick_s = now_s
                next_tick_s = now_s + period_s
                if now_s - last_display_s >= 0.2:
                    state = "STOP" if command.stopped else (
                        f"vx={command.x:+.3f} vy={command.y:+.3f} wz={command.yaw:+.3f}"
                    )
                    state += (
                        f" | linear={control.linear_speed:.3f} m/s"
                        f" yaw={control.angular_speed:.3f} rad/s"
                    )
                    if state != last_display_state:
                        width = max(1, shutil.get_terminal_size((80, 24)).columns - 1)
                        print("\r\x1b[2K" + state[:width], end="", flush=True)
                        last_display_state = state
                    last_display_s = now_s
            timeout_s = max(0.0, min(next_tick_s - time.monotonic(), 0.02))
            readable, _, _ = select.select([input_fd], [], [], timeout_s)
            if not readable:
                continue
            data = os.read(input_fd, 4096)
            if not data:
                break
            if not protocol_enabled:
                result = control.feed(data, time.monotonic())
                if result == "reset":
                    node.reset_drive()
                if result in ("stop", "exit"):
                    node.set_command(Command())
                if result == "exit":
                    running = False
                continue
            for event in parser.feed(data):
                if event == "focus_out":
                    control.clear()
                    node.set_command(Command())
                elif isinstance(event, KeyEvent):
                    result = control.handle(event, time.monotonic())
                    if result == "reset":
                        node.reset_drive()
                    if result in ("stop", "exit"):
                        node.set_command(Command())
                    if result == "exit":
                        running = False
                        break
    except KeyboardProtocolError as exc:
        print(f"\r\nERROR: {exc}", file=sys.stderr)
        exit_code = 2
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                if rclpy.ok():
                    node.set_command(Command())
                    for _ in range(3):
                        node.publish()
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        finally:
            if protocol_enabled:
                os.write(output_fd, b"\x1b[?1004l\x1b[<u")
            termios.tcsetattr(input_fd, termios.TCSADRAIN, terminal_settings)
            print("\nSTOP")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
