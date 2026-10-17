import importlib.util
import sys
import types
import unittest
from pathlib import Path


class _DummyNode:
    pass


fake_rclpy = types.ModuleType("rclpy")
fake_rclpy_node = types.ModuleType("rclpy.node")
fake_rclpy_node.Node = _DummyNode
fake_geometry = types.ModuleType("geometry_msgs")
fake_geometry_msg = types.ModuleType("geometry_msgs.msg")
fake_geometry_msg.Twist = object
sys.modules.setdefault("rclpy", fake_rclpy)
sys.modules.setdefault("rclpy.node", fake_rclpy_node)
sys.modules.setdefault("geometry_msgs", fake_geometry)
sys.modules.setdefault("geometry_msgs.msg", fake_geometry_msg)
fake_std = types.ModuleType("std_msgs")
fake_std_msg = types.ModuleType("std_msgs.msg")
fake_std_msg.String = object
sys.modules.setdefault("std_msgs", fake_std)
sys.modules.setdefault("std_msgs.msg", fake_std_msg)

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "manual_keyboard.py"
SPEC = importlib.util.spec_from_file_location("manual_keyboard", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class ManualKeyboardTests(unittest.TestCase):
    def test_reset_key_stops_and_does_not_repeat_while_held(self):
        state = MODULE.TerminalKeyboardControl(0.1, 0.15, 0.2, 0.3)
        state.feed(b"i", 1.0)
        self.assertEqual(state.feed(b"r", 1.1), "reset")
        self.assertTrue(state.target(1.1).stopped)
        self.assertIsNone(state.feed(b"r", 1.2))
        self.assertEqual(state.feed(b"r", 2.1), "reset")
    def test_terminal_timeout_speed_keys_and_escape_sequences(self):
        state = MODULE.TerminalKeyboardControl(0.1, 0.15, 0.2, 0.3)
        state.feed(b"o", 1.0)
        self.assertLess(state.target(1.1).yaw, 0.0)
        state.feed(b"w", 1.2)
        self.assertAlmostEqual(state.target(1.3).x, 0.11)
        self.assertLess(state.target(1.3).yaw, 0.0)
        self.assertTrue(state.target(2.0).stopped)
        state.feed(b"e", 2.1)
        self.assertTrue(state.target(2.2).stopped)
        state.feed(b"\x1b[", 2.3)
        state.feed(b"O", 2.4)
        self.assertTrue(state.target(2.4).stopped)
        state.feed(b"i", 3.0)
        self.assertEqual(state.feed(b" ", 3.1), "stop")
        self.assertTrue(state.target(3.1).stopped)
    def test_kitty_parser_reports_press_release_shift_and_focus(self):
        parser = MODULE.KittyKeyParser()
        self.assertEqual(parser.feed(b"\x1b[?"), [])
        self.assertEqual(parser.feed(b"11u"), [("flags", 11)])
        events = parser.feed(
            b"\x1b[105u\x1b[111;1:2u\x1b[111;1:3u"
            b"\x1b[57441;2u\x1b[O"
        )
        self.assertEqual(events, [
            MODULE.KeyEvent(105, 0, "press"),
            MODULE.KeyEvent(111, 0, "repeat"),
            MODULE.KeyEvent(111, 0, "release"),
            MODULE.KeyEvent(57441, 1, "press"),
            "focus_out",
        ])

    def test_true_held_keys_and_speed_adjustment(self):
        state = MODULE.HeldKeyboardControl(0.1, 0.15, 0.2, 0.3)
        event = MODULE.KeyEvent
        state.handle(event(ord("i"), 0, "press"), 1.0)
        self.assertEqual(state.target(1.0), MODULE.Command(x=0.1))
        state.handle(event(ord("o"), 0, "press"), 1.1)
        self.assertEqual(state.target(1.1), MODULE.Command(x=0.1, yaw=-0.15))
        state.handle(event(ord("e"), 0, "press"), 1.2)
        state.tick(0.1)
        self.assertAlmostEqual(state.target(1.3).yaw, -0.165)
        state.handle(event(ord("e"), 0, "release"), 1.4)
        state.handle(event(ord("o"), 0, "release"), 1.4)
        self.assertAlmostEqual(state.target(1.4).x, 0.1)
        self.assertEqual(state.target(1.4).yaw, 0.0)
        state.handle(event(ord("i"), 0, "release"), 1.5)
        self.assertFalse(state.target(1.55).stopped)
        self.assertTrue(state.target(1.6).stopped)

        state.handle(event(57441, 1, "press"), 2.0)
        state.handle(event(ord("j"), 1, "press"), 2.0)
        self.assertEqual(state.target(2.0), MODULE.Command(y=state.linear_speed))
        state.handle(event(ord("k"), 0, "press"), 2.1)
        self.assertTrue(state.target(2.1).stopped)

    def test_motion_bindings(self):
        self.assertEqual(MODULE.command_for_key("i", 0.1, 0.2), MODULE.Command(x=0.1))
        self.assertEqual(MODULE.command_for_key("j", 0.1, 0.2), MODULE.Command(yaw=0.2))
        self.assertEqual(
            MODULE.command_for_key("m", 0.1, 0.2),
            MODULE.Command(x=-0.1, yaw=-0.2),
        )
        self.assertEqual(
            MODULE.command_for_key("U", 0.1, 0.2), MODULE.Command(x=0.1, y=0.1)
        )

    def test_stop_and_unknown_bindings(self):
        self.assertTrue(MODULE.command_for_key(" ", 0.1, 0.2).stopped)
        self.assertTrue(MODULE.command_for_key("k", 0.1, 0.2).stopped)
        self.assertTrue(MODULE.command_for_key("K", 0.1, 0.2).stopped)
        self.assertIsNone(MODULE.command_for_key("?", 0.1, 0.2))


    def test_speed_keys_update_current_motion_immediately(self):
        control = MODULE.KeyboardControl(0.1, 0.15, 0.2, 0.3)
        self.assertTrue(control.handle_key("u"))
        self.assertAlmostEqual(control.command.x, 0.1)
        self.assertAlmostEqual(control.command.yaw, 0.15)
        self.assertTrue(control.handle_key("w"))
        self.assertAlmostEqual(control.command.x, 0.11)
        self.assertAlmostEqual(control.command.yaw, 0.15)
        self.assertTrue(control.handle_key("e"))
        self.assertAlmostEqual(control.command.x, 0.11)
        self.assertAlmostEqual(control.command.yaw, 0.165)
        self.assertTrue(control.handle_key("q"))
        self.assertAlmostEqual(control.command.x, 0.121)
        self.assertAlmostEqual(control.command.yaw, 0.1815)

    def test_speed_keys_respect_limits_and_stop_state(self):
        control = MODULE.KeyboardControl(0.19, 0.29, 0.2, 0.3)
        control.handle_key("i")
        control.handle_key("q")
        self.assertAlmostEqual(control.command.x, 0.2)
        self.assertAlmostEqual(control.angular_speed, 0.3)
        control.handle_key(" ")
        control.handle_key("x")
        self.assertTrue(control.command.stopped)
        control.handle_key("i")
        self.assertAlmostEqual(control.command.x, 0.2 / 1.1)


if __name__ == "__main__":
    unittest.main()
