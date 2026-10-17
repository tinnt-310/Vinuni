from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from fx5u_controller.open_loop import OpenLoopSwerveKinematics, VelocityCommand


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def load_bridge_module():
    modules = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.executors", "rclpy.node",
        "geometry_msgs", "geometry_msgs.msg",
        "sensor_msgs", "sensor_msgs.msg",
        "std_msgs", "std_msgs.msg",
    )}
    modules["rclpy.executors"].ExternalShutdownException = type(
        "ExternalShutdownException", (Exception,), {}
    )
    modules["rclpy.node"].Node = type("Node", (), {})
    modules["geometry_msgs.msg"].Twist = object
    modules["sensor_msgs.msg"].Joy = object
    modules["std_msgs.msg"].String = object
    name = "fx5u_test_open_loop_bridge"
    spec = importlib.util.spec_from_file_location(
        name, PROJECT_ROOT / "scripts" / "open_loop_bridge.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


BRIDGE_MODULE = load_bridge_module()


class FakeController:
    def __init__(self) -> None:
        self.calls = []

    def stop_drive(self) -> None:
        self.calls.append(("stop_drive", None))

    def move_angles(self, angles) -> None:
        self.calls.append(("move_angles", tuple(angles)))

    def set_directions(self, directions) -> None:
        self.calls.append(("set_directions", tuple(directions)))

    def move_speeds(self, raw) -> None:
        self.calls.append(("move_speeds", tuple(raw)))


class BridgeMotionTests(unittest.TestCase):
    def test_manual_reset_stops_pulses_and_discards_cached_motion(self):
        bridge = BRIDGE_MODULE.Fx5uOpenLoopBridge.__new__(
            BRIDGE_MODULE.Fx5uOpenLoopBridge
        )
        bridge.controller = unittest.mock.Mock()
        bridge.settings = types.SimpleNamespace(command_timeout_s=0.5)
        bridge.stop_raw = 660
        bridge.faulted = False
        bridge.selector = object()
        bridge.get_logger = unittest.mock.Mock()
        bridge.publish_status = unittest.mock.Mock()
        bridge.enter_fault = unittest.mock.Mock()

        bridge.on_reset_drive(types.SimpleNamespace(data="reset"))

        self.assertEqual(bridge.controller.mock_calls, [
            unittest.mock.call.stop_drive(),
            unittest.mock.call.reset_drive_on_start(),
            unittest.mock.call.stop_drive(),
        ])
        self.assertTrue(bridge.drive_stopped)
        self.assertEqual(bridge.last_raw, (660,) * 4)
        self.assertEqual(bridge.smoothed_velocity, VelocityCommand(0.0, 0.0, 0.0))
        self.assertEqual(bridge.selector.select(10.0),
                         (None, VelocityCommand(0.0, 0.0, 0.0)))
        bridge.enter_fault.assert_not_called()
        bridge.publish_status.assert_called_once_with("drive_reset", source="keyboard")
        bridge.controller.reset_mock()
        bridge.faulted = True
        bridge.on_reset_drive(types.SimpleNamespace(data="reset"))
        bridge.controller.assert_not_called()
        self.assertEqual(bridge.controller.mock_calls, [])
    def test_o_then_more_omega_updates_angle_and_speed_without_stopping(self) -> None:
        settings = BRIDGE_MODULE.load_bridge_settings(
            PROJECT_ROOT / "config" / "open_loop_bridge.yaml"
        )
        self.assertTrue(settings.allow_steering_while_driving)
        controller = FakeController()
        bridge = BRIDGE_MODULE.Fx5uOpenLoopBridge.__new__(
            BRIDGE_MODULE.Fx5uOpenLoopBridge
        )
        bridge.controller = controller
        bridge.settings = settings
        bridge.stop_raw = 660
        bridge.kinematics = OpenLoopSwerveKinematics(
            settings.wheelbase_m, settings.track_width_m,
            settings.wheel_radius_m, settings.max_wheel_rate_rad_s,
        )
        bridge.previous_steering = (0.0,) * 4
        bridge.last_angles = None
        bridge.last_angle_float = None
        bridge.last_steering_s = None
        bridge.last_raw = (660,) * 4
        bridge.last_directions = None
        bridge.drive_stopped = True
        bridge.settle_until_s = 0.0
        bridge.last_source = None
        bridge.publish_status = lambda *args, **kwargs: None

        original = VelocityCommand(0.10, 0.0, -0.15)
        bridge.send_motion("keyboard", original, 1.0)
        self.assertEqual([name for name, _ in controller.calls], [
            "move_angles", "set_directions",
        ])
        bridge.send_motion("keyboard", original, 1.25)
        self.assertEqual(controller.calls[-1][0], "move_speeds")
        first_speed = controller.calls[-1][1]
        before = len(controller.calls)
        bridge.send_motion("keyboard", original, 1.35)
        self.assertEqual(
            [name for name, _ in controller.calls[before:]], ["move_speeds"]
        )
        self.assertNotEqual(controller.calls[-1][1], first_speed)

        before = len(controller.calls)
        bridge.send_motion(
            "keyboard", VelocityCommand(0.10, 0.0, -0.165), 1.45
        )
        self.assertEqual([name for name, _ in controller.calls[before:]], [
            "move_angles", "move_speeds",
        ])
        self.assertFalse(bridge.drive_stopped)

    def test_i_o_i_changes_angles_and_speed_gradually_without_motor_disable(self) -> None:
        settings = BRIDGE_MODULE.load_bridge_settings(
            PROJECT_ROOT / "config" / "open_loop_bridge.yaml"
        )
        controller = FakeController()
        bridge = BRIDGE_MODULE.Fx5uOpenLoopBridge.__new__(
            BRIDGE_MODULE.Fx5uOpenLoopBridge
        )
        bridge.controller = controller
        bridge.settings = settings
        bridge.stop_raw = 660
        bridge.kinematics = OpenLoopSwerveKinematics(
            settings.wheelbase_m, settings.track_width_m,
            settings.wheel_radius_m, settings.max_wheel_rate_rad_s,
        )
        bridge.previous_steering = (0.0,) * 4
        bridge.last_angles = None
        bridge.last_angle_float = None
        bridge.last_steering_s = None
        bridge.last_raw = (660,) * 4
        bridge.last_directions = None
        bridge.drive_stopped = True
        bridge.faulted = False
        bridge.settle_until_s = 0.0
        bridge.smoothed_velocity = VelocityCommand(0.0, 0.0, 0.0)
        bridge.last_slew_s = None
        bridge.last_source = None
        bridge.publish_status = lambda *args, **kwargs: None
        selected = [VelocityCommand(0.10, 0.0, 0.0)]
        bridge.selector = types.SimpleNamespace(select=lambda _: ("keyboard", selected[0]))

        for tick in range(15):
            with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=1.0 + tick * 0.1):
                bridge.on_timer()
        straight_angles = [value for name, value in controller.calls if name == "move_angles"]
        self.assertEqual(straight_angles[-1], (90, 90, 90, 90))
        self.assertGreater(len([
            value for name, value in controller.calls if name == "move_speeds"
        ]), 1)

        selected[0] = VelocityCommand(0.10, 0.0, -0.15)
        for tick in range(15, 45):
            with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=1.0 + tick * 0.1):
                bridge.on_timer()
        right_angles = [value for name, value in controller.calls if name == "move_angles"]
        self.assertGreater(len(set(right_angles[len(straight_angles):])), 2)
        self.assertEqual(bridge.smoothed_velocity, selected[0])

        selected[0] = VelocityCommand(0.10, 0.0, 0.0)
        for tick in range(45, 75):
            with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=1.0 + tick * 0.1):
                bridge.on_timer()
        all_angles = [value for name, value in controller.calls if name == "move_angles"]
        self.assertGreater(len(set(all_angles[len(right_angles):])), 2)
        self.assertEqual(all_angles[-1], (90, 90, 90, 90))
        for previous, current in zip(all_angles, all_angles[1:]):
            for before, after in zip(previous, current):
                self.assertLessEqual(
                    BRIDGE_MODULE.steering_distance_deg(before, after), 4
                )
        raw_values = [value for name, value in controller.calls if name == "move_speeds"]
        for previous, current in zip(raw_values, raw_values[1:]):
            self.assertLessEqual(
                max(abs(after - before) for before, after in zip(previous, current)),
                settings.max_raw_step_per_tick,
            )
        self.assertNotIn("stop_drive", [name for name, _ in controller.calls])
        self.assertFalse(bridge.drive_stopped)

        selected[0] = VelocityCommand(0.0, 0.0, 0.0)
        with patch.object(BRIDGE_MODULE.time, "monotonic", return_value=8.6):
            bridge.on_timer()
        self.assertEqual(controller.calls[-1][0], "stop_drive")
        self.assertTrue(bridge.smoothed_velocity.stopped)


    def test_right_left_and_holonomic_transitions_keep_drive_enabled(self) -> None:
        settings = BRIDGE_MODULE.load_bridge_settings(
            PROJECT_ROOT / "config" / "open_loop_bridge.yaml"
        )
        controller = FakeController()
        bridge = BRIDGE_MODULE.Fx5uOpenLoopBridge.__new__(
            BRIDGE_MODULE.Fx5uOpenLoopBridge
        )
        bridge.controller = controller
        bridge.settings = settings
        bridge.stop_raw = 660
        bridge.kinematics = OpenLoopSwerveKinematics(
            settings.wheelbase_m, settings.track_width_m,
            settings.wheel_radius_m, settings.max_wheel_rate_rad_s,
        )
        bridge.previous_steering = (0.0,) * 4
        bridge.last_angles = None
        bridge.last_angle_float = None
        bridge.last_steering_s = None
        bridge.last_raw = (660,) * 4
        bridge.last_directions = None
        bridge.drive_stopped = True
        bridge.faulted = False
        bridge.settle_until_s = 0.0
        bridge.smoothed_velocity = VelocityCommand(0.0, 0.0, 0.0)
        bridge.last_slew_s = None
        bridge.last_source = None
        bridge.publish_status = lambda *args, **kwargs: None
        selected = [VelocityCommand(0.10, 0.0, -0.15)]
        bridge.selector = types.SimpleNamespace(select=lambda _: ("keyboard", selected[0]))

        phases = (
            (0, 30, VelocityCommand(0.10, 0.0, -0.15)),  # O
            (30, 60, VelocityCommand(0.10, 0.0, 0.15)),  # U
            (60, 90, VelocityCommand(0.0, 0.10, 0.0)),   # Shift+J
            (90, 120, VelocityCommand(0.10, 0.10, 0.0)),  # Shift+U
            (120, 160, VelocityCommand(0.0, -0.10, 0.0)),  # Shift+L
        )
        for start, end, target in phases:
            selected[0] = target
            for tick in range(start, end):
                with patch.object(
                    BRIDGE_MODULE.time, "monotonic", return_value=1.0 + tick * 0.1
                ):
                    bridge.on_timer()
        self.assertNotIn("stop_drive", [name for name, _ in controller.calls])
        direction_indices = [
            index for index, (name, _) in enumerate(controller.calls)
            if name == "set_directions"
        ]
        self.assertGreater(len(direction_indices), 1)
        for index in direction_indices[1:]:
            previous_speeds = [
                values for name, values in controller.calls[:index]
                if name == "move_speeds"
            ]
            self.assertEqual(previous_speeds[-1], (660, 660, 660, 660))
        self.assertGreater(
            len([1 for name, _ in controller.calls if name == "move_angles"]), 10
        )
        self.assertGreater(
            len([1 for name, _ in controller.calls if name == "move_speeds"]), 10
        )
        self.assertFalse(bridge.drive_stopped)


if __name__ == "__main__":
    unittest.main()
