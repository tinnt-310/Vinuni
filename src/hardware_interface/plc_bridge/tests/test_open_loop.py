from __future__ import annotations

import math
import unittest
from pathlib import Path

import yaml

from fx5u_controller.open_loop import (
    ManualCommandSelector,
    OpenLoopSwerveKinematics,
    VelocityCommand,
    clamp_velocity,
    logical_to_plc,
    rate_to_raw,
    speed_to_raw,
    slew_velocity,
    slew_heading_deg,
    steering_distance_deg,
    steering_to_plc_degree,
)


class OpenLoopTests(unittest.TestCase):
    def test_provisional_linear_speed_calibration(self):
        for speed, raw in ((0, 660), (0.1, 634), (0.2, 608),
                           (0.5, 530), (1, 400), (-1, 400), (5, 0)):
            self.assertEqual(speed_to_raw(speed, 400), raw)
        for raw in (-1, 660, 700, True):
            with self.assertRaises(ValueError):
                speed_to_raw(1.0, raw)
        with self.assertRaises(ValueError):
            speed_to_raw(float("nan"), 400)

    def test_velocity_slew_limits_linear_vector_and_yaw(self) -> None:
        zero = VelocityCommand(0.0, 0.0, 0.0)
        target = VelocityCommand(0.1, 0.1, -0.15)
        first = slew_velocity(zero, target, 0.1, 0.4, 0.15)
        self.assertAlmostEqual(math.hypot(first.vx, first.vy), 0.04)
        self.assertAlmostEqual(first.wz, -0.015)
        reached = slew_velocity(first, target, 2.0, 0.4, 0.15)
        self.assertAlmostEqual(reached.vx, target.vx)
        self.assertAlmostEqual(reached.vy, target.vy)
        self.assertAlmostEqual(reached.wz, target.wz)
        with self.assertRaises(ValueError):
            slew_velocity(zero, target, -0.1, 0.4, 0.15)

    def test_wrapped_steering_slew_never_skips_intermediate_angles(self) -> None:
        self.assertEqual(slew_heading_deg(90.0, 120.0, 2.0), 92.0)
        self.assertEqual(slew_heading_deg(120.0, 70.0, 2.0), 118.0)
        self.assertEqual(slew_heading_deg(359.0, 2.0, 2.0), 1.0)
        self.assertEqual(slew_heading_deg(1.0, 358.0, 2.0), 359.0)

    def test_raw_speed_mapping_matches_confirmed_voltage_direction(self) -> None:
        self.assertEqual(rate_to_raw(0.0, 10.0), 660)
        self.assertEqual(rate_to_raw(5.0, 10.0), 330)
        self.assertEqual(rate_to_raw(-10.0, 10.0), 0)
        self.assertEqual(rate_to_raw(20.0, 10.0), 0)

    def test_bridge_config_uses_ninety_degrees_for_straight(self) -> None:
        config_path = Path(__file__).resolve().parents[1] / "config" / "open_loop_bridge.yaml"
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        self.assertEqual(config["limits"]["minimum_raw_change"], 1)
        self.assertEqual(
            config["wheels"]["steering_zero_offset_deg"], [90.0, 90.0, 90.0, 90.0]
        )

    def test_forward_command_keeps_all_steering_zero(self) -> None:
        model = OpenLoopSwerveKinematics(0.8, 0.7, 0.21, 10.0)
        output = model.calculate(VelocityCommand(0.21, 0.0, 0.0), [0.0] * 4)
        self.assertEqual(output.steering_rad, (0.0, 0.0, 0.0, 0.0))
        for rate in output.wheel_rate_rad_s:
            self.assertAlmostEqual(rate, 1.0)

    def test_plc_angle_uses_ninety_as_straight_and_wraps(self) -> None:
        self.assertEqual(steering_to_plc_degree(0.0, 1, 90.0), 90)
        self.assertEqual(steering_to_plc_degree(-math.pi / 2.0, 1, 90.0), 0)
        self.assertEqual(steering_to_plc_degree(-math.pi, 1, 90.0), 270)
        self.assertEqual(steering_to_plc_degree(math.radians(-91), 1, 90.0), 359)
        self.assertEqual(steering_distance_deg(359, 0), 1)
        self.assertEqual(steering_distance_deg(0, 359), 1)

    def test_lateral_command_uses_ninety_degree_steering(self) -> None:
        model = OpenLoopSwerveKinematics(0.8, 0.7, 0.21, 10.0)
        output = model.calculate(VelocityCommand(0.0, 0.21, 0.0), [0.0] * 4)
        for angle in output.steering_rad:
            self.assertAlmostEqual(angle, math.pi / 2.0)

    def test_selector_prioritizes_enabled_joystick_and_times_out(self) -> None:
        selector = ManualCommandSelector(0.3)
        keyboard = VelocityCommand(0.1, 0.0, 0.0)
        joystick = VelocityCommand(0.0, 0.1, 0.0)
        selector.update_keyboard(keyboard, 1.0)
        selector.update_joystick(joystick, 1.1, enabled=True)
        self.assertEqual(selector.select(1.2), ("joystick", joystick))
        selector.update_joystick(joystick, 1.21, enabled=False)
        self.assertEqual(selector.select(1.22), ("keyboard", keyboard))
        self.assertEqual(selector.select(1.5)[0], None)

    def test_clamping_and_plc_order(self) -> None:
        command = clamp_velocity(VelocityCommand(3.0, 4.0, 2.0), 1.0, 0.5)
        self.assertAlmostEqual(command.vx, 0.6)
        self.assertAlmostEqual(command.vy, 0.8)
        self.assertEqual(command.wz, 0.5)
        self.assertEqual(logical_to_plc(("FL", "FR", "RL", "RR"), (3, 1, 4, 2)),
                         ("FR", "RR", "FL", "RL"))


if __name__ == "__main__":
    unittest.main()
