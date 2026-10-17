import math
import types
import unittest
from unittest.mock import patch

from fx5u_controller.cyclic import STOP
from fx5u_controller.open_loop import OpenLoopSwerveKinematics, VelocityCommand
from test_bridge_motion import BRIDGE_MODULE, PROJECT_ROOT


class Worker:
    def __init__(self):
        self.calls = []
        self.status = {"error": None}

    def submit(self, angles, raw, alignment_angles=None):
        self.calls.append((tuple(angles), tuple(raw), tuple(alignment_angles)))

    def stop_drive(self):
        self.calls.append("stop")

    def emergency_stop(self):
        self.calls.append("emergency_stop")


def make_bridge():
    bridge = BRIDGE_MODULE.Fx5uCyclicBridge.__new__(BRIDGE_MODULE.Fx5uCyclicBridge)
    bridge.controller = Worker()
    bridge.settings = BRIDGE_MODULE.load_bridge_settings(
        PROJECT_ROOT / "config/open_loop_bridge.yaml")
    bridge.stop_raw = 660
    bridge.kinematics = OpenLoopSwerveKinematics(
        bridge.settings.wheelbase_m, bridge.settings.track_width_m,
        bridge.settings.wheel_radius_m, bridge.settings.max_wheel_rate_rad_s,
        allow_reverse=False)
    bridge.previous_steering = (0.0,) * 4
    bridge.last_angle_float = (90.0,) * 4
    bridge.last_angles = (90,) * 4
    bridge.last_raw = STOP
    bridge.last_steering_s = None
    bridge.drive_stopped = True
    bridge.faulted = False
    bridge.smoothed_velocity = VelocityCommand(0, 0, 0)
    bridge.publish_status = unittest.mock.Mock()
    bridge.get_logger = unittest.mock.Mock()
    return bridge


class CyclicBridgeTests(unittest.TestCase):
    def test_main_without_execute_never_constructs_a_transport(self):
        args = types.SimpleNamespace(execute=False)
        with patch.object(BRIDGE_MODULE, "parse_args", return_value=(args, [])), \
                patch.object(BRIDGE_MODULE, "CyclicWorker") as worker, \
                patch.object(BRIDGE_MODULE.sys, "stderr"):
            self.assertEqual(BRIDGE_MODULE.main(), 2)
        worker.assert_not_called()

    def test_main_stops_transport_before_ros_teardown(self):
        actions = []
        args = types.SimpleNamespace(execute=True, config=PROJECT_ROOT / "config/default.yaml",
                                     bridge_config=PROJECT_ROOT / "config/open_loop_bridge.yaml",
                                     ip=None, port=None)
        ros = types.SimpleNamespace(
            init=lambda **_: actions.append("ros_init"),
            spin=unittest.mock.Mock(side_effect=KeyboardInterrupt),
            ok=lambda: True, shutdown=lambda: actions.append("ros_shutdown"))
        worker = types.SimpleNamespace(start=lambda: actions.append("start"),
                                       close=lambda: actions.append("close"))
        node = types.SimpleNamespace(destroy_node=lambda: actions.append("destroy_node"))
        with patch.object(BRIDGE_MODULE, "parse_args", return_value=(args, [])), \
                patch.object(BRIDGE_MODULE, "CyclicWorker", return_value=worker), \
                patch.object(BRIDGE_MODULE, "Fx5uCyclicBridge", return_value=node), \
                patch.object(BRIDGE_MODULE, "rclpy", ros):
            self.assertEqual(BRIDGE_MODULE.main(), 0)
        self.assertEqual(actions, ["ros_init", "start", "close", "destroy_node", "ros_shutdown"])

    def test_backward_target_is_one_atomic_submission_without_legacy_bits(self):
        bridge = make_bridge()
        bridge.send_motion("keyboard", VelocityCommand(-0.1, 0, 0), 1)
        angles, raw, alignment = bridge.controller.calls[-1]
        self.assertEqual(alignment, (270,) * 4)
        self.assertTrue(all(abs(math.remainder(a - 90, 360)) <= 1.5 for a in angles))
        self.assertEqual(raw, (653,) * 4)
        bridge.send_motion("keyboard", VelocityCommand(-0.1, 0, 0), 1.05)
        self.assertEqual(bridge.controller.calls[-1][1], (646,) * 4)

    def test_mixed_swerve_targets_follow_plc_wheel_order(self):
        bridge = make_bridge()
        command = VelocityCommand(0.1, 0, 0.1)
        wheels = bridge.kinematics.calculate(command, bridge.previous_steering)
        bridge.send_motion("keyboard", command, 1)
        alignment = bridge.controller.calls[-1][2]
        for logical_index, plc_id in enumerate((3, 1, 4, 2)):
            expected = round(math.degrees(wheels.steering_rad[logical_index]) + 90) % 360
            self.assertEqual(alignment[plc_id - 1], expected)

    def test_reset_discards_command_without_any_unmapped_reset_pulse(self):
        bridge = make_bridge()
        bridge.on_reset_drive(types.SimpleNamespace(data="reset"))
        self.assertEqual(bridge.controller.calls, ["stop"])
        self.assertEqual(bridge.last_raw, STOP)
        self.assertIsNone(bridge.selector.select(10)[0])

    def test_worker_fault_is_reported_even_while_robot_is_idle(self):
        bridge = make_bridge()
        bridge.controller.status = {"error": "Hall read timeout"}
        bridge.on_timer()
        self.assertTrue(bridge.faulted)
        self.assertEqual(bridge.controller.calls, ["emergency_stop"])

    def test_status_contains_actual_feedback_and_provisional_speed_calibration(self):
        bridge = make_bridge()
        bridge.controller.status = {"feedback": {"actual_angle_deg": (90,) * 4}}
        bridge.status_pub = unittest.mock.Mock()
        with patch.object(BRIDGE_MODULE, "String", types.SimpleNamespace):
            BRIDGE_MODULE.Fx5uCyclicBridge.publish_status(bridge, "stopped", None)
        message = bridge.status_pub.publish.call_args.args[0]
        import json
        status = json.loads(message.data)
        self.assertEqual(status["plc"]["feedback"]["actual_angle_deg"], [90] * 4)
        self.assertEqual(status["velocity_calibration"], "provisional")


if __name__ == "__main__":
    unittest.main()
