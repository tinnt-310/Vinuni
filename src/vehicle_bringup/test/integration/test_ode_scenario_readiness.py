"""Checks that Gazebo clients start only after required controllers are active."""

import unittest

from ode_scenario_readiness import controllers_activated, required_controllers_for_case


class ControllerReadinessTest(unittest.TestCase):
    def test_requires_each_controller_activation(self):
        log = (
            "[spawner-9] [spawner_joint_state_broadcaster]: "
            "Configured and activated joint_state_broadcaster\n"
            "[spawner-10] [spawner_steering_controller]: "
            "Configured and activated steering_controller\n"
        )
        required = {'joint_state_broadcaster', 'steering_controller',
                    'drive_controller'}
        self.assertFalse(controllers_activated(log, required))
        self.assertTrue(controllers_activated(
            log + "[spawner-11] [spawner_drive_controller]: "
                  "Configured and activated drive_controller\n", required))

    def test_ignores_loading_and_other_controller_messages(self):
        log = (
            "[spawner-9] [spawner_joint_trajectory_controller]: "
            "waiting for service /controller_manager/list_controllers\n"
            "[gzserver-1] [controller_manager]: Loading controller "
            "'joint_trajectory_controller'\n"
            "[spawner-10] [spawner_joint_state_broadcaster]: "
            "Configured and activated joint_state_broadcaster\n"
        )
        self.assertFalse(controllers_activated(
            log, {'joint_trajectory_controller'}))

    def test_accepts_colorized_spawner_line(self):
        log = ("[spawner-12] [spawner_joint_trajectory_controller]: "
               "\x1b[92mConfigured and activated \x1b[1m"
               "joint_trajectory_controller\x1b[0m\n")
        self.assertTrue(controllers_activated(
            log, {'joint_trajectory_controller'}))

    def test_v0_v8_controller_requirements(self):
        expected = {
            'V0': set(),
            'V1': {'joint_state_broadcaster'},
            'V2': {'joint_state_broadcaster', 'steering_controller'},
            'V3': {'joint_state_broadcaster', 'drive_controller'},
        }
        all_active = {'joint_state_broadcaster', 'steering_controller',
                      'drive_controller'}
        expected.update({f'V{i}': all_active for i in range(4, 9)})
        for case, controllers in expected.items():
            self.assertEqual(required_controllers_for_case(case), controllers)
        with self.assertRaises(ValueError):
            required_controllers_for_case('V9')

if __name__ == '__main__':
    unittest.main()
