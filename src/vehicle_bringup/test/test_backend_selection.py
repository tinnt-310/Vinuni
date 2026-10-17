import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions


spec = importlib.util.spec_from_file_location('bringup', Path(__file__).parents[1] / 'launch/bringup.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SelectionTest(unittest.TestCase):
    def test_supported_routes(self):
        for arm, vehicle, mode, filename in (
            (True, False, 'sim', 'simulation.launch.py'),
            (True, False, 'real', 'fr10_pose_controller.launch.py'),
            (False, True, 'sim', 'simulation.launch.py'),
            (True, True, 'sim', 'simulation.launch.py')):
            with self.subTest(arm=arm, vehicle=vehicle, mode=mode):
                self.assertEqual(module.select_backend(arm, vehicle, mode)[1], filename)

    def test_unconfigured_plc_rejected(self):
        for arm in (False, True):
            with self.assertRaisesRegex(RuntimeError, 'missing encoder'):
                module.select_backend(arm, True, 'real')

    def test_empty_and_invalid_rejected(self):
        with self.assertRaises(ValueError):
            module.select_backend(False, False, 'sim')
        with self.assertRaises(ValueError):
            module.select_backend(True, True, 'typo')

    def test_real_arm_routes_to_executor_not_moveit_demo(self):
        self.assertEqual(module.select_backend(True, False, 'real'),
                         ('fr10_pose_controller', 'fr10_pose_controller.launch.py'))


class LaunchSwitchTest(unittest.TestCase):
    def context(self, **overrides):
        context = LaunchContext()
        context.launch_configurations.update(overrides)
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        return context

    def include(self, **overrides):
        context = self.context(**overrides)
        with patch.object(module, 'get_package_share_directory',
                          return_value='/test/share') as lookup:
            actions = module.setup(context)
        include = next(action for action in actions
                       if isinstance(action, IncludeLaunchDescription))
        arguments = {
            name: value if isinstance(value, str)
            else perform_substitutions(context, value)
            for name, value in include.launch_arguments
        }
        return lookup.call_args.args[0], arguments

    def test_public_switches_are_declared(self):
        names = {action.name for action in module.generate_launch_description().entities
                 if isinstance(action, DeclareLaunchArgument)}
        self.assertTrue({'use_arm', 'use_vehicle', 'use_gazebo', 'use_force'} <= names)

    def test_default_is_mounted_gazebo_with_sole_position_controller(self):
        package, arguments = self.include()
        self.assertEqual(package, 'vehicle_bringup')
        self.assertEqual(arguments['enable_arm'], 'true')
        self.assertEqual(arguments['enable_vehicle'], 'true')
        self.assertEqual(arguments['arm_controller_profile'], 'pose')
        self.assertEqual(arguments['enable_force_control'], 'false')
        self.assertEqual(arguments['wrist_layout'], 'calibrated')

    def test_photo_wrist_is_forwarded_for_simulation(self):
        _, arguments = self.include(use_vehicle='false', wrist_layout='photo_dc_d455')
        self.assertEqual(arguments['wrist_layout'], 'photo_dc_d455')

    def test_unmeasured_photo_wrist_rejects_hardware_and_old_contact_fixture(self):
        for overrides in ({'use_gazebo': 'false'}, {'use_force': 'true'}):
            with self.subTest(overrides=overrides), patch.object(
                    module, 'get_package_share_directory') as lookup:
                with self.assertRaisesRegex(ValueError, 'uncalibrated male-connector'):
                    module.setup(self.context(use_vehicle='false',
                                              wrist_layout='photo_dc_d455', **overrides))
                lookup.assert_not_called()

    def test_public_component_switches(self):
        for arm, vehicle in (('true', 'false'), ('false', 'true'), ('true', 'true')):
            with self.subTest(arm=arm, vehicle=vehicle):
                package, arguments = self.include(
                    use_arm=arm, use_vehicle=vehicle, use_gazebo='true')
                self.assertEqual(package, 'vehicle_bringup')
                self.assertEqual(arguments['enable_arm'], arm)
                self.assertEqual(arguments['enable_vehicle'], vehicle)
                self.assertEqual(arguments['arm_controller_profile'], 'pose')

    def test_legacy_aliases_remain_supported(self):
        _, arguments = self.include(arm='false', vehicle='true', mode='sim')
        self.assertEqual(arguments['enable_arm'], 'false')
        self.assertEqual(arguments['enable_vehicle'], 'true')
        package, arguments = self.include(arm='true', vehicle='false', mode='real')
        self.assertEqual(package, 'fr10_pose_controller')
        self.assertEqual(arguments['use_fake_hardware'], 'false')

    def test_public_switches_override_legacy_aliases(self):
        package, arguments = self.include(arm='false', vehicle='true', mode='real',
                                          use_arm='true', use_vehicle='false',
                                          use_gazebo='true')
        self.assertEqual(package, 'vehicle_bringup')
        self.assertEqual(arguments['enable_arm'], 'true')
        self.assertEqual(arguments['enable_vehicle'], 'false')

    def test_runtime_options_are_forwarded(self):
        _, arguments = self.include(gui='false', rviz='false', gazebo_port='11422',
                                    cmd_vel_topic='/teleop/cmd_vel')
        self.assertEqual(arguments['gui'], 'false')
        self.assertEqual(arguments['rviz'], 'false')
        self.assertEqual(arguments['gazebo_port'], '11422')
        self.assertEqual(arguments['cmd_vel_topic'], '/teleop/cmd_vel')

    def test_real_arm_uses_sdk_without_enabling_force(self):
        package, arguments = self.include(use_gazebo='false', use_vehicle='false',
                                          robot_ip='192.0.2.10', rviz='false')
        self.assertEqual(package, 'fr10_pose_controller')
        self.assertEqual(arguments['use_gazebo'], 'false')
        self.assertEqual(arguments['use_fake_hardware'], 'false')
        self.assertEqual(arguments['robot_ip'], '192.0.2.10')
        self.assertEqual(arguments['use_rviz'], 'false')
        self.assertEqual(arguments['enable_force_sensor'], 'false')

    def test_force_is_forwarded_only_for_standalone_gazebo(self):
        package, arguments = self.include(use_arm='true', use_vehicle='false',
                                          use_gazebo='true', use_force='true')
        self.assertEqual(package, 'vehicle_bringup')
        self.assertEqual(arguments['enable_force_control'], 'true')
        self.assertEqual(arguments['ft_wrench_topic'], 'wrench')

    def test_force_is_fail_closed_for_real_or_mounted_launch(self):
        for overrides in (
            {'use_force': 'true', 'use_gazebo': 'false'},
            {'use_force': 'true', 'use_vehicle': 'true'},
        ):
            with self.subTest(overrides=overrides), self.assertRaises(RuntimeError):
                self.include(**overrides)

    def test_matched_mounted_force_uses_same_public_launch(self):
        package, arguments = self.include(use_force='true', use_vehicle='true',
                                          station='front_mid', demo='false')
        self.assertEqual(package, 'vehicle_bringup')
        self.assertEqual(arguments['enable_force_control'], 'true')
        self.assertEqual(arguments['station'], 'front_mid')
        self.assertEqual(arguments['demo'], 'false')

    def test_invalid_switch_is_rejected(self):
        for name in ('use_arm', 'use_vehicle', 'use_gazebo'):
            with self.subTest(name=name), self.assertRaises(RuntimeError):
                self.context(**{name: 'typo'})

    def test_unsafe_or_empty_selection_rejected_before_package_lookup(self):
        for overrides, error in (
            ({'use_arm': 'false', 'use_vehicle': 'false'}, ValueError),
            ({'use_gazebo': 'false', 'use_vehicle': 'true'}, RuntimeError),
        ):
            with self.subTest(overrides=overrides), patch.object(
                    module, 'get_package_share_directory') as lookup:
                with self.assertRaises(error):
                    module.setup(self.context(**overrides))
                lookup.assert_not_called()


class NativeDesktopEnvironmentTest(unittest.TestCase):
    def test_native_shell_needs_no_overrides(self):
        environment = {'PATH': '/usr/bin:/opt/ros/humble/bin',
                       'LD_LIBRARY_PATH': '/opt/vendor/lib',
                       'GTK_MODULES': 'custom-native-module'}
        self.assertEqual(module.native_desktop_environment(environment), ({}, []))

    def test_snap_shell_keeps_ros_paths_and_removes_injected_gui_runtime(self):
        environment = {
            'SNAP': '/snap/code/266',
            'SNAP_NAME': 'code',
            'PATH': '/snap/code/266/usr/bin:/opt/ros/humble/bin:/usr/bin',
            'LD_LIBRARY_PATH': '/snap/core20/current/lib:/workspace/install/lib',
            'GTK_MODULES': 'gail:atk-bridge',
            'QT_PLUGIN_PATH': '/snap/code/266/qt/plugins',
            'LD_PRELOAD': '/snap/code/266/lib/injected.so',
            'DISPLAY': ':1',
            'ROS_DOMAIN_ID': '211',
        }
        original = dict(environment)
        updates, removed = module.native_desktop_environment(environment)
        self.assertEqual(updates['PATH'], '/opt/ros/humble/bin:/usr/bin')
        self.assertEqual(updates['LD_LIBRARY_PATH'], '/workspace/install/lib')
        self.assertTrue({'SNAP', 'SNAP_NAME', 'GTK_MODULES', 'QT_PLUGIN_PATH',
                         'LD_PRELOAD'} <= set(removed))
        self.assertNotIn('DISPLAY', removed)
        self.assertNotIn('ROS_DOMAIN_ID', removed)
        self.assertNotIn('ROS_DOMAIN_ID', updates)
        self.assertEqual(environment, original)

    def test_restore_original_xdg_paths_without_dropping_native_custom_paths(self):
        updates, _ = module.native_desktop_environment({
            'SNAP': '/snap/code/266',
            'XDG_DATA_DIRS': '/snap/code/266/usr/share:/usr/share',
            'XDG_DATA_DIRS_VSCODE_SNAP_ORIG': '/opt/custom/share:/usr/share',
            'XDG_CONFIG_DIRS': '/etc/custom:/etc/xdg',
        })
        self.assertEqual(updates['XDG_DATA_DIRS'], '/opt/custom/share:/usr/share')
        self.assertEqual(updates['XDG_CONFIG_DIRS'], '/etc/custom:/etc/xdg')

    def test_empty_native_xdg_paths_use_host_defaults(self):
        updates, _ = module.native_desktop_environment({
            'SNAP': '/snap/code/266',
            'XDG_DATA_DIRS': '/home/example/snap/code/266/share',
            'XDG_CONFIG_DIRS': '/snap/code/266/etc/xdg',
        })
        self.assertEqual(updates['XDG_DATA_DIRS'], '/usr/local/share:/usr/share')
        self.assertEqual(updates['XDG_CONFIG_DIRS'], '/etc/xdg')

    def test_native_qt_and_preload_overrides_are_preserved(self):
        _, removed = module.native_desktop_environment({
            'SNAP': '/snap/code/266',
            'QT_PLUGIN_PATH': '/opt/custom/qt/plugins',
            'QT_QPA_PLATFORM_PLUGIN_PATH': '/opt/custom/qt/platforms',
            'LD_PRELOAD': '/opt/custom/profiler.so',
        })
        self.assertNotIn('QT_PLUGIN_PATH', removed)
        self.assertNotIn('QT_QPA_PLATFORM_PLUGIN_PATH', removed)
        self.assertNotIn('LD_PRELOAD', removed)

    def test_snap_library_path_without_snap_marker_is_also_detected(self):
        updates, removed = module.native_desktop_environment({
            'LD_LIBRARY_PATH': '/snap/core20/current/lib:/usr/local/lib',
            'GTK_MODULES': 'gail:atk-bridge',
        })
        self.assertEqual(updates['LD_LIBRARY_PATH'], '/usr/local/lib')
        self.assertIn('GTK_MODULES', removed)


if __name__ == '__main__':
    unittest.main()
