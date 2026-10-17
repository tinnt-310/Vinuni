"""The production Gazebo graph must give each actuator one controller owner."""

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from types import SimpleNamespace
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import yaml

spec = importlib.util.spec_from_file_location(
    'vehicle_simulation', Path(__file__).parents[1] / 'launch/simulation.launch.py')
simulation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(simulation)


class ControllerContractTest(unittest.TestCase):
    def setUp(self):
        self.vehicle = {
            'controller_manager': {'ros__parameters': {
                'update_rate': 100,
                'joint_state_broadcaster': {
                    'type': 'joint_state_broadcaster/JointStateBroadcaster'},
                'swerve_controller': {
                    'type': 'swerve_vehicle_control/SwerveDriveController'},
            }},
            'swerve_controller': {'ros__parameters': {'wheel_radius': 0.21}},
        }
        self.arm = {
            'joint_trajectory_controller': {'ros__parameters': {'joints': ['j1']}},
        }

    def test_pose_node_starts_only_after_arm_controller_spawner_succeeds(self):
        pose_node = object()
        self.assertEqual(
            simulation.arm_controller_transition(
                SimpleNamespace(returncode=0), pose_node),
            [pose_node],
        )
        failed = simulation.arm_controller_transition(
            SimpleNamespace(returncode=1), pose_node)
        self.assertEqual(len(failed), 1)
        self.assertIsInstance(failed[0], Shutdown)

    def test_vehicle_has_one_swerve_actuator_owner(self):
        config = simulation.compose_controller_config(self.vehicle, self.arm, True, False)
        types = config['controller_manager']['ros__parameters']
        self.assertEqual(types['swerve_controller']['type'],
                         'swerve_vehicle_control/SwerveDriveController')
        self.assertNotIn('steering_controller', types)
        self.assertNotIn('drive_controller', types)
        self.assertNotIn('joint_trajectory_controller', types)
        self.assertEqual(config['swerve_controller']['ros__parameters']['wheel_radius'], 0.21)

    def test_mounted_preserves_vehicle_and_adds_one_arm_owner(self):
        config = simulation.compose_controller_config(self.vehicle, self.arm, True, True)
        types = config['controller_manager']['ros__parameters']
        self.assertIn('swerve_controller', types)
        self.assertEqual(types['joint_trajectory_controller']['type'],
                         'joint_trajectory_controller/JointTrajectoryController')
        self.assertEqual(config['joint_trajectory_controller'],
                         self.arm['joint_trajectory_controller'])

    def test_mounted_pose_profile_has_one_arm_position_owner(self):
        pose_arm = {
            'controller_manager': {'ros__parameters': {
                'update_rate': 125,
                'fairino10_controller': {
                    'type': 'position_controllers/JointGroupPositionController'}}},
            'fairino10_controller': {'ros__parameters': {
                'joints': ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']}},
        }
        config = simulation.compose_controller_config(
            self.vehicle, pose_arm, True, True, 'pose')
        manager = config['controller_manager']['ros__parameters']
        self.assertIn('swerve_controller', manager)
        self.assertEqual(manager['update_rate'], 125)
        self.assertEqual(manager['fairino10_controller']['type'],
                         'position_controllers/JointGroupPositionController')
        self.assertNotIn('joint_trajectory_controller', manager)
        self.assertEqual(config['fairino10_controller']['ros__parameters']['joints'],
                         ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'])
        self.assertNotIn('swerve_controller', pose_arm['controller_manager']['ros__parameters'])

    def test_arm_only_has_no_swerve_controller(self):
        config = simulation.compose_controller_config(self.vehicle, self.arm, False, True)
        types = config['controller_manager']['ros__parameters']
        self.assertNotIn('swerve_controller', types)
        self.assertNotIn('swerve_controller', config)
        self.assertIn('joint_state_broadcaster', types)
        self.assertIn('joint_trajectory_controller', types)

    def test_does_not_mutate_input_configs(self):
        simulation.compose_controller_config(self.vehicle, self.arm, True, True)
        self.assertNotIn('joint_trajectory_controller',
                         self.vehicle['controller_manager']['ros__parameters'])


class ForceClockLaunchContractTest(unittest.TestCase):
    def test_matched_encoder_read_period_tracks_actual_world_step(self):
        # A 1 ms source interval can take 57 ms wall time during CAD contact.
        # Feedback must instead be read each physics step, without changing the
        # sole 125 Hz MotionExecutor or the 50 ms wall freshness barrier.
        for override in ('', '.00005', '.000125', '.000137', '.001'):
            with self.subTest(step=override):
                context = LaunchContext()
                context.launch_configurations.update({
                    'enable_vehicle': 'false', 'enable_arm': 'true',
                    'arm_controller_profile': 'pose', 'gui': 'false', 'rviz': 'false',
                    'enable_force_control': 'true', 'station': 'front_mid',
                    'matched_physics_step_s': override,
                })
                for action in simulation.generate_launch_description().entities:
                    if isinstance(action, DeclareLaunchArgument): action.execute(context)
                with tempfile.TemporaryDirectory(prefix='vinuni_encoder_contract_') as directory:
                    with patch.object(simulation.socket, 'socket') as port_probe, patch.object(
                            simulation.tempfile, 'mkdtemp', return_value=directory):
                        port_probe.return_value.__enter__.return_value.connect_ex.return_value = 1
                        simulation.launch_setup(context)
                    step = float(ET.parse(Path(directory)/'matched_socket.world').findtext(
                        'world/physics/max_step_size'))
                    config = yaml.safe_load((Path(directory)/'controllers.yaml').read_text())
                    manager = config['controller_manager']['ros__parameters']
                    self.assertEqual(manager['update_rate'], math.ceil(1./step))
                    self.assertLessEqual(1./manager['update_rate'], step)
                    self.assertEqual(manager['fairino10_controller']['type'],
                                     'position_controllers/JointGroupPositionController')
                    self.assertNotIn('joint_trajectory_controller', manager)

    def test_contact_source_tcp_transform_is_not_replaced_by_wrist_offset(self):
        context = LaunchContext()
        overlay = Path(simulation.get_package_share_directory('fr10_pose_controller')) / 'config/gazebo_contact_wrench_commissioning.yaml'
        context.launch_configurations.update({
            'enable_vehicle': 'false', 'enable_arm': 'true', 'arm_controller_profile': 'pose',
            'gui': 'false', 'rviz': 'false', 'enable_force_control': 'true',
            'station': 'front_mid', 'force_controller_overlay': str(overlay),
            'matched_physics_step_s': '.001',
        })
        for action in simulation.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument): action.execute(context)
        with tempfile.TemporaryDirectory(prefix='vinuni_contact_contract_') as directory:
            with patch.object(simulation.socket, 'socket') as port_probe, patch.object(
                    simulation.tempfile, 'mkdtemp', return_value=directory), patch.object(
                    simulation, 'Node', wraps=Node) as nodes:
                port_probe.return_value.__enter__.return_value.connect_ex.return_value = 1
                simulation.launch_setup(context)
            parameters = next(call.kwargs['parameters'] for call in nodes.call_args_list
                              if call.kwargs.get('name') == 'fr10_pose_controller')
            resolved = {key:value for entry in parameters if isinstance(entry,dict)
                        for key,value in entry.items()}
            self.assertEqual(resolved['force.wrench_transform.target_to_source_m'], [0.,0.,0.])
            self.assertEqual(resolved['force.wrench_topic'], '/fr10_simulation/contact_wrench')
            self.assertEqual(resolved['force.gravity_compensation.owner'], 'source')
            self.assertFalse(resolved['force.gravity_compensation.enabled'])

    def test_mounted_precision_config_reaches_swerve_spawner(self):
        context = LaunchContext()
        context.launch_configurations.update({
            'enable_vehicle': 'true', 'enable_arm': 'true',
            'arm_controller_profile': 'pose', 'gui': 'false', 'rviz': 'false',
            'enable_force_control': 'true', 'station': 'front_mid',
        })
        for action in simulation.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        with tempfile.TemporaryDirectory(prefix='vinuni_swerve_contract_') as directory:
            with patch.object(simulation.socket, 'socket') as port_probe, patch.object(
                    simulation.tempfile, 'mkdtemp', return_value=directory), patch.object(
                    simulation, 'IncludeLaunchDescription', wraps=IncludeLaunchDescription) as includes, patch.object(
                    simulation, 'Node', wraps=Node) as nodes:
                port_probe.return_value.__enter__.return_value.connect_ex.return_value = 1
                simulation.launch_setup(context)
            arguments = next(dict(call.kwargs['launch_arguments'])
                for call in includes.call_args_list
                if 'controller_parameters_file' in dict(call.kwargs.get('launch_arguments', [])))
            parameters = arguments['controller_parameters_file']
            with Path(parameters).open() as stream:
                config = yaml.safe_load(stream)
            self.assertFalse(config['swerve_controller']['ros__parameters']['publish_tf'])
            self.assertEqual(config['controller_manager']['ros__parameters']['update_rate'], 16000)
            pose_parameters=next(call.kwargs['parameters'] for call in nodes.call_args_list
                                 if call.kwargs.get('name')=='fr10_pose_controller')
            explicit={key:value for entry in pose_parameters if isinstance(entry,dict)
                      for key,value in entry.items()}
            self.assertGreater(explicit['controller.pose_policies.free_space.linear_scale'],0.)
            self.assertNotIn('controller.pose_policies.free_space.linear_velocity_scale',explicit)
            geometry = yaml.safe_load((Path(__file__).parents[2] /
                'vehicle_description/config/vehicle_geometry.yaml').read_text())['vehicle']
            for policy in ('free_space', 'insertion'):
                self.assertEqual(explicit[f'controller.pose_policies.{policy}.joint_jerk_scale'],
                                 geometry[f'precision_arm_{policy}_jerk_scale'])
                self.assertEqual(explicit[f'controller.pose_policies.{policy}.linear_scale'],
                                 geometry[f'precision_arm_{policy}_linear_velocity_scale'])
            self.assertEqual(explicit.get('force.retract_initial_velocity_m_s', 0.), 0.)
            self.assertEqual(explicit.get('force.retract_initial_distance_m', 0.), 0.)
            # Inspect the actual child launch, including its default declarations.
            # A composed manager YAML alone misses a spawner's -p override.
            child_spec = importlib.util.spec_from_file_location('swerve_launch',
                Path(__file__).parents[2] / 'swerve_vehicle_control/launch/swerve_controller.launch.py')
            child = importlib.util.module_from_spec(child_spec)
            child_spec.loader.exec_module(child)
            description = child.generate_launch_description()
            context.launch_configurations['controller_parameters_file'] = parameters
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            spawner = next(action for action in description.entities if isinstance(action, Node))
            command = [perform_substitutions(context, item) for item in spawner.cmd]
            self.assertEqual(command[command.index('-p') + 1], parameters)

    def test_mounted_commissioning_preserves_loaded_retract_pair(self):
        context = LaunchContext()
        pose_share = simulation.get_package_share_directory('fr10_pose_controller')
        overlay = Path(pose_share) / 'config/matched_socket_commissioning.yaml'
        profile = simulation.load_force_profile(str(overlay), pose_share)
        context.launch_configurations.update({
            'enable_vehicle': 'true', 'enable_arm': 'true',
            'arm_controller_profile': 'pose', 'gui': 'false', 'rviz': 'false',
            'enable_force_control': 'true', 'station': 'front_mid',
            'force_controller_overlay': str(overlay),
        })
        for action in simulation.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        with tempfile.TemporaryDirectory(prefix='vinuni_retract_contract_') as directory:
            with patch.object(simulation.socket, 'socket') as port_probe, patch.object(
                    simulation.tempfile, 'mkdtemp', return_value=directory), patch.object(
                    simulation, 'Node', wraps=Node) as nodes:
                port_probe.return_value.__enter__.return_value.connect_ex.return_value = 1
                simulation.launch_setup(context)
            parameters = next(call.kwargs['parameters'] for call in nodes.call_args_list
                              if call.kwargs.get('name') == 'fr10_pose_controller')
            resolved = {key: value for entry in parameters if isinstance(entry, dict)
                        for key, value in entry.items()}
        geometry = yaml.safe_load((Path(__file__).parents[2] /
            'vehicle_description/config/vehicle_geometry.yaml').read_text())['vehicle']
        self.assertEqual(resolved['force.retract_initial_velocity_m_s'],
                         geometry['precision_arm_retract_initial_velocity_m_s'])
        self.assertEqual(resolved['force.retract_initial_distance_m'],
                         profile['force.retract_initial_distance_m'])
        self.assertGreater(resolved['force.retract_initial_distance_m'], 0.)

    def gazebo_arguments(self, force):
        context = LaunchContext()
        context.launch_configurations.update({
            'enable_vehicle': 'false', 'enable_arm': 'true',
            'arm_controller_profile': 'pose', 'gui': 'false', 'rviz': 'false',
            'enable_force_control': 'true' if force else 'false',
        })
        for action in simulation.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        # Build the actual graph without starting ROS/Gazebo or opening a port.
        with tempfile.TemporaryDirectory(prefix='vinuni_clock_contract_') as directory:
            with patch.object(simulation.socket, 'socket') as port_probe, patch.object(
                    simulation.tempfile, 'mkdtemp', return_value=directory):
                port_probe.return_value.__enter__.return_value.connect_ex.return_value = 1
                actions = simulation.launch_setup(context)
            for action in actions:
                if isinstance(action, IncludeLaunchDescription):
                    arguments = dict(action.launch_arguments)
                    if arguments.get('server_required') == 'true':
                        return arguments
        self.fail('Gazebo server include is missing')

    def test_force_launch_uses_physics_clock_for_125_hz_execution(self):
        arguments = self.gazebo_arguments(True)
        with Path(arguments['params_file']).open() as stream:
            config = yaml.safe_load(stream)
        self.assertEqual(config['gazebo']['ros__parameters']['publish_rate'], 1000.0)

    def test_all_motion_modes_share_physics_clock(self):
        arguments = self.gazebo_arguments(False)
        with Path(arguments['params_file']).open() as stream:
            config = yaml.safe_load(stream)
        self.assertEqual(config['gazebo']['ros__parameters']['publish_rate'], 1000.0)


if __name__ == '__main__':
    unittest.main()
