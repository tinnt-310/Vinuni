"""The effective swerve geometry must match the spawned wheel model."""

from copy import deepcopy
import importlib.util
from pathlib import Path
from xml.dom import minidom
import unittest

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml


spec = importlib.util.spec_from_file_location(
    'vehicle_simulation', Path(__file__).parents[1] / 'launch/simulation.launch.py')
simulation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(simulation)


class VehicleGeometryContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        description_share = Path(get_package_share_directory('vehicle_description'))
        swerve_share = Path(get_package_share_directory('swerve_vehicle_control'))
        cls.bringup_root = Path(__file__).parents[1]
        with (description_share / 'config/vehicle_geometry.yaml').open() as stream:
            cls.geometry = yaml.safe_load(stream)['vehicle']
        with (swerve_share / 'config/swerve_controllers.yaml').open() as stream:
            cls.controller_config = yaml.safe_load(stream)
        cls.robot = xacro.process_file(
            str(Path(__file__).parents[1] / 'config/simulation.xacro'),
            mappings={
                'enable_vehicle': 'true',
                'enable_arm': 'false',
                'sim_base_collision_proxy': 'true',
                'controller_yaml_file': '/tmp/geometry_contract_controllers.yaml',
            },
        )

    def test_production_controller_matches_spawned_wheel_model(self):
        simulation.validate_vehicle_geometry(
            self.geometry, self.controller_config, self.robot)

    def test_rejects_controller_geometry_drift(self):
        changed = deepcopy(self.controller_config)
        changed['swerve_controller']['ros__parameters']['wheel_separation_x'] += 0.01
        with self.assertRaisesRegex(ValueError, 'wheel_separation_x'):
            simulation.validate_vehicle_geometry(self.geometry, changed, self.robot)

    def test_rejects_controller_wheel_offset_drift(self):
        changed = deepcopy(self.controller_config)
        changed['swerve_controller']['ros__parameters']['wheel_offset_x'] = 0.0
        with self.assertRaisesRegex(ValueError, 'wheel_offset_x'):
            simulation.validate_vehicle_geometry(self.geometry, changed, self.robot)

    def test_rejects_model_wheel_position_drift(self):
        changed = minidom.parseString(self.robot.toxml())
        steering = next(
            joint for joint in changed.getElementsByTagName('joint')
            if joint.getAttribute('name') == 'Revolute1_to_base_link'
            and joint.parentNode == changed.documentElement)
        steering.getElementsByTagName('origin')[0].setAttribute('xyz', '0.41 0.35 0.52')
        with self.assertRaisesRegex(ValueError, 'Revolute1_to_base_link'):
            simulation.validate_vehicle_geometry(self.geometry, self.controller_config, changed)

    def test_rejects_model_wheel_axle_height_drift(self):
        changed = minidom.parseString(self.robot.toxml())
        axle = next(
            joint for joint in changed.getElementsByTagName('joint')
            if joint.getAttribute('name') == 'Wheel2_to_Revolute2'
            and joint.parentNode == changed.documentElement)
        axle.getElementsByTagName('origin')[0].setAttribute('xyz', '0 0 -0.30')
        with self.assertRaisesRegex(ValueError, 'Wheel2_to_Revolute2'):
            simulation.validate_vehicle_geometry(self.geometry, self.controller_config, changed)

    def test_rejects_model_wheel_axle_offset_drift(self):
        changed = minidom.parseString(self.robot.toxml())
        axle = next(
            joint for joint in changed.getElementsByTagName('joint')
            if joint.getAttribute('name') == 'Wheel2_to_Revolute2'
            and joint.parentNode == changed.documentElement)
        axle.getElementsByTagName('origin')[0].setAttribute('xyz', '0 0 -0.295439')
        with self.assertRaisesRegex(ValueError, 'Wheel2_to_Revolute2 x'):
            simulation.validate_vehicle_geometry(self.geometry, self.controller_config, changed)

    def test_rejects_wheel_axis_reversal(self):
        changed = minidom.parseString(self.robot.toxml())
        axle = next(
            joint for joint in changed.getElementsByTagName('joint')
            if joint.getAttribute('name') == 'Wheel3_to_Revolute3'
            and joint.parentNode == changed.documentElement)
        axle.getElementsByTagName('axis')[0].setAttribute('xyz', '0 -1 0')
        with self.assertRaisesRegex(ValueError, 'Wheel3_to_Revolute3'):
            simulation.validate_vehicle_geometry(self.geometry, self.controller_config, changed)

    def test_rejects_model_wheel_radius_drift(self):
        changed = minidom.parseString(self.robot.toxml())
        wheel = next(
            link for link in changed.getElementsByTagName('link')
            if link.getAttribute('name') == 'Wheel4_1')
        wheel.getElementsByTagName('collision')[0].getElementsByTagName('cylinder')[0].setAttribute('radius', '0.22')
        with self.assertRaisesRegex(ValueError, 'Wheel4_1'):
            simulation.validate_vehicle_geometry(self.geometry, self.controller_config, changed)

    def test_vehicle_control_xacro_is_owned_by_vehicle_description(self):
        source = (self.bringup_root / 'config/simulation.xacro').read_text()
        self.assertIn(
            '$(find vehicle_description)/urdf/ros2_control.xacro', source)
        self.assertNotIn(
            '$(find swerve_vehicle_control)/config/ros2_control.xacro', source)

    def test_nav2_footprints_match_rb_v3_body_bounds(self):
        with (self.bringup_root.parent / 'vehicle_navigation/config/nav2.yaml').open() as stream:
            nav2 = yaml.safe_load(stream)
        expected = [
            [1.02031, 0.505735],
            [1.02031, -0.505735],
            [-1.02261, -0.505735],
            [-1.02261, 0.505735],
        ]
        for costmap in ('local_costmap', 'global_costmap'):
            value = nav2[costmap][costmap]['ros__parameters']['footprint']
            self.assertEqual(yaml.safe_load(value), expected)


if __name__ == '__main__':
    unittest.main()
