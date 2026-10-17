"""Contract for the sole FR10 geometry, wrist and MoveIt semantic model."""

from pathlib import Path
import math
import struct
import unittest
import xml.etree.ElementTree as ET

import xacro


DESCRIPTION = Path(__file__).resolve().parents[1]
VENDOR = DESCRIPTION.parent / 'fairino_description'
MOVEIT_CONFIG = DESCRIPTION.parent / 'fr10_arm_control' / 'fairino10_v6_moveit2_config' / 'config'
ARM_GEOMETRY_LINKS = {
    'fr10_base_link', 'shoulder_link', 'upperarm_link', 'forearm_link',
    'wrist1_link', 'wrist2_link', 'wrist3_link',
}


def expand(vehicle_enabled):
    xml = xacro.process_file(
        str(DESCRIPTION / 'urdf' / 'vehicle.urdf.xacro'),
        mappings={'enable_vehicle': str(vehicle_enabled).lower(), 'enable_arm': 'true'},
    ).toxml()
    return ET.fromstring(xml)


def arm_subtree(robot):
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}
    descendants = {'fr10_base_link'}
    while True:
        children = {
            joint.find('child').attrib['link']
            for joint in joints.values()
            if joint.find('parent').attrib['link'] in descendants
        }
        if children <= descendants:
            return descendants
        descendants |= children


def arm_chain(robot, tip):
    parents = {
        joint.find('child').attrib['link']: (
            joint.find('parent').attrib['link'], joint.attrib['name']
        )
        for joint in robot.findall('joint')
    }
    chain = []
    link = tip
    while link != 'fr10_base_link':
        parent, joint = parents[link]
        chain.append(joint)
        link = parent
    return list(reversed(chain))


class SingleFr10ModelContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.standalone = expand(False)
        cls.mounted = expand(True)

    def test_both_profiles_use_fr10_base_and_charging_tcp(self):
        for robot in (self.standalone, self.mounted):
            with self.subTest(profile='mounted' if robot is self.mounted else 'standalone'):
                links = {link.attrib['name'] for link in robot.findall('link')}
                self.assertEqual(robot.attrib['name'], 'complete_visual')
                self.assertTrue(ARM_GEOMETRY_LINKS | {'charging_tcp', 'charging_connector_link'} <= links)
                chain = arm_chain(robot, 'charging_tcp')
                self.assertEqual([joint for joint in chain if joint.startswith('j')],
                                 [f'j{index}' for index in range(1, 7)])
                self.assertTrue({'camera_link', 'camera_depth_optical_frame'} <= arm_subtree(robot))

    def test_arm_meshes_resolve_from_vehicle_description(self):
        for robot in (self.standalone, self.mounted):
            with self.subTest(profile='mounted' if robot is self.mounted else 'standalone'):
                links = {link.attrib['name']: link for link in robot.findall('link')}
                for name in ARM_GEOMETRY_LINKS | {'charging_connector_link'}:
                    meshes = links[name].findall('.//mesh')
                    self.assertTrue(meshes, f'{name} has no mesh in the installed model')
                    for mesh in meshes:
                        uri = mesh.attrib['filename']
                        self.assertTrue(
                            uri.startswith('package://vehicle_description/meshes/'),
                            f'{name} resolves outside the sole description package: {uri}',
                        )
                        asset = DESCRIPTION / uri.removeprefix('package://vehicle_description/')
                        self.assertTrue(asset.is_file(), f'{name} mesh is missing: {asset}')

    def test_moveit_semantics_match_both_expanded_profiles(self):
        semantics = list((DESCRIPTION / 'config').glob('*.srdf'))
        self.assertEqual(len(semantics), 1, 'vehicle_description must own one FR10 SRDF')
        semantic = ET.parse(semantics[0]).getroot()
        self.assertEqual(semantic.attrib['name'], 'complete_visual')
        group = semantic.find("group[@name='fairino10_v6_group']")
        self.assertIsNotNone(group)
        chain = group.find('chain')
        self.assertIsNotNone(chain)
        self.assertEqual(chain.attrib, {
            'base_link': 'fr10_base_link', 'tip_link': 'charging_tcp',
        })
        for robot in (self.standalone, self.mounted):
            with self.subTest(profile='mounted' if robot is self.mounted else 'standalone'):
                arm_links = arm_subtree(robot)
                self.assertTrue({'fr10_base_link', 'charging_tcp'} <= arm_links)
                for disabled in semantic.findall('disable_collisions'):
                    self.assertIn(disabled.attrib['link1'], arm_links)
                    self.assertIn(disabled.attrib['link2'], arm_links)

    def test_fake_fairino_and_gazebo_profiles_share_geometry_and_one_arm_owner(self):
        positions = MOVEIT_CONFIG / 'initial_positions.yaml'
        for backend, expected_plugin in (
            ('fake', 'mock_components/GenericSystem'),
            ('fairino', 'fairino_hardware_v3_9_3/FairinoHardwareInterface'),
            ('gazebo', 'gazebo_ros2_control/GazeboSystem'),
        ):
            with self.subTest(backend=backend):
                model = ET.fromstring(xacro.process_file(
                    str(DESCRIPTION / 'urdf' / 'vehicle.urdf.xacro'),
                    mappings={
                        'enable_vehicle': 'false', 'enable_arm': 'true',
                        'control_backend': backend,
                        'initial_positions_file': str(positions),
                        'gazebo_controller_yaml': '/tmp/arm_controllers.yaml',
                    },
                ).toxml())
                self.assertEqual(model.attrib['name'], 'complete_visual')
                self.assertEqual(
                    {link.attrib['name'] for link in model.findall('link')},
                    {link.attrib['name'] for link in self.standalone.findall('link')},
                )
                self.assertEqual(len(model.findall('ros2_control')), 1)
                control = model.find('ros2_control')
                self.assertEqual(control.find('hardware/plugin').text, expected_plugin)
                self.assertEqual(
                    [joint.attrib['name'] for joint in control.findall('joint')],
                    [f'j{index}' for index in range(1, 7)],
                )
                self.assertEqual(arm_chain(model, 'charging_tcp'),
                                 arm_chain(self.standalone, 'charging_tcp'))

    def test_gazebo_base_collision_profile_is_shared(self):
        simulation = DESCRIPTION.parent / 'vehicle_bringup' / 'config' / 'simulation.xacro'
        geometries = []
        for vehicle in ('false', 'true'):
            model = ET.fromstring(xacro.process_file(
                str(simulation), mappings={
                    'enable_vehicle': vehicle, 'enable_arm': 'true',
                    'controller_yaml_file': '/tmp/controllers.yaml',
                    'sim_base_collision_proxy': 'true',
                }).toxml())
            collision = model.find("link[@name='fr10_base_link']/collision")
            self.assertIsNotNone(collision.find('geometry/cylinder'))
            geometries.append(ET.canonicalize(ET.tostring(collision, encoding='unicode')))
        self.assertEqual(geometries[0], geometries[1])
        physical = self.standalone.find("link[@name='fr10_base_link']/collision")
        self.assertIsNotNone(physical.find('geometry/mesh'))

    def test_force_sensor_is_gazebo_only_and_opt_in(self):
        positions = MOVEIT_CONFIG / 'initial_positions.yaml'
        common = {
            'enable_vehicle': 'false', 'enable_arm': 'true',
            'control_backend': 'gazebo',
            'initial_positions_file': str(positions),
            'gazebo_controller_yaml': '/tmp/arm_controllers.yaml',
        }
        default_model = ET.fromstring(xacro.process_file(
            str(DESCRIPTION / 'urdf' / 'vehicle.urdf.xacro'),
            mappings=common,
        ).toxml())
        self.assertIsNone(default_model.find("link[@name='ft_sensor_link']"))
        self.assertEqual(
            default_model.findall("gazebo/plugin[@filename='libgazebo_ros_ft_sensor.so']"),
            [],
        )

        force_model = ET.fromstring(xacro.process_file(
            str(DESCRIPTION / 'urdf' / 'vehicle.urdf.xacro'),
            mappings={**common, 'enable_force_sensor': 'true'},
        ).toxml())
        self.assertIsNotNone(force_model.find("link[@name='ft_sensor_link']"))
        self.assertIsNotNone(force_model.find("joint[@name='plate_to_ft_sensor']"))
        self.assertEqual(
            force_model.find("joint[@name='ft_sensor_to_connector']").attrib['type'],
            'fixed',
        )
        self.assertIsNotNone(force_model.find("joint[@name='connector_to_tcp']"))
        gazebo_joint = force_model.find("gazebo[@reference='ft_sensor_to_connector']")
        self.assertIsNotNone(gazebo_joint)
        self.assertEqual(gazebo_joint.findtext('disableFixedJointLumping'), 'true')
        plugins = force_model.findall(
            "gazebo/plugin[@filename='libgazebo_ros_ft_sensor.so']"
        )
        self.assertEqual(len(plugins), 1)
        self.assertEqual(plugins[0].findtext('joint_name'),
                         'ft_sensor_to_connector')
        # Unlimited sensor publication retains every physical substep, even
        # when the numerical integration is refined beyond 1 kHz.
        self.assertEqual(plugins[0].findtext('update_rate'), '0')
        self.assertEqual(plugins[0].findall('ros/remapping'), [])

    def test_force_world_has_gravity_and_test_socket(self):
        world_path = (DESCRIPTION.parent / 'fr10_arm_control' /
                      'fairino10_v6_moveit2_config' / 'worlds' /
                      'fr10_force_test.world')
        world = ET.parse(world_path).getroot().find('world')
        self.assertEqual(world.findtext('gravity'), '0 0 -9.80665')
        socket = world.find("model[@name='fr10_force_insertion_socket']")
        self.assertIsNotNone(socket)
        self.assertEqual(len(socket.findall('link/collision')), 5)
        self.assertEqual(
            socket.findtext('pose'),
            '0.55 0 0.0938 -3.14159265359 0 -1.57079632679',
        )
        cad_visual = socket.find(
            "link[@name='socket']/visual[@name='charging_port_cad_visual']"
        )
        self.assertIsNotNone(cad_visual)
        self.assertEqual(
            cad_visual.findtext('geometry/mesh/uri'),
            'model://cong_sac/meshes/cong_sac.stl',
        )
        self.assertEqual(
            cad_visual.findtext('geometry/mesh/scale'), '0.001 0.001 0.001'
        )
        self.assertIsNotNone(socket.find("link[@name='charging_port_tcp']"))
        tcp_joint = socket.find(
            "joint[@name='socket_to_charging_port_tcp']"
        )
        self.assertIsNotNone(tcp_joint)
        self.assertEqual(tcp_joint.findtext('parent'), 'socket')
        self.assertEqual(tcp_joint.findtext('child'), 'charging_port_tcp')

    def test_force_proxies_are_not_rendered_but_keep_their_physics(self):
        worlds = MOVEIT_CONFIG.parent / 'worlds'
        home = ET.parse(worlds / 'fr10_force_home_test.world').getroot().find('world')
        plate = home.find("model[@name='force_contact_proxy']/link[@name='plate']")
        self.assertEqual(plate.findall('visual'), [])
        self.assertEqual(plate.findtext('collision/geometry/box/size'), '0.30 0.30 0.01')
        self.assertEqual(plate.findtext('pose'), '0 0 0.005 0 0 0')
        spring = home.find("model[@name='force_contact_proxy']/joint[@name='compliance']")
        self.assertEqual(spring.findtext('axis/dynamics/spring_stiffness'), '2000')
        self.assertEqual(spring.findtext('axis/dynamics/damping'), '80')
        self.assertIsNotNone(home.find("model[@name='charging_port_visual']/link/visual/geometry/mesh"))

        socket = ET.parse(worlds / 'fr10_force_test.world').getroot().find(
            "world/model[@name='fr10_force_insertion_socket']/link[@name='socket']")
        self.assertEqual([visual.attrib['name'] for visual in socket.findall('visual')],
                         ['charging_port_cad_visual'])
        rear = socket.find("collision[@name='rear_plate_collision']")
        self.assertEqual(rear.findtext('pose'), '0 0 0.055 0 0 0')
        self.assertEqual(rear.findtext('geometry/box/size'), '0.18 0.20 0.01')
        self.assertEqual(len(socket.findall('collision')), 5)

    def test_force_port_rear_face_rests_on_ground_for_both_profiles(self):
        # Check the actual supplied triangles after scale + visual/model pose,
        # rather than assuming that the STL origin is its bottom surface.
        mesh_path = DESCRIPTION / 'meshes/cong_sac/meshes/cong_sac.stl'
        vertices = []
        with mesh_path.open('rb') as stream:
            stream.read(80)
            triangle_count = struct.unpack('<I', stream.read(4))[0]
            for _ in range(triangle_count):
                triangle = struct.unpack('<12fH', stream.read(50))
                for start in (3, 6, 9):
                    vertices.append(triangle[start:start + 3])

        def transform(vertex, pose):
            x, y, z, roll, pitch, yaw = pose
            vx, vy, vz = vertex
            vy, vz = (math.cos(roll) * vy - math.sin(roll) * vz,
                      math.sin(roll) * vy + math.cos(roll) * vz)
            vx, vz = (math.cos(pitch) * vx + math.sin(pitch) * vz,
                      -math.sin(pitch) * vx + math.cos(pitch) * vz)
            vx, vy = (math.cos(yaw) * vx - math.sin(yaw) * vy,
                      math.sin(yaw) * vx + math.cos(yaw) * vy)
            return (vx + x, vy + y, vz + z)

        world_directory = (DESCRIPTION.parent / 'fr10_arm_control' /
                           'fairino10_v6_moveit2_config' / 'worlds')
        for filename, model_name in (
            ('fr10_force_home_test.world', 'charging_port_visual'),
            ('fr10_force_test.world', 'fr10_force_insertion_socket'),
        ):
            with self.subTest(world=filename):
                world = ET.parse(world_directory / filename).getroot().find('world')
                ground = world.find("model[@name='ground_plane']")
                self.assertIsNotNone(ground)
                self.assertIsNotNone(ground.find('link/collision/geometry/plane'))
                model = world.find(f"model[@name='{model_name}']")
                model_pose = [float(value) for value in model.findtext('pose').split()]
                visual = model.find('link/visual')
                visual_pose = [float(value) for value in visual.findtext('pose').split()]
                scale = [float(value) for value in visual.findtext('geometry/mesh/scale').split()]
                heights = [transform(transform(
                    tuple(value * factor for value, factor in zip(vertex, scale)),
                    visual_pose), model_pose)[2] for vertex in vertices]
                self.assertAlmostEqual(min(heights), 0.0, delta=1e-7)
                self.assertAlmostEqual(max(heights), 0.0938, delta=1e-6)

    def test_retired_vendor_and_moveit_model_copies_are_absent(self):
        retired = [
            VENDOR / 'urdf' / 'fairino10_v6.urdf',
            VENDOR / 'urdf' / 'fr10_end_effector.xacro',
            MOVEIT_CONFIG / 'fairino10_v6_robot.urdf.xacro',
            MOVEIT_CONFIG / 'fairino10_v6_robot.srdf',
        ]
        self.assertEqual([str(path) for path in retired if path.exists()], [])


if __name__ == '__main__':
    unittest.main()
