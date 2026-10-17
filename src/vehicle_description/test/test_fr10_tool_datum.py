"""The controller's front-face TCP must agree across every model consumer."""
import importlib.util
import itertools
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

DESCRIPTION = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = DESCRIPTION.parent
SPEC = importlib.util.spec_from_file_location(
    'tool_geometry', DESCRIPTION / 'launch/tool_geometry.py')
GEOMETRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEOMETRY)


def expand(profile, force=False, mounted=False, backend='none', simulation=False):
    entry = (WORKSPACE_SRC / 'vehicle_bringup/config/simulation.xacro' if simulation
             else DESCRIPTION / 'urdf/vehicle.urdf.xacro')
    return ET.fromstring(xacro.process_file(str(entry), mappings={
        'enable_vehicle': str(mounted).lower(), 'enable_arm': 'true',
        'charging_tool_profile': profile, 'enable_force_sensor': str(force).lower(),
        'control_backend': backend, 'controller_yaml_file': '/tmp/tool_controllers.yaml',
        'gazebo_controller_yaml': '/tmp/tool_controllers.yaml',
        'initial_positions_file': str(WORKSPACE_SRC /
            'fr10_arm_control/fairino10_v6_moveit2_config/config/initial_positions.yaml'),
    }).toxml())


@pytest.mark.parametrize('profile,force,mounted',
                         itertools.product(('legacy', 'matched_socket_v2'), (False, True), (False, True)))
def test_front_face_matches_confirmed_controller_tool1(profile, force, mounted):
    robot = expand(profile, force, mounted)
    actual = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', 'charging_tcp'))
    # Independent acceptance datum supplied by the user, not computed from YAML.
    np.testing.assert_allclose(actual[:3, 3], [.055140, -.046496, .215297], atol=1e-12)
    yaw = math.radians(136.840)
    expected = [[math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    np.testing.assert_allclose(actual[:3, :3], expected, atol=1e-12)
    # Connector geometry and collision normals must follow the TCP clocking.
    body = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', 'charging_connector_link'))
    np.testing.assert_allclose(body[:3, :3], actual[:3, :3], atol=1e-12)
    offset = (json_geometry()['simulation_design']['tcp_connector_z_m']
              if profile == 'matched_socket_v2' else .0625)
    face = body[:3, 3] + body[:3, :3] @ [0, 0, offset]
    np.testing.assert_allclose(face, actual[:3, 3], atol=1e-12)
    mount = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', 'charging_mount_frame'))
    np.testing.assert_allclose(mount[:3, :3], actual[:3, :3], atol=1e-12)
    params = GEOMETRY.wrench_parameters(robot, 'charging_connector_link', 'charging_tcp')
    np.testing.assert_allclose(params['force.wrench_transform.target_to_source_m'],
                               [0, 0, -offset], atol=1e-12)


def json_geometry():
    import json
    return json.loads((DESCRIPTION / 'models/matched_socket_v2/geometry.json').read_text())


@pytest.mark.parametrize('profile', ('legacy', 'matched_socket_v2'))
def test_same_tool_transform_in_fake_fairino_and_gazebo(profile):
    transforms = []
    for backend in ('fake', 'fairino', 'gazebo'):
        robot = expand(profile, backend=backend)
        transforms.append(GEOMETRY.fixed_transform(robot, 'wrist3_link', 'charging_tcp'))
    for mounted in (False, True):
        robot = expand(profile, mounted=mounted, simulation=True)
        transforms.append(GEOMETRY.fixed_transform(robot, 'wrist3_link', 'charging_tcp'))
    for transform in transforms[1:]:
        np.testing.assert_allclose(transform, transforms[0], atol=1e-12)


def test_wrench_transform_rotates_force_and_preserves_correct_moment_arm():
    robot = '''<robot name="fixture"><link name="sensor"/><link name="tcp"/>
      <joint name="offset" type="fixed"><parent link="sensor"/><child link="tcp"/>
        <origin xyz="0.1 0.2 0.3" rpy="0 0 1.5707963267948966"/>
      </joint></robot>'''
    values = GEOMETRY.wrench_parameters(robot, 'sensor', 'tcp')
    rotation = np.asarray(values['force.wrench_transform.rotation_target_from_source']).reshape(3, 3)
    lever = np.asarray(values['force.wrench_transform.target_to_source_m'])
    np.testing.assert_allclose(rotation @ [1., 0., 0.], [0., -1., 0.], atol=1e-12)
    np.testing.assert_allclose(lever, [-.2, .1, -.3], atol=1e-12)
    # Force through the target TCP has zero moment after transformation.
    force_source = np.array([2., 3., 4.])
    torque_source = np.cross([.1, .2, .3], force_source)
    torque_target = rotation @ torque_source + np.cross(lever, rotation @ force_source)
    np.testing.assert_allclose(torque_target, [0., 0., 0.], atol=1e-12)
    identity = GEOMETRY.wrench_parameters(robot, 'tcp', 'tcp')
    np.testing.assert_allclose(identity['force.wrench_transform.target_to_source_m'], [0, 0, 0])
    with pytest.raises(ValueError):
        GEOMETRY.fixed_transform(robot, 'sensor', 'missing')
    moving = robot.replace('type="fixed"', 'type="revolute"')
    with pytest.raises(ValueError):
        GEOMETRY.fixed_transform(moving, 'sensor', 'tcp')


@pytest.mark.parametrize('profile', ('legacy', 'matched_socket_v2'))
def test_rotated_support_columns_clear_the_camera(profile):
    robot = expand(profile)

    def bounds(link_name):
        collision = robot.find(f"link[@name='{link_name}']/collision")
        half = np.fromstring(collision.find('geometry/box').get('size'), sep=' ') / 2
        pose = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', link_name))
        pose = pose @ np.asarray(GEOMETRY._origin(collision.find('origin')))
        corners = np.asarray(list(itertools.product((-1, 1), repeat=3))) * half
        points = corners @ pose[:3, :3].T + pose[:3, 3]
        return points.min(axis=0), points.max(axis=0)

    camera_min, camera_max = bounds('camera_link')
    for name in ('charger_support_left', 'charger_support_right'):
        low, high = bounds(name)
        assert np.any((low > camera_max + .001) | (high < camera_min - .001)), name


@pytest.mark.parametrize('profile,force,mounted',
                         itertools.product(('legacy', 'matched_socket_v2'), (False, True), (False, True)))
def test_camera_is_centred_on_flange_and_parallel_to_insertion(profile, force, mounted):
    import sys
    sys.path.insert(0, str(WORKSPACE_SRC / 'vehicle_bringup/test/integration'))
    from check_fr10_model import collada_positions
    robot = expand(profile, force, mounted)
    parents = {joint.find('child').get('link'): joint.find('parent').get('link')
               for joint in robot.findall('joint')}
    frame = 'camera_link'
    while frame != 'plate_frame':
        assert frame not in ('ft_sensor_link', 'charging_connector_link')
        frame = parents[frame]
    plate_to_tcp = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', 'charging_tcp'))
    def from_tcp(frame):
        return np.linalg.inv(plate_to_tcp) @ np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', frame))
    optical = from_tcp('camera_depth_optical_frame')
    np.testing.assert_allclose(optical[:3, 2], [0, 0, 1], atol=1e-12)
    camera = from_tcp('camera_link')
    # Long side along connector X; camera up along connector Y.
    np.testing.assert_allclose(camera[:3, 1], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(camera[:3, 2], [0, 1, 0], atol=1e-12)
    points = collada_positions('/opt/ros/humble/share/realsense2_description/meshes/d435.dae')
    visual = robot.find("link[@name='camera_link']/visual")
    plate_to_camera = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', 'camera_link'))
    visual_pose = plate_to_camera @ np.asarray(GEOMETRY._origin(visual.find('origin')))
    # Transform the mesh's own housing-box centre. A world-aligned AABB of
    # the rotated, asymmetric lens details is not the physical housing centre.
    local_centre = (points.max(axis=0) + points.min(axis=0)) / 2
    centre = visual_pose[:3, :3] @ local_centre + visual_pose[:3, 3]
    points = points @ visual_pose[:3, :3].T + visual_pose[:3, 3]
    # The CAD body centre, not camera_link's offset origin, must lie on the
    # flange axis. Tool 1's 55/-46 mm offset must not move the camera.
    np.testing.assert_allclose(centre[:2], [0., 0.], atol=1e-7)
    assert points[:, 2].min() == pytest.approx(.030, abs=1e-7)
    assert points[:, 2].max() == pytest.approx(.0550547, abs=1e-7)
    mount = robot.find("joint[@name='plate_to_camera_support']")
    assert mount.find('parent').get('link') == 'plate_frame'
    np.testing.assert_allclose(np.fromstring(mount.find('origin').get('xyz'), sep=' '), [0., 0., 0.])


@pytest.mark.parametrize('profile', ('legacy', 'matched_socket_v2'))
def test_camera_support_columns_are_seated_on_backplate(profile):
    robot = expand(profile)
    plate = robot.find("link[@name='wrist_mount_backplate']/collision/geometry/box")
    half = np.fromstring(plate.get('size'), sep=' ') / 2
    pose = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', 'wrist_mount_backplate'))
    for name in ('camera_support_left', 'camera_support_right'):
        box = robot.find(f"link[@name='{name}']/collision/geometry/box")
        column_half = np.fromstring(box.get('size'), sep=' ') / 2
        column = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', name))
        bottom = np.asarray(list(itertools.product((-1, 1), (-1, 1), (-1,)))) * column_half
        points = bottom @ column[:3, :3].T + column[:3, 3]
        np.testing.assert_allclose(points[:, 2], 0., atol=1e-12)
        assert np.all(points[:, :2] >= pose[:2, 3] - half[:2] - 1e-12)
        assert np.all(points[:, :2] <= pose[:2, 3] + half[:2] + 1e-12)
