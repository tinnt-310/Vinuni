"""Standalone and mounted Gazebo profiles must share one FR10 arm model."""

from pathlib import Path
import xml.etree.ElementTree as ET

import xacro


SOURCE = Path(__file__).parents[1] / 'config' / 'simulation.xacro'


def expanded(vehicle_enabled):
    xml = xacro.process_file(str(SOURCE), mappings={
        'enable_vehicle': str(vehicle_enabled).lower(),
        'enable_arm': 'true',
        'controller_yaml_file': '/tmp/controllers.yaml',
    }).toxml()
    return ET.fromstring(xml)


def arm_signature(robot):
    links = {element.attrib['name']: element for element in robot.findall('link')}
    joints = {element.attrib['name']: element for element in robot.findall('joint')}
    descendants = {'fr10_base_link'}
    selected_joints = set()
    while True:
        added = False
        for name, joint in joints.items():
            parent = joint.find('parent').attrib['link']
            child = joint.find('child').attrib['link']
            if parent in descendants and child not in descendants:
                descendants.add(child)
                selected_joints.add(name)
                added = True
        if not added:
            break
    canonical = lambda element: ET.canonicalize(ET.tostring(element, encoding='unicode'))
    return (
        {name: canonical(links[name]) for name in descendants},
        {name: canonical(joints[name]) for name in selected_joints},
    )


def test_standalone_and_mounted_share_exact_arm_links_joints_and_tcp():
    standalone = expanded(False)
    mounted = expanded(True)
    arm_a = arm_signature(standalone)
    arm_c = arm_signature(mounted)
    assert arm_a == arm_c
    assert {'fr10_base_link', 'wrist3_link', 'charging_tcp'} <= arm_a[0].keys()
    assert {f'j{i}' for i in range(1, 7)} <= arm_a[1].keys()
    assert standalone.find("joint[@name='world_to_fr10']") is not None
    assert mounted.find("joint[@name='base_to_fr10']") is not None
    assert standalone.find("link[@name='base_footprint']") is None
    assert mounted.find("link[@name='base_footprint']") is not None


def test_disabling_arm_depth_sensor_preserves_physical_robot_and_camera_frames():
    enabled = expanded(False)
    disabled = ET.fromstring(xacro.process_file(str(SOURCE), mappings={
        'enable_vehicle': 'false', 'enable_arm': 'true',
        'enable_arm_camera': 'false', 'controller_yaml_file': '/tmp/controllers.yaml',
    }).toxml())
    assert arm_signature(disabled) == arm_signature(enabled)
    assert enabled.find(".//sensor[@name='arm_camera_sensor']") is not None
    assert disabled.find(".//sensor[@name='arm_camera_sensor']") is None


def test_simulation_has_one_robot_model_entry_point():
    source = SOURCE.read_text()
    assert source.count('vehicle.urdf.xacro') == 1
    assert 'arm_only.xacro' not in source


def test_precision_mount_observes_floating_physical_base_without_world_constraint():
    robot=ET.fromstring(xacro.process_file(str(SOURCE),mappings={
        'enable_vehicle':'true','enable_arm':'true',
        'charging_tool_profile':'matched_socket_v2',
        'controller_yaml_file':'/tmp/controllers.yaml'}).toxml())
    sensor=robot.find(".//plugin[@name='matched_base_feedback']")
    assert sensor is not None
    assert sensor.findtext('body_name')=='base_footprint'
    assert sensor.findtext('frame_name')=='world'
    assert float(sensor.findtext('update_rate'))==0  # every physics update
    assert robot.find(".//plugin[@name='wheel_contact_surface']/slip").text=='0.0'
    assert all(j.find('parent').attrib['link']!='world' for j in robot.findall('joint'))
    assert robot.find('.//static') is None
    systems=robot.findall('ros2_control')
    assert sorted(s.findtext('hardware/plugin') for s in systems)==[
        'fr10_simulation_hardware/GazeboTorqueSystem',
        'fr10_simulation_hardware/GazeboWheelMotorSystem']
    joint_owners=[j.attrib['name'] for s in systems for j in s.findall('joint')]
    assert len(joint_owners)==len(set(joint_owners))==14
    legacy=expanded(True)
    assert legacy.find(".//plugin[@name='matched_base_feedback']") is None
    assert float(legacy.find(".//plugin[@name='wheel_contact_surface']/slip").text)==1e-5
    assert legacy.findtext('ros2_control/hardware/plugin')=='gazebo_ros2_control/GazeboSystem'
