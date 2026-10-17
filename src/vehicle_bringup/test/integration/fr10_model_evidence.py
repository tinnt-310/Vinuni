"""Read live robot_description and fingerprint the common FR10 arm subtree."""

import hashlib
from pathlib import Path
import time
import xml.etree.ElementTree as ET


def fixed_link_transform(robot, parent, child):
    """Read an actual URDF fixed chain, rather than duplicating tool offsets."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    joints={joint.find('child').get('link'):joint for joint in robot.findall('joint')}
    transform=np.eye(4);seen=set()
    while child!=parent:
        if child in seen: raise ValueError('Cycle in fixed tool chain')
        seen.add(child);joint=joints[child]
        if joint.get('type')!='fixed': raise ValueError('Tool chain contains a moving joint')
        origin=joint.find('origin');link=np.eye(4)
        if origin is not None:
            xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
            if xyz.shape!=(3,) or rpy.shape!=(3,) or not np.all(np.isfinite(np.r_[xyz,rpy])):
                raise ValueError('Malformed fixed transform')
            link[:3,3]=xyz;link[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix()
        transform=link@transform;child=joint.find('parent').get('link')
    return transform


def arm_signature(robot_description):
    robot = ET.fromstring(robot_description)
    links = {element.attrib['name']: element for element in robot.findall('link')}
    joints = {element.attrib['name']: element for element in robot.findall('joint')}
    if 'fr10_base_link' not in links:
        raise ValueError('fr10_base_link missing from live robot_description')
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
    if not {'wrist3_link', 'charging_tcp'} <= descendants:
        raise ValueError('Shared FR10 wrist/TCP missing')
    if not {f'j{i}' for i in range(1, 7)} <= selected_joints:
        raise ValueError('FR10 six-joint chain missing')
    digest = hashlib.sha256()
    for kind, names, elements in (
        ('link', descendants, links), ('joint', selected_joints, joints)
    ):
        for name in sorted(names):
            digest.update(f'{kind}:{name}\n'.encode())
            digest.update(ET.canonicalize(
                ET.tostring(elements[name], encoding='unicode')).encode())
    return {'shared_arm_signature_sha256': digest.hexdigest(),
            'shared_arm_link_count': len(descendants),
            'shared_arm_joint_count': len(selected_joints),
            'model_robot_name': robot.attrib.get('name')}


def capture_live_model(node, output_dir, timeout_s=5.0):
    from rcl_interfaces.srv import GetParameters

    client = node.create_client(GetParameters,
                                '/robot_state_publisher/get_parameters')
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError('robot_state_publisher parameter service unavailable')
    future = client.call_async(GetParameters.Request(names=['robot_description']))
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)  # ProbeContext executor runs in another thread.
    if not future.done() or future.result() is None:
        raise RuntimeError('robot_description parameter request timed out')
    values = future.result().values
    if not values or not values[0].string_value.strip().startswith('<'):
        raise RuntimeError('Live robot_description is missing or not XML')
    description = values[0].string_value
    Path(output_dir, 'model.urdf').write_text(description)
    planning_client = node.create_client(GetParameters,'/fr10_pose_controller/get_parameters')
    if not planning_client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError('Planning model parameter service unavailable')
    planning_future = planning_client.call_async(GetParameters.Request(names=['robot_description']))
    deadline=time.monotonic()+timeout_s
    while not planning_future.done() and time.monotonic()<deadline:time.sleep(.02)
    if not planning_future.done() or planning_future.result() is None:
        raise RuntimeError('Planning model parameter request timed out')
    planning_description=planning_future.result().values[0].string_value
    Path(output_dir,'planning_model.urdf').write_text(planning_description)
    return {'live_model_sha256': hashlib.sha256(description.encode()).hexdigest(),
            'planning_model_sha256':hashlib.sha256(planning_description.encode()).hexdigest(),
            **arm_signature(description)}
