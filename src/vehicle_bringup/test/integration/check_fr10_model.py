#!/usr/bin/env python3
"""Audit expanded FR10 URDFs against the pre-migration mechanical fingerprint.

Usage: check_fr10_model.py ARM.urdf [--compare MOUNTED.urdf]

The fingerprint in docs/model_migration/before.json freezes the seven FR10
rigid links, six moving joints, and seven CAD meshes before their move into
vehicle_description. Mesh URIs are reduced to their basename for XML hashes so
the description-package move does not change the mechanical signature.
"""

import argparse
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET

import numpy as np
import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
FINGERPRINT = WORKSPACE / 'docs/model_migration/before.json'
DESCRIPTION_SOURCE = WORKSPACE / 'src/vehicle_description'


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def xml_hash(element, mesh_basename_only=False, normalize_sim_proxy=False):
    # Cloning avoids changing the parsed model used by later geometry checks.
    element = ET.fromstring(ET.tostring(element, encoding='unicode'))
    if normalize_sim_proxy and element.tag == 'link' and element.get('name') == 'fr10_base_link':
        collision = element.find('collision')
        if collision is not None and collision.find('geometry/cylinder') is not None:
            cylinder = collision.find('geometry/cylinder')
            pose = collision.find('origin')
            require(cylinder.attrib == {'radius': '0.095', 'length': '0.0838'} and
                    pose is not None and pose.get('xyz') == '0 0 0.0429' and
                    pose.get('rpy') == '0 0 0',
                    'Simulation FR10 base collision proxy changed')
            index = list(element).index(collision)
            element.remove(collision)
            collision = ET.Element('collision')
            ET.SubElement(collision, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(geometry, 'mesh', {'filename': 'base_link.STL'})
            element.insert(index, collision)
    if mesh_basename_only:
        for mesh in element.findall('.//mesh'):
            mesh.set('filename', PurePosixPath(mesh.get('filename', '')).name)
    canonical = ET.canonicalize(ET.tostring(element, encoding='unicode'))
    return hashlib.sha256(canonical.encode()).hexdigest()


def resolve(uri):
    if uri.startswith('file://'):
        path = Path(uri[7:])
        require(path.is_file(), f'Missing mesh: {uri}')
        return path
    require(uri.startswith('package://'), f'Unsupported mesh URI: {uri}')
    package, separator, relative = uri[len('package://'):].partition('/')
    require(separator and package and relative, f'Invalid package mesh URI: {uri}')
    candidates = []
    if package == 'vehicle_description':
        candidates.append(DESCRIPTION_SOURCE)
    for install_dir in ('install_grouped', 'install'):
        candidates.append(WORKSPACE / install_dir / package / 'share' / package)
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / 'share' / package)
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory(package)))
    except Exception:
        # A source-tree check can run without an initialized ROS environment.
        pass
    candidates.append(Path('/opt/ros/humble/share') / package)
    for base in candidates:
        path = base / relative
        if path.is_file():
            return path
    raise AssertionError(f'Missing mesh: {uri}')


@lru_cache(maxsize=4)
def collada_positions(path):
    """Read POSITION vertices from the official D435 DAE without trimesh."""
    root = ET.parse(path).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    up_axis = root.find('.//c:asset/c:up_axis', ns)
    unit = root.find('.//c:asset/c:unit', ns)
    require(up_axis is not None and up_axis.text == 'Z_UP' and
            unit is not None and float(unit.get('meter')) == 1.0,
            'Unexpected D435 Collada coordinate system or units')
    groups = []
    for geometry in root.findall('.//c:library_geometries/c:geometry', ns):
        mesh = geometry.find('c:mesh', ns)
        if mesh is None:
            continue
        for vertices in mesh.findall('c:vertices', ns):
            for item in vertices.findall('c:input', ns):
                if item.get('semantic') != 'POSITION':
                    continue
                source_id = item.get('source', '')[1:]
                vertex_source = mesh.find(f"c:source[@id='{source_id}']", ns)
                require(vertex_source is not None, f'D435 vertex source missing: {source_id}')
                float_array = vertex_source.find('c:float_array', ns)
                require(float_array is not None, f'D435 position array missing: {source_id}')
                values = np.fromstring(float_array.text or '', sep=' ')
                require(values.size > 0 and values.size % 3 == 0 and np.isfinite(values).all(),
                        f'Invalid D435 position array: {source_id}')
                groups.append(values.reshape(-1, 3))
    require(groups, 'No D435 POSITION vertices found')
    return np.concatenate(groups)


def origin(element):
    pose = element.find('origin')
    if pose is None:
        return np.eye(4)
    roll, pitch, yaw = map(float, pose.get('rpy', '0 0 0').split())
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    transform = np.eye(4)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = list(map(float, pose.get('xyz', '0 0 0').split()))
    return transform


def transform(parents, parent, child):
    if parent == child:
        return np.eye(4)
    require(child in parents, f'{child} is not descended from {parent}')
    joint = parents[child]
    return transform(parents, parent, joint.find('parent').get('link')) @ origin(joint)


def arm_subtree(links, joints):
    children = {}
    for joint in joints:
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    result = {}
    pending = ['fr10_base_link']
    while pending:
        parent = pending.pop()
        require(parent in links, f'FR10 descendant link missing: {parent}')
        result[f'link:{parent}'] = xml_hash(links[parent])
        for joint in children.get(parent, []):
            result[f'joint:{joint.get("name")}'] = xml_hash(joint)
            pending.append(joint.find('child').get('link'))
    require('link:charging_tcp' in result, 'charging_tcp is not in the FR10 tree')
    return result


def audit(path, baseline):
    robot = ET.parse(path).getroot()
    require(robot.tag == 'robot', 'Expected expanded URDF robot root')
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = robot.findall('joint')
    require(len(links) == len(robot.findall('link')), 'Duplicate link names')
    require(len({joint.get('name') for joint in joints}) == len(joints), 'Duplicate joint names')
    parents = {}
    for joint in joints:
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        require(parent in links and child in links, f'Joint has unknown endpoint: {joint.get("name")}')
        require(child not in parents, f'Multiple parents for {child}')
        parents[child] = joint
    require('fr10_base_link' in links and 'charging_tcp' in links, 'Shared FR10 base/TCP missing')
    require('tool0' not in links, 'Legacy tool0 model is present')

    mesh_paths = {}
    for mesh in robot.findall('.//mesh'):
        uri = mesh.get('filename')
        mesh_paths[uri] = resolve(uri)
    expected_meshes = baseline['mesh_sha256']
    for basename, expected in expected_meshes.items():
        uri = f'package://vehicle_description/meshes/fairino10_v6/{basename}'
        require(uri in mesh_paths, f'FR10 mesh is not owned by vehicle_description: {basename}')
        actual = hashlib.sha256(mesh_paths[uri].read_bytes()).hexdigest()
        require(actual == expected, f'FR10 CAD mesh changed: {basename}')

    elements = {f'link:{name}': link for name, link in links.items()}
    elements.update({f'joint:{joint.get("name")}': joint for joint in joints})
    for key, expected in baseline['core_element_hashes_mesh_basename_only'].items():
        require(key in elements, f'FR10 mechanical element missing: {key}')
        actual = xml_hash(elements[key], mesh_basename_only=True, normalize_sim_proxy=True)
        require(actual == expected, f'FR10 mechanical element changed: {key}')

    # One active command owner for each moving FR10 joint in controlled models.
    systems = robot.findall('ros2_control')
    require(len(systems) <= 1, 'Multiple ros2_control systems claim the robot')
    if systems:
        controlled = [joint.get('name') for joint in systems[0].findall('joint')]
        require(len(controlled) == len(set(controlled)), 'Duplicate ros2_control joint claim')
        require(all(controlled.count(f'j{i}') == 1 for i in range(1, 7)),
                'ros2_control must claim each FR10 moving joint exactly once')

    for name in ('plate_frame', 'charging_connector_link', 'camera_link',
                 'camera_depth_optical_frame', 'd435_mount_frame'):
        require(name in links, f'D435/CCS2 link missing: {name}')
    require('jlr_ccs2_centered.stl' == PurePosixPath(
        links['charging_connector_link'].find('visual/geometry/mesh').get('filename')).name,
        'Charging connector must use the CCS2 CAD mesh')
    camera_uri = links['camera_link'].find('visual/geometry/mesh').get('filename')
    require(camera_uri == 'package://realsense2_description/meshes/d435.dae',
            'Wrist camera must use the D435 CAD mesh')
    np.testing.assert_allclose(transform(parents, 'wrist3_link', 'plate_frame')[:3, 3],
                               [0, 0, .106], atol=1e-8)
    datum = yaml.safe_load((DESCRIPTION_SOURCE / 'config/fr10_tool.yaml').read_text())
    expected_tcp = ET.Element('joint')
    ET.SubElement(expected_tcp, 'origin',
                  xyz=' '.join(map(str, datum['translation_m'])),
                  rpy=' '.join(str(np.deg2rad(v)) for v in datum['rotation_fixed_xyz_deg']))
    np.testing.assert_allclose(transform(parents, 'plate_frame', 'charging_tcp'),
                               origin(expected_tcp), atol=1e-8)
    np.testing.assert_allclose(transform(parents, 'charging_connector_link', 'charging_tcp')[:3, 3],
                               [0, 0, datum['legacy_mesh_front_z_m']], atol=1e-8)
    np.testing.assert_allclose(transform(parents, 'plate_frame', 'charging_connector_link')[:3, :3],
                               origin(expected_tcp)[:3, :3], atol=1e-8)
    plate_to_connector = transform(parents, 'plate_frame', 'charging_connector_link')
    def from_connector(frame):
        return np.linalg.inv(plate_to_connector) @ transform(parents, 'plate_frame', frame)
    np.testing.assert_allclose(from_connector('camera_depth_optical_frame')[:3, 2],
                               [0, 0, 1], atol=1e-8)
    np.testing.assert_allclose(from_connector('camera_link')[:3, 2],
                               [0, 1, 0], atol=1e-8)
    camera_visual = links['camera_link'].find('visual')
    visual_tf = transform(parents, 'plate_frame', 'camera_link') @ origin(camera_visual)
    vertices = collada_positions(mesh_paths[camera_uri])
    positions_in_plate = vertices @ visual_tf[:3, :3].T + visual_tf[:3, 3]
    bounds = np.stack((positions_in_plate.min(axis=0), positions_in_plate.max(axis=0)))
    camera_mount = yaml.safe_load((DESCRIPTION_SOURCE / 'config/fr10_camera_mount.yaml').read_text())
    centre = visual_tf[:3, :3] @ ((vertices.min(axis=0)+vertices.max(axis=0))/2) + visual_tf[:3, 3]
    np.testing.assert_allclose(centre[:2], camera_mount['housing_center_xy_in_flange_m'],
                               atol=1e-7, err_msg='D435 centre on flange axis')
    np.testing.assert_allclose(bounds[0, 2], camera_mount['rear_face_from_flange_m'],
                               atol=1e-7, err_msg='D435 rear face from flange')

    supports = ('wrist_mount_backplate', 'camera_support_left', 'camera_support_right',
                'camera_rear_mount', 'charger_support_left', 'charger_support_right',
                'charger_rear_mount', 'charger_cradle_left', 'charger_cradle_right')
    for name in supports:
        require(parents[name].get('type') == 'fixed', f'{name} is not fixed')
        require(links[name].find('collision') is not None, f'{name} has no collision')
        require(float(links[name].find('inertial/mass').get('value')) > 0,
                f'{name} has invalid mass')

    return {
        'path': str(path),
        'links': len(links),
        'joints': len(joints),
        'control_systems': len(systems),
        'core_fingerprint_elements': len(baseline['core_element_hashes_mesh_basename_only']),
        'cad_meshes_verified': len(expected_meshes),
        'arm_subtree': arm_subtree(links, joints),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path, help='expanded FR10 URDF')
    parser.add_argument('--compare', type=Path, help='expanded second profile, e.g. mounted')
    args = parser.parse_args()
    baseline = json.loads(FINGERPRINT.read_text())
    primary = audit(args.urdf, baseline)
    if args.compare:
        second = audit(args.compare, baseline)
        require(primary['arm_subtree'] == second['arm_subtree'],
                'FR10 arm subtree differs across the two profiles')
        print(f'PASS: shared FR10 arm subtree identical: {len(primary["arm_subtree"])} links/joints')
        print(f'PASS: second profile {args.compare}: {second["links"]} links, {second["joints"]} joints')
    print(f'PASS: {args.urdf}: {primary["core_fingerprint_elements"]} frozen mechanical elements, '
          f'{primary["cad_meshes_verified"]} CAD meshes, D435/CCS2/TCP geometry and tree')


if __name__ == '__main__':
    main()
