"""Resolve fixed tool geometry from the expanded canonical robot description.

Runtime consumers must not copy flange/TCP or wrench moment-arm constants.
All distances here are metres, all URDF angles fixed-axis XYZ radians.
"""
import math
import xml.etree.ElementTree as ET


def _multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4))
             for j in range(4)] for i in range(4)]


def _identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def _origin(element):
    result = _identity()
    if element is None:
        return result
    xyz = [float(v) for v in element.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in element.get('rpy', '0 0 0').split()]
    if len(xyz) != 3 or len(rpy) != 3 or not all(map(math.isfinite, xyz + rpy)):
        raise ValueError('Malformed tool transform')
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    rotation = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                [-sp, cp*sr, cp*cr]]
    for i in range(3):
        result[i][:3] = rotation[i]
        result[i][3] = xyz[i]
    return result


def fixed_transform(description, parent, child):
    """Return parent_T_child; reject unknown frames or moving/cyclic chains."""
    robot = ET.fromstring(description) if isinstance(description, str) else description
    links = {link.get('name') for link in robot.findall('link')}
    if parent not in links or child not in links:
        raise ValueError(f'Unknown tool frame: {parent} -> {child}')
    joints = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    result, seen = _identity(), set()
    while child != parent:
        if child in seen or child not in joints:
            raise ValueError(f'No acyclic fixed chain: {parent} -> {child}')
        seen.add(child)
        joint = joints[child]
        if joint.get('type') != 'fixed':
            raise ValueError('Tool transform must not cross a moving joint')
        result = _multiply(_origin(joint.find('origin')), result)
        child = joint.find('parent').get('link')
    return result


def wrench_parameters(description, source_frame, target_frame):
    """Wrench rotation and target->source lever arm, both in target axes."""
    source_from_target = fixed_transform(description, source_frame, target_frame)
    rotation = [[source_from_target[j][i] for j in range(3)] for i in range(3)]
    translation = [-sum(rotation[i][j] * source_from_target[j][3]
                        for j in range(3)) for i in range(3)]
    return {
        'force.wrench_transform.rotation_target_from_source':
            [value for row in rotation for value in row],
        'force.wrench_transform.target_to_source_m': translation,
    }
