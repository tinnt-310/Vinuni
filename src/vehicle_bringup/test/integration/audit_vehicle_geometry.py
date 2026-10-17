"""Audit expanded URDF and converted SDF; report, never modify model parameters."""
import argparse
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def pose(element, sdf=False):
    if sdf:
        values = list(map(float, element.findtext('pose', '0 0 0 0 0 0').split()))
    else:
        origin = element.find('origin')
        values = ([0.] * 6 if origin is None else
                  list(map(float, origin.get('xyz', '0 0 0').split())) +
                  list(map(float, origin.get('rpy', '0 0 0').split())))
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def inertial(link, sdf=False):
    data = link.find('inertial')
    if data is None:
        return None
    mass = float(data.findtext('mass') if sdf else data.find('mass').get('value'))
    tensor = data.find('inertia')
    def value(key):
        return float(tensor.findtext(key, '0') if sdf else tensor.get(key, '0'))
    matrix = np.array([[value('ixx'), value('ixy'), value('ixz')],
                       [value('ixy'), value('iyy'), value('iyz')],
                       [value('ixz'), value('iyz'), value('izz')]])
    eig = np.linalg.eigvalsh(matrix)
    return dict(mass=mass, com=pose(data, sdf)[:3, 3].tolist(), tensor=matrix.tolist(),
                eigenvalues=eig.tolist(), valid=bool(np.isfinite(matrix).all()
                    and np.isfinite(mass) and mass > 0 and min(eig) > 0
                    and eig[-1] <= eig[0] + eig[1] + 1e-9))


def audit(directory):
    directory = Path(directory)
    result = subprocess.run(['gz', 'sdf', '-p', str(directory / 'model.urdf')],
                            text=True, capture_output=True, check=True)
    (directory / 'model.sdf').write_text(result.stdout)
    (directory / 'sdf_conversion.log').write_text(result.stderr)
    urdf = ET.parse(directory / 'model.urdf').getroot()
    transforms = {link.get('name'): np.eye(4) for link in urdf.findall('link')}
    joints = urdf.findall('joint')
    children = {j.find('child').get('link') for j in joints}
    roots = set(transforms) - children
    resolved = set(roots)
    while len(resolved) < len(transforms):
        before = len(resolved)
        for joint in joints:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent in resolved:
                transforms[child] = transforms[parent] @ pose(joint)
                resolved.add(child)
        if len(resolved) == before:
            raise ValueError('Unresolved or cyclic URDF joint graph')
    collisions = []
    for link in urdf.findall('link'):
        for c in link.findall('collision'):
            transform = transforms[link.get('name')] @ pose(c)
            geometry = c.find('geometry')[0]
            half = None
            if geometry.tag == 'box':
                half = abs(transform[:3, :3]) @ (np.array(list(map(float, geometry.get('size').split()))) / 2)
            elif geometry.tag == 'cylinder':
                axis = transform[:3, 2]
                half = abs(axis) * float(geometry.get('length')) / 2 + float(geometry.get('radius')) * np.sqrt(np.maximum(0, 1-axis*axis))
            elif geometry.tag == 'sphere':
                half = np.ones(3) * float(geometry.get('radius'))
            center = transform[:3, 3]
            collisions.append(dict(link=link.get('name'), geometry=geometry.tag,
                attributes=geometry.attrib, origin=pose(c).tolist(),
                minimum=None if half is None else (center-half).tolist(),
                maximum=None if half is None else (center+half).tolist(),
                note='Mesh bounds require separate mesh audit' if half is None else 'Exact primitive AABB at zero joint angles'))
    sdf = ET.fromstring(result.stdout)
    report = dict(roots=sorted(roots), collisions=collisions,
        urdf_inertia={l.get('name'):inertial(l) for l in urdf.findall('link')},
        sdf_inertia={l.get('name'):inertial(l, True) for l in sdf.findall('.//model/link')},
        joints=[dict(name=j.get('name'), type=j.get('type'),
                     fields={c.tag:c.attrib for c in j}) for j in joints])
    (directory / 'geometry_audit.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(dict(collisions=len(collisions),
          unresolved_mesh_bounds=sum(c['minimum'] is None for c in collisions),
          invalid_urdf_inertia=[k for k,v in report['urdf_inertia'].items() if v and not v['valid']],
          invalid_sdf_inertia=[k for k,v in report['sdf_inertia'].items() if v and not v['valid']]), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    audit(parser.parse_args().directory)
