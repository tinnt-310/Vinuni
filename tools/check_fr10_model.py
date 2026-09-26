"""Geometry and link audit. Run with numpy/trimesh/pycollada after xacro export."""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh

workspace = Path(__file__).resolve().parents[1]
robot = ET.parse(sys.argv[1]).getroot()
links = {e.attrib['name']: e for e in robot.findall('link')}
joints = robot.findall('joint')
assert len(links) == len(robot.findall('link')), 'duplicate link names'
assert len({j.attrib['name'] for j in joints}) == len(joints), 'duplicate joints'
parents = {j.find('child').attrib['link']: j for j in joints}
assert len(parents) == len(joints), 'multiple parents'
for j in joints:
    assert j.find('parent').attrib['link'] in links
    assert j.find('child').attrib['link'] in links

def origin(e):
    e = e.find('origin')
    if e is None:
        return np.eye(4)
    m = trimesh.transformations.euler_matrix(*map(float, e.get('rpy','0 0 0').split()))
    m[:3,3] = list(map(float,e.get('xyz','0 0 0').split()))
    return m

def tf(parent, child):
    if parent == child:
        return np.eye(4)
    j = parents[child]
    return tf(parent, j.find('parent').attrib['link']) @ origin(j)

def resolve(uri):
    if uri.startswith('file://'):
        return Path(uri[7:])
    assert uri.startswith('package://'), uri
    package, rel = uri[10:].split('/',1)
    for base in [workspace/'install'/package/'share'/package,Path('/opt/ros/humble/share')/package]:
        if (base/rel).is_file():
            return base/rel
    raise AssertionError('Missing mesh: '+uri)

for mesh in robot.findall('.//mesh'):
    assert resolve(mesh.attrib['filename']).is_file()

# Compare full moving-joint XML, permitting only the root mount rename.
source = ET.parse(workspace/'src/vehicle_description/fairino_description/urdf/fairino10_v6.urdf').getroot()
def normalized(e):
    return (e.tag,dict(e.attrib), (e.text or '').strip(),[normalized(c) for c in e])
for j in source.findall('joint'):
    if j.find('parent').get('link') == 'base_link':
        j.find('parent').set('link','fr10_base_link')
    actual = next(k for k in joints if k.get('name') == j.get('name'))
    assert normalized(actual) == normalized(j), 'FR10 joint changed: '+j.get('name')
assert len(robot.findall('ros2_control')) == 1
assert len(robot.findall('ros2_control/joint')) == 14
np.testing.assert_allclose(tf('wrist3_link','plate_frame')[:3,3],[0,0,.106],atol=1e-8)
np.testing.assert_allclose(tf('plate_frame','charging_connector_link')[:3,3],[0,.09,.09],atol=1e-8)
np.testing.assert_allclose(tf('plate_frame','charging_connector_link')[:3,:3],np.eye(3),atol=1e-8)
# Supports must remain attached by fixed joints and have physical properties.
support_names = ['wrist_mount_backplate', 'camera_support_left', 'camera_support_right',
                 'camera_rear_mount', 'charger_support_left', 'charger_support_right',
                 'charger_rear_mount', 'charger_cradle_left', 'charger_cradle_right']
for name in support_names:
    assert parents[name].get('type') == 'fixed'
    assert links[name].find('collision') is not None
    assert float(links[name].find('inertial/mass').get('value')) > 0

visual = links['camera_link'].find('visual')
mesh = trimesh.load(str(resolve(visual.find('geometry/mesh').get('filename'))),force='mesh')
mesh.apply_transform(tf('plate_frame','camera_link') @ origin(visual))
print('D435 actual visual bounds in plate (m):',mesh.bounds)
np.testing.assert_allclose(mesh.bounds[1,1],0,atol=1e-7,err_msg='camera top')
np.testing.assert_allclose(mesh.bounds[0,2],.030,atol=1e-7,err_msg='camera rear')
np.testing.assert_allclose(mesh.bounds[:,0].mean(),0,atol=1e-7,err_msg='camera centered X')
np.testing.assert_allclose(tf('plate_frame','camera_depth_optical_frame')[:3,2],[0,0,1],atol=1e-8)
np.testing.assert_allclose(tf('plate_frame','camera_link')[:3,2],[0,1,0],atol=1e-8)
print('PASS: joint preservation, paths, tree, plate, connector, camera geometry and optical direction')
