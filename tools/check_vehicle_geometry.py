"""Validate expanded vehicle URDF against measured wheel/official sensor geometry."""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh

workspace=Path(__file__).resolve().parents[1]
root=ET.parse(sys.argv[1]).getroot()
links={e.get('name'):e for e in root.findall('link')}
joints={e.find('child').get('link'):e for e in root.findall('joint')}
def origin(e):
    o=e.find('origin')
    if o is None:return np.eye(4)
    t=trimesh.transformations.euler_matrix(*map(float,o.get('rpy','0 0 0').split()))
    t[:3,3]=list(map(float,o.get('xyz','0 0 0').split()))
    return t
def tf(parent,child):
    if parent==child:return np.eye(4)
    j=joints[child]
    return tf(parent,j.find('parent').get('link'))@origin(j)
def mesh(e):
    m=e.find('geometry/mesh');pkg,rel=m.get('filename').split('package://')[1].split('/',1)
    p=workspace/'install'/pkg/'share'/pkg/rel
    if not p.exists():p=Path('/opt/ros/humble/share')/pkg/rel
    value=trimesh.load(str(p),force='mesh')
    value.apply_scale(list(map(float,m.get('scale','1 1 1').split())))
    value.apply_transform(origin(e));return value
radius=.210
half_length=.4+np.hypot(radius,.05)+.020
half_width=half_length-.05
arm_enabled='plate_frame' in links
assert len(root.findall('ros2_control/joint'))==(14 if arm_enabled else 8)
for i,(x,y) in enumerate([(.4,.35),(.4,-.35),(-.4,.35),(-.4,-.35)],1):
    name=f'Wheel{i}_1'
    np.testing.assert_allclose(tf('base_link',name)[:3,3],[x,y,radius],atol=1e-8)
    for tag in ['visual','collision']:
        c=links[name].find(tag+'/geometry/cylinder')
        np.testing.assert_allclose([float(c.get('radius')),float(c.get('length'))],[radius,.1])
    assert radius*2<.52, 'tire intersects chassis deck'
    assert abs(x)+np.hypot(radius,.05)<half_length
    assert abs(y)+np.hypot(radius,.05)<half_width
    fork=links[f'Revolute{i}_1'].findall('collision')
    assert len(fork)==3
    for side in fork[1:]:
        center=origin(side)[:3,3]
        width=float(side.find('geometry/box').get('size').split()[1])
        assert abs(center[1])-width/2>.05, 'fork intersects tire sides'
v=links['base_link'].findall('visual');c=links['base_link'].findall('collision')
assert len(v)==len(c)==10, 'deck, roof, four posts and four enclosure panels required'
for a,b in zip(v,c):
    np.testing.assert_allclose(origin(a),origin(b))
    assert a.find('geometry/box').attrib==b.find('geometry/box').attrib
np.testing.assert_allclose(tf('base_link','roof_frame')[:3,3],[0,0,1])
np.testing.assert_allclose(tf('base_link','lidar_link')[:3,3],[half_length/2,0,1.0415])
np.testing.assert_allclose(tf('roof_frame','laser_frame')[:3,:3],np.eye(3))
np.testing.assert_allclose(tf('base_link','d455_depth_optical_frame')[2,3],.4)
np.testing.assert_allclose(tf('base_link','d455_depth_optical_frame')[:3,2],[1,0,0],atol=1e-8)
m=mesh(links['d455_link'].find('visual'));m.apply_transform(tf('base_link','d455_link'))
np.testing.assert_allclose(m.bounds.mean(axis=0),[half_length+.025,0,.4],atol=1e-6)
np.testing.assert_allclose(m.bounds[0,0]-half_length,.012,atol=1e-6)
print('D455 outside-front bounds',m.bounds)
if arm_enabled:
    np.testing.assert_allclose(tf('roof_frame','robot_base')[:3,3],[0,0,0])
    q={j.get('name'):float(j.find("state_interface[@name='position']/param[@name='initial_value']").text)
       for j in root.findall('ros2_control/joint') if j.get('name').startswith('j')}
    def home(name):
        if name=='base_footprint':return np.eye(4)
        j=joints[name];t=origin(j)
        if j.get('name') in q:
            angle=q[j.get('name')];limit=j.find('limit')
            assert float(limit.get('lower'))<=angle<=float(limit.get('upper'))
            t=t@trimesh.transformations.rotation_matrix(angle,list(map(float,j.find('axis').get('xyz').split())))
        return home(j.find('parent').get('link'))@t
    cam=home('camera_link');opt=home('camera_depth_optical_frame');charger=home('charging_connector_link')
    assert charger[2,3]>cam[2,3], 'charger must be above camera at home'
    np.testing.assert_allclose(cam[:3,2],[0,0,1],atol=1e-5,err_msg='camera housing up')
    np.testing.assert_allclose(opt[:3,1],[0,0,-1],atol=1e-5,err_msg='image pixel +Y points DOWN')
    np.testing.assert_allclose(opt[:3,2],home('plate_frame')[:3,2],atol=1e-8)
    print('Home camera/charger height',cam[2,3],charger[2,3],'; optical pixel-down',opt[:3,1])
else:
    assert not any(n in links for n in ['fr10_base_link','plate_frame','camera_link','charging_connector_link'])
print('PASS: 420mm wheels, fork clearance, closed chassis, external D455, optical orientation, optional arm')
