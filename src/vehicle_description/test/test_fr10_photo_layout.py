"""Photo proposal: vertical CCS2 DC male / D455 stack, independent of calibration."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

DESCRIPTION = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('photo_tool_geometry', DESCRIPTION/'launch/tool_geometry.py')
GEOMETRY = importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(GEOMETRY)


@pytest.mark.parametrize('mounted', (False, True))
def test_photo_stack_has_nine_male_contacts_and_flange_centred_d455(mounted):
    robot = ET.fromstring(xacro.process_file(str(DESCRIPTION/'urdf/vehicle.urdf.xacro'), mappings={
        'enable_arm': 'true', 'enable_vehicle': str(mounted).lower(),
        'wrist_layout': 'photo_dc_d455'}).toxml())
    tool = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', 'charging_tcp'))
    np.testing.assert_allclose(tool[:3, 3], [0, .095, .185], atol=1e-12)
    np.testing.assert_allclose(tool[:3, :3], np.eye(3), atol=1e-12)
    optical = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', 'camera_depth_optical_frame'))
    np.testing.assert_allclose(optical[:3, 2], [0, 0, 1], atol=1e-12)
    mesh = robot.find("link[@name='camera_link']/visual/geometry/mesh")
    assert mesh.get('filename').endswith('/d455.stl')
    # Read the actual official D455 triangles rather than treating its optical
    # frame or bottom screw as the centre of its housing.
    import struct
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('realsense2_description'))/'meshes/d455.stl'
    with path.open('rb') as stream:
        stream.seek(80);count = struct.unpack('<I', stream.read(4))[0]
        records = np.fromfile(stream, dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]), count=count)
    points = records['v'].reshape(-1, 3).astype(float)*.001
    pose = np.asarray(GEOMETRY.fixed_transform(robot, 'plate_frame', 'camera_link')) @ np.asarray(
        GEOMETRY._origin(robot.find("link[@name='camera_link']/visual/origin")))
    points = points @ pose[:3,:3].T + pose[:3,3]
    centre = (points.min(axis=0)+points.max(axis=0))/2
    np.testing.assert_allclose(centre[:2], [0, 0], atol=1e-7)
    assert points[:,2].min() == pytest.approx(.015, abs=1e-7)
    assert points[:,2].max() < .042 < tool[2,3]
    assert points[:,1].max()+.010 < tool[1,3]-.0675
    connector = robot.find("link[@name='charging_connector_link']")
    contacts = connector.findall("collision")
    contacts = [c for c in contacts if c.get('name','').endswith('_male')]
    assert len(contacts) == 9
    dc = [c for c in contacts if c.get('name').startswith('dc_')]
    upper = [c for c in contacts if c not in dc]
    assert len(dc) == 2 and len(upper) == 7
    assert all(float(c.find('origin').get('xyz').split()[1]) < 0 for c in dc)
    assert all(float(c.find('origin').get('xyz').split()[1]) > 0 for c in upper)
    assert min(float(c.find('geometry/cylinder').get('radius')) for c in dc) > max(
        float(c.find('geometry/cylinder').get('radius')) for c in upper)
    for contact in contacts:
        z = float(contact.find('origin').get('xyz').split()[2])
        half = float(contact.find('geometry/cylinder').get('length'))/2
        assert z+half+.075 == pytest.approx(.185)


def test_photo_layout_does_not_replace_the_measured_tool_file():
    import yaml
    data = yaml.safe_load((DESCRIPTION/'config/fr10_tool.yaml').read_text())
    assert data['translation_m'] == [.055140, -.046496, .215297]
    assert data['rotation_fixed_xyz_deg'] == [0., 0., 136.840]
