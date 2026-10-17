"""The FCL BVH must contain every canonical facet, including internal faces."""
import importlib.util
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import pytest
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
DIRECTORY=ROOT/'src/vehicle_description/models/matched_socket_v2'
SPEC=importlib.util.spec_from_file_location('planning_collision_model',
    ROOT/'src/fr10_arm_control/fr10_pose_controller/launch/planning_collision_model.py')
MODEL=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(MODEL)


def test_shoulder_box_has_identical_cad_support_and_no_inflation():
    from fr10_mating_geometry import load_stl
    collision=ET.parse(DIRECTORY/'matched_connector.urdf').find("link[@name='charging_mating_shoulder']/collision")
    center=np.fromstring(collision.find('origin').get('xyz'),sep=' ')
    half=np.fromstring(collision.find('geometry/box').get('size'),sep=' ')*.5
    vertices=np.unique(load_stl(DIRECTORY/'seat.stl').reshape(-1,3),axis=0)
    directions=np.random.default_rng(26).normal(size=(1000,3))
    directions=np.vstack([directions,np.eye(3),-np.eye(3)])
    mesh_support=(directions@vertices.T).max(axis=1)
    box_support=directions@center+np.abs(directions)@half
    assert np.max(np.abs(mesh_support-box_support)) < 1e-14


def test_all_facets_preserved_and_only_collision_partition_changes():
    description=(DIRECTORY/'matched_connector.urdf').read_text()
    physics=ET.fromstring(description)
    planning=ET.fromstring(MODEL.planning_collision_description(description,DIRECTORY))
    a=physics.find("link[@name='charging_mating_nose']")
    b=planning.find("link[@name='charging_mating_nose']")
    manifest=json.loads((DIRECTORY/'planning_collision.json').read_text())
    original=b''.join((DIRECTORY/s['uri'][len(MODEL.PREFIX):]).read_bytes()[84:] for s in manifest['sources'])
    merged=(DIRECTORY/manifest['mesh_uri'][len(MODEL.PREFIX):]).read_bytes()
    assert merged[84:]==original
    assert struct.unpack_from('<I',merged,80)[0]==4368
    assert len(a.findall('collision'))==364 and len(b.findall('collision'))==1
    for link in (a,b):
        for collision in link.findall('collision'):link.remove(collision)
    # Whitespace serialization has no effect on URDF element attributes/text.
    def value(e): return (e.tag,e.attrib,(e.text or '').strip(),[value(c) for c in e])
    assert value(physics)==value(planning)


@pytest.mark.parametrize('change',['cell','bvh','transform','missing'])
def test_mismatched_geometry_fails_closed(tmp_path,change):
    shutil.copytree(DIRECTORY,tmp_path/'model');directory=tmp_path/'model'
    description=(directory/'matched_connector.urdf').read_text()
    manifest=json.loads((directory/'planning_collision.json').read_text())
    if change in ('cell','bvh'):
        uri=manifest['sources'][0]['uri'] if change=='cell' else manifest['mesh_uri']
        file=directory/uri[len(MODEL.PREFIX):];data=bytearray(file.read_bytes());data[96]^=1;file.write_bytes(data)
    else:
        robot=ET.fromstring(description);link=robot.find("link[@name='charging_mating_nose']")
        if change=='transform':ET.SubElement(link.find('collision'),'origin',xyz='0 0 .001')
        else:link.remove(link.find('collision'))
        description=ET.tostring(robot,encoding='unicode')
    with pytest.raises(ValueError):MODEL.planning_collision_description(description,directory)


def test_legacy_model_is_unchanged():
    description='<robot name="legacy"><link name="tool"/></robot>'
    assert MODEL.planning_collision_description(description,DIRECTORY)==description


def test_gazebo_resolved_file_uri_is_same_canonical_asset():
    robot=ET.parse(DIRECTORY/'matched_connector.urdf').getroot()
    for mesh in robot.findall("link[@name='charging_mating_nose']/collision/geometry/mesh"):
        mesh.set('filename','file://'+str((DIRECTORY/mesh.get('filename')[len(MODEL.PREFIX):]).resolve()))
    description=ET.tostring(robot,encoding='unicode')
    assert len(ET.fromstring(MODEL.planning_collision_description(description,DIRECTORY)).findall(
        "link[@name='charging_mating_nose']/collision"))==1
    robot.find("link[@name='charging_mating_nose']/collision/geometry/mesh").set('filename','file:///tmp/unrelated.stl')
    with pytest.raises(ValueError):MODEL.planning_collision_description(ET.tostring(robot,encoding='unicode'),DIRECTORY)
