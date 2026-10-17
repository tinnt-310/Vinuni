#!/usr/bin/env python3
"""Build the matched CAD MuJoCo plant from canonical Xacro and station JSON."""
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from fr10_mujoco_bridge import ROOT, DESCRIPTION, XACRO, JOINT_NAMES, HOME, _format, _matrix_to_quaternion
from fr10_mujoco_bridge import simulation_pd_gains
from fr10_model_evidence import fixed_link_transform as fixed_transform


def matched_contact_parameters():
    values = yaml.safe_load((DESCRIPTION/'config/fr10_simulation_actuators.yaml').read_text())
    contact = values['matched_mujoco_contact']
    reference = np.asarray(contact['solref'], dtype=float)
    impedance = np.asarray(contact['solimp'], dtype=float)
    tolerance = contact['ccd_tolerance']
    iterations = contact['ccd_iterations']
    if reference.shape != (2,) or impedance.shape != (5,) or \
            not np.isfinite(reference).all() or not np.isfinite(impedance).all() or \
            np.any(reference >= 0) or not (0 <= impedance[0] < impedance[1] < 1) or \
            not (0 < impedance[2] <= .00005) or not (0 < impedance[3] < 1) or impedance[4] < 1:
        raise ValueError('Invalid bounded matched MuJoCo contact calibration')
    # Slender CAD cells against zero-radius triangles need a converged convex
    # distance solve. The default 1 um termination can return spurious deep
    # contacts; changing impedance or clipping their depth cannot repair it.
    if not isinstance(tolerance, (int, float)) or not np.isfinite(tolerance) or \
            not (0 < tolerance <= 1e-10) or isinstance(iterations, bool) or \
            not isinstance(iterations, int) or not (200 <= iterations <= 1000):
        raise ValueError('Invalid matched MuJoCo collision precision')
    return _format(reference), _format(impedance), float(tolerance), iterations


def configure_matched_plant(plant, station_file, physics_step_s, include_visuals=False):
    station = json.loads(Path(station_file).read_text())
    if station.get('profile_id') != 'matched_socket_v2' or station.get('mounted'):
        raise ValueError('Matched MuJoCo plant currently requires canonical arm-only station')
    import importlib.util
    spec = importlib.util.spec_from_file_location('charging_station', ROOT/'src/vehicle_bringup/launch/charging_station.py')
    station_module = importlib.util.module_from_spec(spec); spec.loader.exec_module(station_module)
    initial = np.asarray(station_module.initial_configuration(station, station.get('initial_state', 'home')))
    contact_reference, contact_impedance, ccd_tolerance, ccd_iterations = matched_contact_parameters()
    model_dir = Path(station['model_path'])
    import hashlib
    manifest = dict(algorithm='MuJoCo rigid nonconvex triangle flex',radius_m=0.0,
        sources={name:hashlib.sha256((model_dir/name).read_bytes()).hexdigest()
                 for name in ('socket_contact_surface.stl','socket_protected_rear.stl')},
        contact_solref=contact_reference, contact_solimp=contact_impedance,
        ccd_tolerance=ccd_tolerance, ccd_iterations=ccd_iterations,
        limitations=['triangle surface contacts; penetration bounded independently'])
    plant._temporary_directory = tempfile.TemporaryDirectory(prefix='fr10_matched_mujoco_')
    directory = Path(plant._temporary_directory.name)
    urdf_path = directory/'matched.urdf'
    subprocess.run(['xacro', str(XACRO), 'enable_vehicle:=false', 'enable_arm:=true',
        'enable_force_sensor:=true', 'charging_tool_profile:=matched_socket_v2',
        'control_backend:=none', '-o', str(urdf_path)], cwd=ROOT, check=True)
    text = urdf_path.read_text().replace('package://vehicle_description/', f'{DESCRIPTION}/')
    robot = ET.fromstring(text)
    for link in robot.findall('link'):
        for index, collision in enumerate(link.findall('collision')):
            collision.set('name', link.get('name')+'__'+collision.get('name', str(index)))
    if include_visuals:
        from fr10_mujoco_visuals import retain_urdf_visuals
        retain_urdf_visuals(robot, directory)
    ET.ElementTree(robot).write(urdf_path, encoding='unicode')
    tool = fixed_transform(robot, 'wrist3_link', 'charging_tcp')
    plant.tcp_in_wrist, plant.rotation_wrist_to_tcp = tool[:3, 3], tool[:3, :3]
    base_model = mujoco.MjModel.from_xml_path(str(urdf_path))
    if base_model.nq != 6: raise ValueError('Unexpected matched arm joint count')
    base_xml = directory/'base.xml'; mujoco.mj_saveLastXML(str(base_xml), base_model)
    tree = ET.parse(base_xml); xml = tree.getroot()
    if include_visuals:
        xml.set('model', 'FR10 original visual CAD - read-only viewer')
    option = xml.find('option')
    if option is None: option = ET.SubElement(xml, 'option')
    option.attrib.update(timestep=str(physics_step_s), gravity='0 0 -9.80665', integrator='implicitfast',
                         ccd_tolerance=str(ccd_tolerance), ccd_iterations=str(ccd_iterations))
    compiler = xml.find('compiler')
    if compiler is not None: compiler.set('inertiafromgeom', 'false')
    world = xml.find('worldbody'); asset = xml.find('asset')
    if asset is None: asset = ET.SubElement(xml, 'asset')
    for geom in world.iter('geom'):
        name = geom.get('name','')
        if name.startswith('visual__'):
            # Viewer-only mesh; never introduce additional contact surfaces.
            geom.attrib.update(contype='0', conaffinity='0', group='2', density='0')
            continue
        intentional = name.startswith(('charging_mating_nose__','charging_mating_shoulder__'))
        # Robot/fixture physical contacts are enabled. Self-contact authority
        # remains the complete, validated PlanningScene used by all backends.
        geom.attrib.update(contype='2' if intentional else '1', conaffinity='4',
            margin='0', gap='0', friction='0.4 0.005 0.0001',
            solref=contact_reference, solimp=contact_impedance)
    rotation = Rotation.from_euler('xyz', station['world_mouth_rpy']).as_matrix()
    port = ET.SubElement(world, 'body', name='matched_socket',
        pos=_format(np.asarray(station['world_mouth_position_m'])),
        quat=_format(_matrix_to_quaternion(rotation)))
    # A fully rigid flex preserves the original nonconvex triangles and bores.
    # Radius zero introduces no inflated surface or convex-hull hole filling.
    # Unlike thousands of overlapping shells it also avoids internal cell faces.
    for name,filename in (('socket_surface_exact','socket_contact_surface.stl'),
                          ('socket_protected_rear_exact','socket_protected_rear.stl')):
        flex=ET.SubElement(port,'flexcomp',name=name,type='mesh',rigid='true',
            dim='2',radius='0',file=str(model_dir/filename),rgba='0.1 0.1 0.12 1')
        ET.SubElement(flex,'contact',contype='4',conaffinity='3',selfcollide='none',internal='false',
            friction='0.4 0.005 0.0001',solref=contact_reference,solimp=contact_impedance)
    for primitive in station['world_primitives']:
        ET.SubElement(world,'geom',name=primitive['id'],type='box',
            pos=_format(np.asarray(primitive['position_world_m'])),
            size=_format(np.asarray(primitive['size_m'])/2),contype='4',conaffinity='3',group='1')
    kp,kd = simulation_pd_gains()
    limit = np.array([150.,150.,150.,28.,28.,28.])
    actuators = ET.SubElement(xml,'actuator')
    for index,name in enumerate(JOINT_NAMES):
        ET.SubElement(actuators,'position',name=name+'_position',joint=name,kp=str(kp[index]),
            kv=str(kd[index]),ctrllimited='true',ctrlrange=_format(base_model.jnt_range[index]),
            forcelimited='true',forcerange=_format(np.array([-limit[index],limit[index]])))
    path = directory/'matched.xml'; tree.write(path,encoding='unicode')
    plant.model = mujoco.MjModel.from_xml_path(str(path)); plant.data = mujoco.MjData(plant.model)
    plant.data.qpos[:6] = initial; plant.data.ctrl[:] = initial; mujoco.mj_forward(plant.model,plant.data)
    plant.wrist_body_id = plant.model.body('wrist3_link').id
    plant.tool_geom_ids = {i for i in range(plant.model.ngeom) if plant.model.geom(i).name.startswith(
        ('charging_connector_link__','charging_mating_nose__','charging_mating_shoulder__'))}
    plant.fixture_geom_ids = {i for i in range(plant.model.ngeom) if plant.model.geom_contype[i]==4}
    plant.fixture_flex_ids = set(range(plant.model.nflex))
    plant.command = initial.copy(); plant.last_wrench = np.zeros(6)
    plant.kp,plant.kd,plant.torque_limit = kp,kd,limit
    plant.contact_tcp_position = np.asarray(station['world_mouth_position_m'])
    plant.insertion_axis_world = rotation[:,2]
    plant.home_tcp_position,plant.home_tcp_rotation = plant.tcp_pose()
    plant.matched_station = station; plant.surface_manifest = manifest
    plant.model_path = path
