"""Single station/CAD datum source for world, scene, TF and test goals."""
import json
import itertools
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotate(r, v):
    return [sum(r[i][k]*v[k] for k in range(3)) for i in range(3)]


def rpy_matrix(rpy):
    r, p, y = rpy; cr, sr = math.cos(r), math.sin(r); cp, sp = math.cos(p), math.sin(p); cy, sy = math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def matrix_rpy(r):
    return [math.atan2(r[2][1], r[2][2]), math.atan2(-r[2][0], math.hypot(r[0][0], r[1][0])), math.atan2(r[1][0], r[0][0])]


def rpy_quaternion(rpy):
    r, p, y = [angle/2 for angle in rpy]
    cr, sr = math.cos(r), math.sin(r); cp, sp = math.cos(p), math.sin(p); cy, sy = math.cos(y), math.sin(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def arm_mount(robot_xml, mounted):
    translation = [0., 0., 0.]; rotation = rpy_matrix([0., 0., 0.])
    if not mounted: return translation, rotation
    # Resolve the fixed chain from the actual expanded model. Never duplicate
    # arm mount coordinates in station YAML/world or compensate upstream joints.
    root = ET.fromstring(robot_xml)
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain, child = [], 'fr10_base_link'
    while child in joints:
        joint = joints[child]
        if joint.get('type') != 'fixed': raise ValueError('Arm base mount must be a fixed chain')
        chain.append(joint); child = joint.find('parent').get('link')
    for joint in reversed(chain):
        origin = joint.find('origin')
        if origin is None: continue
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
        delta = rotate(rotation, xyz)
        translation = [translation[i]+delta[i] for i in range(3)]
        rotation = multiply(rotation, rpy_matrix(rpy))
    return translation, rotation


def keyed_yaw_half_span(geometry):
    """Nominal centered yaw clearance; not proof of actual contact/fit.

    A pin center at radius r travels 2*r*sin(yaw/2). Use the canonical
    design's minimum radial clearance and largest pin-center radius. Real
    assemblies require measured clearance and FT commissioning separately.
    """
    clearance = float(geometry['simulation_design']['radial_clearance_mm'])
    radius = max(math.hypot(*pin['center_mm']) for pin in geometry['pins'])
    if not math.isfinite(clearance) or not math.isfinite(radius) or not 0 < clearance < 2*radius:
        raise ValueError('Invalid mechanical keyed yaw clearance')
    return 2*math.asin(clearance/(2*radius))


def select_station(registry_path, geometry_path, name, robot_xml, mounted,
                   preinsert_distance_m=None):
    geometry_path = Path(geometry_path)
    registry = yaml.safe_load(Path(registry_path).read_text())
    approach = registry['approach_distance_m'] if preinsert_distance_m is None else preinsert_distance_m
    if isinstance(approach, bool) or not isinstance(approach, (int, float)) or \
            not math.isfinite(approach) or not .005 <= approach <= .4:
        raise ValueError('Matched preinsert clearance must be 5..400 mm')
    geometry = json.loads(Path(geometry_path).read_text())
    if name not in registry['stations']: raise ValueError(f'Unknown charging station {name}')
    entry = registry['stations'][name]
    position = entry['mouth_arm_m']
    rotation = multiply(rpy_matrix([0., math.radians(entry['tilt_up_deg']), 0.]), rpy_matrix([-math.pi, 0., -math.pi/2]))
    mount_xyz, mount_r = arm_mount(robot_xml, mounted)
    world_delta = rotate(mount_r, position)
    world_position = [mount_xyz[i]+world_delta[i] for i in range(3)]
    world_rotation = multiply(mount_r, rotation)
    normal = rotate(world_rotation, [0., 0., -1.])
    to_arm = [mount_xyz[i]-world_position[i] for i in range(3)]
    if normal[2] < -1e-9 or (math.hypot(*normal[:2]) > 1e-6 and
            sum(normal[i]*to_arm[i] for i in range(2)) <= 0):
        raise ValueError('Socket face points down or away from the approach region')
    seated = geometry['simulation_design']['nose_length_mm']*.001
    axis = rotate(rotation, [0., 0., 1.])
    bounds = geometry['mating_bounds_mm']
    corners = [rotate(world_rotation, [v*.001 for v in corner])
               for corner in itertools.product(*zip(bounds[0], bounds[1]))]
    bottom = min(world_position[2]+corner[2] for corner in corners)
    if bottom <= .04: raise ValueError('Station CAD/support intersects the floor')
    center = [world_position[i]+sum(corner[i] for corner in corners)/len(corners) for i in range(2)]
    primitives = [
        {'id': 'matched_ground', 'size_m': [10.,10.,.02], 'position_world_m': [0.,0.,-.01]},
        {'id': 'station_support_base', 'size_m': [.22,.24,.02], 'position_world_m': [*center,.01]},
        {'id': 'station_support_column', 'size_m': [.035,.035,bottom-.04],
         'position_world_m': [*center,.02+(bottom-.04)/2]},
        {'id': 'station_support_mount', 'size_m': [.10,.14,.02],
         'position_world_m': [*center,bottom-.01]}]
    return {'name': name, 'mounted': mounted, 'profile_id': geometry_path.parent.name,
            'keyed_yaw_free_half_span_rad': keyed_yaw_half_span(geometry),
            'lateral_free_half_span_m': float(geometry['simulation_design']['radial_clearance_mm'])*.001,
            'canonical_sha256': geometry['socket_sha256'], 'acceptance': registry['acceptance'],
            'world_mouth_position_m': world_position, 'world_mouth_rpy': matrix_rpy(world_rotation),
            'mouth_position_arm_m': position, 'orientation_arm_xyzw': rpy_quaternion(matrix_rpy(rotation)),
            'preinsert_position_arm_m': [position[i]-axis[i]*approach for i in range(3)],
            'seated_position_arm_m': [position[i]+axis[i]*seated for i in range(3)],
            'preinsert_distance_m': approach, 'insertion_depth_m': seated,
            'initial_joint_rad': registry['initial_joint_rad'], 'world_primitives': primitives}


def initial_configuration(station, name):
    configurations = station['initial_joint_rad']
    if name not in configurations:
        raise ValueError(f'Unknown initial configuration {name}')
    values = configurations[name]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError('Initial configuration must contain six finite radians')
    return list(values)


def initialize_arm_description(robot_xml, station, name):
    """Set initial state once; runtime joint feedback remains measured."""
    values = initial_configuration(station, name)
    robot = ET.fromstring(robot_xml)
    found = set()
    for component in robot.findall('ros2_control'):
        for joint in component.findall('joint'):
            joint_name = joint.get('name')
            if joint_name not in {f'j{i}' for i in range(1, 7)}:
                continue
            if joint_name in found:
                raise ValueError('Multiple hardware owners for an arm joint')
            interface = joint.find("state_interface[@name='position']")
            if interface is None:
                raise ValueError('Arm joint has no position state interface')
            param = interface.find("param[@name='initial_value']")
            if param is None:
                param = ET.SubElement(interface, 'param', name='initial_value')
            param.text = str(values[int(joint_name[1:])-1])
            found.add(joint_name)
    if len(found) != 6:
        raise ValueError('Initial configuration requires exactly six arm hardware joints')
    return ET.tostring(robot, encoding='unicode')


def matched_physics_step(model, step_s=None):
    """One validated integration step for the world and encoder read cadence."""
    calibration = yaml.safe_load((model.parents[1]/'config/fr10_simulation_actuators.yaml').read_text())
    step = calibration['matched_gazebo_physics']['max_step_size_s'] if step_s is None else step_s
    if isinstance(step, bool) or not isinstance(step, (int, float)) or \
            not math.isfinite(step) or not (0.00005 <= step <= 0.001):
        raise ValueError('Invalid matched Gazebo integration step')
    return step


def write_world(station, model, path, step_s=None, solver_iterations=None):
    if solver_iterations is None:
        solver_iterations = 2000
    if isinstance(solver_iterations, bool) or not isinstance(solver_iterations, int) or \
            not 50 <= solver_iterations <= 2000:
        raise ValueError('Matched solver iterations must be an integer in 50..2000')
    step = matched_physics_step(model, step_s)
    calibration = yaml.safe_load((model.parents[1]/'config/fr10_simulation_actuators.yaml').read_text())
    contact_calibration = calibration['matched_gazebo_contact']
    for value in contact_calibration.values():
        if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or value<=0.:
            raise ValueError('Invalid matched Gazebo contact calibration')
    sdf = ET.Element('sdf', version='1.6'); world = ET.SubElement(sdf, 'world', name='default')
    ET.SubElement(world, 'gravity').text = '0 0 -9.80665'
    physics = ET.SubElement(world, 'physics', type='ode', name='matched_contact')
    ET.SubElement(physics, 'max_step_size').text = str(step)
    ET.SubElement(physics, 'real_time_update_rate').text = str(1./step)
    ET.SubElement(physics, 'real_time_factor').text = '1'
    ode = ET.SubElement(physics, 'ode'); solver = ET.SubElement(ode, 'solver')
    # Contact meshes introduce redundant constraints: Dantzig/world produced
    # LCP errors at seating. Converge QuickStep instead of hiding transverse
    # hinge tilt in joint feedback; physical TCP/FK consistency is measured.
    ET.SubElement(solver, 'type').text = 'quick'
    ET.SubElement(solver, 'iters').text = str(solver_iterations)
    ET.SubElement(solver, 'sor').text = '1.0'
    constraints = ET.SubElement(ode, 'constraints'); ET.SubElement(constraints, 'contact_surface_layer').text = '0'
    ET.SubElement(constraints, 'contact_max_correcting_vel').text = str(contact_calibration['max_correcting_velocity_m_s'])
    light = ET.SubElement(world, 'light', type='directional', name='matched_sun')
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'direction').text = '-.5 .1 -.9'
    ET.SubElement(light, 'diffuse').text = '.8 .8 .8 1'
    ET.SubElement(light, 'cast_shadows').text = 'true'
    # Inline ground and support: never depend on a network model:// download.
    # These same primitives are published to PlanningScene.
    for primitive in station['world_primitives']:
        fixture = ET.SubElement(world, 'model', name=primitive['id'])
        ET.SubElement(fixture, 'static').text = 'true'
        ET.SubElement(fixture, 'pose').text = ' '.join(map(str, primitive['position_world_m']))+' 0 0 0'
        fixed = ET.SubElement(fixture, 'link', name='support')
        for kind in ('visual', 'collision'):
            item = ET.SubElement(fixed, kind, name=primitive['id'])
            box = ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = ' '.join(map(str, primitive['size_m']))
    socket = ET.SubElement(world, 'model', name='matched_charging_station'); ET.SubElement(socket, 'static').text = 'true'
    ET.SubElement(socket, 'pose').text = ' '.join(map(str, station['world_mouth_position_m']+station['world_mouth_rpy']))
    link = ET.SubElement(socket, 'link', name='socket')
    for tag, name, asset in [('visual', 'canonical_cad', 'socket_mating.stl'),
                              ('collision', 'mating_surface', 'socket_contact_surface.stl'),
                              ('collision', 'protected_rear', 'socket_protected_rear.stl')]:
        item = ET.SubElement(link, tag, name=name); geometry = ET.SubElement(item, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh'); ET.SubElement(mesh, 'uri').text = (model/asset).resolve().as_uri()
        if tag == 'visual':
            material = ET.SubElement(item, 'material'); ET.SubElement(material, 'ambient').text = '.12 .14 .16 1'
            ET.SubElement(material, 'diffuse').text = '.12 .14 .16 1'
        else:
            surface = ET.SubElement(item, 'surface'); contact = ET.SubElement(surface, 'contact'); params = ET.SubElement(contact, 'ode')
            ET.SubElement(params, 'kp').text = str(contact_calibration['stiffness_n_m'])
            ET.SubElement(params, 'kd').text = str(contact_calibration['damping_n_s_m'])
            ET.SubElement(params, 'max_vel').text = str(contact_calibration['max_correcting_velocity_m_s'])
            ET.SubElement(params, 'min_depth').text = '0'
            friction = ET.SubElement(ET.SubElement(surface, 'friction'), 'ode')
            ET.SubElement(friction, 'mu').text = '.2'; ET.SubElement(friction, 'mu2').text = '.2'
    # Evaluation only: actual CAD contact pairs/depth, never pose feedback to
    # the admittance controller. Both mating and protected rear are observed.
    sensor = ET.SubElement(link, 'sensor', type='contact', name='cad_station_contacts')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '125'
    contacts = ET.SubElement(sensor, 'contact')
    for name in ('mating_surface', 'protected_rear'):
        ET.SubElement(contacts, 'collision').text = name
    plugin = ET.SubElement(sensor, 'plugin', name='cad_contact_evidence', filename='libgazebo_ros_bumper.so')
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'remapping').text = 'bumper_states:=/fr10_sim/station_contacts'
    ET.SubElement(plugin, 'frame_name').text = 'world'
    # No plate is inserted into the mouth. Support ends at the CAD rear datum.
    Path(path).write_text(ET.tostring(sdf, encoding='unicode')+'\n')
