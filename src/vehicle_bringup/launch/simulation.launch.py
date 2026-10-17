import os
import sys
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import yaml
import tempfile
import socket
import math
import json
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path
from copy import deepcopy
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import AppendEnvironmentVariable, SetEnvironmentVariable


def load_force_profile(path, pose_share):
    spec = importlib.util.spec_from_file_location('controller_profile',
        os.path.join(pose_share, 'launch', 'controller_profile.py'))
    loader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loader)
    return loader.load_controller_profile(path)


def station_spawn_transition(event, next_spawn):
    """Only advance vehicle startup after a successful prerequisite."""
    if event.returncode != 0:
        return [Shutdown(reason=f'Vehicle startup prerequisite failed: {event.returncode}')]
    return [next_spawn]


def arm_controller_transition(event, pose_node):
    """The pose node may publish only after its position controller is active."""
    if event.returncode != 0:
        return [Shutdown(reason=f'FR10 controller activation failed: {event.returncode}')]
    return [pose_node]


def compose_controller_config(vehicle_config, arm_config, vehicle_enabled, arm_enabled, arm_profile="jtc"):
    """Keep one controller manager and exactly one owner per selected joint set."""
    if not vehicle_enabled and not arm_enabled:
        raise ValueError('At least one robot component is required')
    if vehicle_enabled:
        config = deepcopy(vehicle_config)
    else:
        config = {'controller_manager': {'ros__parameters': {
            'update_rate': 100, 'use_sim_time': True,
            'joint_state_broadcaster': {
                'type': 'joint_state_broadcaster/JointStateBroadcaster'}}}}
    manager = config['controller_manager']['ros__parameters']
    if arm_enabled:
        if arm_profile == 'jtc':
            config.update(deepcopy(arm_config))
            manager['joint_trajectory_controller'] = {
                'type': 'joint_trajectory_controller/JointTrajectoryController'}
        elif arm_profile == 'pose':
            arm_manager = arm_config['controller_manager']['ros__parameters']
            manager['update_rate'] = arm_manager['update_rate']
            manager['fairino10_controller'] = deepcopy(
                arm_manager['fairino10_controller'])
            config['fairino10_controller'] = deepcopy(
                arm_config['fairino10_controller'])
        else:
            raise ValueError(f'Unknown arm controller profile: {arm_profile}')
    return config


def validate_vehicle_geometry(geometry, controller_config, robot):
    """Fail before spawn if swerve kinematics differ from the wheel model.

    The 1e-12 tolerance only accounts for XML float serialization; it is not
    a motion or physical acceptance budget.
    """
    params = controller_config['swerve_controller']['ros__parameters']
    for key in ('wheelbase', 'track_width', 'wheel_radius', 'steering_axis_height',
                'wheel_axle_drop'):
        try:
            value = float(geometry[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'{key}: missing or nonnumeric vehicle geometry') from exc
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{key}: invalid vehicle geometry')
    try:
        wheel_offset_x = float(geometry['wheel_offset_x'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('wheel_offset_x: missing or nonnumeric vehicle geometry') from exc
    if not math.isfinite(wheel_offset_x) or wheel_offset_x < 0.0:
        raise ValueError('wheel_offset_x: invalid vehicle geometry')

    def same(actual, expected, label):
        try:
            actual = float(actual)
            expected = float(expected)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'{label}: missing or nonnumeric geometry') from exc
        if not math.isfinite(actual) or not math.isfinite(expected) or not math.isclose(
                actual, expected, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(f'{label}: controller/model geometry mismatch ({actual} != {expected})')

    for model_key, controller_key in (
            ('wheelbase', 'wheel_separation_x'),
            ('track_width', 'wheel_separation_y'),
            ('wheel_radius', 'wheel_radius'),
            ('wheel_offset_x', 'wheel_offset_x')):
        same(params[controller_key], geometry[model_key], controller_key)

    def axis_is(element, expected, label):
        axes = element.getElementsByTagName('axis')
        if len(axes) != 1:
            raise ValueError(f'{label}: missing joint axis')
        coordinates = axes[0].getAttribute('xyz').split()
        if len(coordinates) != 3:
            raise ValueError(f'{label}: invalid joint axis')
        for actual, target, coordinate in zip(coordinates, expected, 'xyz'):
            same(actual, target, f'{label} axis {coordinate}')

    def named(tag, name):
        matches = [element for element in robot.documentElement.childNodes
                   if element.nodeType == element.ELEMENT_NODE
                   and element.tagName == tag
                   and element.getAttribute('name') == name]
        if len(matches) != 1:
            raise ValueError(f'{name}: expected one {tag}, found {len(matches)}')
        return matches[0]

    for index, signs in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1)), 1):
        joint_name = f'Revolute{index}_to_base_link'
        joint = named('joint', joint_name)
        parent = joint.getElementsByTagName('parent')
        child = joint.getElementsByTagName('child')
        origins = joint.getElementsByTagName('origin')
        axis_is(joint, (0, 0, 1), joint_name)
        if (len(parent) != 1 or parent[0].getAttribute('link') != 'base_link'
                or len(child) != 1 or child[0].getAttribute('link') != f'Revolute{index}_1'
                or len(origins) != 1):
            raise ValueError(f'{joint_name}: wrong wheel parent, child, or origin')
        xyz = origins[0].getAttribute('xyz').split()
        if len(xyz) != 3:
            raise ValueError(f'{joint_name}: invalid origin xyz')
        same(xyz[0], signs[0] * geometry['wheelbase'] / 2, joint_name + ' x')
        same(xyz[1], signs[1] * geometry['track_width'] / 2, joint_name + ' y')
        same(xyz[2], geometry['steering_axis_height'],
             joint_name + ' z')

        axle_name = f'Wheel{index}_to_Revolute{index}'
        axle = named('joint', axle_name)
        axle_origins = axle.getElementsByTagName('origin')
        axle_parents = axle.getElementsByTagName('parent')
        axle_children = axle.getElementsByTagName('child')
        axis_is(axle, (0, 1, 0), axle_name)
        if (len(axle_parents) != 1 or axle_parents[0].getAttribute('link') != f'Revolute{index}_1'
                or len(axle_children) != 1 or axle_children[0].getAttribute('link') != f'Wheel{index}_1'
                or len(axle_origins) != 1):
            raise ValueError(f'{axle_name}: wheel axle frame or axis differs from controller convention')
        axle_xyz = axle_origins[0].getAttribute('xyz').split()
        if len(axle_xyz) != 3:
            raise ValueError(f'{axle_name}: invalid wheel axle origin')
        same(axle_xyz[0], geometry['wheel_offset_x'], axle_name + ' x')
        same(axle_xyz[1], 0.0, axle_name + ' y')
        same(axle_xyz[2], -geometry['wheel_axle_drop'], axle_name + ' z')

        wheel_name = f'Wheel{index}_1'
        wheel = named('link', wheel_name)
        collisions = wheel.getElementsByTagName('collision')
        cylinders = (collisions[0].getElementsByTagName('cylinder')
                     if len(collisions) == 1 else [])
        if len(cylinders) != 1:
            raise ValueError(f'{wheel_name}: expected one cylindrical wheel collision')
        same(cylinders[0].getAttribute('radius'), geometry['wheel_radius'],
             wheel_name + ' radius')


def launch_setup(context):
    gazebo_port = int(LaunchConfiguration('gazebo_port').perform(context))
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', gazebo_port)) == 0:
            raise RuntimeError(f'Gazebo port {gazebo_port} is already in use. Stop the previous launch or set gazebo_port:=another_port.')
    vehicle_enabled = LaunchConfiguration('enable_vehicle').perform(context).lower() == 'true'
    arm_enabled = LaunchConfiguration('enable_arm').perform(context).lower() == 'true'
    arm_profile = LaunchConfiguration('arm_controller_profile').perform(context)
    force_enabled = LaunchConfiguration('enable_force_control').perform(context).lower() == 'true'
    if force_enabled and not arm_enabled:
        raise RuntimeError(
            'Force insertion test requires enable_arm:=true.')
    pkg_name = 'vehicle_description'
    navigation_mode = LaunchConfiguration('navigation_mode').perform(context)
    if navigation_mode != 'disabled' and (not vehicle_enabled or force_enabled):
        raise ValueError('Navigation requires enable_vehicle:=true enable_force_control:=false')
    # Resolve invalid files before spawning Gazebo or creating any controllers.
    if navigation_mode != 'disabled':
        navigation_share = Path(get_package_share_directory('vehicle_navigation'))
        for argument, fallback in [('nav2_params_file', 'config/nav2.yaml'),
                                   ('slam_params_file', 'config/slam.yaml')]:
            filename = LaunchConfiguration(argument).perform(context) or str(navigation_share / fallback)
            if not Path(filename).expanduser().is_file():
                raise ValueError(f'Parameter file does not exist: {filename}')
        if navigation_mode in ('localization', 'navigation'):
            spec = importlib.util.spec_from_file_location('vehicle_navigation_launch',
                navigation_share / 'launch/navigation.launch.py')
            navigation_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(navigation_module)
            navigation_module.validate_map(LaunchConfiguration('map').perform(context)
                                           or navigation_share / 'maps/vinfast.yaml')

    with open(os.path.join(get_package_share_directory(pkg_name), 'config', 'vehicle_geometry.yaml')) as stream:
        geometry = yaml.safe_load(stream)['vehicle']
    vehicle_geometry = geometry

    # Đường dẫn đến file Xacro gốc
    xacro_path = os.path.join(get_package_share_directory('vehicle_bringup'), 'config', 'simulation.xacro')
    combined_config_path = os.path.join(
        get_package_share_directory('swerve_vehicle_control'), 'config',
        'swerve_controllers.yaml')
    with open(combined_config_path) as stream:
        vehicle_config = yaml.safe_load(stream)
    arm_config = {}
    if arm_enabled:
        if arm_profile == 'pose':
            arm_config_path = os.path.join(
                get_package_share_directory('fr10_pose_controller'),
                'config', 'ros2_controllers.yaml')
        else:
            arm_config_path = os.path.join(
                get_package_share_directory('fr10_arm_control'),
                'config', 'controllers.yaml')
        with open(arm_config_path) as stream:
            arm_config = yaml.safe_load(stream)
    controller_config = compose_controller_config(
        vehicle_config, arm_config, vehicle_enabled, arm_enabled, arm_profile)
    runtime_dir = tempfile.mkdtemp(prefix='vinuni_sim_')
    runtime_config = os.path.join(runtime_dir, 'controllers.yaml')
    with open(runtime_config, 'w') as stream:
        yaml.safe_dump(controller_config, stream)
    force_world = LaunchConfiguration('force_world').perform(context)
    force_fixture_profile = LaunchConfiguration('force_fixture_profile').perform(context)
    force_overlay = LaunchConfiguration('force_controller_overlay').perform(context)
    station_name = LaunchConfiguration('station').perform(context)
    matched_station = force_enabled and bool(station_name)
    if matched_station:
        module_path = os.path.join(get_package_share_directory('vehicle_bringup'), 'launch', 'charging_station.py')
        spec = importlib.util.spec_from_file_location('charging_station', module_path)
        station_module = importlib.util.module_from_spec(spec); spec.loader.exec_module(station_module)
        model_path = Path(get_package_share_directory('vehicle_description'))/'models/matched_socket_v2'
        step_text = LaunchConfiguration('matched_physics_step_s').perform(context)
        physics_step = station_module.matched_physics_step(
            model_path, float(step_text) if step_text else None)
        # Physics can progress below real time during exact CAD contact. Read
        # actual encoders each physics step so wall-time safety sees unique,
        # recent measurements; the motion executor/command stream stays 125 Hz.
        controller_config['controller_manager']['ros__parameters']['update_rate'] = math.ceil(1./physics_step)
        if vehicle_enabled:
            # Wheel odometry cannot own a second physical root TF.
            controller_config['swerve_controller']['ros__parameters']['publish_tf'] = False
        with open(runtime_config, 'w') as stream:
            yaml.safe_dump(controller_config, stream)
    if force_enabled and vehicle_enabled and not matched_station:
        raise RuntimeError('Mounted force requires a matched station:=front_mid; legacy proxy is standalone only.')
    if LaunchConfiguration('demo').perform(context).lower() == 'true':
        raise RuntimeError('Automatic matched demo is locked until geometry, motion and contact acceptance gates pass.')
    ft_wrench_topic = LaunchConfiguration('ft_wrench_topic').perform(context)
    # A parent bringup may declare these pass-through arguments as empty
    # strings.  Resolve the force-only defaults here so the child launch does
    # not turn an omitted override into an invalid path.
    if force_enabled and not force_world:
        force_world = os.path.join(
            get_package_share_directory('fairino10_v6_moveit2_config'),
            'worlds', 'fr10_force_test.world' if force_fixture_profile == 'socket'
            else 'fr10_force_home_test.world')
    if force_enabled and not force_overlay:
        force_overlay = os.path.join(
            get_package_share_directory('fr10_pose_controller'), 'config',
            'matched_socket_v2.yaml' if matched_station else 'gazebo_force_test.yaml')
    force_parameters = load_force_profile(force_overlay,
        get_package_share_directory('fr10_pose_controller')) if force_enabled else {}
    wrist_layout = LaunchConfiguration('wrist_layout', default='calibrated').perform(context)
    if wrist_layout == 'photo_dc_d455' and force_enabled:
        raise ValueError('photo_dc_d455 needs its own female contact fixture; start this model with enable_force_control:=false')
    doc = xacro.process_file(xacro_path, mappings={
        'controller_yaml_file': runtime_config, 'enable_arm': str(arm_enabled).lower(),
        'enable_arm_camera': LaunchConfiguration('enable_arm_camera').perform(context),
        'enable_vehicle': str(vehicle_enabled).lower(),
        'sim_base_collision_proxy': 'true',
        'wrist_layout': wrist_layout,
        'charging_tool_profile': 'matched_socket_v2' if matched_station else 'legacy',
        'enable_force_sensor': str(force_enabled).lower(),
        'force_sensor_sim_gazebo': str(force_enabled).lower(),
        'ft_wrench_topic': ft_wrench_topic})
    if vehicle_enabled:
        validate_vehicle_geometry(geometry, controller_config, doc)
    # Source Xacros retain package://. Resolve the expanded description for Gazebo
    # without exposing every ROS package as an Insert Model candidate.
    for mesh in doc.getElementsByTagName('mesh'):
        uri = mesh.getAttribute('filename')
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            mesh.setAttribute('filename', 'file://' + os.path.join(get_package_share_directory(package), relative))
    # gazebo_ros2_control passes robot_description through ROS parameter parsing.
    # XML comments may contain YAML tokens (notably ': ') that break that path.
    # Strip comments from the runtime XML only; keep source documentation intact.
    def strip_comments(element):
        for child in list(element.childNodes):
            if child.nodeType == child.COMMENT_NODE:
                element.removeChild(child)
            else:
                strip_comments(child)
    strip_comments(doc)
    robot_description = {'robot_description': doc.toxml()}
    station_file = ''
    matched_parameters = {}
    if matched_station:
        station = station_module.select_station(
            Path(get_package_share_directory('vehicle_bringup'))/'config/charging_stations.yaml',
            model_path/'geometry.json', station_name, doc.toxml(), vehicle_enabled,
            preinsert_distance_m=force_parameters.get('force.goal.approach_distance_m'))
        station['mounted'] = vehicle_enabled
        station['model_path'] = str(model_path)
        station['initial_state'] = LaunchConfiguration('initial_state').perform(context)
        robot_description['robot_description'] = station_module.initialize_arm_description(
            doc.toxml(), station, station['initial_state'])
        station_file = os.path.join(runtime_dir, 'charging_station.json')
        with open(station_file, 'w') as stream: json.dump(station, stream, indent=2)
        force_world = os.path.join(runtime_dir, 'matched_socket.world')
        iterations_text = LaunchConfiguration('matched_solver_iterations').perform(context)
        iterations = int(iterations_text) if iterations_text else None
        station_module.write_world(station, model_path, force_world, physics_step,
                                   solver_iterations=iterations)
        geometry = json.loads((model_path/'geometry.json').read_text())
        matched_parameters = {
            'force.goal.approach_distance_m': station['preinsert_distance_m'],
            'force.goal.insertion_depth_m': station['insertion_depth_m'],
            'force.gravity_compensation.tool_mass_kg': geometry['tool_mass_kg'],
            'force.gravity_compensation.tool_cog_m': geometry['tool_cog_sensor_m'],
            'force.geometry.mating_clearance_m': geometry['simulation_design']['radial_clearance_mm']*.001,
            'force.search.xy_free_half_span_m': force_parameters.get(
                'force.search.xy_free_half_span_m', station['lateral_free_half_span_m']),
            'force.search.yaw_free_half_span_rad': force_parameters.get(
                'force.search.yaw_free_half_span_rad', station['keyed_yaw_free_half_span_rad']),
            'force.geometry.mating_feature_lever_m': .001*max(
                math.hypot(math.hypot(*pin['center_mm'])+pin['bore_inscribed_radius_mm'],
                           geometry['simulation_design']['nose_length_mm']) for pin in geometry['pins'])}
        if vehicle_enabled:
            matched_parameters.update({'force.base_motion_topic': '/fr10_sim/base_odometry',
                                       'force.base_motion_frame': 'world',
                                       'force.task_frame': 'world',
                                       'force.base_motion_twist_source': 'pose'})
            for policy in ('free_space', 'insertion'):
                for parameter, suffix in (
                        ('joint_velocity_scale', 'velocity_scale'),
                        ('joint_acceleration_scale', 'acceleration_scale'),
                        ('joint_jerk_scale', 'jerk_scale'),
                        ('linear_scale', 'linear_velocity_scale')):
                    key = f'precision_arm_{policy}_{suffix}'
                    value = vehicle_geometry[key]
                    if not math.isfinite(value) or not 0. < value <= 1.:
                        raise ValueError(f'Invalid mounted {policy} limit: {key}')
                    matched_parameters[f'controller.pose_policies.{policy}.{parameter}'] = value
            unload_speed = vehicle_geometry['precision_arm_retract_initial_velocity_m_s']
            if not math.isfinite(unload_speed) or not 0. < unload_speed <= .000025:
                raise ValueError('Invalid mounted loaded-retract speed')
            # Loaded retraction is enabled by a positive speed/distance pair in
            # the selected force profile. Keep it disabled in the draft profile;
            # overriding only its speed would make controller startup fail.
            if (force_parameters.get('force.retract_initial_velocity_m_s', 0.) > 0.
                    and force_parameters.get('force.retract_initial_distance_m', 0.) > 0.):
                matched_parameters['force.retract_initial_velocity_m_s'] = unload_speed

    # 3. Node: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, 
        {'use_sim_time': True}]
    )

    # Python helpers (including Gazebo's env-python3 spawner) must share the
    # launch interpreter's ROS ABI even when Conda comes first on PATH.
    # 4. Node: Spawn Entity
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        prefix=[sys.executable],
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_mobile_manipulator',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    world_file_path = (force_world if force_enabled else
                      LaunchConfiguration('world').perform(context) or os.path.join(
                          get_package_share_directory(pkg_name), 'worlds', 'vinfast_map.world'))
    world_file_path = os.path.expanduser(world_file_path)
    if not os.path.isfile(world_file_path):
        raise ValueError(f'Gazebo world does not exist: {world_file_path}')
    if force_enabled and not os.path.isfile(world_file_path):
        raise RuntimeError(f'Force test world does not exist: {world_file_path}')
    if force_enabled:
        # Gazebo Classic does not consistently resolve a package share as a
        # model:// URI across installations.  Materialize the selected force
        # world with an absolute file:// URI so the detailed charging-port CAD
        # visual is always loaded, including when the workspace is copied.
        canonical_mesh = os.path.join(
            get_package_share_directory('vehicle_description'), 'meshes',
            'cong_sac', 'meshes', 'cong_sac.stl')
        if not os.path.isfile(canonical_mesh):
            raise RuntimeError(f'Canonical CCS2 mesh does not exist: {canonical_mesh}')
        with open(world_file_path, encoding='utf-8') as stream:
            force_world_xml = stream.read()
        for mesh_uri in (
                'model://cong_sac/meshes/cong_sac.stl',
                'model://vehicle_description/meshes/jlr_ccs2_centered.stl'):
            force_world_xml = force_world_xml.replace(
                mesh_uri, 'file://' + canonical_mesh)
        runtime_world_path = os.path.join(runtime_dir, 'force_world.world')
        with open(runtime_world_path, 'w', encoding='utf-8') as stream:
            stream.write(force_world_xml)
        world_file_path = runtime_world_path

    # Force execution advances one 8 ms sample per successful tick and pauses
    # while ROS simulation time is unchanged. Gazebo's default 10 Hz /clock
    # would therefore throttle the whole preinsert path to about 10 ticks/s.
    # Use the same physics-clock configuration as the force regression runs.
    gazebo_arguments = {
        'world': world_file_path, 'server_required': 'true',
        'use_sim_time': 'true', 'gui': LaunchConfiguration('gui'),
    }
    gazebo_arguments['params_file'] = os.path.join(
        get_package_share_directory('fr10_pose_controller'), 'config',
        'gazebo_runtime_clock.yaml')

    # 5. Include Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments=gazebo_arguments.items()
    )

    # 6. Spawn Controllers
    
    # 6.1 Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    def include(package, filename, **kwargs):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', filename)), **kwargs)
    vehicle_controllers = include('swerve_vehicle_control', 'swerve_controller.launch.py',
        launch_arguments={'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                          'controller_parameters_file': runtime_config}.items())
    if arm_profile == 'pose' and arm_enabled:
        pose_share = get_package_share_directory('fr10_pose_controller')
        semantic_path = os.path.join(
            get_package_share_directory('vehicle_description'), 'config',
            'fr10.srdf')
        with open(semantic_path) as stream:
            semantic_description = stream.read()
        if matched_station:
            # Only actual assembled tool interfaces; keep all camera/arm/world
            # collision pairs active. No broad tool/environment ACM exemption.
            assembly = ET.parse(model_path/'matched_connector.srdf').getroot()
            entries = ''.join(ET.tostring(entry,encoding='unicode') for entry in assembly)
            semantic_description = semantic_description.replace('</robot>', entries+'</robot>')
        with open(os.path.join(
                get_package_share_directory('fairino10_v6_moveit2_config'),
                'config', 'kinematics.yaml')) as stream:
            kinematics_config = yaml.safe_load(stream)
        arm_controllers = Node(
            package='controller_manager', executable='spawner',
            arguments=['fairino10_controller'], output='screen')
        spec = importlib.util.spec_from_file_location('planning_collision_model',
            os.path.join(pose_share,'launch','planning_collision_model.py'))
        collision_model = importlib.util.module_from_spec(spec); spec.loader.exec_module(collision_model)
        planning_description = {'robot_description': collision_model.planning_collision_description(
            robot_description['robot_description'],
            os.path.join(get_package_share_directory('vehicle_description'),'models','matched_socket_v2'))}
        tool_spec = importlib.util.spec_from_file_location('tool_geometry',
            os.path.join(get_package_share_directory('vehicle_description'),
                         'launch', 'tool_geometry.py'))
        tool_geometry = importlib.util.module_from_spec(tool_spec)
        tool_spec.loader.exec_module(tool_geometry)
        tool_parameters = tool_geometry.wrench_parameters(
            robot_description['robot_description'],
            force_parameters.get('force.wrench_source_frame', 'charging_connector_link'),
            force_parameters.get('force.wrench_target_frame', 'charging_tcp'))
        pose_node = Node(
            package='fr10_pose_controller',
            executable='fr10_pose_controller_node',
            name='fr10_pose_controller', output='screen',
            parameters=[
                os.path.join(pose_share, 'config', 'controller.yaml'),
                os.path.join(get_package_share_directory('vehicle_bringup'),
                             'config', 'fr10_vehicle_pose.yaml'),
                *([force_parameters] if force_enabled else []),
                robot_description,
                planning_description,
                matched_parameters,
                tool_parameters,
                {'robot_description_semantic': semantic_description,
                 'robot_description_kinematics': kinematics_config,
                 'use_sim_time': True},
            ])
    else:
        arm_controllers = include('fr10_arm_control', 'controllers.launch.py')
        pose_node = None
    # 7. Sự kiện (Event Handlers)
    # Đợi spawn robot xong -> Bật Joint State Broadcaster
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_spawn_entity,
            on_exit=lambda event, context: station_spawn_transition(event, joint_state_broadcaster_spawner),
        )
    )

    # Đợi Joint State Broadcaster xong -> Bật 3 Controller còn lại (Tay + Chân)
    load_base_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=lambda event, context: (
                [Shutdown(reason=f'Joint state broadcaster failed: {event.returncode}')]
                if event.returncode != 0 else
                ([vehicle_controllers] if vehicle_enabled else []) +
                ([arm_controllers] if arm_enabled else [])),
        )
    )
    load_pose_after_arm_controller = (
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=arm_controllers,
            on_exit=lambda event, context: arm_controller_transition(event, pose_node),
        )) if pose_node is not None else None
    )
    description_share_path = get_package_share_directory('vehicle_description')
    external_models_path = os.path.join(description_share_path, 'models')
    # Keep the package share parent available for legacy model:// assets; the
    # selected force world itself is rewritten to an absolute mesh URI below.
    description_model_root = os.path.dirname(description_share_path)

    station_sdf_path = os.path.join(external_models_path, 'tram_sac_VF', 'model.sdf')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.pathsep.join([external_models_path, description_model_root]))
    # Gazebo Classic's public model database is no longer reliable. Missing
    # legacy meshes should not block this local simulation during startup.
    disable_online_model_database = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )
    rviz_profile = 'simulation_vehicle.rviz' if vehicle_enabled else 'simulation_arm.rviz'
    rviz_config_file = os.path.join(
        get_package_share_directory('vehicle_bringup'), 'rviz', rviz_profile)
    if navigation_mode != 'disabled':
        rviz_config_file = str(navigation_share / 'rviz/navigation.rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file').perform(context) or rviz_config_file
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file, '-f', 'world' if matched_station and vehicle_enabled else
                   'map' if navigation_mode != 'disabled' else
                   'odom' if vehicle_enabled else 'fr10_base_link'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    detection = include('charger_detection', 'detection.launch.py',
        condition=IfCondition(LaunchConfiguration('enable_detection')))
    node_clock_ready = Node(
        package='vehicle_bringup', executable='wait_for_gazebo_clock.py',
        prefix=[sys.executable], output='screen')
    force_fixture = Node(
        package='vehicle_bringup', executable='matched_station_publisher.py' if matched_station else 'force_fixture_publisher.py',
        prefix=[sys.executable], output='screen',
        parameters=[{'fixture_profile': force_fixture_profile, 'station_file': station_file,
                     'use_sim_time': True}])
    node_spawn_station = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        prefix=[sys.executable],
        arguments=[
            '-entity', 'tram_sac_VF1',
            '-file', station_sdf_path, # Dùng đường dẫn tuyệt đối vừa tạo
            '-x', '10.5',   # Tọa độ X
            '-y', '0.0',   # Tọa độ Y
            '-z', '0.0',
            '-Y', '-1.57'   # Quay ngược lại nhìn robot
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    node_spawn_station2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        prefix=[sys.executable],
        arguments=[
            '-entity', 'tram_sac_VF2',
            '-file', station_sdf_path, 
            '-x', '10.5',  
            '-y', '4.0',   
            '-z', '0.0',
            '-Y', '-1.57'  
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    node_spawn_station3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        prefix=[sys.executable],
        arguments=[
            '-entity', 'tram_sac_VF3',
            '-file', station_sdf_path, 
            '-x', '0.0',   # Tọa độ X
            '-y', '10.5',   # Tọa độ Y
            '-z', '0.0',
            '-Y', '0.0'   # Quay vuông góc so với xe 
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    def start_next_on_success(current, following):
        return RegisterEventHandler(event_handler=OnProcessExit(
            target_action=current,
            on_exit=lambda event, context: station_spawn_transition(event, following)))

    navigation_actions = []
    if navigation_mode != 'disabled':
        gate = Node(package='vehicle_bringup', executable='wait_for_navigation.py',
                    prefix=[sys.executable], output='screen', parameters=[{'use_sim_time': True,
                        'scan_topic': LaunchConfiguration('scan_topic').perform(context),
                        'odom_topic': LaunchConfiguration('odom_topic').perform(context),
                        'timeout': float(LaunchConfiguration('navigation_startup_timeout').perform(context))}])
        navigation = include('vehicle_navigation', 'navigation.launch.py',
            launch_arguments={name: LaunchConfiguration(name) for name in (
                'navigation_mode', 'map', 'nav2_params_file', 'slam_params_file',
                'scan_topic', 'odom_topic', 'cmd_vel_topic', 'autostart', 'use_respawn',
                'log_level')}.items())
        navigation_actions = [start_next_on_success(gate, navigation), gate]

    return [
        SetEnvironmentVariable(name='GAZEBO_MASTER_URI', value=f'http://127.0.0.1:{gazebo_port}'),
        AppendEnvironmentVariable(name='GAZEBO_PLUGIN_PATH',
                                  value=os.path.join(get_package_prefix('vehicle_bringup'), 'lib')),
        set_gazebo_model_path,
        disable_online_model_database,
        gazebo,
        # publish_initial_pose,
        node_robot_state_publisher,
        *([] if vehicle_enabled else [node_spawn_entity]),
        load_joint_state_broadcaster,
        load_base_controllers,
        *([load_pose_after_arm_controller] if load_pose_after_arm_controller else []),
        *([start_next_on_success(node_clock_ready, node_spawn_station),
           start_next_on_success(node_spawn_station, node_spawn_station2),
           start_next_on_success(node_spawn_station2, node_spawn_station3),
           start_next_on_success(node_spawn_station3, node_spawn_entity),
           node_clock_ready, detection] if vehicle_enabled and not matched_station else []),
        *([start_next_on_success(node_clock_ready, node_spawn_entity), node_clock_ready]
          if vehicle_enabled and matched_station else []),
        *([force_fixture] if force_enabled else []),
        rviz,
        *navigation_actions,

    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_vehicle', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_detection', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('gazebo_port', default_value='11355', description='Private Gazebo port for the vehicle simulation'),
        DeclareLaunchArgument('enable_arm', default_value='true', choices=['true', 'false'],
                              description='Include FR10, D435, charger and arm controller'),
        DeclareLaunchArgument('arm_controller_profile', default_value='jtc', choices=['jtc', 'pose']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('navigation_mode', default_value='disabled',
                             choices=['disabled', 'mapping', 'localization', 'navigation']),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=''),
        DeclareLaunchArgument('slam_params_file', default_value=''),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_respawn', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('navigation_startup_timeout', default_value='90.0'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('rviz_config_file', default_value=''),
        DeclareLaunchArgument(
            'enable_force_control', default_value='false', choices=['true', 'false'],
            description='Test-only Gazebo force insertion; requires standalone arm.'),
        DeclareLaunchArgument(
            'force_world', default_value='',
            description='Optional world override; otherwise selected by force_fixture_profile.'),
        DeclareLaunchArgument(
            'force_fixture_profile', default_value='home',
            choices=['home', 'socket'],
            description='Planning proxy pose: canonical home fixture or the standalone socket fixture.'),
        DeclareLaunchArgument(
            'force_controller_overlay', default_value=''),
        DeclareLaunchArgument('station', default_value=''),
        DeclareLaunchArgument('initial_state', default_value='home'),
        DeclareLaunchArgument('demo', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('ft_wrench_topic', default_value='wrench'),
        DeclareLaunchArgument('matched_physics_step_s', default_value='',
                             description='Explicit matched ODE convergence study step, 50 us..1 ms; empty uses calibrated default.'),
        DeclareLaunchArgument('matched_solver_iterations', default_value='',
                             description='Commissioning-only ODE iteration study, 50..2000; empty retains 2000.'),
        DeclareLaunchArgument('enable_arm_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('wrist_layout', default_value='calibrated', choices=['calibrated', 'photo_dc_d455']),
        OpaqueFunction(function=launch_setup),
    ])
