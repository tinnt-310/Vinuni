import os
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
from copy import deepcopy
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import AppendEnvironmentVariable, SetEnvironmentVariable


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
    pkg_name = 'vehicle_description'
    cpp_pkg_name = "swerve_vehicle_control"
    nav2_pkg_name = 'nav2_bringup'

    map_file_path = os.path.join(get_package_share_directory('vehicle_bringup'), 'maps', 'my_vinfast_map.yaml')
    nav2_params_path = os.path.join(get_package_share_directory('vehicle_bringup'), 'config', 'nav2_params.yaml')



    with open(os.path.join(get_package_share_directory(pkg_name), 'config', 'vehicle_geometry.yaml')) as stream:
        geometry = yaml.safe_load(stream)['vehicle']

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
    doc = xacro.process_file(xacro_path, mappings={
        'controller_yaml_file': runtime_config, 'enable_arm': str(arm_enabled).lower(),
        'enable_vehicle': str(vehicle_enabled).lower(),
        'sim_base_collision_proxy': 'true'})
    if vehicle_enabled:
        validate_vehicle_geometry(geometry, controller_config, doc)
    # doc.documentElement.appendChild(__import__('xml.dom.minidom', fromlist=['parseString']).parseString('<gazebo><plugin name="ode_contact_probe" filename="/tmp/libode_contact_probe.so"/></gazebo>').documentElement)
    from pathlib import Path
    Path(os.environ['ODE_OUT']+'/model.urdf').write_text(doc.toxml())
    Path(os.environ['ODE_OUT']+'/controllers.yaml').write_text(yaml.safe_dump(controller_config))
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

    # 3. Node: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, 
        {'use_sim_time': True}]
    )

    # 4. Node: Spawn Entity
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_mobile_manipulator',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    world_file_path = os.path.join(
        get_package_share_directory(pkg_name), '/home/tinnt/Vinuni/docs/ode_debug/rb_v3_passive_20260929/world.sdf'
    )

    # 5. Include Gazebo 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path, 'server_required': 'true', 'use_sim_time': 'true', 'gui': LaunchConfiguration('gui')}.items()
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
        launch_arguments={'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')}.items())
    if arm_profile == 'pose' and arm_enabled:
        pose_share = get_package_share_directory('fr10_pose_controller')
        semantic_path = os.path.join(
            get_package_share_directory('vehicle_description'), 'config',
            'fr10.srdf')
        with open(semantic_path) as stream:
            semantic_description = stream.read()
        with open(os.path.join(
                get_package_share_directory('fairino10_v6_moveit2_config'),
                'config', 'kinematics.yaml')) as stream:
            kinematics_config = yaml.safe_load(stream)
        arm_controllers = Node(
            package='controller_manager', executable='spawner',
            arguments=['fairino10_controller'], output='screen')
        pose_node = Node(
            package='fr10_pose_controller',
            executable='fr10_pose_controller_node',
            name='fr10_pose_controller', output='screen',
            parameters=[
                os.path.join(pose_share, 'config', 'controller.yaml'),
                os.path.join(get_package_share_directory('vehicle_bringup'),
                             'config', 'fr10_vehicle_pose.yaml'),
                robot_description,
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
    external_models_path = os.path.join(get_package_share_directory('vehicle_description'), 'models')

    station_sdf_path = os.path.join(external_models_path, 'tram_sac_VF', 'model.sdf')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=external_models_path)
    # Gazebo Classic's public model database is no longer reliable. Missing
    # legacy meshes should not block this local simulation during startup.
    disable_online_model_database = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )
    rviz_config_file = os.path.join(get_package_share_directory('vehicle_bringup'), 'rviz', 'SLAM.rviz')
    rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file, '-f', 'odom' if vehicle_enabled else 'fr10_base_link'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    detection = include('charger_detection', 'detection.launch.py',
        condition=IfCondition(LaunchConfiguration('enable_detection')))
    node_clock_ready = Node(
        package='vehicle_bringup', executable='wait_for_gazebo_clock.py',
        output='screen')
    node_spawn_station = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
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


        *([load_pose_after_arm_controller] if load_pose_after_arm_controller else []),
        *([start_next_on_success(node_clock_ready, node_spawn_station),
           start_next_on_success(node_spawn_station, node_spawn_station2),
           start_next_on_success(node_spawn_station2, node_spawn_station3),
           start_next_on_success(node_spawn_station3, node_spawn_entity),
           node_clock_ready, detection] if vehicle_enabled else []),
        rviz,

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
        OpaqueFunction(function=launch_setup),
    ])
