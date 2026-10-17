import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import yaml
import tempfile
import socket
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import AppendEnvironmentVariable, SetEnvironmentVariable


def launch_setup(context):
    gazebo_port = int(LaunchConfiguration('gazebo_port').perform(context))
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', gazebo_port)) == 0:
            raise RuntimeError(f'Gazebo port {gazebo_port} is already in use. Stop the previous launch or set gazebo_port:=another_port.')
    vehicle_enabled = LaunchConfiguration('enable_vehicle').perform(context).lower() == 'true'
    arm_enabled = LaunchConfiguration('enable_arm').perform(context).lower() == 'true'
    pkg_name = 'vehicle_description'
    cpp_pkg_name = "swerve_vehicle_control"
    nav2_pkg_name = 'nav2_bringup'

    map_file_path = os.path.join(get_package_share_directory('vehicle_bringup'), 'maps', 'my_vinfast_map.yaml')
    nav2_params_path = os.path.join(get_package_share_directory('vehicle_bringup'), 'config', 'nav2_params.yaml')



    with open(os.path.join(get_package_share_directory(pkg_name), 'config', 'vehicle_geometry.yaml')) as stream:
        geometry = yaml.safe_load(stream)['vehicle']

    # Đường dẫn đến file Xacro gốc
    xacro_path = os.path.join(get_package_share_directory('vehicle_bringup'), 'config', 'simulation.xacro')
    combined_config_path = os.path.join(get_package_share_directory('swerve_vehicle_control'), 'config', 'controllers.yaml')
    # Filter controller configuration together with the conditional URDF.
    with open(combined_config_path) as stream:
        controller_config = yaml.safe_load(stream)
    if not vehicle_enabled:
        controller_config = {'controller_manager': {'ros__parameters': {'update_rate': 100,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'}}}}
    if arm_enabled:
        with open(os.path.join(get_package_share_directory('fr10_arm_control'), 'config', 'controllers.yaml')) as stream:
            controller_config.update(yaml.safe_load(stream))
        controller_config['controller_manager']['ros__parameters']['joint_trajectory_controller'] = {
            'type': 'joint_trajectory_controller/JointTrajectoryController'}
    runtime_dir = tempfile.mkdtemp(prefix='vinuni_sim_')
    runtime_config = os.path.join(runtime_dir, 'controllers.yaml')
    with open(runtime_config, 'w') as stream:
        yaml.safe_dump(controller_config, stream)
    doc = xacro.process_file(xacro_path, mappings={
        'controller_yaml_file': runtime_config, 'enable_arm': str(arm_enabled).lower(),
        'enable_vehicle': str(vehicle_enabled).lower()})
    # Gazebo-only base collision: vendor STL reaches the ground plane while
    # the standalone base is already constrained by world_to_fr10. Avoid that
    # redundant contact with a 1 mm clearance. Bounds: radius .095, top .0848 m.
    # Visual pose, inertial data and all articulated/tool collisions stay intact.
    for link in doc.getElementsByTagName('link'):
        if link.getAttribute('name') != 'fr10_base_link':
            continue
        for collision in link.getElementsByTagName('collision'):
            geometry_node = collision.getElementsByTagName('geometry')[0]
            for child in list(geometry_node.childNodes):
                geometry_node.removeChild(child)
            cylinder = doc.createElement('cylinder')
            cylinder.setAttribute('radius', '0.095')
            cylinder.setAttribute('length', '0.0838')
            geometry_node.appendChild(cylinder)
            origin = collision.getElementsByTagName('origin')[0]
            origin.setAttribute('xyz', '0 0 0.0429')
            origin.setAttribute('rpy', '0 0 0')
    doc.documentElement.appendChild(__import__('xml.dom.minidom', fromlist=['parseString']).parseString('<gazebo><plugin name="ode_contact_probe" filename="/tmp/libode_contact_probe.so"/></gazebo>').documentElement)
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
                   '-x', '0.0', '-y', '0.0', '-z', '0.1' if vehicle_enabled else '0.0'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    world_file_path = os.path.join(
        get_package_share_directory(pkg_name), '/home/tin/Vinuni/docs/ode_debug/vehicle_prod_matrix_V3_20260927/world.sdf'
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
    vehicle_controllers = Node(package='controller_manager', executable='spawner', arguments=['drive_controller'])
    arm_controllers = include('fr10_arm_control', 'controllers.launch.py')
    # 7. Sự kiện (Event Handlers)
    # Đợi spawn robot xong -> Bật Joint State Broadcaster
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Đợi Joint State Broadcaster xong -> Bật 3 Controller còn lại (Tay + Chân)
    load_base_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=([vehicle_controllers] if vehicle_enabled else []) + ([arm_controllers] if arm_enabled else []), 
        )
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

    node_swerve_drive = include('swerve_vehicle_control', 'drive.launch.py',
        launch_arguments={'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')}.items())
    detection = include('charger_detection', 'detection.launch.py',
        condition=IfCondition(LaunchConfiguration('enable_detection')))
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


    return [
        SetEnvironmentVariable(name='GAZEBO_MASTER_URI', value=f'http://127.0.0.1:{gazebo_port}'),
        AppendEnvironmentVariable(name='GAZEBO_PLUGIN_PATH',
                                  value=os.path.join(get_package_prefix('vehicle_bringup'), 'lib')),
        set_gazebo_model_path,
        disable_online_model_database,
        gazebo,
        # publish_initial_pose,
        node_robot_state_publisher,
        node_spawn_entity,
        load_joint_state_broadcaster,
        load_base_controllers,
        *([node_spawn_station, node_spawn_station2, node_spawn_station3, detection] if vehicle_enabled else []),
        rviz,

    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_vehicle', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_detection', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('gazebo_port', default_value='11355', description='Private Gazebo port for the vehicle simulation'),
        DeclareLaunchArgument('enable_arm', default_value='true', choices=['true', 'false'],
                              description='Include FR10, D435, charger and arm controller'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        OpaqueFunction(function=launch_setup),
    ])
