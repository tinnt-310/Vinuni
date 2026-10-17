"""Internal navigation stack; vehicle_bringup owns the robot and RViz."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def validate_map(filename):
    path = Path(filename).expanduser()
    if not path.is_file():
        raise ValueError(f'Map YAML does not exist: {path}')
    with path.open() as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('image'), str):
        raise ValueError(f'Map YAML must contain an image path: {path}')
    image = Path(data['image']).expanduser()
    if not image.is_absolute():
        image = path.parent / image
    if not image.is_file():
        raise ValueError(f'Map image does not exist: {image}')
    return str(path.resolve())


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value('navigation_mode')
    if mode == 'disabled':
        return []
    share = Path(get_package_share_directory('vehicle_navigation'))
    sim_time = value('use_sim_time').lower() == 'true'
    autostart = value('autostart').lower() == 'true'
    respawn = value('use_respawn').lower() == 'true'
    raw_topic = '/navigation/cmd_vel'
    output_topic = value('cmd_vel_topic')
    if output_topic.lstrip('/') in ('navigation/cmd_vel', 'cmd_vel_nav'):
        raise ValueError('cmd_vel_topic conflicts with an internal velocity topic')
    params_path = Path(value('nav2_params_file') or share / 'config/nav2.yaml').expanduser()
    slam_path = Path(value('slam_params_file') or share / 'config/slam.yaml').expanduser()
    for path in ([slam_path] if mode == 'mapping' else [params_path]):
        if not path.is_file():
            raise ValueError(f'Parameter file does not exist: {path}')
    rewrites = {'use_sim_time': str(sim_time).lower(),
                'scan_topic': value('scan_topic'), 'odom_topic': value('odom_topic'),
                'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': value('scan_topic'),
                'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': value('scan_topic')}
    if mode in ('localization', 'navigation'):
        rewrites['yaml_filename'] = validate_map(value('map') or share / 'maps/vinfast.yaml')
    params = ParameterFile(RewrittenYaml(
        source_file=str(params_path), param_rewrites=rewrites, convert_types=True))
    actions = []

    def node(package, executable, name=None, remappings=None, parameters=None):
        return Node(package=package, executable=executable, name=name or executable,
                    output='screen', respawn=respawn, respawn_delay=2.0,
                    parameters=parameters if parameters is not None else [params],
                    remappings=remappings or [],
                    arguments=['--ros-args', '--log-level', value('log_level')])

    def manager(name, nodes):
        return node('nav2_lifecycle_manager', 'lifecycle_manager', name=name,
                    parameters=[{'use_sim_time': sim_time, 'autostart': autostart,
                                 'node_names': nodes}])

    if mode == 'mapping':
        slam_params = ParameterFile(RewrittenYaml(
            source_file=str(slam_path), param_rewrites={
                'use_sim_time': str(sim_time).lower(), 'scan_topic': value('scan_topic')},
            convert_types=True))
        actions.append(node('slam_toolbox', 'async_slam_toolbox_node', 'slam_toolbox',
                            parameters=[slam_params]))
    else:
        actions += [node('nav2_map_server', 'map_server'), node('nav2_amcl', 'amcl'),
                    manager('lifecycle_manager_localization', ['map_server', 'amcl'])]
    if mode == 'navigation':
        specifications = [
            ('nav2_controller', 'controller_server'),
            ('nav2_smoother', 'smoother_server'),
            ('nav2_planner', 'planner_server'),
            ('nav2_behaviors', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator'),
            ('nav2_waypoint_follower', 'waypoint_follower'),
            ('nav2_velocity_smoother', 'velocity_smoother'),
        ]
        for package, executable in specifications:
            remaps = [('cmd_vel', raw_topic)] if executable in (
                'controller_server', 'behavior_server', 'velocity_smoother') else []
            if executable == 'velocity_smoother':
                remaps += [('cmd_vel_smoothed', output_topic)]
            actions.append(node(package, executable, remappings=remaps))
        actions.append(manager('lifecycle_manager_navigation', [name for _, name in specifications]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('navigation_mode', default_value='disabled',
                              choices=['disabled', 'mapping', 'localization', 'navigation']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=''),
        DeclareLaunchArgument('slam_params_file', default_value=''),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_respawn', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=setup),
    ])
