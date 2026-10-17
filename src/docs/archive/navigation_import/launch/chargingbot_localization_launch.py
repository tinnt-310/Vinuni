# Copyright 2024 cei
#
# Licensed under the MIT License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    map_yaml_file = LaunchConfiguration('map')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the localization stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes.')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([FindPackageShare('chargingbot_nav_ros2'),
                                           'maps', 'eai_map.yaml']),
        description='Full path to map yaml file to load')

    declare_amcl_params_file_cmd = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=PathJoinSubstitution([FindPackageShare('chargingbot_nav_ros2'),
                                           'config', 'nav2_params', 'amcl_params.yaml']),
        description='Full path to the ROS2 parameters file to use for AMCL')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    # Include the ChargingBot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chargingbot_description_ros2'),
                'launch',
                'chargingbot_description_launch.py'
            ])
        ]),
        condition=IfCondition(use_robot_state_pub)
    )

    # Map server
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}],
        remappings=remappings)

    # AMCL
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[amcl_params_file,
                    {'use_sim_time': use_sim_time}],
        remappings=remappings)

    # Lifecycle manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server', 'amcl']}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_amcl_params_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to launch
    ld.add_action(robot_description_launch)
    ld.add_action(map_server_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
