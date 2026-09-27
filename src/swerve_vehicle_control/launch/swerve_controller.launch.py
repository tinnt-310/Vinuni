"""Attach the swerve plugin to a running controller_manager and normalize Nav2 Twist."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('swerve_vehicle_control'),
                          'config', 'swerve_controllers.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('controller_manager', default_value='/controller_manager'),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['swerve_controller', '-c', LaunchConfiguration('controller_manager'),
                        '-t', 'swerve_vehicle_control/SwerveDriveController', '-p', params,
                        '--controller-manager-timeout', '60']),
        Node(package='swerve_vehicle_control', executable='cmd_vel_normalizer.py',
             output='screen', parameters=[{'use_sim_time': True,
                                           'input_topic': LaunchConfiguration('cmd_vel_topic'),
                                           'output_topic': '/cmd_vel_nav'}]),
    ])
