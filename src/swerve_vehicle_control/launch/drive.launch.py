"""Deprecated compatibility wrapper: only normalizes legacy Twist commands."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        LogInfo(msg='drive.launch.py is deprecated; use swerve_controller.launch.py'),
        Node(package='swerve_vehicle_control', executable='cmd_vel_normalizer.py',
             output='screen', parameters=[{'use_sim_time': True,
                                           'input_topic': LaunchConfiguration('cmd_vel_topic'),
                                           'output_topic': '/cmd_vel_nav'}]),
    ])
