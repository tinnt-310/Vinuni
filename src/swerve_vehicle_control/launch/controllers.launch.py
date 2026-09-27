"""Deprecated compatibility wrapper for vehicle_bringup's existing launch graph."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('swerve_vehicle_control'),
                          'config', 'swerve_controllers.yaml')
    return LaunchDescription([
        LogInfo(msg='controllers.launch.py is deprecated; use swerve_controller.launch.py'),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['swerve_controller', '-c', '/controller_manager',
                        '-t', 'swerve_vehicle_control/SwerveDriveController', '-p', params,
                        '--controller-manager-timeout', '60']),
    ])
