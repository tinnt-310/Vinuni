"""Start the mission manager with stations and route loaded from ROS parameters."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    stations = os.path.join(get_package_share_directory('swerve_vehicle_control'),
                            'config', 'stations.yaml')
    return LaunchDescription([
        Node(package='swerve_vehicle_control', executable='mission_manager',
             output='screen', emulate_tty=True, parameters=[stations]),
    ])
