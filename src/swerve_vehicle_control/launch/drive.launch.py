import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    with open(os.path.join(get_package_share_directory('vehicle_description'), 'config', 'vehicle_geometry.yaml')) as stream:
        geometry = yaml.safe_load(stream)['vehicle']
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        Node(package='swerve_vehicle_control', executable='swerve_drive', output='screen',
             parameters=[{'use_sim_time': True, 'wheelbase': geometry['wheelbase'],
                          'track_width': geometry['track_width'], 'wheel_radius': geometry['wheel_radius'],
                          'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')}]),
    ])
