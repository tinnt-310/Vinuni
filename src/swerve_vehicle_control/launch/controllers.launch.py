from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package='controller_manager', executable='spawner',
        arguments=[name], parameters=[{'use_sim_time': True}], output='screen')
        for name in ['steering_controller', 'drive_controller']])
