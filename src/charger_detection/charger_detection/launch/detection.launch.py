from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package='charger_detection', executable='ccs2_detector_node',
        parameters=[{'use_sim_time': True}], output='screen',
        remappings=[('/base_camera/base_camera_sensor/image_raw', '/body_camera/d455/image_raw'),
                    ('/base_camera/base_camera_sensor/depth/image_raw', '/body_camera/d455/depth/image_raw')])])
