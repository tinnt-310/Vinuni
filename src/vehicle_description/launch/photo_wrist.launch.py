"""Read-only RViz view of the nominal photo-based CCS2 DC male / D455 wrist."""
from pathlib import Path
import math
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('vehicle_description'))
    robot = xacro.process_file(str(share/'urdf/vehicle.urdf.xacro'), mappings={
        'enable_arm': 'true', 'enable_vehicle': 'false', 'enable_arm_camera': 'false',
        'wrist_layout': 'photo_dc_d455', 'control_backend': 'none'}).toxml()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'robot_description': robot, 'zeros.j2': -math.pi/2,
                          'zeros.j3': math.pi/2, 'zeros.j4': -math.pi/2, 'zeros.j5': -math.pi/2}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'rviz/photo_wrist.rviz')])])
