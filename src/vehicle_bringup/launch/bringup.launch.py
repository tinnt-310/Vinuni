"""Select components and backend; never substitute mock feedback for PLC feedback."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def select_backend(arm, vehicle, mode):
    if mode not in ('sim', 'real'):
        raise ValueError('mode must be sim or real')
    if not arm and not vehicle:
        raise ValueError('Select at least one component: arm:=true or vehicle:=true')
    if vehicle and mode == 'real':
        raise RuntimeError('PLC backend not commissioned: missing encoder/angle/speed/alarm '
                           'feedback registers, speed calibration and watchdog mapping. '
                           'No hardware nodes have been started.')
    if mode == 'sim':
        return 'vehicle_bringup', 'simulation.launch.py'
    return 'fairino10_v6_moveit2_config', ('gazebo.launch.py' if mode == 'sim' else 'demo.launch.py')


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    arm, vehicle, mode = value('arm') == 'true', value('vehicle') == 'true', value('mode')
    package, filename = select_backend(arm, vehicle, mode)
    if mode == 'sim':
        arguments = {'enable_vehicle': str(vehicle).lower(), 'enable_arm': str(arm).lower(), 'gui': value('gui'),
                     'rviz': value('rviz'), 'gazebo_port': value('gazebo_port'),
                     'cmd_vel_topic': value('cmd_vel_topic')}
    else:
        arguments = {'use_fake_hardware': 'false', 'use_gazebo': 'false',
                     'robot_ip': value('robot_ip')}
    actions = []
    if mode == 'real' and not vehicle:
        actions.append(LogInfo(msg='Standalone FR10 uses the vendor MoveIt tool configuration; '
                               'it does not yet use the vehicle D435/CCS2 assembly.'))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename)),
        launch_arguments=arguments.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('vehicle', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument('robot_ip', default_value='192.168.58.2'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_port', default_value='11355'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        OpaqueFunction(function=setup),
    ])
