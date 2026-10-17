"""Save an active SLAM map with the standard Nav2 map saver."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    filename = Path(LaunchConfiguration('map_name').perform(context)).expanduser().resolve()
    filename.parent.mkdir(parents=True, exist_ok=True)
    if Path(str(filename) + '.yaml').exists() or Path(str(filename) + '.pgm').exists():
        raise ValueError(f'Map already exists; choose a new map_name: {filename}')
    saver = Node(package='nav2_map_server', executable='map_saver_cli', output='screen',
                 arguments=['-f', str(filename), '-t', LaunchConfiguration('map_topic'),
                            '--fmt', 'pgm'],
                 parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                              'save_map_timeout': 10.0}])

    def finished(event, context):
        if event.returncode:
            raise RuntimeError(f'Map save failed with exit code {event.returncode}')
        return [LogInfo(msg=f'Map saved: {filename}.yaml'), Shutdown(reason='Map saved')]

    return [RegisterEventHandler(OnProcessExit(target_action=saver, on_exit=finished)), saver]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name', description='Output path without extension; existing maps are protected.'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
