import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


GEOMETRY_ARGUMENT_DEFAULTS = {
    "enable_custom_collisions": "true",
    "flange_xyz": "0 0 0.106",
    "flange_rpy": "0 0 0",
    "ft_sensor_xyz": "0 0 0",
    "ft_sensor_rpy": "0 0 0",
    "tool_plate_xyz": "0 0 0.030",
    "tool_plate_rpy": "0 0 0",
    "d455_xyz": "-0.0475 0 0.012",
    "d455_rpy": "0 -1.57079632679 1.57079632679",
    "charging_mount_xyz": "0.060 0 0.012",
    "charging_mount_rpy": "0 0 0",
    "charging_connector_xyz": "0 0 0.040",
    "charging_connector_rpy": "0 0 0",
    "tool0_xyz": "0 0 0.080",
    "tool0_rpy": "0 0 0",
    "ft_sensor_radius": "0.045",
    "ft_sensor_length": "0.030",
    "tool_plate_size_x": "0.140",
    "tool_plate_size_y": "0.100",
    "tool_plate_size_z": "0.012",
    "charging_mount_size_x": "0.040",
    "charging_mount_size_y": "0.050",
    "charging_mount_size_z": "0.040",
    "connector_body_size_x": "0.045",
    "connector_body_size_y": "0.035",
    "connector_body_length": "0.060",
    "connector_tip_radius": "0.012",
    "connector_tip_length": "0.020",
}


def generate_launch_description():
    package_name = "fairino10_v6_moveit2_config"
    package_share = get_package_share_directory(package_name)
    use_rviz = LaunchConfiguration("use_rviz")
    world = LaunchConfiguration("world")
    model_roots = [
        os.path.dirname(get_package_share_directory("fairino_description")),
        os.path.dirname(get_package_share_directory("realsense2_description")),
    ]
    gazebo_model_path = os.pathsep.join(
        path for path in [os.environ.get("GAZEBO_MODEL_PATH", ""), *model_roots]
        if path
    )

    moveit_config = (
        MoveItConfigsBuilder("fairino10_v6_robot", package_name=package_name)
        .robot_description(
            mappings={
                "use_fake_hardware": "false",
                "use_gazebo": "true",
                "initial_positions_file": os.path.join(package_share, "config", "sim_ready_positions.yaml"),
                **{
                    name: LaunchConfiguration(name)
                    for name in GEOMETRY_ARGUMENT_DEFAULTS
                },
            }
        )
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "false"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "fr10"],
    )

    spawn_controllers = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["fairino10_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(package_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(package_share, "worlds", "fr10_force_test.world"),
            ),
            *[
                DeclareLaunchArgument(name, default_value=default)
                for name, default in GEOMETRY_ARGUMENT_DEFAULTS.items()
            ],
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            spawn_controllers,
            move_group,
            rviz,
        ]
    )
