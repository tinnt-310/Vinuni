import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = get_package_share_directory("fairino10_v6_moveit2_config")
    description_share = get_package_share_directory("vehicle_description")
    controller_yaml = LaunchConfiguration("controller_yaml")
    model_roots = [
        os.path.dirname(description_share),
        os.path.join(description_share, "models"),
        os.path.dirname(get_package_share_directory("realsense2_description")),
    ]
    gazebo_model_path = os.pathsep.join(
        path for path in [os.environ.get("GAZEBO_MODEL_PATH", ""), *model_roots]
        if path
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "complete_visual", package_name="fairino10_v6_moveit2_config"
        )
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "vehicle.urdf.xacro"),
            mappings={
                "enable_vehicle": "false",
                "enable_arm": "true",
                "control_backend": "gazebo",
                "sim_base_collision_proxy": "true",
                "enable_force_sensor": LaunchConfiguration("enable_force_sensor"),
                "charging_tool_profile": LaunchConfiguration("charging_tool_profile"),
                "ft_wrench_topic": LaunchConfiguration("ft_wrench_topic"),
                "gazebo_controller_yaml": controller_yaml,
                "initial_positions_file": os.path.join(
                    package_share, "config", "sim_ready_positions.yaml"
                ),
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(description_share, "config", "fr10.srdf")
        )
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": "false",
            "gui": LaunchConfiguration("gui"),
            "params_file": LaunchConfiguration("gazebo_clock_params"),
        }.items(),
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
        period=LaunchConfiguration("controller_start_delay"),
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
        condition=IfCondition(LaunchConfiguration("start_move_group")),
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
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("controller_start_delay", default_value="4.0"),
            DeclareLaunchArgument("ft_wrench_topic", default_value="wrench"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("start_move_group", default_value="true"),
            DeclareLaunchArgument("enable_force_sensor", default_value="false"),
            DeclareLaunchArgument("charging_tool_profile", default_value="legacy",
                                  choices=['legacy', 'matched_socket_v2']),
            DeclareLaunchArgument("gazebo_clock_params", default_value=""),
            DeclareLaunchArgument(
                "controller_yaml",
                default_value=os.path.join(package_share, "config", "ros2_controllers.yaml"),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(package_share, "worlds", "fr10_force_test.world"),
            ),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
            gazebo,
            robot_state_publisher,
            spawn_robot,
            spawn_controllers,
            move_group,
            rviz,
        ]
    )
