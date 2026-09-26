import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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


def as_bool(value):
    return value.lower() in ("true", "1", "yes")


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("fr10_pose_controller")
    moveit_share = get_package_share_directory("fairino10_v6_moveit2_config")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    use_gazebo = LaunchConfiguration("use_gazebo").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    start_robot_stack = LaunchConfiguration("start_robot_stack").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    hardware_plugin = LaunchConfiguration("hardware_plugin").perform(context)
    requested_backend = LaunchConfiguration("backend").perform(context)
    geometry_mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in GEOMETRY_ARGUMENT_DEFAULTS
    }

    backend = "ros_control" if requested_backend == "auto" else requested_backend
    if backend != "ros_control":
        raise RuntimeError("Pose/Joint mode supports only the ros_control backend")

    moveit_config = (
        MoveItConfigsBuilder(
            "fairino10_v6_robot",
            package_name="fairino10_v6_moveit2_config",
        )
        .robot_description(
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "use_gazebo": use_gazebo,
                "robot_ip": robot_ip,
                "hardware_plugin": hardware_plugin,
                **geometry_mappings,
            }
        )
        .to_moveit_configs()
    )

    actions = []
    if as_bool(start_robot_stack):
        if as_bool(use_gazebo):
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(moveit_share, "launch", "gazebo.launch.py")
                    ),
                    launch_arguments={
                        "use_rviz": use_rviz,
                        **geometry_mappings,
                    }.items(),
                )
            )
        else:
            # The controller uses MoveIt only as an in-process IK/model library.
            # Starting move_group would create a second, unused motion authority
            # and is unnecessary for either FakeSystem or the physical plugin.
            robot_stack_actions = [
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        output="screen",
                        parameters=[
                            moveit_config.robot_description,
                            {"publish_frequency": 125.0},
                        ],
                    ),
                    Node(
                        package="controller_manager",
                        executable="ros2_control_node",
                        output="screen",
                        parameters=[
                            moveit_config.robot_description,
                            os.path.join(package_share, "config", "ros2_controllers.yaml"),
                        ],
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        output="screen",
                        arguments=[
                            "joint_state_broadcaster",
                            "--controller-manager",
                            "/controller_manager",
                        ],
                    ),
                ]
            robot_stack_actions.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    output="screen",
                    arguments=[
                        "fairino10_controller",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                )
            )
            actions.extend(robot_stack_actions)
            if as_bool(use_rviz):
                actions.append(
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="log",
                        arguments=[
                            "-d",
                            os.path.join(moveit_share, "config", "moveit.rviz"),
                        ],
                        parameters=[
                            moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics,
                            moveit_config.planning_pipelines,
                            moveit_config.joint_limits,
                        ],
                    )
                )

    actions.append(
        Node(
            package="fr10_pose_controller",
            executable="fr10_pose_controller_node",
            name="fr10_pose_controller",
            output="screen",
            parameters=[
                os.path.join(package_share, "config", "controller.yaml"),
                *([LaunchConfiguration("controller_overlay").perform(context)]
                  if LaunchConfiguration("controller_overlay").perform(context) else []),
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {
                    "controller.backend": backend,
                    "controller.robot_ip": robot_ip,
                },
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="true: FakeSystem; false: FAIRINO SimMachine/robot",
            ),
            DeclareLaunchArgument(
                "use_gazebo",
                default_value="false",
                description="Use Gazebo Classic through ros_control backend.",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "backend",
                default_value="auto",
                choices=["auto", "ros_control"],
                description="Pose/Joint motion always uses ros2_control.",
            ),
            DeclareLaunchArgument("robot_ip", default_value="192.168.58.2"),
            DeclareLaunchArgument("controller_overlay", default_value="",
                                  description="Optional controller parameter overlay (e.g. SimMachine profile)"),
            DeclareLaunchArgument(
                "hardware_plugin",
                default_value="fairino_hardware_v3_9_3/FairinoHardwareInterface",
                choices=["fairino_hardware_v3_9_3/FairinoHardwareInterface"],
                description=(
                    "The sole supported FAIRINO runtime plugin for the current "
                    "v3.9.3.x controller/SimMachine target."
                ),
            ),
            DeclareLaunchArgument(
                "start_robot_stack",
                default_value="true",
                description=(
                    "Start robot_state_publisher, controller_manager, controllers, "
                    "and the selected ros2_control hardware."
                ),
            ),
            *[
                DeclareLaunchArgument(name, default_value=default)
                for name, default in GEOMETRY_ARGUMENT_DEFAULTS.items()
            ],
            OpaqueFunction(function=launch_setup),
        ]
    )
