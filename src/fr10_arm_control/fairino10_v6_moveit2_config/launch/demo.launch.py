from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


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
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="true: simulation (mock_components); false: real FR10 hardware",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.58.2",
        description="IP address of the real FR10 robot (only used when use_fake_hardware:=false)",
    )
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo",
        default_value="false",
        description="Use GazeboSystem instead of the fake or real hardware plugin.",
    )

    moveit_config = (
        MoveItConfigsBuilder("fairino10_v6_robot", package_name="fairino10_v6_moveit2_config")
        .robot_description(
            mappings={
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "use_gazebo": LaunchConfiguration("use_gazebo"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                **{
                    name: LaunchConfiguration(name)
                    for name in GEOMETRY_ARGUMENT_DEFAULTS
                },
            }
        )
        .to_moveit_configs()
    )

    demo_ld = generate_demo_launch(moveit_config)
    return LaunchDescription(
        [
            use_fake_hardware_arg,
            use_gazebo_arg,
            robot_ip_arg,
            *[
                DeclareLaunchArgument(name, default_value=default)
                for name, default in GEOMETRY_ARGUMENT_DEFAULTS.items()
            ],
            *demo_ld.entities,
        ]
    )
