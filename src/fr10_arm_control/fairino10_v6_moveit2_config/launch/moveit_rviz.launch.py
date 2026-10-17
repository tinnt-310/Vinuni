import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    description_share = get_package_share_directory("vehicle_description")
    config_share = get_package_share_directory("fairino10_v6_moveit2_config")
    moveit_config = (
        MoveItConfigsBuilder(
            "complete_visual", package_name="fairino10_v6_moveit2_config"
        )
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "vehicle.urdf.xacro"),
            mappings={
                "enable_vehicle": "false",
                "enable_arm": "true",
                "control_backend": IfElseSubstitution(
                    LaunchConfiguration("use_fake_hardware"), "fake", "fairino"
                ),
                "initial_positions_file": os.path.join(
                    config_share, "config", "initial_positions.yaml"
                ),
                "robot_ip": LaunchConfiguration("robot_ip"),
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(description_share, "config", "fr10.srdf")
        )
        .to_moveit_configs()
    )
    generated = generate_moveit_rviz_launch(moveit_config)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument("robot_ip", default_value="192.168.58.2"),
            *generated.entities,
        ]
    )
