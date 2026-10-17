import os
import importlib.util
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def as_bool(value):
    return value.lower() in ("true", "1", "yes")


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("fr10_pose_controller")
    profile_spec = importlib.util.spec_from_file_location('controller_profile',
        os.path.join(package_share, 'launch', 'controller_profile.py'))
    profile_loader = importlib.util.module_from_spec(profile_spec)
    profile_spec.loader.exec_module(profile_loader)
    overlay_path = LaunchConfiguration("controller_overlay").perform(context)
    overlay_parameters = profile_loader.load_controller_profile(overlay_path) if overlay_path else {}
    moveit_share = get_package_share_directory("fairino10_v6_moveit2_config")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    use_gazebo = LaunchConfiguration("use_gazebo").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    enable_force_sensor = LaunchConfiguration("enable_force_sensor").perform(context)
    start_robot_stack = LaunchConfiguration("start_robot_stack").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    hardware_plugin = LaunchConfiguration("hardware_plugin").perform(context)
    native_proof = LaunchConfiguration('native_simulator_proof').perform(context)
    native_parameters = {}
    if native_proof:
        if as_bool(use_gazebo) or as_bool(use_fake_hardware) or robot_ip != '192.168.58.2':
            raise RuntimeError('Native simulator proof requires the verified Docker native endpoint')
        spec = importlib.util.spec_from_file_location('native_simulator_gate',
            os.path.join(package_share,'launch','native_simulator_gate.py'))
        gate = importlib.util.module_from_spec(spec); spec.loader.exec_module(gate)
        native_parameters = gate.verify_simulator_proof(native_proof)
    requested_backend = LaunchConfiguration("backend").perform(context)

    backend = "ros_control" if requested_backend == "auto" else requested_backend
    if backend != "ros_control":
        raise RuntimeError("Pose/Joint mode supports only the ros_control backend")

    description_share = get_package_share_directory("vehicle_description")
    control_backend = ("gazebo" if as_bool(use_gazebo) else
                       "fake" if as_bool(use_fake_hardware) else "fairino")
    positions_file = os.path.join(
        moveit_share, "config",
        "sim_ready_positions.yaml" if as_bool(use_gazebo) else "initial_positions.yaml")
    moveit_config = (
        MoveItConfigsBuilder(
            "complete_visual", package_name="fairino10_v6_moveit2_config")
        .robot_description(
            file_path=os.path.join(description_share, "urdf", "vehicle.urdf.xacro"),
            mappings={
                "enable_vehicle": "false",
                "enable_arm": "true",
                "sim_base_collision_proxy": "true" if as_bool(use_gazebo) else "false",
                "control_backend": control_backend,
                "enable_force_sensor": enable_force_sensor,
                "charging_tool_profile": LaunchConfiguration('charging_tool_profile').perform(context),
                "ft_wrench_topic": LaunchConfiguration("ft_wrench_topic").perform(context),
                "initial_positions_file": positions_file,
                "gazebo_controller_yaml": os.path.join(
                    package_share, "config", "ros2_controllers.yaml"),
                "robot_ip": robot_ip,
                "hardware_plugin": hardware_plugin,
            })
        .robot_description_semantic(
            file_path=os.path.join(description_share, "config", "fr10.srdf"))
        .to_moveit_configs()
    )
    tool_spec = importlib.util.spec_from_file_location('tool_geometry',
        os.path.join(description_share, 'launch', 'tool_geometry.py'))
    tool_geometry = importlib.util.module_from_spec(tool_spec)
    tool_spec.loader.exec_module(tool_geometry)
    tool_parameters = tool_geometry.wrench_parameters(
        moveit_config.robot_description['robot_description'],
        overlay_parameters.get('force.wrench_source_frame', 'charging_connector_link'),
        overlay_parameters.get('force.wrench_target_frame', 'charging_tcp'))

    actions = []
    spec = importlib.util.spec_from_file_location('planning_collision_model',
        os.path.join(package_share,'launch','planning_collision_model.py'))
    collision_model = importlib.util.module_from_spec(spec); spec.loader.exec_module(collision_model)
    planning_description = {'robot_description': collision_model.planning_collision_description(
        moveit_config.robot_description['robot_description'],
        os.path.join(description_share,'models','matched_socket_v2'))}
    if LaunchConfiguration('charging_tool_profile').perform(context) == 'matched_socket_v2':
        semantic = moveit_config.robot_description_semantic['robot_description_semantic']
        assembly = ET.parse(os.path.join(description_share,'models','matched_socket_v2','matched_connector.srdf')).getroot()
        entries = ''.join(ET.tostring(entry,encoding='unicode') for entry in assembly)
        moveit_config.robot_description_semantic['robot_description_semantic'] = semantic.replace('</robot>',entries+'</robot>')
    if not as_bool(start_robot_stack):
        actions.append(Node(package='robot_state_publisher',executable='robot_state_publisher',
            parameters=[moveit_config.robot_description,{'use_sim_time':as_bool(use_gazebo)}]))
    if as_bool(start_robot_stack):
        if as_bool(use_gazebo):
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(moveit_share, "launch", "gazebo.launch.py")
                    ),
                    launch_arguments={
                        "use_rviz": use_rviz,
                        "gui": gui,
                        "start_move_group": "false",
                        "enable_force_sensor": enable_force_sensor,
                        "charging_tool_profile": LaunchConfiguration("charging_tool_profile").perform(context),
                        "ft_wrench_topic": LaunchConfiguration("ft_wrench_topic").perform(context),
                        "gazebo_clock_params": LaunchConfiguration("gazebo_clock_params").perform(context),
                        "controller_start_delay": LaunchConfiguration("gazebo_controller_start_delay").perform(context),
                        "controller_yaml": os.path.join(package_share, "config", "ros2_controllers.yaml"),
                        **({"world": LaunchConfiguration("gazebo_world").perform(context)}
                           if LaunchConfiguration("gazebo_world").perform(context) else {}),
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
                overlay_parameters,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                planning_description,
                {
                    "controller.backend": backend,
                    "controller.robot_ip": robot_ip,
                    "controller.require_driver_health": not as_bool(use_gazebo),
                    "use_sim_time": as_bool(use_gazebo),
                },
                native_parameters,
                tool_parameters,
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_world", default_value=""),
            DeclareLaunchArgument('charging_tool_profile', default_value='legacy',
                choices=['legacy','matched_socket_v2']),
            DeclareLaunchArgument("ft_wrench_topic", default_value="wrench"),
            DeclareLaunchArgument("gazebo_clock_params", default_value=""),
            DeclareLaunchArgument("gazebo_controller_start_delay", default_value="4.0"),
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
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "enable_force_sensor",
                default_value="false",
                description="Opt-in test-only Gazebo F/T load-cell chain.",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="auto",
                choices=["auto", "ros_control"],
                description="Pose/Joint motion always uses ros2_control.",
            ),
            DeclareLaunchArgument("robot_ip", default_value="192.168.58.2"),
            DeclareLaunchArgument('native_simulator_proof', default_value=''),
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
            OpaqueFunction(function=launch_setup),
        ]
    )
