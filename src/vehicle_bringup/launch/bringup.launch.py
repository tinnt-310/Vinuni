"""Single entry point for arm/vehicle Gazebo or commissioned native hardware.

The public switches are use_arm, use_vehicle and use_gazebo.  Legacy arm,
vehicle and mode arguments remain aliases; explicit public switches win.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    SetEnvironmentVariable, UnsetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def native_desktop_environment(environment):
    """Isolate host ROS/Qt children from a VS Code Snap runtime, not the shell."""
    path_keys = ('PATH', 'LD_LIBRARY_PATH', 'QT_PLUGIN_PATH',
                 'QT_QPA_PLATFORM_PLUGIN_PATH', 'GTK_PATH', 'LD_PRELOAD',
                 'XDG_DATA_DIRS', 'XDG_CONFIG_DIRS')
    if not (any(name.startswith('SNAP') for name in environment)
            or any('/snap/' in environment.get(name, '') for name in path_keys)):
        return {}, []

    def native_paths(value):
        return ':'.join(path for path in value.split(':')
                        if path and '/snap/' not in path)

    updates = {}
    removed = [name for name in environment if name.startswith('SNAP')]
    for name in ('PATH', 'LD_LIBRARY_PATH'):
        if name in environment:
            updates[name] = native_paths(environment[name])
    for name in ('XDG_DATA_DIRS', 'XDG_CONFIG_DIRS'):
        default = '/usr/local/share:/usr/share' if name == 'XDG_DATA_DIRS' else '/etc/xdg'
        original = environment.get(name + '_VSCODE_SNAP_ORIG')
        updates[name] = native_paths(original or environment.get(name, default)) or default
    # GTK module names can be path-free (gail:atk-bridge) yet resolve to core20.
    # Preserve explicitly native Qt/preload paths; remove only Snap versions.
    for name in ('GTK_MODULES', 'GTK_PATH', 'GTK_EXE_PREFIX',
                 'GDK_PIXBUF_MODULEDIR', 'GDK_PIXBUF_MODULE_FILE',
                 'GIO_MODULE_DIR', 'GSETTINGS_SCHEMA_DIR', 'LOCPATH'):
        if name in environment:
            removed.append(name)
    for name in ('QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH', 'LD_PRELOAD'):
        if '/snap/' in environment.get(name, ''):
            removed.append(name)
    return updates, sorted(set(removed))


def select_backend(arm, vehicle, mode):
    if mode not in ('sim', 'real'):
        raise ValueError('mode must be sim or real')
    if not arm and not vehicle:
        raise ValueError('Select at least one component: use_arm:=true or use_vehicle:=true')
    if vehicle and mode == 'real':
        raise RuntimeError('PLC backend not commissioned: missing encoder/angle/speed/alarm '
                           'feedback registers, speed calibration and watchdog mapping. '
                           'No hardware nodes have been started.')
    if mode == 'sim':
        return 'vehicle_bringup', 'simulation.launch.py'
    return 'fr10_pose_controller', 'fr10_pose_controller.launch.py'


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    arm = value('use_arm') == 'true'
    vehicle = value('use_vehicle') == 'true'
    mode = 'sim' if value('use_gazebo') == 'true' else 'real'
    force = value('use_force') == 'true'
    wrist_layout = LaunchConfiguration('wrist_layout', default='calibrated').perform(context)
    if wrist_layout == 'photo_dc_d455' and (mode != 'sim' or force):
        raise ValueError('photo_dc_d455 is an uncalibrated male-connector model; use simulation with use_force:=false. Existing contact fixture is not its female counterpart.')
    navigation_mode = value('navigation_mode')
    if navigation_mode != 'disabled' and (not vehicle or mode != 'sim' or force):
        raise ValueError('Navigation requires use_vehicle:=true use_gazebo:=true use_force:=false')
    if value('cmd_vel_topic').lstrip('/') in ('cmd_vel_nav', 'navigation/cmd_vel'):
        raise ValueError('cmd_vel_topic conflicts with an internal velocity topic')
    if force and (mode != 'sim' or not arm):
        raise RuntimeError(
            'use_force:=true requires use_arm:=true use_gazebo:=true; native contact remains locked')
    if force and vehicle and not value('station'):
        raise RuntimeError('Mounted force requires a matched station:=front_mid; legacy proxy is standalone only.')
    package, filename = select_backend(arm, vehicle, mode)
    if mode == 'sim':
        arguments = {'enable_vehicle': str(vehicle).lower(), 'enable_arm': str(arm).lower(),
                     'arm_controller_profile': 'pose', 'gui': value('gui'),
                     'rviz': value('rviz'), 'gazebo_port': value('gazebo_port'),
                     'cmd_vel_topic': value('cmd_vel_topic'),
                     'enable_force_control': str(force).lower(),
                     'ft_wrench_topic': value('ft_wrench_topic'),
                     'matched_physics_step_s': value('matched_physics_step_s'),
                     'matched_solver_iterations': value('matched_solver_iterations'),
                     'enable_arm_camera': value('use_arm_camera'),
                     'wrist_layout': wrist_layout,
                     'force_fixture_profile': value('force_fixture_profile'),
                     'station': value('station'), 'demo': value('demo'),
                     'initial_state': value('initial_state')}
        arguments.update({name: value(name) for name in (
            'navigation_mode', 'map', 'nav2_params_file', 'slam_params_file',
            'scan_topic', 'odom_topic', 'autostart', 'use_respawn', 'log_level',
            'navigation_startup_timeout', 'world', 'rviz_config_file')})
        arguments['enable_detection'] = value('use_detection')
        # Leave simulation.launch.py's package-resolved defaults untouched
        # unless the caller explicitly overrides a force fixture path.  This
        # also keeps fail-closed route validation free of package lookups.
        if force and value('force_world'):
            arguments['force_world'] = value('force_world')
        if force and value('force_controller_overlay'):
            arguments['force_controller_overlay'] = value('force_controller_overlay')
    else:
        arguments = {'use_fake_hardware': 'false', 'use_gazebo': 'false',
                     'robot_ip': value('robot_ip'), 'use_rviz': value('rviz'),
                     'enable_force_sensor': 'false'}
    updates, removed = native_desktop_environment(context.environment)
    actions = [UnsetEnvironmentVariable(name=name) for name in removed]
    actions.extend(SetEnvironmentVariable(name=name, value=value)
                   for name, value in updates.items())
    if mode == 'real' and not vehicle:
        actions.append(LogInfo(msg='Standalone FR10 uses the canonical vehicle_description FR10 D435/CCS2 model.'))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename)),
        launch_arguments=arguments.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='true', choices=['true', 'false'],
                              description='Legacy alias; prefer use_arm.'),
        DeclareLaunchArgument('vehicle', default_value='true', choices=['true', 'false'],
                              description='Legacy alias; prefer use_vehicle.'),
        DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real'],
                              description='Legacy alias; prefer use_gazebo.'),
        DeclareLaunchArgument('use_arm', default_value=LaunchConfiguration('arm'),
                              choices=['true', 'false'],
                              description='Enable the FR10 arm (default: true).'),
        DeclareLaunchArgument('use_vehicle', default_value=LaunchConfiguration('vehicle'),
                              choices=['true', 'false'],
                              description='Enable the vehicle (default: true).'),
        DeclareLaunchArgument('use_gazebo', default_value=PythonExpression([
            "'true' if '", LaunchConfiguration('mode'), "' == 'sim' else 'false'"
        ]), choices=['true', 'false'],
                              description='Use Gazebo (default: true); false selects native '
                                          'FAIRINO hardware, not a mock or MuJoCo backend.'),
        DeclareLaunchArgument(
            'use_force', default_value='false', choices=['true', 'false'],
            description='Test-only Gazebo insertion into the CCS2 fixture; force is '
                        'never enabled on native hardware.'),
        DeclareLaunchArgument('force_world', default_value=''),
        DeclareLaunchArgument('station', default_value='',
                             description='Opt-in matched CAD station; e.g. front_mid. Candidates require validation.'),
        DeclareLaunchArgument('initial_state', default_value='home'),
        DeclareLaunchArgument('demo', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'force_fixture_profile', default_value='home',
            choices=['home', 'socket']),
        DeclareLaunchArgument('force_controller_overlay', default_value=''),
        DeclareLaunchArgument('ft_wrench_topic', default_value='wrench'),
        DeclareLaunchArgument('matched_physics_step_s', default_value=''),
        DeclareLaunchArgument('matched_solver_iterations', default_value=''),
        DeclareLaunchArgument('use_arm_camera', default_value='true', choices=['true', 'false'],
                             description='Generate arm depth images; false preserves camera geometry/TF.'),
        DeclareLaunchArgument('wrist_layout', default_value='calibrated', choices=['calibrated', 'photo_dc_d455'],
                             description='Measured wrist layout or photo-based CCS2 DC male + D455 simulation proposal.'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.58.2'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_port', default_value='11355'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('navigation_mode', default_value='disabled',
                             choices=['disabled', 'mapping', 'localization', 'navigation'],
                             description='SLAM mapping, AMCL localization, or AMCL plus Nav2.'),
        DeclareLaunchArgument('map', default_value='', description='Map YAML; defaults to the packaged VinFast map.'),
        DeclareLaunchArgument('nav2_params_file', default_value='', description='Override vehicle_navigation/config/nav2.yaml.'),
        DeclareLaunchArgument('slam_params_file', default_value='', description='Override vehicle_navigation/config/slam.yaml.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='LaserScan input for localization and costmaps.'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', description='Odometry input for navigation.'),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false'], description='Automatically activate Nav2/AMCL lifecycle nodes.'),
        DeclareLaunchArgument('use_respawn', default_value='false', choices=['true', 'false'], description='Restart crashed navigation processes.'),
        DeclareLaunchArgument('log_level', default_value='info', description='Navigation ROS logging severity.'),
        DeclareLaunchArgument('navigation_startup_timeout', default_value='90.0', description='Wall-time seconds to wait for vehicle navigation inputs.'),
        DeclareLaunchArgument('world', default_value='', description='Gazebo world override.'),
        DeclareLaunchArgument('rviz_config_file', default_value='', description='RViz profile override; empty selects the current mode profile.'),
        DeclareLaunchArgument('use_detection', default_value='false', choices=['true', 'false'], description='Enable CCS2 detection when the vehicle is enabled.'),
        OpaqueFunction(function=setup),
    ])
