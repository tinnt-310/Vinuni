"""Reproducibility manifest for shared-model FR10 Gazebo runs."""

import hashlib
import json
from pathlib import Path
import subprocess

WORKSPACE = Path(__file__).resolve().parents[4]
SOURCE_FILES = (
    'src/vehicle_bringup/launch/bringup.launch.py',
    'src/vehicle_bringup/launch/simulation.launch.py',
    'src/vehicle_bringup/config/simulation.xacro',
    'src/vehicle_description/urdf/vehicle.urdf.xacro',
    'src/vehicle_description/urdf/vehicle_dimensions.xacro',
    'src/vehicle_description/urdf/vehicle_chassis.xacro',
    'src/vehicle_description/urdf/body_sensors.xacro',
    'src/vehicle_description/urdf/vehicle.gazebo.xacro',
    'src/vehicle_description/config/vehicle_geometry.yaml',
    'src/vehicle_description/urdf/fr10_arm.urdf',
    'src/vehicle_description/urdf/fr10_wrist.xacro',
    'src/vehicle_description/urdf/fr10_ros2_control.xacro',
    'src/vehicle_description/config/fr10.srdf',
    'src/vehicle_description/worlds/vinfast_map.world',
    'src/fr10_arm_control/fr10_pose_controller/config/controller.yaml',
    'src/fr10_arm_control/fr10_pose_controller/src/motion_adapter.cpp',
    'src/fr10_arm_control/fr10_pose_controller/src/moveit_ik_provider.cpp',
    'src/swerve_vehicle_control/src/swerve_controller_plugin.cpp',
    'src/swerve_vehicle_control/include/swerve_vehicle_control/temporal_validity.hpp',
    'src/swerve_vehicle_control/config/swerve_controllers.yaml',
)
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def new_manifest(output, command, env, mode):
    head = subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=WORKSPACE, text=True).strip()
    diff = subprocess.check_output(['git', 'diff', '--binary'], cwd=WORKSPACE)
    from ament_index_python.packages import get_package_prefix
    binaries = {
        'fr10_pose_controller_node': Path(get_package_prefix(
            'fr10_pose_controller')) / 'lib/fr10_pose_controller/fr10_pose_controller_node',
        'swerve_controller_plugin': Path(get_package_prefix(
            'swerve_vehicle_control')) / 'lib/libswerve_controller_plugin.so',
        'wheel_contact_surface': Path(get_package_prefix(
            'vehicle_bringup')) / 'lib/libvinuni_wheel_contact_surface.so',
    }
    missing = [str(path) for path in binaries.values() if not path.is_file()]
    if missing:
        raise RuntimeError('Missing runtime binaries: ' + ', '.join(missing))
    manifest = {
        'mode': mode, 'git_head': head,
        'tracked_diff_sha256': hashlib.sha256(diff).hexdigest(),
        'source_sha256': {name: digest(WORKSPACE / name)
                          for name in SOURCE_FILES if (WORKSPACE / name).exists()},
        'binary_path': {name: str(path) for name, path in binaries.items()},
        'binary_sha256': {name: digest(path) for name, path in binaries.items()},
        'command': command,
        'environment': {key: env.get(key) for key in (
            'ROS_DOMAIN_ID', 'RMW_LOCALHOST_ONLY', 'GAZEBO_MASTER_URI',
            'ROS_DISTRO', 'AMENT_PREFIX_PATH')},
        'clock_sources': ['wall_monotonic', 'ROS_sim', 'Gazebo_transport'],
    }
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    return manifest


def finish_manifest(output, manifest, summary):
    manifest['live_model_sha256'] = summary.get('live_model_sha256')
    manifest['shared_arm_signature_sha256'] = summary.get('shared_arm_signature_sha256')
    manifest['process_identity'] = {
        key: summary.get(key) for key in (
            'launch_pid', 'launch_exit', 'observer_pid', 'observer_exit',
            'vehicle_observer_pid', 'vehicle_observer_exit',
            'wrist_observer_pid', 'wrist_observer_exit')
        if key in summary}
    manifest['gzserver_exit_code'] = summary.get('gzserver_exit_code')
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
