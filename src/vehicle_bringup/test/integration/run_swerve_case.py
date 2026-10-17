#!/usr/bin/env python3
"""Measured production-swerve host-Gazebo diagnostic, one fresh server/case."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import threading
import time

from barrier_contact_diagnostics import (
    DEFAULT_TARGET, analyze_directory, build_probe, inject_probe)
from ode_transport_runtime import observer_binary
from vehicle_trace_contract import evaluate_vehicle_ros_trace

WORKSPACE = Path(__file__).resolve().parents[4]
CASES = {
    'zero': (0.0, 0.0, 0.0),
    'forward': (0.12, 0.0, 0.0),
    'reverse': (-0.12, 0.0, 0.0),
    'lateral': (0.0, 0.12, 0.0),
    'diagonal': (0.085, 0.085, 0.0),
    'rotation': (0.0, 0.0, 0.15),
    'combined': (0.10, 0.05, 0.10),
}


def evaluate_motion_contract(case, delta_xyz, delta_yaw):
    """Apply the Gazebo ground-truth motion budgets from the E2E plan."""
    vx, vy, omega = CASES[case]
    dx, dy = delta_xyz[:2]
    reasons = []
    linear_command = math.hypot(vx, vy)
    if linear_command == 0.0 and omega == 0.0:
        if math.hypot(dx, dy) > 0.03:
            reasons.append('zero command drift exceeds 0.03 m')
        if abs(delta_yaw) > 0.03:
            reasons.append('zero command yaw drift exceeds 0.03 rad')
    else:
        if linear_command > 0.0:
            projection = (dx * vx + dy * vy) / linear_command
            minimum = 0.4 if linear_command >= 0.119 else 0.3
            if projection < minimum:
                reasons.append(
                    f'translation along commanded direction {projection:.6f} m < {minimum:.3f} m')
        else:
            projection = None
        if omega != 0.0:
            signed_yaw = delta_yaw * math.copysign(1.0, omega)
            minimum_yaw = 0.5 if abs(omega) >= 0.149 else 0.3
            if signed_yaw < minimum_yaw:
                reasons.append(
                    f'rotation in commanded direction {signed_yaw:.6f} rad < {minimum_yaw:.3f} rad')
    return {'valid': not reasons, 'reasons': reasons,
            'delta_xyz_m': delta_xyz, 'delta_yaw_rad': delta_yaw}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def run(out, case, domain, port, omit_ray_sensor=False, gpu_ray_sensor=False,
        defer_ray_sensor=False, activate_ray_sensor=False,
        late_scan_subscriber=False,
        contact_probe=False, contact_target=DEFAULT_TARGET,
        drive_duration_s=5.0, settling_duration_s=16.0, spawn_z=0.0,
        contact_kd=None, surface_slip=None):
    if late_scan_subscriber and not defer_ray_sensor:
        raise ValueError('Late scan subscription requires deferred CPU ray')
    if drive_duration_s <= 0 or settling_duration_s <= 0:
        raise ValueError('Drive and settling durations must be positive')
    if not math.isfinite(spawn_z):
        raise ValueError('Spawn height must be finite')
    if contact_kd is not None and (not math.isfinite(contact_kd) or contact_kd <= 0.0):
        raise ValueError('Contact damping must be finite and positive')
    if surface_slip is not None and (not math.isfinite(surface_slip) or
                                     surface_slip <= 0.0 or surface_slip > 0.1):
        raise ValueError('Surface slip must be finite and in (0, 0.1]')
    out.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), RMW_LOCALHOST_ONLY='1',
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
               GAZEBO_MODEL_DATABASE_URI='', ROS_LOG_DIR=str(out/'ros_log'))
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['RMW_LOCALHOST_ONLY'] = '1'
    launch_target = ['vehicle_bringup', 'simulation.launch.py']
    if omit_ray_sensor or gpu_ray_sensor or defer_ray_sensor or activate_ray_sensor:
        # Diagnostic copy only. Keep geometry, controllers, camera, world and
        # station/vehicle spawn order identical to the production launch.
        source = WORKSPACE / 'src/vehicle_bringup/launch/simulation.launch.py'
        launch_text = source.read_text()
        anchor = '    # Source Xacros retain package://.'
        if launch_text.count(anchor) != 1:
            raise RuntimeError('Cannot locate unique diagnostic injection point')
        injection = '''    # Diagnostic only: remove the Gazebo ray sensor, not its physical link.
    removed = 0
    for gazebo_element in doc.getElementsByTagName('gazebo'):
        if gazebo_element.getAttribute('reference') == 'laser_frame':
            for sensor_element in list(gazebo_element.getElementsByTagName('sensor')):
                if sensor_element.getAttribute('type') == 'ray':
                    gazebo_element.removeChild(sensor_element)
                    removed += 1
    if removed != 1:
        raise RuntimeError(f'Expected one Gazebo ray sensor, found {removed}')
    print('DIAGNOSTIC OMIT_RAY_SENSOR=1', flush=True)
'''
        if gpu_ray_sensor:
            injection = '''    # Diagnostic only: retain scan geometry/topic but use Gazebo GPU ray.
    changed = 0
    for gazebo_element in doc.getElementsByTagName('gazebo'):
        if gazebo_element.getAttribute('reference') == 'laser_frame':
            for sensor_element in gazebo_element.getElementsByTagName('sensor'):
                if sensor_element.getAttribute('type') == 'ray':
                    sensor_element.setAttribute('type', 'gpu_ray')
                    changed += 1
    if changed != 1:
        raise RuntimeError(f'Expected one Gazebo ray sensor, found {changed}')
    print('DIAGNOSTIC GPU_RAY_SENSOR=1', flush=True)
'''
        if defer_ray_sensor:
            injection = '''    # Diagnostic only: CPU ray sensor starts with always_on=false.
    changed = 0
    for gazebo_element in doc.getElementsByTagName('gazebo'):
        if gazebo_element.getAttribute('reference') == 'laser_frame':
            for sensor_element in gazebo_element.getElementsByTagName('sensor'):
                if sensor_element.getAttribute('type') == 'ray':
                    for flag in sensor_element.getElementsByTagName('always_on'):
                        flag.firstChild.nodeValue = 'false'
                        changed += 1
    if changed != 1:
        raise RuntimeError(f'Expected one CPU ray always_on, found {changed}')
    print('DIAGNOSTIC DEFER_RAY_SENSOR=1', flush=True)
'''
        if activate_ray_sensor:
            injection = '''    # Diagnostic only: force the production CPU ray sensor active.
    changed = 0
    for gazebo_element in doc.getElementsByTagName('gazebo'):
        if gazebo_element.getAttribute('reference') == 'laser_frame':
            for sensor_element in gazebo_element.getElementsByTagName('sensor'):
                if sensor_element.getAttribute('type') == 'ray':
                    for flag in sensor_element.getElementsByTagName('always_on'):
                        flag.firstChild.nodeValue = 'true'
                        changed += 1
    if changed != 1:
        raise RuntimeError(f'Expected one CPU ray always_on, found {changed}')
    print('DIAGNOSTIC ACTIVATE_RAY_SENSOR=1', flush=True)
'''
        generated = out / ('diagnostic_gpu_ray.launch.py' if gpu_ray_sensor
                           else 'diagnostic_defer_ray.launch.py' if defer_ray_sensor
                           else 'diagnostic_active_ray.launch.py' if activate_ray_sensor
                           else 'diagnostic_no_ray.launch.py')
        generated.write_text(launch_text.replace(anchor, injection + anchor, 1))
        launch_target = [str(generated)]
    if contact_kd is not None or surface_slip is not None:
        launch_source = (Path(launch_target[0]).read_text()
                         if len(launch_target) == 1 else
                         (WORKSPACE / 'src/vehicle_bringup/launch/simulation.launch.py').read_text())
        anchor = '    # Source Xacros retain package://.'
        if launch_source.count(anchor) != 1:
            raise RuntimeError('Cannot locate unique contact diagnostic injection point')
        lines = []
        if contact_kd is not None:
            lines.extend([
                "    changed_contact_kd = 0",
                "    for gazebo_element in doc.getElementsByTagName('gazebo'):",
                "        if gazebo_element.getAttribute('reference') in {f'Wheel{i}_1' for i in range(1, 5)}:",
                "            kd_elements = gazebo_element.getElementsByTagName('kd')",
                "            if not kd_elements:",
                "                continue",
                "            if len(kd_elements) != 1 or kd_elements[0].firstChild is None:",
                "                raise RuntimeError('Expected at most one kd element per wheel Gazebo tag')",
                f"            kd_elements[0].firstChild.nodeValue = '{contact_kd:.12g}'",
                "            changed_contact_kd += 1",
                "    if changed_contact_kd != 4:",
                "        raise RuntimeError(f'Expected four wheel kd values, changed {changed_contact_kd}')",
                f"    print('DIAGNOSTIC CONTACT_KD={contact_kd:.12g}', flush=True)",
            ])
        if surface_slip is not None:
            lines.extend([
                "    changed_surface_slip = 0",
                "    for plugin_element in doc.getElementsByTagName('plugin'):",
                "        if plugin_element.getAttribute('name') == 'wheel_contact_surface':",
                "            existing = plugin_element.getElementsByTagName('slip')",
                "            if existing:",
                f"                existing[0].firstChild.nodeValue = '{surface_slip:.12g}'",
                "            else:",
                "                slip_element = doc.createElement('slip')",
                f"                slip_element.appendChild(doc.createTextNode('{surface_slip:.12g}'))",
                "                plugin_element.appendChild(slip_element)",
                "            changed_surface_slip += 1",
                "    if changed_surface_slip != 1:",
                "        raise RuntimeError(f'Expected one wheel contact plugin, changed {changed_surface_slip}')",
                f"    print('DIAGNOSTIC SURFACE_SLIP={surface_slip:.12g}', flush=True)",
            ])
        diagnostic_launch = out / 'diagnostic_contact_parameters.launch.py'
        diagnostic_launch.write_text(launch_source.replace(
            anchor, '\n'.join(lines) + '\n' + anchor, 1))
        launch_target = [str(diagnostic_launch)]
    probe_binary = None
    if contact_probe:
        probe_binary = build_probe(WORKSPACE / 'src/vehicle_bringup/test/integration/ode_contact_probe.cpp',
                                   out / 'libode_contact_probe.so')
        launch_source = (Path(launch_target[0]).read_text()
                         if len(launch_target) == 1 else
                         (WORKSPACE / 'src/vehicle_bringup/launch/simulation.launch.py').read_text())
        diagnostic_launch = out / 'diagnostic_contact.launch.py'
        diagnostic_launch.write_text(inject_probe(launch_source, probe_binary))
        launch_target = [str(diagnostic_launch)]
        env['ODE_OUT'] = str(out)
    if spawn_z != 0.0:
        launch_source = (Path(launch_target[0]).read_text()
                         if len(launch_target) == 1 else
                         (WORKSPACE / 'src/vehicle_bringup/launch/simulation.launch.py').read_text())
        original = "'-x', '0.0', '-y', '0.0', '-z', '0.0'],"
        replacement = f"'-x', '0.0', '-y', '0.0', '-z', '{spawn_z:.9f}'],"
        if launch_source.count(original) != 1:
            raise RuntimeError('Cannot locate unique vehicle spawn height')
        diagnostic_launch = out / 'diagnostic_spawn_height.launch.py'
        diagnostic_launch.write_text(launch_source.replace(original, replacement, 1))
        launch_target = [str(diagnostic_launch)]
    command = ['ros2', 'launch', *launch_target,
               'enable_vehicle:=true', 'enable_arm:=false', 'gui:=false',
               'rviz:=false', f'gazebo_port:={port}']
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=WORKSPACE,
                                   text=True).strip()
    diff = subprocess.check_output(['git', 'diff', '--binary'], cwd=WORKSPACE)
    files = ['src/vehicle_bringup/launch/simulation.launch.py',
             'src/swerve_vehicle_control/src/swerve_controller_plugin.cpp',
             'src/swerve_vehicle_control/config/swerve_controllers.yaml',
             'src/vehicle_description/worlds/vinfast_map.world',
             'src/vehicle_description/config/vehicle_geometry.yaml',
             'src/vehicle_description/urdf/vehicle_chassis.xacro',
             'src/vehicle_description/urdf/vehicle.gazebo.xacro',
             'src/vehicle_description/urdf/body_sensors.xacro']
    from ament_index_python.packages import get_package_prefix
    binary_paths = {
        'swerve_controller_plugin': Path(get_package_prefix(
            'swerve_vehicle_control')) / 'lib/libswerve_controller_plugin.so',
        'wheel_contact_surface': Path(get_package_prefix(
            'vehicle_bringup')) / 'lib/libvinuni_wheel_contact_surface.so',
    }
    missing_binaries = [str(path) for path in binary_paths.values()
                        if not path.is_file()]
    if missing_binaries:
        raise RuntimeError('Missing runtime binaries: ' + ', '.join(missing_binaries))
    manifest = {'git_head':head,'tracked_diff_sha256':hashlib.sha256(diff).hexdigest(),
                'file_sha256':{name:sha256(WORKSPACE/name) for name in files},
                'binary_path':{name:str(path) for name,path in binary_paths.items()},
                'binary_sha256':{name:sha256(path) for name,path in binary_paths.items()},
                'command':command,'domain':domain,'port':port,'case':case,
                'twist':CASES[case],'omit_ray_sensor':omit_ray_sensor,
                'gpu_ray_sensor':gpu_ray_sensor,
                'defer_ray_sensor':defer_ray_sensor,
                'activate_ray_sensor':activate_ray_sensor,
                'late_scan_subscriber':late_scan_subscriber,
                'contact_probe':contact_probe,
                'contact_target':contact_target,
                'drive_duration_s':drive_duration_s,
                'settling_duration_s':settling_duration_s,
                'spawn_z':spawn_z,
                'contact_kd':contact_kd,
                'surface_slip':surface_slip,
                'contact_probe_binary_sha256':sha256(probe_binary) if probe_binary else None,
                'contact_probe_source_sha256':sha256(WORKSPACE / 'src/vehicle_bringup/test/integration/ode_contact_probe.cpp') if contact_probe else None,
                'clock_sources':['wall_monotonic','ROS_sim','Gazebo_transport'],
                'environment':{key:env.get(key) for key in (
                    'ROS_DOMAIN_ID','RMW_LOCALHOST_ONLY','GAZEBO_MASTER_URI',
                    'ROS_DISTRO','AMENT_PREFIX_PATH')}}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    summary = {'case':case,'twist':CASES[case],'command':command}
    observer_dir = out/'vehicle'
    observer_dir.mkdir()
    observer_env = env.copy()
    observer_env.update(ODE_OUT=str(observer_dir), ODE_WORLD_NAME='default',
                        ODE_TARGET_ENTITY='my_mobile_manipulator')
    with (out/'runtime.log').open('w') as launch_log, \
         (observer_dir/'observer.log').open('w') as observer_log:
        launch = subprocess.Popen(command,cwd=WORKSPACE,env=env,
                                  stdout=subprocess.PIPE if contact_probe else launch_log,
                                  stderr=subprocess.STDOUT,start_new_session=True,
                                  text=contact_probe,bufsize=1 if contact_probe else -1)
        log_thread = None
        if contact_probe:
            def collect_runtime():
                with (out/'runtime_events.jsonl').open('w') as events:
                    for line in launch.stdout:
                        wall = time.monotonic()
                        launch_log.write(line)
                        events.write(json.dumps({'wall':wall,'line':line.rstrip()})+'\n')
            log_thread = threading.Thread(target=collect_runtime, daemon=True)
            log_thread.start()
        master_deadline = time.monotonic()+15
        master_ready = False
        while time.monotonic()<master_deadline and launch.poll() is None:
            try:
                with socket.create_connection(('127.0.0.1',port),timeout=0.2):
                    master_ready = True
                    break
            except OSError:
                time.sleep(0.05)
        summary['gazebo_master_ready'] = master_ready
        observer = subprocess.Popen([str(observer_binary())],cwd=WORKSPACE,
                                    env=observer_env,stdout=observer_log,
                                    stderr=subprocess.STDOUT,start_new_session=True)
        summary['launch_pid'] = launch.pid
        summary['observer_pid'] = observer.pid
        node = None
        try:
            import rclpy
            from controller_manager_msgs.srv import ListControllers
            from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
            from nav_msgs.msg import Odometry
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import JointState, LaserScan
            from tf2_msgs.msg import TFMessage
            from rclpy.qos import qos_profile_sensor_data
            from rcl_interfaces.srv import GetParameters
            rclpy.init()
            node = rclpy.create_node('swerve_case_observer')
            clocks, odom, wheels, commands, transforms, scans = [], [], [], [], [], []
            published_commands = []
            wheel_names = tuple(f'Wheel{i}_to_Revolute{i}' for i in range(1,5))
            steer_names = tuple(f'Revolute{i}_to_base_link' for i in range(1,5))
            def stamp(msg):
                return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            def wheel_cb(msg):
                names = {name:i for i,name in enumerate(msg.name)}
                if not all(name in names for name in wheel_names+steer_names):
                    return
                wheels.append({'wall':time.monotonic(),'source':stamp(msg),
                    'speed':[msg.velocity[names[name]] if names[name]<len(msg.velocity)
                             else None for name in wheel_names],
                    'steer':[msg.position[names[name]] if names[name]<len(msg.position)
                             else None for name in steer_names]})
            def odom_cb(msg):
                odom.append({'wall':time.monotonic(),'source':stamp(msg),
                    'x':msg.pose.pose.position.x,'y':msg.pose.pose.position.y,
                    'vx':msg.twist.twist.linear.x,'vy':msg.twist.twist.linear.y})
            def command_cb(msg):
                commands.append({'wall':time.monotonic(),'source':stamp(msg),
                    'vx':msg.twist.linear.x,'vy':msg.twist.linear.y,
                    'omega':msg.twist.angular.z})
            def tf_cb(msg):
                for tf in msg.transforms:
                    if tf.header.frame_id=='odom' and tf.child_frame_id=='base_footprint':
                        transforms.append({'wall':time.monotonic(),
                            'source':stamp(tf),'x':tf.transform.translation.x,
                            'y':tf.transform.translation.y})
            node.create_subscription(Clock,'/clock',lambda m:clocks.append(
                (time.monotonic(),m.clock.sec+m.clock.nanosec*1e-9)),
                qos_profile_sensor_data)
            node.create_subscription(JointState,'/joint_states',wheel_cb,
                                     qos_profile_sensor_data)
            node.create_subscription(Odometry,'/odom',odom_cb,10)
            node.create_subscription(TwistStamped,'/cmd_vel_nav',command_cb,10)
            node.create_subscription(TFMessage,'/tf',tf_cb,10)
            def scan_cb(msg):
                scans.append({'wall':time.monotonic(),'source':stamp(msg),
                    'frame':msg.header.frame_id,'count':len(msg.ranges),
                    'finite_count':sum(math.isfinite(v) for v in msg.ranges),
                    'angle_min':msg.angle_min,'angle_max':msg.angle_max,
                    'range_min':msg.range_min,'range_max':msg.range_max})
            scan_subscription = None
            if not late_scan_subscriber:
                scan_subscription = node.create_subscription(
                    LaserScan,'/scan',scan_cb,qos_profile_sensor_data)
            publisher = node.create_publisher(Twist,'/cmd_vel',10)
            client = node.create_client(ListControllers,
                                        '/controller_manager/list_controllers')
            def spin_for(seconds, drive=None):
                until = time.monotonic()+seconds
                next_publish = time.monotonic()
                while time.monotonic()<until and launch.poll() is None:
                    now = time.monotonic()
                    if drive is not None and now >= next_publish:
                        publisher.publish(drive)
                        if contact_probe:
                            published_commands.append({'wall':now,
                                'vx':drive.linear.x,'vy':drive.linear.y,
                                'omega':drive.angular.z})
                        next_publish = now + 0.05  # 20 Hz wall-clock command
                    timeout = min(0.05, max(0.0, next_publish-now)) \
                        if drive is not None else 0.05
                    rclpy.spin_once(node,timeout_sec=timeout)
            deadline = time.monotonic()+90
            future = None
            next_query = time.monotonic()
            query_count = 0
            while time.monotonic()<deadline and launch.poll() is None:
                now = time.monotonic()
                if future is None and now >= next_query and client.service_is_ready():
                    future = client.call_async(ListControllers.Request())
                    query_count += 1
                    next_query = now + 0.5
                rclpy.spin_once(node,timeout_sec=0.1)
                if future is not None and future.done():
                    response = future.result()
                    future = None  # At most one request in flight.
                    if response:
                        states={c.name:c.state for c in response.controller}
                        if states.get('swerve_controller')=='active' and \
                           states.get('joint_state_broadcaster')=='active' and \
                           states.get('steering_controller')!='active' and \
                           states.get('drive_controller')!='active' and \
                           len(wheels)>5 and len(odom)>5 and len(clocks)>5:
                            summary['controllers']=states
                            break
            summary['controller_query_count'] = query_count
            summary['ready']='controllers' in summary
            if summary['ready']:
                if late_scan_subscriber:
                    summary['scan_subscription_started_after_controller_ready']=True
                    scan_subscription = node.create_subscription(
                        LaserScan,'/scan',scan_cb,qos_profile_sensor_data)
                robot_params = node.create_client(
                    GetParameters, '/robot_state_publisher/get_parameters')
                if robot_params.wait_for_service(timeout_sec=2.0):
                    future = robot_params.call_async(
                        GetParameters.Request(names=['robot_description']))
                    rclpy.spin_until_future_complete(node,future,timeout_sec=3.0)
                    if future.done() and future.result() and future.result().values:
                        urdf = future.result().values[0].string_value
                        if urdf.strip().startswith('<'):
                            (out/'model.urdf').write_text(urdf)
                            summary['expanded_urdf_sha256'] = hashlib.sha256(
                                urdf.encode()).hexdigest()
                            stable_urdf = re.sub(
                                r'/tmp/vinuni_sim_[^/]+/controllers\.yaml',
                                '/tmp/vinuni_sim_CONTROLLERS/controllers.yaml',
                                urdf)
                            summary['canonical_urdf_sha256'] = hashlib.sha256(
                                stable_urdf.encode()).hexdigest()
                spin_for(3)
                transport_path = observer_dir/'transport_state.jsonl'
                pose_deadline = time.monotonic()+8
                while time.monotonic()<pose_deadline and launch.poll() is None \
                        and (not transport_path.exists() or
                             transport_path.stat().st_size==0):
                    spin_for(0.2)
                summary['observer_pose_ready'] = (
                    transport_path.exists() and transport_path.stat().st_size>0)
                if summary['observer_pose_ready']:
                    twist=Twist()
                    twist.linear.x,twist.linear.y,twist.angular.z=CASES[case]
                    summary['drive_start_wall_s']=time.monotonic()
                    spin_for(drive_duration_s,twist)
                    summary['drive_stop_wall_s']=time.monotonic()
                    publisher.publish(Twist())
                    if contact_probe:
                        published_commands.append({'wall':summary['drive_stop_wall_s'],
                                                   'vx':0.0,'vy':0.0,'omega':0.0})
                    spin_for(settling_duration_s)
            summary['launch_alive_at_measurement']=launch.poll() is None
            summary['observer_alive_at_measurement']=observer.poll() is None
            summary['clock_samples']=len(clocks)
            if len(clocks)>=2:
                summary['clock_sim_elapsed_s']=clocks[-1][1]-clocks[0][1]
                summary['real_time_factor']=summary['clock_sim_elapsed_s']/(
                    clocks[-1][0]-clocks[0][0])
            summary['wheel_samples']=len(wheels)
            summary['odom_samples']=len(odom)
            summary['tf_samples']=len(transforms)
            summary['scan_samples']=len(scans)
            if scans:
                summary['scan_source_monotonic'] = all(
                    b['source'] > a['source'] for a,b in zip(scans,scans[1:]))
                summary['scan_max_gap_wall_s'] = max(
                    (b['wall']-a['wall'] for a,b in zip(scans,scans[1:])),
                    default=0.0)
                summary['scan_frame']=scans[-1]['frame']
                summary['scan_ranges']=scans[-1]['count']
                summary['scan_finite_ranges']=scans[-1]['finite_count']
                summary['scan_any_finite']=any(
                    row['finite_count']>0 for row in scans)
            summary['normalized_command_samples']=len(commands)
            (out/'ros_trace.json').write_text(json.dumps({
                'clock':clocks,'wheel_states':wheels,'odom':odom,
                'tf':transforms,'normalized_commands':commands,
                'published_commands':published_commands,
                'scan_metadata':scans})+'\n')
        except Exception as error:
            summary['exception']=repr(error)
        finally:
            if node is not None:
                node.destroy_node()
                if rclpy.ok(): rclpy.shutdown()
            if observer.poll() is None: os.killpg(observer.pid,signal.SIGINT)
            try: summary['observer_exit']=observer.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(observer.pid,signal.SIGTERM)
                try:
                    summary['observer_exit']=observer.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(observer.pid,signal.SIGKILL)
                    summary['observer_exit']=observer.wait(timeout=5)
                    summary['observer_forced_shutdown']=True
            if launch.poll() is None: os.killpg(launch.pid,signal.SIGINT)
            try: summary['launch_exit']=launch.wait(timeout=18)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid,signal.SIGTERM)
                summary['launch_exit']=launch.wait(timeout=5)
                summary['forced_shutdown']=True
            if log_thread is not None:
                log_thread.join(timeout=5)
    path=observer_dir/'transport_state.jsonl'
    rows=read_jsonl(path) if path.exists() else []
    summary['gazebo_samples']=len(rows)
    if len(rows)>=2:
        models=[row['models'][0] for row in rows]
        first,last=models[0],models[-1]
        summary['gazebo_sim_elapsed_s']=rows[-1]['sim']-rows[0]['sim']
        summary['gazebo_wall_elapsed_s']=rows[-1]['wall']-rows[0]['wall']
        summary['gazebo_max_gap_wall_s']=max(b['wall']-a['wall']
            for a,b in zip(rows,rows[1:]))
        summary['gazebo_finite']=all(all(math.isfinite(v) for v in
            m['position']+m['rpy']+m['linear']+m['angular']) for m in models)
        summary['gazebo_delta_xyz_m']=[b-a for a,b in zip(
            first['position'],last['position'])]
        summary['gazebo_delta_yaw_rad']=math.remainder(
            last['rpy'][2]-first['rpy'][2],2*math.pi)
        summary['gazebo_max_tilt_rad']=max(abs(m['rpy'][axis])
            for m in models for axis in (0,1))
        summary['gazebo_final_max_speed_m_s']=max(math.sqrt(sum(v*v for v in m['linear']))
            for row,m in zip(rows,models) if row['wall']>=rows[-1]['wall']-1)
        stop=summary.get('drive_stop_wall_s')
        if stop is not None:
            at_stop=min(rows,key=lambda row:abs(row['wall']-stop))['models'][0]
            summary['gazebo_post_stop_displacement_m']=math.dist(
                at_stop['position'],last['position'])
    ros_trace_path = out/'ros_trace.json'
    summary['trace_contract'] = evaluate_vehicle_ros_trace(
        json.loads(ros_trace_path.read_text()) if ros_trace_path.exists() else {},
        summary.get('drive_start_wall_s', float('nan')),
        summary.get('drive_stop_wall_s', float('nan')))
    summary['trace_contract_valid'] = summary['trace_contract']['valid']
    runtime=(out/'runtime.log').read_text(errors='replace')
    summary['ode_lcp_count']=len(re.findall(r'ODE Message.*LCP|LCP internal error',runtime))
    abnormal_exit = re.search(
        r'\[ERROR\] \[gzserver-\d+\]: process has died .*?exit code (-?\d+)',
        runtime)
    summary['gzserver_exit_code'] = (int(abnormal_exit.group(1))
                                     if abnormal_exit else None)
    summary['gzserver_crash'] = abnormal_exit is not None
    summary['scan_contract_valid'] = bool(
        omit_ray_sensor or (summary.get('scan_samples',0)>5 and
                            summary.get('scan_frame')=='laser_frame' and
                            summary.get('scan_ranges')==500 and
                            summary.get('scan_any_finite') and
                            summary.get('scan_source_monotonic')))
    summary['motion_contract'] = evaluate_motion_contract(
        case, summary.get('gazebo_delta_xyz_m', [float('nan')] * 3),
        summary.get('gazebo_delta_yaw_rad', float('nan')))
    summary['motion_contract_valid'] = summary['motion_contract']['valid']
    summary['diagnostic_valid']=bool(summary.get('ready') and
        summary.get('gazebo_master_ready') and
        summary.get('observer_pose_ready') and
        summary.get('launch_alive_at_measurement') and
        summary.get('observer_alive_at_measurement') and
        summary.get('observer_exit')==0 and
        summary.get('gazebo_samples',0)>=100 and
        summary.get('gazebo_wall_elapsed_s',0)>=20 and
        summary.get('gazebo_sim_elapsed_s',0)>=20 and
        summary.get('gazebo_max_gap_wall_s',999)<1.0 and
        summary.get('gazebo_finite') and
        summary.get('gazebo_max_tilt_rad',999)<0.05 and
        summary.get('gazebo_final_max_speed_m_s',999)<0.03 and
        summary['ode_lcp_count']==0 and not summary['gzserver_crash'] and
        summary['scan_contract_valid'] and
        summary['trace_contract_valid'] and
        summary['motion_contract_valid'] and
        summary.get('expanded_urdf_sha256'))
    # Ground-truth direction and minimum motion are enforced above; odometry
    # remains corroborating evidence rather than the acceptance source.
    manifest['expanded_urdf_sha256'] = summary.get('expanded_urdf_sha256')
    manifest['canonical_urdf_sha256'] = summary.get('canonical_urdf_sha256')
    manifest['process_identity'] = {
        'launch_pid':summary.get('launch_pid'),
        'observer_pid':summary.get('observer_pid'),
        'launch_exit':summary.get('launch_exit'),
        'observer_exit':summary.get('observer_exit')}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    (out/'summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
    if contact_probe:
        barrier_analysis = analyze_directory(out, contact_target)
        summary['barrier_measurement_status'] = barrier_analysis['measurement_status']
        (out/'summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
    print(json.dumps({k:summary.get(k) for k in (
        'case','diagnostic_valid','gazebo_delta_xyz_m','gazebo_delta_yaw_rad',
        'gazebo_post_stop_displacement_m','gazebo_final_max_speed_m_s',
        'ode_lcp_count','gzserver_crash','exception')},indent=2))
    return 0 if summary['diagnostic_valid'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('output',type=Path)
    parser.add_argument('case',choices=CASES)
    parser.add_argument('--domain',type=int,default=96)
    parser.add_argument('--port',type=int,default=11396)
    sensor_group=parser.add_mutually_exclusive_group()
    sensor_group.add_argument('--omit-ray-sensor',action='store_true',
                        help='Generated diagnostic launch omits only the Gazebo ray sensor')
    sensor_group.add_argument('--gpu-ray-sensor',action='store_true',
                        help='Generated diagnostic launch uses GPU ray, preserving /scan')
    sensor_group.add_argument('--defer-ray-sensor',action='store_true',
                        help='Generated diagnostic launch sets CPU ray always_on=false')
    sensor_group.add_argument('--activate-ray-sensor',action='store_true',
                        help='Generated diagnostic launch sets CPU ray always_on=true')
    parser.add_argument('--late-scan-subscriber',action='store_true',
                        help='Subscribe to /scan only after controller readiness')
    parser.add_argument('--contact-probe',action='store_true',
                        help='Inject the existing diagnostic ODE contact trace into a generated launch')
    parser.add_argument('--contact-target',default=DEFAULT_TARGET,
                        help='Collision name substring to align with first LCP')
    parser.add_argument('--drive-duration',type=float,default=5.0,
                        help='Wall seconds of commanded motion after readiness; diagnostic only')
    parser.add_argument('--settling-duration',type=float,default=16.0,
                        help='Wall seconds of post-command observation; diagnostic only')
    parser.add_argument('--spawn-z',type=float,default=0.0,
                        help='Diagnostic-only vehicle spawn height override')
    parser.add_argument('--contact-kd',type=float,default=None,
                        help='Diagnostic-only damping override for all four wheel contacts')
    parser.add_argument('--surface-slip',type=float,default=None,
                        help='Diagnostic-only wheel contact plugin slip override')
    args=parser.parse_args()
    raise SystemExit(run(args.output,args.case,args.domain,args.port,
                         args.omit_ray_sensor,args.gpu_ray_sensor,
                         args.defer_ray_sensor,args.activate_ray_sensor,
                         args.late_scan_subscriber,
                         args.contact_probe,args.contact_target,
                         args.drive_duration,args.settling_duration,
                         args.spawn_z,args.contact_kd,args.surface_slip))
