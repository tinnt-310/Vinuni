#!/usr/bin/env python3
"""Measured coupled FR10 pose-node and swerve-plugin host-Gazebo smoke."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

WORKSPACE = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(WORKSPACE / 'src/fr10_arm_control/fr10_pose_controller/test/simmachine'))
from ros_health import ProbeContext  # noqa: E402
from ode_transport_runtime import observer_binary  # noqa: E402
from fr10_model_evidence import capture_live_model  # noqa: E402
from fr10_feedback_trace import write_feedback_trace  # noqa: E402
from fr10_run_manifest import new_manifest, finish_manifest  # noqa: E402
from metrics import max_abs_error, position_error, quaternion_error  # noqa: E402


def read_transport(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def list_controllers(node, timeout=15.0):
    """Read controller ownership from controller_manager without ros2controlcli."""
    from controller_manager_msgs.srv import ListControllers
    client = node.create_client(
        ListControllers, '/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=timeout):
        return []
    future = client.call_async(ListControllers.Request())
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not future.done() or future.result() is None:
        return []
    return [{'name': controller.name, 'type': controller.type,
             'state': controller.state}
            for controller in future.result().controller]


def run(out, domain, port, diagnostic=False, drive_only=False):
    if diagnostic and drive_only:
        raise ValueError('--diagnostic and --drive-only are mutually exclusive')
    scenario = ('hold_drive' if drive_only else
                'arm_stationary' if diagnostic else 'simultaneous')
    out.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), RMW_LOCALHOST_ONLY='1',
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
               GAZEBO_MODEL_DATABASE_URI='', ROS_LOG_DIR=str(out/'ros_log'))
    # The in-process observer and child launch must share the DDS transport.
    os.environ['RMW_LOCALHOST_ONLY'] = env['RMW_LOCALHOST_ONLY']
    command = ['ros2','launch','vehicle_bringup','bringup.launch.py',
               'use_vehicle:=true','use_arm:=true',
               'use_gazebo:=true','gui:=false','rviz:=false',
               f'gazebo_port:={port}']
    summary = {'command': command, 'domain': domain, 'port': port,
               'diagnostic': diagnostic, 'drive_only': drive_only,
               'scenario': scenario}
    manifest = new_manifest(out, command, env, 'C_mounted')
    observers = []
    with (out/'runtime.log').open('w') as log:
        launch = subprocess.Popen(command,cwd=WORKSPACE,env=env,stdout=log,
                                  stderr=subprocess.STDOUT,start_new_session=True)
        summary['launch_pid'] = launch.pid
        for label,target in [('vehicle','my_mobile_manipulator'),
                             ('wrist','my_mobile_manipulator::wrist3_link')]:
            directory = out / label
            directory.mkdir()
            observer_env = env.copy()
            observer_env.update(ODE_OUT=str(directory),ODE_WORLD_NAME='default',
                                ODE_TARGET_ENTITY=target)
            handle = (directory/'observer.log').open('w')
            process = subprocess.Popen([str(observer_binary())],cwd=WORKSPACE,
                                       env=observer_env,stdout=handle,
                                       stderr=subprocess.STDOUT,start_new_session=True)
            observers.append((label,process,handle))
            summary[label+'_observer_pid'] = process.pid
        try:
            with ProbeContext(domain) as probe:
                from geometry_msgs.msg import Twist, TwistStamped
                from nav_msgs.msg import Odometry
                from sensor_msgs.msg import JointState, LaserScan
                from rosgraph_msgs.msg import Clock
                from rclpy.qos import qos_profile_sensor_data
                clock = []
                odom = []
                normalized_commands = []
                wheel_states = []
                scans = []
                wheels = tuple(f'Wheel{i}_to_Revolute{i}' for i in range(1,5))
                steering = tuple(f'Revolute{i}_to_base_link' for i in range(1,5))
                def wheel_callback(message):
                    names = {name:i for i,name in enumerate(message.name)}
                    if not all(name in names for name in wheels+steering):
                        return
                    wheel_states.append({
                        'wall':time.monotonic(),
                        'source':message.header.stamp.sec+
                                 message.header.stamp.nanosec*1e-9,
                        'wheel_velocity':[message.velocity[names[name]]
                                          if names[name]<len(message.velocity)
                                          else None for name in wheels],
                        'steering_position':[message.position[names[name]]
                                             if names[name]<len(message.position)
                                             else None for name in steering]})
                def odom_callback(message):
                    odom.append({'wall':time.monotonic(),
                        'source':message.header.stamp.sec+
                                 message.header.stamp.nanosec*1e-9,
                        'x':message.pose.pose.position.x,
                        'y':message.pose.pose.position.y,
                        'vx':message.twist.twist.linear.x,
                        'vy':message.twist.twist.linear.y})
                def command_callback(message):
                    normalized_commands.append({'wall':time.monotonic(),
                        'source':message.header.stamp.sec+
                                 message.header.stamp.nanosec*1e-9,
                        'vx':message.twist.linear.x,
                        'vy':message.twist.linear.y,
                        'omega':message.twist.angular.z})
                probe.create_subscription(Clock,'/clock',lambda m: clock.append(
                    (time.monotonic(),m.clock.sec+m.clock.nanosec*1e-9)),
                    qos_profile_sensor_data)
                probe.create_subscription(Odometry,'/odom',odom_callback,10)
                probe.create_subscription(TwistStamped,'/cmd_vel_nav',
                                          command_callback,10)
                probe.create_subscription(JointState,'/joint_states',
                                          wheel_callback,qos_profile_sensor_data)
                def scan_callback(message):
                    scans.append({'wall':time.monotonic(),
                        'source':message.header.stamp.sec+
                                 message.header.stamp.nanosec*1e-9,
                        'frame':message.header.frame_id,
                        'count':len(message.ranges),
                        'finite_count':sum(math.isfinite(x)
                                           for x in message.ranges)})
                probe.create_subscription(LaserScan,'/scan',scan_callback,
                                          qos_profile_sensor_data)
                drive = probe.create_publisher(Twist,'/cmd_vel',10)
                summary['ready'] = probe.wait_ready(timeout=90)
                summary['initial_status'] = probe.status()
                if summary['ready'] and launch.poll() is None:
                    summary.update(capture_live_model(probe, out))
                    time.sleep(3)
                    summary['initial_joint'] = probe.joint_position()
                    summary['initial_tcp'] = probe.tcp_pose()
                    ready_target = [0.0, -math.pi / 2, math.pi / 2,
                                    -math.pi / 2, -math.pi / 2, 0.0]
                    summary['ready_target_rad'] = ready_target
                    summary['ready_max_joint_error_rad'] = (
                        max_abs_error(summary['initial_joint'], ready_target)
                        if summary['initial_joint'] else None)
                    summary['charging_tcp_tf_connected'] = probe.tf_connected()
                    try:
                        import rclpy
                        probe.tf_buffer.lookup_transform(
                            'odom', 'charging_tcp', rclpy.time.Time())
                        summary['world_to_tcp_tf_connected'] = True
                    except Exception as error:
                        summary['world_to_tcp_tf_connected'] = False
                        summary['world_to_tcp_tf_error'] = repr(error)
                    publishers = probe.get_publishers_info_by_topic('/joint_states')
                    summary['joint_state_publishers'] = [
                        {'node_name': info.node_name,
                         'node_namespace': info.node_namespace,
                         'topic_type': info.topic_type}
                        for info in publishers]
                    summary['controllers'] = list_controllers(probe)
                    summary['controller_state_map'] = {
                        row['name']: {'type': row['type'], 'state': row['state']}
                        for row in summary['controllers']}
                    initial = summary['initial_tcp']
                    target = list(initial[0]); target[0] += 0.02
                    summary['pose_target'] = [target,initial[1]]
                    with probe.lock:
                        history_start = len(probe.tcp_history)
                        joint_history_start = len(probe.joint_history)
                    scenario_start = time.monotonic()
                    summary['scenario_start_wall_s'] = scenario_start
                    drive_start = None
                    drive_stop = None
                    if scenario == 'hold_drive':
                        summary['pose_reached'] = None
                        probe.publish_joint(ready_target)
                        summary['hold_ready_reached'] = probe.wait_joint(
                            ready_target, tolerance=0.002, timeout=15)
                        drive_start = time.monotonic()
                        summary['drive_start_wall_s'] = drive_start
                        command = Twist(); command.linear.x = 0.12
                        while time.monotonic()-drive_start < 5.0:
                            drive.publish(command)
                            time.sleep(0.05)
                        drive_stop = time.monotonic()
                        summary['drive_stop_wall_s'] = drive_stop
                        drive.publish(Twist())
                        summary['drive_command_duration_s'] = drive_stop-drive_start
                        time.sleep(3)
                    elif scenario == 'arm_stationary':
                        probe.publish_pose_message(
                            target, initial[1], 'fr10_base_link', stamp_now=False)
                        summary['pose_reached'] = probe.wait_pose(
                            target, initial[1], timeout=65)
                        summary['pose_arrival_tcp'] = probe.tcp_pose()
                        time.sleep(2)
                    else:
                        probe.publish_pose_message(
                            target, initial[1], 'fr10_base_link', stamp_now=False)
                        drive_start = time.monotonic()
                        summary['drive_start_wall_s'] = drive_start
                        command = Twist(); command.linear.x = 0.12
                        while time.monotonic()-drive_start < 5.0:
                            drive.publish(command)
                            time.sleep(0.05)
                        drive_stop = time.monotonic()
                        summary['drive_stop_wall_s'] = drive_stop
                        drive.publish(Twist())
                        summary['drive_command_duration_s'] = drive_stop-drive_start
                        summary['pose_reached'] = probe.wait_pose(
                            target, initial[1], timeout=65)
                        summary['pose_arrival_tcp'] = probe.tcp_pose()
                        time.sleep(2)
                    remaining_observation = 15.0 - (time.monotonic() - scenario_start)
                    if remaining_observation > 0:
                        time.sleep(remaining_observation)
                    summary['scenario_stop_wall_s'] = time.monotonic()
                    summary['hold_tcp'] = probe.tcp_pose()
                    summary['final_tcp'] = summary['hold_tcp']
                    summary['final_joint'] = probe.joint_position()
                    summary['final_status'] = probe.status()
                    with probe.lock:
                        tcp_rows = list(probe.tcp_history[history_start:])
                        joint_rows = list(probe.joint_history[joint_history_start:])
                    scenario_rows = [row for row in tcp_rows
                                     if scenario_start <= row[0] <=
                                     summary['scenario_stop_wall_s']]
                    summary['tcp_samples_during_scenario'] = len(scenario_rows)
                    if len(scenario_rows) >= 2:
                        summary['tcp_motion_during_scenario_m'] = math.dist(
                            scenario_rows[0][1], scenario_rows[-1][1])
                    if drive_start is not None and drive_stop is not None:
                        during = [row for row in tcp_rows
                                  if drive_start <= row[0] <= drive_stop]
                        summary['tcp_samples_during_drive'] = len(during)
                        if len(during) >= 2:
                            summary['tcp_motion_during_drive_m'] = math.dist(
                                during[0][1], during[-1][1])
                    if scenario == 'hold_drive':
                        if summary['initial_tcp'] and summary['hold_tcp']:
                            summary['hold_position_drift_m'] = position_error(
                                summary['hold_tcp'][0], summary['initial_tcp'][0])
                            summary['hold_orientation_drift_rad'] = quaternion_error(
                                summary['hold_tcp'][1], summary['initial_tcp'][1])
                        if summary['initial_joint'] and summary['final_joint']:
                            summary['hold_joint_max_drift_rad'] = max_abs_error(
                                summary['final_joint'], summary['initial_joint'])
                    else:
                        arrival = summary.get('pose_arrival_tcp')
                        if arrival:
                            summary['pose_position_error_m'] = position_error(
                                arrival[0], target)
                            summary['pose_orientation_error_rad'] = quaternion_error(
                                arrival[1], initial[1])
                        if arrival and summary['hold_tcp']:
                            summary['hold_position_drift_m'] = position_error(
                                summary['hold_tcp'][0], arrival[0])
                            summary['hold_orientation_drift_rad'] = quaternion_error(
                                summary['hold_tcp'][1], arrival[1])
                    summary['joint_samples_during_scenario'] = len(joint_rows)
                    (out/'tcp_trace.json').write_text(json.dumps(tcp_rows)+'\n')
                summary['clock_samples'] = len(clock)
                if len(clock) >= 2:
                    summary['clock_sim_elapsed_s'] = clock[-1][1]-clock[0][1]
                    summary['real_time_factor'] = summary['clock_sim_elapsed_s'] / (
                        clock[-1][0]-clock[0][0])
                summary['odom_samples'] = len(odom)
                summary['wheel_state_samples'] = len(wheel_states)
                summary['scan_samples'] = len(scans)
                summary['scan_contract_valid'] = bool(len(scans)>5 and
                    all(row['frame']=='laser_frame' and row['count']==500
                        for row in scans) and
                    all(b['source']>a['source']
                        for a,b in zip(scans,scans[1:])) and
                    any(row['finite_count']>0 for row in scans))
                (out/'scan_metadata.json').write_text(json.dumps(scans)+'\n')
                summary['normalized_command_samples'] = len(normalized_commands)
                (out/'vehicle_ros_trace.json').write_text(json.dumps({
                    'odom':odom,'wheel_states':wheel_states,
                    'normalized_commands':normalized_commands})+'\n')
                summary.update(write_feedback_trace(probe, out))
                summary['launch_alive_at_measurement'] = launch.poll() is None
                summary['observers_alive_at_measurement'] = all(
                    process.poll() is None for _,process,_ in observers)
        except Exception as error:
            summary['exception'] = repr(error)
        finally:
            for _,process,_ in observers:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
            for label,process,handle in observers:
                try:
                    summary[label+'_observer_exit'] = process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:
                        summary[label+'_observer_exit'] = process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        summary[label+'_observer_exit'] = process.wait(timeout=5)
                        summary[label+'_observer_forced_shutdown'] = True
                handle.close()
            if launch.poll() is None:
                os.killpg(launch.pid,signal.SIGINT)
            try:
                summary['launch_exit'] = launch.wait(timeout=18)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid,signal.SIGTERM)
                try:
                    summary['launch_exit'] = launch.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    summary['launch_exit'] = launch.wait(timeout=5)
                summary['forced_shutdown'] = True
    for label in ('vehicle','wrist'):
        rows = read_transport(out/label/'transport_state.jsonl')
        summary[label+'_samples'] = len(rows)
        if len(rows) >= 2:
            start = rows[0]['models'][0]['position']
            finish = rows[-1]['models'][0]['position']
            summary[label+'_actual_displacement_m'] = math.dist(start,finish)
            summary[label+'_sim_elapsed_s'] = rows[-1]['sim']-rows[0]['sim']
            summary[label+'_finite'] = all(all(math.isfinite(x) for x in
                (row['models'][0]['position']+row['models'][0]['rpy']+
                 row['models'][0]['linear']+row['models'][0]['angular']))
                for row in rows)
            summary[label+'_max_gap_wall_s'] = max(b['wall']-a['wall']
                for a,b in zip(rows,rows[1:]))
            if label == 'vehicle':
                summary['vehicle_max_tilt_rad'] = max(
                    abs(row['models'][0]['rpy'][axis])
                    for row in rows for axis in (0,1))
                summary['vehicle_final_max_speed_m_s'] = max(
                    math.sqrt(sum(v*v for v in row['models'][0]['linear']))
                    for row in rows if row['wall'] >= rows[-1]['wall']-1.0)
                stop = summary.get('drive_stop_wall_s')
                if stop is not None:
                    at_stop = min(rows,key=lambda row:abs(row['wall']-stop))
                    summary['vehicle_post_stop_displacement_m'] = math.dist(
                        at_stop['models'][0]['position'],
                        rows[-1]['models'][0]['position'])
    content = (out/'runtime.log').read_text(errors='replace')
    summary['ode_lcp_count'] = len(re.findall(r'ODE Message.*LCP|LCP internal error',content))
    abnormal_exit = re.search(
        r'\[ERROR\] \[gzserver-\d+\]: process has died .*?exit code (-?\d+)',
        content)
    summary['gzserver_exit_code'] = (int(abnormal_exit.group(1))
                                     if abnormal_exit else None)
    summary['gzserver_crash'] = abnormal_exit is not None
    controller_states = summary.get('controller_state_map', {})
    common_pass = bool(summary.get('ready')
        and summary.get('ready_max_joint_error_rad', float('inf')) <= 0.02
        and summary.get('charging_tcp_tf_connected')
        and summary.get('world_to_tcp_tf_connected')
        and len(summary.get('joint_state_publishers', [])) == 1
        and summary.get('launch_alive_at_measurement')
        and summary.get('model_robot_name') == 'complete_visual'
        and summary.get('shared_arm_signature_sha256')
        and summary.get('tcp_source_monotonic')
        and summary.get('tcp_source_matches_joint')
        and summary.get('observers_alive_at_measurement')
        and summary.get('vehicle_observer_exit') == 0
        and summary.get('wrist_observer_exit') == 0
        and controller_states.get('swerve_controller', {}).get('state') == 'active'
        and controller_states.get('fairino10_controller') == {
            'type': 'position_controllers/JointGroupPositionController',
            'state': 'active'}
        and controller_states.get('joint_state_broadcaster') == {
            'type': 'joint_state_broadcaster/JointStateBroadcaster',
            'state': 'active'}
        and 'joint_trajectory_controller' not in controller_states
        and summary.get('vehicle_samples',0) >= 100
        and summary.get('wrist_samples',0) >= 100
        and summary.get('scan_contract_valid')
        and summary.get('vehicle_finite') and summary.get('wrist_finite')
        and summary.get('vehicle_max_tilt_rad',float('inf')) < 0.05
        and summary.get('vehicle_final_max_speed_m_s',float('inf')) < 0.03
        and summary.get('clock_sim_elapsed_s',0) >= 20
        and summary.get('tcp_samples_during_scenario',0) >= 20
        and 'state=HOLDING' in summary.get('final_status', '')
        and summary['ode_lcp_count'] == 0 and not summary['gzserver_crash'])
    if scenario == 'hold_drive':
        scenario_pass = bool(
            summary.get('hold_ready_reached')
            and summary.get('vehicle_actual_displacement_m', 0) >= 0.40
            and summary.get('tcp_samples_during_drive', 0) >= 20
            and summary.get('hold_position_drift_m', float('inf')) <= 0.01
            and summary.get('hold_orientation_drift_rad', float('inf')) <=
                math.radians(2.0)
            and summary.get('hold_joint_max_drift_rad', float('inf')) <= 0.02)
    elif scenario == 'arm_stationary':
        scenario_pass = bool(
            summary.get('vehicle_actual_displacement_m', float('inf')) <= 0.03
            and summary.get('pose_reached')
            and summary.get('tcp_motion_during_scenario_m', 0) > 0.005
            and summary.get('pose_position_error_m', float('inf')) <= 0.01
            and summary.get('pose_orientation_error_rad', float('inf')) <=
                math.radians(2.0)
            and summary.get('hold_position_drift_m', float('inf')) <= 0.002
            and summary.get('hold_orientation_drift_rad', float('inf')) <= 0.01)
    else:
        scenario_pass = bool(
            summary.get('vehicle_actual_displacement_m', 0) >= 0.40
            and summary.get('tcp_samples_during_drive', 0) >= 20
            and summary.get('tcp_motion_during_drive_m', 0) > 0.005
            and summary.get('pose_reached')
            and summary.get('pose_position_error_m', float('inf')) <= 0.01
            and summary.get('pose_orientation_error_rad', float('inf')) <=
                math.radians(2.0)
            and summary.get('hold_position_drift_m', float('inf')) <= 0.002
            and summary.get('hold_orientation_drift_rad', float('inf')) <= 0.01)
    summary['common_pass'] = common_pass
    summary['scenario_pass'] = scenario_pass
    summary['pass'] = common_pass and scenario_pass
    finish_manifest(out, manifest, summary)
    (out/'summary.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
    print(json.dumps({k:summary.get(k) for k in ('pass','common_pass',
        'scenario_pass','scenario','ready','pose_reached',
        'ready_max_joint_error_rad','pose_position_error_m',
        'tcp_motion_during_drive_m','tcp_motion_during_scenario_m',
        'hold_position_drift_m','hold_joint_max_drift_rad',
        'vehicle_actual_displacement_m',
        'vehicle_samples','wrist_samples','clock_sim_elapsed_s',
        'ode_lcp_count','gzserver_crash','exception')},indent=2))
    return 0 if summary['pass'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('output',type=Path)
    parser.add_argument('--domain',type=int,default=95)
    parser.add_argument('--port',type=int,default=11395)
    parser.add_argument('--diagnostic',action='store_true')
    parser.add_argument('--drive-only',action='store_true')
    args=parser.parse_args()
    raise SystemExit(run(args.output,args.domain,args.port,args.diagnostic,args.drive_only))
