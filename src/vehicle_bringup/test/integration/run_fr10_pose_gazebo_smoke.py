#!/usr/bin/env python3
"""Measured shared-model FR10 arm-only pose node and Gazebo position bridge."""
import argparse
import csv
import math
import json
import os
from pathlib import Path
import re
import signal
import socket
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


def run(out: Path, domain: int, port: int, return_joint_target: bool = False) -> int:
    out.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), RMW_LOCALHOST_ONLY='1',
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
               GAZEBO_MODEL_DATABASE_URI='', ROS_LOG_DIR=str(out / 'ros_log'))
    command = ['ros2', 'launch', 'vehicle_bringup',
               'bringup.launch.py', 'use_vehicle:=false',
               'use_arm:=true', 'use_gazebo:=true',
               'gui:=false', 'rviz:=false', f'gazebo_port:={port}']
    summary = {'command': command, 'domain': domain, 'gazebo_port': port}
    manifest = new_manifest(out, command, env, 'A_arm_only')
    observer_env = env.copy()
    observer_env.update(ODE_OUT=str(out), ODE_WORLD_NAME='default',
                        ODE_TARGET_ENTITY='my_mobile_manipulator::wrist3_link')
    with (out / 'runtime.log').open('w') as log, (out / 'transport.log').open('w') as transport_log:
        launch = subprocess.Popen(command, cwd=WORKSPACE, env=env, stdout=log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        summary['launch_pid'] = launch.pid
        master_deadline = time.monotonic() + 15
        master_ready = False
        while time.monotonic() < master_deadline and launch.poll() is None:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=0.2):
                    master_ready = True
                    break
            except OSError:
                time.sleep(0.05)
        summary['gazebo_master_ready'] = master_ready
        observer = subprocess.Popen([str(observer_binary())], cwd=WORKSPACE,
                                    env=observer_env, stdout=transport_log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
        summary['observer_pid'] = observer.pid
        try:
            with ProbeContext(domain) as probe:
                from rosgraph_msgs.msg import Clock
                from rclpy.qos import qos_profile_sensor_data
                clock_trace = []
                def on_clock(message):
                    wall = time.monotonic()
                    if not clock_trace or wall - clock_trace[-1][0] >= 0.05:
                        clock_trace.append((wall, message.clock.sec +
                                            message.clock.nanosec * 1e-9))
                probe.create_subscription(Clock, '/clock', on_clock,
                                          qos_profile_sensor_data)
                started = time.monotonic()
                ready = probe.wait_ready(timeout=80)
                summary['ready'] = ready
                summary['startup_wall_s'] = time.monotonic() - started
                summary['status'] = probe.status()
                summary['initial_joint_rad'] = probe.joint_position()
                summary['initial_tcp'] = probe.tcp_pose()
                ready_target = [0.0, -math.pi / 2, math.pi / 2,
                                -math.pi / 2, -math.pi / 2, 0.0]
                summary['ready_target_rad'] = ready_target
                summary['ready_max_joint_error_rad'] = (
                    max_abs_error(summary['initial_joint_rad'], ready_target)
                    if summary['initial_joint_rad'] else None)
                summary['charging_tcp_tf_connected'] = probe.tf_connected()
                publishers = probe.get_publishers_info_by_topic('/joint_states')
                summary['joint_state_publishers'] = [
                    {'node_name': info.node_name, 'node_namespace': info.node_namespace,
                     'topic_type': info.topic_type} for info in publishers]
                if ready and master_ready and launch.poll() is None:
                    summary.update(capture_live_model(probe, out))
                    transport_path = out / 'transport_state.jsonl'
                    pose_deadline = time.monotonic() + 8
                    while (time.monotonic() < pose_deadline and
                           launch.poll() is None and observer.poll() is None and
                           (not transport_path.exists() or
                            transport_path.stat().st_size == 0)):
                        time.sleep(0.2)
                    summary['observer_pose_ready'] = (
                        transport_path.exists() and transport_path.stat().st_size > 0)
                if (ready and master_ready and launch.poll() is None and
                        summary.get('observer_pose_ready')):
                    summary['controllers'] = list_controllers(probe)
                    summary['controller_state_map'] = {
                        row['name']: {'type': row['type'], 'state': row['state']}
                        for row in summary['controllers']}
                    initial = probe.joint_position()
                    target = list(initial)
                    target[0] += 0.03
                    summary['target_joint_rad'] = target
                    probe.publish_joint(target)
                    summary['joint_reached'] = probe.wait_joint(target, timeout=25)
                    summary['single_joint_final_rad'] = probe.joint_position()
                    summary['single_joint_max_error_rad'] = (
                        max_abs_error(summary['single_joint_final_rad'], target)
                        if summary['single_joint_final_rad'] else None)
                    multi_target = list(target)
                    multi_target[1] += 0.02
                    multi_target[2] -= 0.02
                    multi_target[3] += 0.015
                    summary['multi_joint_target_rad'] = multi_target
                    if summary['joint_reached']:
                        probe.publish_joint(multi_target)
                        summary['multi_joint_reached'] = probe.wait_joint(
                            multi_target, timeout=25)
                    else:
                        summary['multi_joint_reached'] = False
                    summary['multi_joint_final_rad'] = probe.joint_position()
                    summary['multi_joint_max_error_rad'] = (
                        max_abs_error(summary['multi_joint_final_rad'], multi_target)
                        if summary['multi_joint_final_rad'] else None)
                    summary['joint_final_tcp'] = probe.tcp_pose()
                    if summary['multi_joint_reached'] and summary['joint_final_tcp']:
                        pose_start = summary['joint_final_tcp']
                        pose_target = list(pose_start[0])
                        pose_target[0] += 0.02
                        summary['pose_target'] = [pose_target, pose_start[1]]
                        probe.publish_pose_message(
                            pose_target, pose_start[1], 'fr10_base_link', stamp_now=False)
                        summary['pose_reached'] = probe.wait_pose(
                            pose_target, pose_start[1], timeout=60)
                    else:
                        summary['pose_reached'] = False
                    summary['pose_arrival_tcp'] = probe.tcp_pose()
                    if summary['pose_arrival_tcp'] and summary.get('pose_target'):
                        summary['pose_position_error_m'] = position_error(
                            summary['pose_arrival_tcp'][0], summary['pose_target'][0])
                        summary['pose_orientation_error_rad'] = quaternion_error(
                            summary['pose_arrival_tcp'][1], summary['pose_target'][1])
                    time.sleep(2)
                    summary['hold_tcp'] = probe.tcp_pose()
                    summary['final_joint_rad'] = probe.joint_position()
                    summary['final_tcp'] = summary['hold_tcp']
                    summary['command_state'] = probe.command_state()
                    summary['final_status'] = probe.status()
                    if summary['pose_arrival_tcp'] and summary['hold_tcp']:
                        summary['hold_position_drift_m'] = position_error(
                            summary['hold_tcp'][0], summary['pose_arrival_tcp'][0])
                        summary['hold_orientation_drift_rad'] = quaternion_error(
                            summary['hold_tcp'][1], summary['pose_arrival_tcp'][1])
                    summary.update(write_feedback_trace(probe, out))
                    if return_joint_target:
                        # Regression: the same JOINT goal is new intent after POSE.
                        probe.publish_joint(multi_target)
                        summary['joint_after_pose_reached'] = probe.wait_joint(
                            multi_target, timeout=25)
                        summary['joint_after_pose_final_rad'] = probe.joint_position()
                        summary['joint_after_pose_status'] = probe.status()
                summary['server_alive_at_measurement'] = launch.poll() is None
                summary['observer_alive_at_measurement'] = observer.poll() is None
                summary['clock_samples'] = len(clock_trace)
                if len(clock_trace) >= 2:
                    sim_elapsed = clock_trace[-1][1] - clock_trace[0][1]
                    wall_elapsed = clock_trace[-1][0] - clock_trace[0][0]
                    summary['clock_sim_elapsed_s'] = sim_elapsed
                    summary['clock_wall_elapsed_s'] = wall_elapsed
                    summary['gazebo_real_time_factor'] = (
                        sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0)
                with (out / 'clock_trace.csv').open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(['wall_monotonic_s', 'sim_s'])
                    writer.writerows(clock_trace)
        except Exception as error:
            summary['exception'] = repr(error)
        finally:
            if observer.poll() is None:
                os.killpg(observer.pid, signal.SIGINT)
            try:
                summary['observer_exit'] = observer.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(observer.pid, signal.SIGTERM)
                try:
                    summary['observer_exit'] = observer.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(observer.pid, signal.SIGKILL)
                    summary['observer_exit'] = observer.wait(timeout=5)
                    summary['observer_forced_shutdown'] = True
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
            try:
                summary['launch_exit'] = launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                summary['launch_exit'] = launch.wait(timeout=5)
                summary['forced_shutdown'] = True
    transport_path = out / 'transport_state.jsonl'
    transport_rows = [json.loads(line) for line in transport_path.read_text().splitlines()]
    summary['gazebo_wrist_samples'] = len(transport_rows)
    wall_gaps = [next_row['wall'] - row['wall']
                 for row, next_row in zip(transport_rows, transport_rows[1:])]
    summary['gazebo_wrist_max_gap_wall_s'] = max(wall_gaps, default=None)
    summary['gazebo_wrist_sim_monotonic'] = all(
        next_row['sim'] > row['sim']
        for row, next_row in zip(transport_rows, transport_rows[1:]))
    summary['gazebo_wrist_finite'] = all(
        all(math.isfinite(v) for v in row['models'][0]['position'])
        for row in transport_rows)
    if len(transport_rows) >= 2:
        start = transport_rows[0]['models'][0]['position']
        finish = transport_rows[-1]['models'][0]['position']
        summary['gazebo_wrist_start'] = start
        summary['gazebo_wrist_final'] = finish
        summary['gazebo_wrist_motion_m'] = math.dist(start, finish)
        summary['gazebo_wrist_sim_elapsed_s'] = (
            transport_rows[-1]['sim'] - transport_rows[0]['sim'])
    content = (out / 'runtime.log').read_text(errors='replace')
    summary['ode_lcp_count'] = len(re.findall(r'ODE Message.*LCP|LCP internal error', content))
    abnormal_exit = re.search(
        r'\[ERROR\] \[gzserver-\d+\]: process has died .*?exit code (-?\d+)',
        content)
    summary['gzserver_exit_code'] = (int(abnormal_exit.group(1))
                                     if abnormal_exit else None)
    summary['gzserver_crash'] = abnormal_exit is not None
    controller_states = summary.get('controller_state_map', {})
    summary['pass'] = bool(summary.get('ready') and summary.get('joint_reached')
        and summary.get('multi_joint_reached') and summary.get('pose_reached')
        and summary.get('ready_max_joint_error_rad', float('inf')) <= 0.02
        and summary.get('single_joint_max_error_rad', float('inf')) <= 0.02
        and summary.get('multi_joint_max_error_rad', float('inf')) <= 0.02
        and summary.get('pose_position_error_m', float('inf')) <= 0.01
        and summary.get('pose_orientation_error_rad', float('inf')) <= math.radians(2.0)
        and summary.get('hold_position_drift_m', float('inf')) <= 0.002
        and summary.get('hold_orientation_drift_rad', float('inf')) <= 0.01
        and 'state=HOLDING' in summary.get('final_status', '')
        and summary.get('charging_tcp_tf_connected')
        and len(summary.get('joint_state_publishers', [])) == 1
        and summary.get('server_alive_at_measurement')
        and summary.get('gazebo_master_ready') and summary.get('observer_pose_ready')
        and summary.get('model_robot_name') == 'complete_visual'
        and summary.get('shared_arm_signature_sha256')
        and summary.get('tcp_source_monotonic')
        and summary.get('tcp_source_matches_joint')
        and controller_states.get('fairino10_controller') == {
            'type': 'position_controllers/JointGroupPositionController',
            'state': 'active'}
        and controller_states.get('joint_state_broadcaster') == {
            'type': 'joint_state_broadcaster/JointStateBroadcaster',
            'state': 'active'}
        and 'joint_trajectory_controller' not in controller_states
        and summary.get('gazebo_wrist_samples', 0) >= 100
        and summary.get('observer_alive_at_measurement')
        and summary.get('observer_exit') == 0
        and summary.get('gazebo_wrist_sim_monotonic')
        and summary.get('gazebo_wrist_finite')
        and summary.get('gazebo_wrist_max_gap_wall_s', 999) < 1.0
        and summary.get('clock_sim_elapsed_s', 0) >= 20
        and summary['ode_lcp_count'] == 0 and not summary['gzserver_crash'])
    if return_joint_target:
        summary['pass'] = summary['pass'] and bool(
            summary.get('joint_after_pose_reached'))
    finish_manifest(out, manifest, summary)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
    print(json.dumps({key: summary.get(key) for key in ('pass', 'ready', 'joint_reached',
         'multi_joint_reached', 'pose_reached', 'ready_max_joint_error_rad',
         'single_joint_max_error_rad', 'multi_joint_max_error_rad',
         'pose_position_error_m', 'pose_orientation_error_rad',
         'hold_position_drift_m', 'final_status', 'ode_lcp_count',
         'gzserver_crash', 'controllers', 'exception')}, indent=2))
    return 0 if summary['pass'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--domain', type=int, default=94)
    parser.add_argument('--port', type=int, default=11394)
    parser.add_argument('--return-joint-target', action='store_true')
    args = parser.parse_args()
    raise SystemExit(run(args.output, args.domain, args.port, args.return_joint_target))
