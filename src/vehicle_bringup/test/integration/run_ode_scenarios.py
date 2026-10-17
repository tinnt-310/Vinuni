"""Run arm, vehicle, or mounted Gazebo regressions in an isolated ROS domain.

Source install/setup.bash first. Never connects to a
FAIRINO cabinet. Usage: python3 src/vehicle_bringup/test/integration/run_ode_scenarios.py LABEL MODE
where MODE is arm, vehicle, or mounted.
"""
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from evaluate_gazebo_pose import evaluate_pose_trace
from ode_scenario_readiness import wait_for_controllers
from ode_transport_runtime import observer_binary

workspace_root = Path(__file__).resolve().parents[4]
os.chdir(workspace_root)

label, mode = sys.argv[1:3]
if mode not in ('arm', 'vehicle', 'mounted'):
    raise SystemExit('MODE must be arm, vehicle, or mounted')
out = workspace_root / 'docs' / 'ode_debug' / label
out.mkdir(parents=True, exist_ok=False)
launch_source = Path('src/vehicle_bringup/launch/simulation.launch.py').read_text()
launch_source = launch_source.replace(
    '    # Source Xacros retain package://.',
    "    from pathlib import Path\n"
    "    Path(os.environ['ODE_OUT']+'/model.urdf').write_text(doc.toxml())\n"
    '    # Source Xacros retain package://.')
if os.environ.get('ODE_WHEEL_KP'):
    launch_source = launch_source.replace(
        '    # Gazebo-only base collision:',
        "    for gazebo in doc.getElementsByTagName('gazebo'):\n"
        "        if gazebo.getAttribute('reference').startswith('Wheel'):\n"
        "            for kp in gazebo.getElementsByTagName('kp'):\n"
        "                kp.firstChild.nodeValue = os.environ['ODE_WHEEL_KP']\n"
        '    # Gazebo-only base collision:')
if os.environ.get('ODE_SOLVER') == 'quick':
    world_source = Path('src/vehicle_description/worlds/vinfast_map.world').read_text()
    old_solver = '<type>world</type>'
    if old_solver not in world_source:
        raise SystemExit('Expected ODE world solver marker was not found')
    (out / 'world.sdf').write_text(world_source.replace(
        old_solver, '<type>quick</type><iters>100</iters><sor>1.0</sor>', 1))
    launch_source = launch_source.replace(
        "'worlds', 'vinfast_map.world'", repr(str((out / 'world.sdf').resolve())))
if os.environ.get('ODE_CFM'):
    world_source = Path('src/vehicle_description/worlds/vinfast_map.world').read_text()
    old_cfm = '<cfm>0.00001</cfm>'
    if old_cfm not in world_source:
        raise SystemExit('Expected ODE CFM marker was not found')
    (out / 'world.sdf').write_text(world_source.replace(
        old_cfm, '<cfm>' + os.environ['ODE_CFM'] + '</cfm>', 1))
    launch_source = launch_source.replace(
        "'worlds', 'vinfast_map.world'", repr(str((out / 'world.sdf').resolve())))
if os.environ.get('ODE_GROUND_MAX_CONTACTS'):
    world_path = out / 'world.sdf'
    world_source = world_path.read_text() if world_path.exists() else Path('src/vehicle_description/worlds/vinfast_map.world').read_text()
    old_contacts = '<max_contacts>10</max_contacts>'
    if old_contacts not in world_source:
        raise SystemExit('Expected ground contact marker was not found')
    world_path.write_text(world_source.replace(
        old_contacts, '<max_contacts>' + os.environ['ODE_GROUND_MAX_CONTACTS'] + '</max_contacts>', 1))
    launch_source = launch_source.replace(
        "'worlds', 'vinfast_map.world'", repr(str(world_path.resolve())))
if os.environ.get('ODE_VEHICLE_SPAWN_Z'):
    launch_source = launch_source.replace(
        "'-z', '0.0'],",
        "'-z', os.environ['ODE_VEHICLE_SPAWN_Z']],", 1)
launch_path = out / 'repro.launch.py'
launch_path.write_text(launch_source)
env = os.environ.copy()
env.update(ROS_DOMAIN_ID='93', GAZEBO_MASTER_URI='http://127.0.0.1:11393',
           ODE_OUT=str(out.resolve()))
arm = mode != 'vehicle'
vehicle = mode != 'arm'
model_observer_executable = observer_binary() if vehicle else None
command = ['ros2', 'launch', str(launch_path),
           'enable_arm:=' + str(arm).lower(),
           'enable_vehicle:=' + str(vehicle).lower(),
           'gui:=false', 'rviz:=false', 'gazebo_port:=11393']
start = time.monotonic()
with (out / 'gazebo.log').open('w') as launch_log:
    launch = subprocess.Popen(command, env=env, stdout=launch_log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    transport = None
    transport_log = None
    controllers_ready = False
    required_controllers = {'joint_state_broadcaster'}
    if vehicle:
        required_controllers.add('swerve_controller')
    if arm:
        required_controllers.add('joint_trajectory_controller')
    try:
        if vehicle:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                try:
                    with socket.create_connection(('127.0.0.1', 11393), timeout=0.2):
                        break
                except OSError:
                    time.sleep(0.05)
            else:
                raise RuntimeError('Gazebo master did not open port 11393')
            transport_log = (out / 'transport.log').open('w')
            transport = subprocess.Popen([str(model_observer_executable)], env=env,
                                         stdout=transport_log, stderr=subprocess.STDOUT)
        controllers_ready = wait_for_controllers(
            out / 'gazebo.log', required_controllers, launch)
        clients = []
        if not controllers_ready:
            print('Required controllers did not activate before the test client',
                  sorted(required_controllers), file=sys.stderr)
        elif mode == 'mounted':
            clients.append(('mounted', subprocess.Popen(
                [sys.executable, 'src/vehicle_bringup/test/integration/ode_mounted_coupled_client.py'], env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)))
        else:
            if arm:
                clients.append(('arm', subprocess.Popen(
                    ['python3', 'src/vehicle_bringup/test/integration/ode_repro_client.py'], env=env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)))
            if vehicle:
                clients.append(('vehicle', subprocess.Popen(
                    ['python3', 'src/vehicle_bringup/test/integration/ode_vehicle_client.py'], env=env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)))
        for name, client in clients:
            try:
                output, _ = client.communicate(timeout=65)
            except subprocess.TimeoutExpired:
                client.kill()
                output, _ = client.communicate()
            (out / (name + '_client.log')).write_text(output)
        time.sleep(15)
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGTERM)
            launch.wait(timeout=10)
        if transport is not None:
            transport.terminate()
            transport.wait(timeout=5)
        if transport_log is not None:
            transport_log.close()

log = (out / 'gazebo.log').read_text()
arm_result = json.loads((out / 'result.json').read_text()) if arm and (out / 'result.json').exists() else {}
vehicle_result = json.loads((out / 'vehicle_result.json').read_text()) if vehicle and (out / 'vehicle_result.json').exists() else {}
vehicle_pose = evaluate_pose_trace(out / 'transport_state.jsonl') if vehicle else None
positions = dict(zip(arm_result.get('final', {}).get('names', []),
                     arm_result.get('final', {}).get('p', [])))
angle_error = max((abs(positions.get(f'j{i}', float('inf')) - target)
                   for i, target in enumerate(arm_result.get('target', []), 1)),
                  default=0.0 if not arm else float('inf'))
summary = {
    'mode': mode, 'duration_s': round(time.monotonic() - start, 2),
    'controllers_ready': controllers_ready,
    'required_controllers': sorted(required_controllers),
    'launch_command': command, 'lcp_errors': log.count('LCP internal error'),
    'nonfinite_log_tokens': len(re.findall(r'(?i)\b(?:nan|[-+]?inf(?:inity)?)\b', log)),
    'gzserver_crashed': bool(re.search(r'\[ERROR\] \[gzserver-\d+\]: process has died', log)),
    'arm_action_status': arm_result.get('status') if arm else None,
    'arm_max_final_error_rad': angle_error if arm else None,
    'arm_finite': arm_result.get('finite') if arm else None,
    'mounted_rest_goal_completed_before_drive': arm_result.get('rest_goal_completed_before_drive') if mode == 'mounted' else None,
    'mounted_moving_goal_drive_overlap_s': arm_result.get('moving_goal_drive_overlap_s') if mode == 'mounted' else None,
    'vehicle': vehicle_result if vehicle else None,
    'warnings': [line for line in log.splitlines() if '[WARN]' in line or '[ERROR]' in line][:30],
    'vehicle_pose': vehicle_pose,
}
summary['pass'] = (
    controllers_ready and summary['lcp_errors'] == 0
    and summary['nonfinite_log_tokens'] == 0
    and not summary['gzserver_crashed']
    and (not arm or (arm_result.get('finite', False)
                     and arm_result.get('status') == 4
                     and arm_result.get('error_code') == 0
                     and angle_error < 0.01))
    and (mode != 'mounted' or (arm_result.get('rest_goal_completed_before_drive', False)
                              and arm_result.get('moving_goal_drive_overlap_s', 0.0) >= 2.0))
    and (not vehicle or (vehicle_result.get('pass', False)
                         and vehicle_result.get('finite', False)
                         and vehicle_pose.get('pass', False))))
(out / 'summary.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
if not summary['pass']:
    raise SystemExit(1)
