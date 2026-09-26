"""Run arm, vehicle, or mounted Gazebo regressions in an isolated ROS domain.

Source ROS Humble and install_grouped/setup.bash first. Never connects to a
FAIRINO cabinet. Usage: python3 tools/run_ode_scenarios.py LABEL MODE
where MODE is arm, vehicle, or mounted.
"""
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

label, mode = sys.argv[1:3]
if mode not in ('arm', 'vehicle', 'mounted'):
    raise SystemExit('MODE must be arm, vehicle, or mounted')
out = Path('/home/tin/Vinuni/docs/ode_debug') / label
out.mkdir(parents=True, exist_ok=False)
launch_source = Path('src/vehicle_bringup/launch/simulation.launch.py').read_text()
launch_source = launch_source.replace(
    '    # Source Xacros retain package://.',
    "    from pathlib import Path\n"
    "    Path(os.environ['ODE_OUT']+'/model.urdf').write_text(doc.toxml())\n"
    '    # Source Xacros retain package://.')
launch_path = out / 'repro.launch.py'
launch_path.write_text(launch_source)
env = os.environ.copy()
env.update(ROS_DOMAIN_ID='93', GAZEBO_MASTER_URI='http://127.0.0.1:11393',
           ODE_OUT=str(out.resolve()))
arm = mode != 'vehicle'
vehicle = mode != 'arm'
command = ['ros2', 'launch', str(launch_path),
           'enable_arm:=' + str(arm).lower(),
           'enable_vehicle:=' + str(vehicle).lower(),
           'gui:=false', 'rviz:=false', 'gazebo_port:=11393']
start = time.monotonic()
with (out / 'gazebo.log').open('w') as launch_log:
    launch = subprocess.Popen(command, env=env, stdout=launch_log,
                              stderr=subprocess.STDOUT, start_new_session=True)
    try:
        clients = []
        if arm:
            clients.append(('arm', subprocess.Popen(
                ['python3', 'tools/ode_repro_client.py'], env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)))
        if vehicle:
            clients.append(('vehicle', subprocess.Popen(
                ['python3', 'tools/ode_vehicle_client.py'], env=env,
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

log = (out / 'gazebo.log').read_text()
arm_result = json.loads((out / 'result.json').read_text()) if arm and (out / 'result.json').exists() else {}
vehicle_result = json.loads((out / 'vehicle_result.json').read_text()) if vehicle and (out / 'vehicle_result.json').exists() else {}
positions = dict(zip(arm_result.get('final', {}).get('names', []),
                     arm_result.get('final', {}).get('p', [])))
angle_error = max((abs(positions.get(f'j{i}', float('inf')) - target)
                   for i, target in enumerate(arm_result.get('target', []), 1)),
                  default=0.0 if not arm else float('inf'))
summary = {
    'mode': mode, 'duration_s': round(time.monotonic() - start, 2),
    'launch_command': command, 'lcp_errors': log.count('LCP internal error'),
    'nonfinite_log_tokens': len(re.findall(r'(?i)\b(?:nan|[-+]?inf(?:inity)?)\b', log)),
    'arm_action_status': arm_result.get('status') if arm else None,
    'arm_max_final_error_rad': angle_error if arm else None,
    'arm_finite': arm_result.get('finite') if arm else None,
    'vehicle': vehicle_result if vehicle else None,
    'warnings': [line for line in log.splitlines() if '[WARN]' in line or '[ERROR]' in line][:30],
}
summary['pass'] = (
    summary['lcp_errors'] == 0 and summary['nonfinite_log_tokens'] == 0
    and (not arm or (arm_result.get('finite', False)
                     and arm_result.get('status') == 4
                     and arm_result.get('error_code') == 0
                     and angle_error < 0.01))
    and (not vehicle or (vehicle_result.get('pass', False)
                         and vehicle_result.get('finite', False))))
(out / 'summary.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
if not summary['pass']:
    raise SystemExit(1)
