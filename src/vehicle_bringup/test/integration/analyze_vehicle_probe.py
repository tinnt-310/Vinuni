"""Summarize actual Gazebo motion and ROS coverage; never infer it from odometry alone."""
import argparse
import json
import math
from pathlib import Path

from ode_scenario_readiness import required_controllers_for_case


def finite(value):
    if isinstance(value, dict):
        return all(finite(v) for v in value.values())
    if isinstance(value, list):
        return all(finite(v) for v in value)
    return not isinstance(value, float) or math.isfinite(value)


def analyze(directory):
    directory = Path(directory)
    manifest = json.loads((directory / 'manifest.json').read_text())
    summary = json.loads((directory / 'summary.json').read_text())
    samples = []
    all_finite = True
    trace = directory / ('transport_state.jsonl' if manifest.get('no_probe') else 'physics.jsonl')
    if not trace.exists():
        return {'pass': False, 'reason': 'No Gazebo model state trace'}
    for line in trace.open():
        data = json.loads(line)
        all_finite &= finite(data)
        for model in data['models']:
            if model['name'] == 'my_mobile_manipulator':
                samples.append((data['wall'], model))
    topics = {}
    controllers = []
    commands = []
    ros_finite = True
    for line in (directory / 'ros_state.jsonl').open():
        event = json.loads(line)
        topic = event['topic']
        topics[topic] = topics.get(topic, 0) + 1
        data = event['data']
        # Effort is not exported for vehicle joints; do not treat unregistered
        # effort NaNs as measured numeric failures. Retain them in the raw log.
        if topic == '/joint_states':
            ros_finite &= finite(data['position']) and finite(data['velocity'])
        else:
            ros_finite &= finite(data)
        if topic == 'controller_states':
            controllers = data['controller']
        if topic == '/cmd_vel' and any(data[k][a] != 0 for k in ('linear', 'angular') for a in ('x', 'y', 'z')):
            commands.append(event['wall'])
    if not samples:
        return {'pass': False, 'reason': 'No actual model state samples'}
    start = commands[0] if commands else samples[0][0] + 5
    settled = [(t, m) for t, m in samples if t >= start]
    if not settled:
        return {'pass': False, 'reason': 'No post-start model samples'}
    first, last = settled[0][1], settled[-1][1]
    delta = [b-a for a, b in zip(first['position'], last['position'])]
    yaw = last['rpy'][2] - first['rpy'][2]
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    tilt = max(abs(m['rpy'][i]) for _, m in settled for i in (0, 1))
    tail = [m for t, m in samples if t > samples[-1][0] - 2]
    final_speed = max(math.sqrt(sum(v*v for v in m['linear'])) for m in tail)
    active = {c['name'] for c in controllers if c['state'] == 'active'}
    case = manifest['case']
    expected = required_controllers_for_case(case)
    dynamic = case in ('V5', 'V6', 'V7', 'V8')
    moved = abs(yaw) > .1 if case == 'V7' else math.hypot(*delta[:2]) > .1
    result = dict(case=case, lcp_count=summary['lcp_count'], samples=len(samples),
                  trace_source=trace.name,
                  duration_s=samples[-1][0]-samples[0][0], physics_finite=all_finite,
                  ros_finite=ros_finite, topics=topics, active_controllers=sorted(active),
                  actual_displacement_m=delta, actual_yaw_change_rad=yaw,
                  max_settled_tilt_rad=tilt, final_max_speed_m_s=final_speed,
                  final_pose=last['position'], final_rpy=last['rpy'])
    result['pass'] = bool(not summary.get('gzserver_crashed', False)
                          and summary.get('controllers_ready', True)
                          and summary['lcp_count'] == 0 and all_finite and ros_finite
                          and expected == active and result['duration_s'] >= 20
                          and tilt < .05 and final_speed < .03
                          and summary['launch_returncode'] == 0
                          and summary['observer_returncode'] == 0
                          and (not dynamic or (commands and moved and topics.get('/odom', 0) > 0)))
    result['scope'] = 'Diagnostic gate; motion accuracy and full acceptance matrix still require review.'
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    args = parser.parse_args()
    result = analyze(args.directory)
    (Path(args.directory) / 'measured_result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
