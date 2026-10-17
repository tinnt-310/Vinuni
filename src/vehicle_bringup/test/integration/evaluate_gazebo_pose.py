"""Evaluate independently observed Gazebo chassis poses for regressions."""

import json
import math
from pathlib import Path


def evaluate_pose_trace(path: Path) -> dict:
    if not path.is_file():
        return {'pass': False, 'reason': 'Gazebo transport pose trace is missing'}
    samples = []
    try:
        for line in path.open():
            frame = json.loads(line)
            for model in frame.get('models', []):
                if model.get('name') == 'my_mobile_manipulator':
                    samples.append((frame['wall'], frame['sim'], model))
    except (OSError, ValueError, KeyError) as error:
        return {'pass': False, 'reason': f'Invalid Gazebo pose trace: {error}'}
    if len(samples) < 100:
        return {'pass': False, 'reason': 'Too few actual chassis pose samples',
                'samples': len(samples)}

    first, last = samples[0][2], samples[-1][2]
    delta = [b - a for a, b in zip(first['position'], last['position'])]
    wall_duration = samples[-1][0] - samples[0][0]
    sim_duration = samples[-1][1] - samples[0][1]
    finite = all(math.isfinite(value) for _, sim, model in samples
                 for value in [sim, *model['position'], *model['rpy'],
                               *model['linear'], *model['angular']])
    max_tilt = max(abs(model['rpy'][axis]) for _, _, model in samples
                   for axis in (0, 1))
    final_speed = max(math.sqrt(sum(value * value for value in model['linear']))
                      for wall, _, model in samples if wall >= samples[-1][0] - 1.0)
    result = {
        'samples': len(samples), 'wall_duration_s': wall_duration,
        'sim_duration_s': sim_duration, 'finite': finite,
        'actual_displacement_m': delta,
        'actual_planar_distance_m': math.hypot(*delta[:2]),
        'max_tilt_rad': max_tilt, 'final_max_speed_m_s': final_speed,
        'final_pose': last['position'], 'final_rpy': last['rpy'],
    }
    result['pass'] = (finite and wall_duration >= 20.0 and sim_duration >= 20.0
                      and result['actual_planar_distance_m'] >= 0.5
                      and max_tilt < 0.05 and final_speed < 0.03)
    return result
