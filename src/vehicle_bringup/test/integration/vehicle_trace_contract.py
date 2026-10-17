"""Evidence-validity checks for measured swerve motion and settling traces."""

import math


def evaluate_vehicle_ros_trace(trace, drive_start, drive_stop):
    if not (math.isfinite(drive_start) and math.isfinite(drive_stop)
            and drive_stop > drive_start):
        return {'valid': False, 'reason': 'missing drive window'}
    reasons = []
    fields = {
        'wheel_states': ('speed', 'steer'),
        'odom': ('x', 'y', 'vx', 'vy'),
        'tf': ('x', 'y'),
        'normalized_commands': ('vx', 'vy', 'omega'),
    }
    for name, values in fields.items():
        rows = trace.get(name, [])
        if not rows:
            reasons.append(name + ': no samples')
            continue
        if not any(drive_start <= row['wall'] <= drive_stop for row in rows):
            reasons.append(name + ': no motion-window sample')
        if name != 'normalized_commands' and not any(
                row['wall'] > drive_stop for row in rows):
            reasons.append(name + ': no settling-window sample')
        # A zero stop may intentionally be forwarded at the last /clock
        # stamp after the drive window. The controller rejects it closed.
        stamped_rows = ([row for row in rows
                         if drive_start <= row['wall'] <= drive_stop]
                        if name == 'normalized_commands' else rows)
        if not all(math.isfinite(row['source']) and row['source'] > 0
                   for row in stamped_rows) or not all(
                b['source'] > a['source']
                for a, b in zip(stamped_rows, stamped_rows[1:])):
            reasons.append(name + ': stale/replayed source stamp')
        if name == 'normalized_commands' and not all(
                rows[-1].get(field) == 0.0 for field in ('vx', 'vy', 'omega')):
            reasons.append('normalized_commands: final stop not observed')
        for row in rows:
            for field in values:
                raw = row.get(field)
                entries = raw if isinstance(raw, list) else [raw]
                if any(value is None or not math.isfinite(value)
                       for value in entries):
                    reasons.append(name + ': nonfinite or missing ' + field)
                    break
    return {'valid': not reasons, 'reasons': sorted(set(reasons)),
            'samples': {name: len(trace.get(name, [])) for name in fields}}
