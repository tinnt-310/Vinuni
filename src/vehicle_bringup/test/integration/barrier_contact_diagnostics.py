"""Read-only alignment of Gazebo contact, command and LCP evidence.

Gazebo log timestamps are recorded on receipt by the runner. They are not
solver-call timestamps; the nearby physics sample cannot identify an exact
failing ODE constraint row.
"""

import json
import math
from pathlib import Path
import shlex
import subprocess


DEFAULT_TARGET = 'drc_practice_angled_barrier_45'


def build_probe(source: Path, output: Path) -> Path:
    """Build the existing diagnostic ModelPlugin outside the installed tree."""
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs', 'gazebo'], text=True))
    subprocess.run(['g++', '-std=c++17', '-O2', '-fPIC', '-shared',
                    str(source), '-o', str(output), *flags], check=True)
    return output


def inject_probe(launch_text: str, plugin: Path) -> str:
    """Make a generated launch copy that adds only a measurement plugin."""
    anchor = '    # Source Xacros retain package://.'
    if launch_text.count(anchor) != 1:
        raise ValueError('Expected one model injection point in current launch')
    snippet = (
        '    # Diagnostic-only contact trace; production source is unchanged.\n'
        '    from xml.dom.minidom import parseString\n'
        '    probe_xml = \'<gazebo><plugin name="ode_contact_probe" '
        'filename="' + str(plugin) + '"/></gazebo>\'\n'
        '    doc.documentElement.appendChild(parseString(probe_xml).documentElement)\n'
    )
    return launch_text.replace(anchor, snippet + anchor, 1)


def _nearest(rows, wall):
    if wall is None or not rows:
        return None
    return min(rows, key=lambda row: abs(row['wall'] - wall))


def _latest_before(rows, wall):
    if wall is None:
        return None
    return next((row for row in reversed(rows) if row['wall'] <= wall), None)


def _target_contacts(sample, target):
    return [contact for contact in sample.get('contacts', [])
            if target in contact.get('a', '') or target in contact.get('b', '')]


def _contact_measurement(sample, contacts):
    if sample is None:
        return None
    points = [point for contact in contacts for point in contact.get('points', [])]
    return {
        'wall_monotonic_s': sample['wall'],
        'gazebo_sim_s': sample.get('sim'),
        'collision_pairs': [{'a': contact['a'], 'b': contact['b']}
                            for contact in contacts],
        'points': points,
        'max_depth_m': max((point['depth'] for point in points), default=None),
        'max_body1_force_n': max((math.sqrt(sum(value * value for value in
            point['body1_force'])) for point in points), default=None),
        'chassis': next((model for model in sample.get('models', [])
                         if model.get('name') == 'my_mobile_manipulator'), None),
    }


def analyze_trace(physics_rows, event_rows, ros_trace, target=DEFAULT_TARGET):
    """Return evidence, not an acceptance verdict or a causal solver claim."""
    first_contact = next(((sample, _target_contacts(sample, target))
                          for sample in physics_rows
                          if _target_contacts(sample, target)), None)
    first_lcp = next((event for event in event_rows
                      if 'LCP internal error' in event.get('line', '')), None)
    contact_sample, contacts = first_contact if first_contact else (None, [])
    lcp_sample = _nearest(physics_rows, first_lcp['wall']) if first_lcp else None
    contact_wall = contact_sample['wall'] if contact_sample else None
    lcp_wall = first_lcp['wall'] if first_lcp else None
    commands = ros_trace.get('normalized_commands', [])
    published = ros_trace.get('published_commands', [])
    result = {
        'scope': 'Diagnostic only: log arrival and Gazebo update times are distinct; '
                 'the exact failing ODE row is not observable.',
        'target_collision_substring': target,
        'physics_samples': len(physics_rows),
        'event_samples': len(event_rows),
        'first_target_contact': _contact_measurement(contact_sample, contacts),
        'first_lcp_event': first_lcp,
        'nearest_physics_at_first_lcp': _contact_measurement(
            lcp_sample, lcp_sample.get('contacts', []) if lcp_sample else []),
        'first_lcp_nearest_physics_wall_offset_s': (
            lcp_sample['wall'] - lcp_wall if lcp_sample is not None
            and lcp_wall is not None else None),
        'contact_to_lcp_log_arrival_s': (
            lcp_wall - contact_wall if lcp_wall is not None
            and contact_wall is not None else None),
        'normalized_command_at_contact': _latest_before(commands, contact_wall),
        'published_command_at_contact': _latest_before(published, contact_wall),
        'normalized_command_at_lcp': _latest_before(commands, lcp_wall),
        'published_command_at_lcp': _latest_before(published, lcp_wall),
        'wheel_state_at_contact': _latest_before(ros_trace.get('wheel_states', []), contact_wall),
        'wheel_state_at_lcp': _latest_before(ros_trace.get('wheel_states', []), lcp_wall),
        'odom_at_contact': _latest_before(ros_trace.get('odom', []), contact_wall),
    }
    if not physics_rows or not event_rows:
        result['measurement_status'] = 'INVALID: missing physics or timestamped runtime events'
    elif first_contact is None:
        result['measurement_status'] = 'NO_TARGET_CONTACT_OBSERVED'
    elif first_lcp is None:
        result['measurement_status'] = 'TARGET_CONTACT_WITHOUT_LCP'
    elif lcp_wall < contact_wall:
        result['measurement_status'] = 'LCP_PRECEDES_TARGET_CONTACT'
    else:
        result['measurement_status'] = 'TARGET_CONTACT_AND_LCP_OBSERVED'
    result['command_evidence_complete'] = bool(
        result['published_command_at_contact'] and
        result['normalized_command_at_contact']) if contact_wall is not None else None
    result['chassis_evidence_complete'] = bool(
        result['first_target_contact'] and
        result['first_target_contact']['chassis']) if contact_wall is not None else None
    return result


def analyze_directory(directory: Path, target=DEFAULT_TARGET):
    directory = Path(directory)
    def jsonl(path):
        if not path.exists():
            return []
        with path.open() as stream:
            return [json.loads(line) for line in stream if line.strip()]
    events = jsonl(directory / 'runtime_events.jsonl')
    trace_path = directory / 'ros_trace.json'
    trace = json.loads(trace_path.read_text()) if trace_path.exists() else {}
    first_lcp = next((event for event in events
                      if 'LCP internal error' in event.get('line', '')), None)
    first_contact = None
    nearest_lcp = None
    count = 0
    physics_path = directory / 'physics.jsonl'
    if physics_path.exists():
        with physics_path.open() as stream:
            for line in stream:
                if not line.strip():
                    continue
                sample = json.loads(line)
                count += 1
                if first_contact is None and _target_contacts(sample, target):
                    first_contact = sample
                if first_lcp and (nearest_lcp is None or
                        abs(sample['wall'] - first_lcp['wall']) <
                        abs(nearest_lcp['wall'] - first_lcp['wall'])):
                    nearest_lcp = sample
    selected = sorted((row for row in (first_contact, nearest_lcp)
                       if row is not None), key=lambda row: row['wall'])
    result = analyze_trace(selected, events, trace, target)
    result['physics_samples'] = count
    if not count or not events:
        result['measurement_status'] = (
            'INVALID: missing physics or timestamped runtime events')
    (directory / 'barrier_contact_analysis.json').write_text(
        json.dumps(result, indent=2, sort_keys=True) + '\n')
    # These fixed-width evidence slices are for inspection, not acceptance limits.
    for name, wall in [('first_contact', result['first_target_contact']),
                       ('first_lcp', result['first_lcp_event'])]:
        stamp = (wall.get('wall_monotonic_s') if name == 'first_contact'
                 else wall.get('wall')) if wall else None
        if stamp is not None and physics_path.exists():
            with (directory / f'barrier_{name}_physics_window.jsonl').open('w') as stream, \
                 physics_path.open() as physics_stream:
                for line in physics_stream:
                    if not line.strip():
                        continue
                    sample = json.loads(line)
                    if abs(sample['wall'] - stamp) <= 0.25:
                        stream.write(line)
    return result
