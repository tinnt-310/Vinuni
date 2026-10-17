#!/usr/bin/python3
"""Run isolated matched-CAD pose-error trials and export force/alignment plots."""
import argparse
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
CASES = {
    'aligned': ((0., 0.), (0., 0., 0.)),
    'translation_x_minus2': ((-2., 0.), (0., 0., 0.)),
    'tilt_ry_minus2': ((0., 0.), (0., -2., 0.)),
    'combined_x_minus2_ry_minus2': ((-2., 0.), (0., -2., 0.)),
    'yaw_rz_plus2': ((0., 0.), (0., 0., 2.)),
    'translation_x_minus5': ((-5., 0.), (0., 0., 0.)),
    'tilt_ry_minus5': ((0., 0.), (0., -5., 0.)),
    'combined_x_minus5_ry_minus5': ((-5., 0.), (0., -5., 0.)),
    'yaw_rz_plus5': ((0., 0.), (0., 0., 5.)),
}
for level, label in ((.5, '0p5'), (1., '1'), (2., '2'), (3., '3'), (5., '5')):
    CASES['combined_diagonal_' + label] = ((.6 * level, -.8 * level), (.6 * level, -.8 * level, 0.))
CASES = {name: ((tuple(position) + (0.,)) if len(position) == 2 else tuple(position), rotation)
         for name, (position, rotation) in CASES.items()}
CASES.update({
    'translation_z_minus5': ((0., 0., -5.), (0., 0., 0.)),
    'translation_z_plus5': ((0., 0., 5.), (0., 0., 0.)),
    'combined_spatial_5': ((3., 0., 4.), (3., -4., 0.)),
})
AXIS_CASES = {1: [], 5: []}
for level in (1, 5):
    for dof, label in enumerate(('x', 'y', 'z', 'rx', 'ry', 'rz')):
        for sign, word in ((-1, 'minus'), (1, 'plus')):
            prefix = 'translation' if dof < 3 else 'yaw' if dof == 5 else 'tilt'
            name = f'{prefix}_{label}_{word}{level}'
            error = [0.] * 6
            error[dof] = float(sign * level)
            CASES[name] = (tuple(error[:3]), tuple(error[3:]))
            AXIS_CASES[level].append(name)
DEFAULT_CASES = ['aligned', 'combined_diagonal_0p5', 'combined_diagonal_1', 'combined_diagonal_2',
                 'combined_diagonal_3', 'translation_x_minus5', 'tilt_ry_minus5', 'combined_diagonal_5',
                 'translation_z_minus5', 'translation_z_plus5', 'combined_spatial_5']
CASE_GROUPS = {
    'smoke': ['aligned', 'combined_diagonal_0p5'],
    'axes': AXIS_CASES[1],
    'combined': ['combined_diagonal_' + label for label in ('0p5', '1', '2', '3', '5')],
    'boundary': AXIS_CASES[5] + ['combined_diagonal_5', 'combined_spatial_5'],
}


def trial_profile(mode, extended, overlay=None):
    config = ROOT / 'src/fr10_arm_control/fr10_pose_controller/config'
    default = ('mujoco_passive_5mm_5deg.yaml' if mode == 'passive' else
               'mujoco_pose_uncertainty_5mm_5deg.yaml' if extended else 'mujoco_pose_uncertainty.yaml')
    path = Path(overlay).resolve() if overlay else config / default
    spec = importlib.util.spec_from_file_location('capture_profile', config.parent / 'launch/controller_profile.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    parameters = module.load_controller_profile(path)
    spiral = parameters.get('force.search.spiral_enabled', True)
    if mode == 'passive' and (spiral is not False or
            parameters.get('force.search.xy_probe_force_n', 0.) != 0. or
            parameters.get('force.search.yaw_probe_torque_nm', 0.) != 0.):
        raise ValueError('Passive profile must disable spiral, XY probe force and yaw probe torque')
    if mode == 'search' and spiral is not True:
        raise ValueError('Search profile must enable force.search.spiral_enabled')
    return path, parameters


def load(path):
    return json.loads(path.read_text())


def runtime_conflicts(node):
    """Observers may share a domain; an existing plant/controller may not."""
    conflicts = []
    for topic in ('/clock', '/joint_states', '/controller_status'):
        for endpoint in node.get_publishers_info_by_topic(topic):
            conflicts.append(f'{topic} published by {endpoint.node_name}')
    for name, types in node.get_service_names_and_types():
        if name == '/get_planning_scene':
            conflicts.append(name)
    return sorted(set(conflicts))


def require_unused_domain(domain):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('fr10_mujoco_preflight', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        # Discovery is asynchronous; one immediate graph query can miss an
        # existing plant and mix its clock/joints into the new experiment.
        deadline = time.monotonic() + 2.
        conflicts = set()
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
            conflicts.update(runtime_conflicts(node))
        if conflicts:
            raise ValueError(f'ROS domain {domain} already has an active runtime: '
                             + '; '.join(sorted(conflicts))
                             + '. Choose an unused --domain (and use it for viewer/plots). '
                             'No simulation or motion was started.')
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def phase_motion(physical, feedback, states):
    selected = [s for s in feedback if s['state'] in states]
    if len(selected) < 2:
        return None
    rows = [s for s in physical if selected[0]['wall'] <= s['wall'] <= selected[-1]['wall']]
    if len(rows) < 2:
        return None
    wall = np.asarray([s['wall'] for s in rows])
    source = np.asarray([s['source'] for s in rows])
    xyz = np.asarray([s['position'] for s in rows])
    elapsed = wall[-1] - wall[0]
    if elapsed <= 0:
        return None
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))]
    earlier = np.searchsorted(wall, wall - 2.)
    duration = wall - wall[earlier]
    valid = duration >= 1.5
    speeds = 1000. * (distance[valid] - distance[earlier[valid]]) / duration[valid]
    return {'duration_wall_s': float(elapsed),
            'path_mm': float(1000. * distance[-1]),
            'mean_path_speed_wall_mm_s': float(1000. * distance[-1] / elapsed),
            'p95_2s_path_speed_wall_mm_s': float(np.percentile(speeds, 95)) if len(speeds) else None,
            'rtf': float((source[-1] - source[0]) / elapsed)}


def matrix_outcomes(records):
    # A timed-out attempt is still a failed trial. Excluding it because the
    # action never returned would inflate the observed capture success rate.
    completed = [r for r in records if r.get('run_summary_received')]
    passed = sum(r['pass'] is True for r in completed)
    return {'all_pass': bool(records) and all(r['pass'] is True for r in records),
            'requested_cases': len(records), 'completed_cases': len(completed),
            'action_results_received': sum(r.get('insert_result') is not None for r in completed),
            'passed_cases': passed,
            'observed_success_fraction': passed / len(completed) if completed else None}


def report(output, cases):
    os.environ.setdefault('MPLCONFIGDIR', str(output / 'plot_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
    wrench_fig, wrench_axes = plt.subplots(6, 1, figsize=(12, 14), sharex=True)
    records = []
    for name in cases:
        directory = output / name
        if not (directory / 'summary.json').is_file():
            records.append({'case': name, 'pass': False, 'run_summary_received': False,
                            'error': 'No complete run summary'})
            continue
        summary = load(directory / 'summary.json')
        feedback = load(directory / 'insert_feedback.json')
        physical = load(directory / 'physical_tcp.json')
        record = {key: summary.get(key) for key in (
            'pass', 'error', 'insert_result', 'force_self_alignment_gate',
            'physical_seating_gate', 'physical_depth_range_m',
            'physical_max_lateral_error_m', 'physical_max_angular_error_rad',
            'physical_max_mating_feature_error_m', 'station_peak_penetration_m',
            'unexpected_station_contacts', 'physical_retract_gate', 'retract_pass',
            'shutdown_hold_confirmed', 'motion_shutdown_before_physics', 'pose_uncertainty')}
        record.update(case=name, run_summary_received=True, summary_file=str(directory / 'summary.json'))
        import yaml
        parameters = yaml.safe_load((directory / 'mujoco_overlay.yaml').read_text())[
            'fr10_pose_controller']['ros__parameters']
        record['profile_parameters'] = {key: parameters.get(key) for key in (
            'controller.pose_policies.free_space.linear_scale', 'force.goal.approach_distance_m',
            'force.goal.insertion_speed_m_s', 'force.max_nominal_lead_m', 'force.filter_cutoff_hz',
            'force.goal.selected_axes', 'force.admittance.mass', 'force.admittance.stiffness',
            'force.admittance.max_correction', 'force.admittance.drive_deadband',
            'force.admittance.damping', 'force.admittance.max_velocity', 'force.search.spiral_enabled',
            'force.search.yaw_probe_torque_nm', 'force.search.xy_probe_force_n')}
        raw = load(directory / 'raw_wrench.json')
        record['peak_raw_force_norm_n'] = max((float(np.linalg.norm(s['wrench'][:3])) for s in raw), default=None)
        record['peak_raw_torque_norm_nm'] = max((float(np.linalg.norm(s['wrench'][3:])) for s in raw), default=None)
        record['free_motion'] = phase_motion(physical, feedback, {1})
        record['approach_motion'] = phase_motion(physical, feedback, {16})
        if feedback:
            start = feedback[0]['wall']
            record['phase_start_wall_s'] = {}
            for row in feedback:
                record['phase_start_wall_s'].setdefault(row['detail'].split(';')[0], row['wall'] - start)
            if 'result_received_wall' in summary:
                record['time_to_result_wall_s'] = summary['result_received_wall'] - start
            held = [s for s in feedback if s['state'] == 9 and
                    summary.get('hold_observation_start_wall', float('inf')) <= s['wall'] <=
                    summary.get('hold_evidence_end_wall', -float('inf'))]
            record['hold_fz_range_n'] = [min(s['wrench'][2] for s in held),
                                         max(s['wrench'][2] for s in held)] if held else None
            # Keep the chart bounded while retaining every sample on disk.
            sampled = feedback[::max(1, len(feedback) // 10000)]
            line, = axes[0].plot([s['wall'] - start for s in sampled],
                                [s['wrench'][2] for s in sampled], label=name, linewidth=.8)
            color = line.get_color()
            for component, axis in enumerate(wrench_axes):
                axis.plot([s['wall'] - start for s in sampled],
                          [s['wrench'][component] for s in sampled],
                          color=color, label=name, linewidth=.8)
            if physical:
                station = load(directory / 'selected_station.json')
                turn = Rotation.from_euler('xyz', station['world_mouth_rpy'])
                approach_start = next((s['wall'] for s in feedback if s['state'] == 16), float('inf'))
                insertion = [s for s in physical if s['wall'] >= approach_start]
                sampled = insertion[::max(1, len(insertion) // 10000)]
                if not sampled:
                    records.append(record)
                    continue
                wall = np.asarray([s['wall'] for s in sampled])
                source = np.asarray([s['source'] for s in sampled])
                local = turn.inv().apply(np.asarray([s['position'] for s in sampled]) -
                                         station['world_mouth_position_m'])
                angle = (turn.inv() * Rotation.from_quat(
                    [s['orientation_xyzw'] for s in sampled])).magnitude()
                axes[1].plot(wall - start, local[:, 2] * 1000., color=color, linewidth=.8)
                axes[2].plot(wall - start, np.linalg.norm(local[:, :2], axis=1) * 1000., color=color, linewidth=.8)
                axes[3].plot(wall - start, np.rad2deg(angle), color=color, linewidth=.8)
                old = np.searchsorted(wall, wall - 2.)
                valid = wall - wall[old] >= 1.5
                axes[4].plot(wall[valid] - start,
                    (source[valid] - source[old[valid]]) / (wall[valid] - wall[old[valid]]),
                    color=color, linewidth=.8)
        records.append(record)
    axes[0].axhline(5., color='black', linestyle='--', linewidth=.7, label='Seating reference 5 N')
    axes[1].axhline(15., color='black', linestyle='--', linewidth=.7)
    for axis, label in zip(axes, ('Fz [N]', 'True depth [mm]', 'True lateral error [mm]',
                                   'True angle error [deg]', 'Sim / wall time')):
        axis.set_ylabel(label)
        axis.grid(alpha=.25)
    axes[0].legend(fontsize=8, loc='best')
    axes[-1].set_xlabel('Wall time since single pose input [s]')
    fig.suptitle('MuJoCo matched CAD: measured force and independent physical alignment')
    fig.tight_layout()
    fig.savefig(output / 'force_alignment.png', dpi=150)
    plt.close(fig)
    for axis, label in zip(wrench_axes, ('Fx [N]', 'Fy [N]', 'Fz [N]',
                                        'Tx [Nm]', 'Ty [Nm]', 'Tz [Nm]')):
        axis.set_ylabel(label)
        axis.grid(alpha=.25)
    wrench_axes[0].legend(fontsize=8, loc='best')
    wrench_axes[-1].set_xlabel('Wall time since single pose input [s]')
    wrench_fig.suptitle('Conditioned six-axis contact wrench in the fixed insertion frame')
    wrench_fig.tight_layout()
    wrench_fig.savefig(output / 'wrench_components.png', dpi=150)
    plt.close(wrench_fig)
    document = {'cases': records, **matrix_outcomes(records),
                'units': 'Pose error in true socket frame: mm and rotation-vector degrees',
                'coverage': 'Observed trials only; no guarantee over the continuous 5 mm / 5 degree region',
                'acceptance': 'Original CAD; <=50 um penetration; independent keyed fit, HOLD and explicit withdrawal'}
    (output / 'matrix_summary.json').write_text(json.dumps(document, indent=2) + '\n')
    for row in records:
        print(f"{row['case']}: {'PASS' if row['pass'] else 'FAIL'}; "
              f"result after {row.get('time_to_result_wall_s', '?')} wall seconds", flush=True)
    print(f"Plot: {output / 'force_alignment.png'}", flush=True)
    print(f"Six-axis wrench: {output / 'wrench_components.png'}", flush=True)
    return document['all_pass']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'test_results' /
                        ('fr10_capture_mujoco_' + datetime.now().strftime('%Y%m%dT%H%M%S')))
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--cases', nargs='+', choices=CASES)
    selection.add_argument('--case-group', choices=CASE_GROUPS)
    parser.add_argument('--mode', choices=('passive', 'search'), default='passive')
    parser.add_argument('--overlay', type=Path, help='Use this controller YAML instead of the default profile')
    parser.add_argument('--domain', type=int, default=212)
    parser.add_argument('--port', type=int, default=11996)
    parser.add_argument('--timeout', type=float, default=1800.)
    parser.add_argument('--report-only', action='store_true')
    parser.add_argument('--dry-run', action='store_true', help='Save resolved profile and cases without starting ROS or physics')
    args = parser.parse_args()
    args.cases = args.cases or CASE_GROUPS.get(args.case_group, DEFAULT_CASES)
    output = args.output.resolve()
    if len(set(args.cases)) != len(args.cases):
        parser.error('Choose each case once')
    if not 0 <= args.domain <= 232 or not 1024 <= args.port <= 65535 or not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Invalid domain, port or timeout')
    if args.report_only and args.dry_run:
        parser.error('--report-only and --dry-run cannot be combined')
    if not args.report_only:
        if any((output / name).exists() for name in args.cases):
            parser.error('Case output already exists; choose a fresh output or --report-only')
        extended = any(max(np.linalg.norm(CASES[name][0]), np.linalg.norm(CASES[name][1])) > 2.
                       for name in args.cases)
        try:
            profile, parameters = trial_profile(args.mode, extended, args.overlay)
        except (OSError, ValueError) as error:
            parser.error(str(error))
        output.mkdir(parents=True, exist_ok=True)
        plan = {'backend': 'mujoco', 'mode': args.mode, 'ros_domain': args.domain,
                'profile': str(profile), 'resolved_profile_parameters': parameters,
                'cases': [{'case': name, 'translation_mm': CASES[name][0],
                           'rotation_vector_deg': CASES[name][1]} for name in args.cases]}
        (output / 'trial_plan.json').write_text(json.dumps(plan, indent=2) + '\n')
        print(f'Profile: {profile}; plan: {output / "trial_plan.json"}', flush=True)
        if args.dry_run:
            print('Plan only: no ROS or physics started.', flush=True)
            return 0
        # Fail before starting a case or producing empty failure plots when a
        # new terminal has only sourced /opt/ros rather than this workspace.
        try:
            from ament_index_python.packages import get_packages_with_prefixes
            from fr10_matched_run_manifest import runtime_binary_inventory
            runtime_binary_inventory(get_packages_with_prefixes())
        except (ImportError, ValueError) as error:
            parser.error(str(error) + '\nSource /opt/ros/humble/setup.bash and install/setup.bash before running.')
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        try:
            require_unused_domain(args.domain)
        except ValueError as error:
            parser.error(str(error))
        for name in args.cases:
            position, angle = CASES[name]
            command = [sys.executable, str(Path(__file__).with_name('run_matched_station_gazebo.py')),
                str(output / name), '--backend', 'mujoco', '--domain', str(args.domain),
                '--port', str(args.port), '--station', 'front_mid', '--initial-state', 'home',
                '--insert', '--pose-input', '--retract-after-hold', '--run-timeout', str(args.timeout),
                '--overlay', str(profile),
                '--pose-error-limit', '5' if extended else '2',
                '--pose-lateral-mm', *map(str, position[:2]), '--pose-axial-mm', str(position[2]),
                '--pose-rotation-deg', *map(str, angle)]
            print(f'Running {name}; ROS domain {args.domain}; evidence {output / name}', flush=True)
            print(f'3D viewer (another terminal, after selected_station.json appears):\n'
                  f'ROS_DOMAIN_ID={args.domain} ROS_LOCALHOST_ONLY=1 /usr/bin/python3 '
                  'src/vehicle_bringup/test/integration/view_fr10_force_mujoco.py '
                  f'--run-directory {output / name} --live --close-up', flush=True)
            with (output / (name + '.console.log')).open('w') as log:
                result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            summary = output / name / 'summary.json'
            if not summary.is_file():
                print(f'No summary; stopped after exit {result.returncode}. See {log.name}', flush=True)
                break
            evidence = load(summary)
            if not (evidence.get('shutdown_hold_confirmed') and evidence.get('motion_shutdown_before_physics')):
                print('Run did not confirm its stop/shutdown; no next case launched.', flush=True)
                break
    return 0 if report(output, args.cases) else 1


if __name__ == '__main__':
    raise SystemExit(main())
