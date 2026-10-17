"""Recorded speed uses physical path length and both measured clocks."""
import json
from pathlib import Path

import pytest
import yaml

import run_fr10_capture_mujoco as runner
from run_fr10_capture_mujoco import CASES, matrix_outcomes, phase_motion
from fr10_pose_uncertainty import input_socket_pose
from view_fr10_force_mujoco import load_run_frames


def test_round_trip_motion_does_not_disappear_as_zero_net_displacement():
    rows = [{'wall': t, 'source': t / 10., 'position': [x, 0., 0.]}
            for t, x in ((0., 0.), (1., .001), (2., .002), (3., .001), (4., 0.))]
    feedback = [{'wall': 0., 'state': 16}, {'wall': 4., 'state': 16}]
    result = phase_motion(rows, feedback, {16})
    assert result['path_mm'] == pytest.approx(4.)
    assert result['mean_path_speed_wall_mm_s'] == pytest.approx(1.)
    assert result['p95_2s_path_speed_wall_mm_s'] == pytest.approx(1.)
    assert result['rtf'] == pytest.approx(.1)
    assert phase_motion(rows, feedback, {1}) is None


def test_all_capture_presets_respect_existing_input_error_bounds():
    for position, rotation in CASES.values():
        input_socket_pose([0., 0., 0.], [0., 0., 0., 1.], position[:2], rotation,
                          axial_mm=position[2], max_position_mm=5., max_rotation_deg=5.)


def test_single_axis_groups_cover_both_signs_of_all_six_dofs():
    for level, group in ((1, runner.CASE_GROUPS['axes']), (5, runner.AXIS_CASES[5])):
        observed = set()
        for name in group:
            position, angle = CASES[name]
            vector = position + angle
            nonzero = [(i, value) for i, value in enumerate(vector) if value]
            assert len(nonzero) == 1
            observed.add(nonzero[0])
        assert observed == {(i, float(sign * level)) for i in range(6) for sign in (-1, 1)}


def test_custom_gains_survive_profile_resolution_and_dry_run_never_starts_ros(tmp_path, monkeypatch):
    profile = tmp_path / 'new_gains.yaml'
    gains = [2., 3., 4., .2, .3, .4]
    profile.write_text(yaml.safe_dump({'extends': str(runner.ROOT /
        'src/fr10_arm_control/fr10_pose_controller/config/mujoco_passive_5mm_5deg.yaml'),
        'fr10_pose_controller': {'ros__parameters': {'force.admittance.mass': gains}}}))
    output = tmp_path / 'plan'
    monkeypatch.setattr(runner.sys, 'argv', ['runner', '--dry-run', '--case-group', 'axes',
        '--overlay', str(profile), '--output', str(output)])
    def forbidden(*args, **kwargs):
        raise AssertionError('Dry-run attempted to launch a subprocess')
    monkeypatch.setattr(runner.subprocess, 'run', forbidden)
    assert runner.main() == 0
    plan = json.loads((output / 'trial_plan.json').read_text())
    assert plan['resolved_profile_parameters']['force.admittance.mass'] == gains
    assert len(plan['cases']) == 12
    assert not any(p.is_dir() for p in output.iterdir())


def test_profiles_cannot_silently_enable_search_in_passive_trials(tmp_path):
    profile = tmp_path / 'wrong_mode.yaml'
    profile.write_text(yaml.safe_dump({'fr10_pose_controller': {'ros__parameters': {
        'force.search.spiral_enabled': True}}}))
    with pytest.raises(ValueError, match='Passive profile'):
        runner.trial_profile('passive', True, profile)
    profile.write_text(yaml.safe_dump({'fr10_pose_controller': {'ros__parameters': {
        'force.search.spiral_enabled': False, 'force.search.yaw_probe_torque_nm': .1}}}))
    with pytest.raises(ValueError, match='Passive profile'):
        runner.trial_profile('passive', True, profile)
    with pytest.raises(ValueError, match='Search profile'):
        runner.trial_profile('search', True, profile)


def test_missing_workspace_stops_before_launch_or_empty_failure_plots(tmp_path, monkeypatch, capsys):
    from ament_index_python import packages
    output = tmp_path / 'missing_runtime'
    monkeypatch.setattr(packages, 'get_packages_with_prefixes', lambda: {})
    monkeypatch.setattr(runner.sys, 'argv', ['runner', '--case-group', 'smoke',
                                           '--output', str(output)])
    def forbidden(*args, **kwargs):
        raise AssertionError('Missing workspace must not launch a trial')
    monkeypatch.setattr(runner.subprocess, 'run', forbidden)
    with pytest.raises(SystemExit) as error:
        runner.main()
    assert error.value.code == 2
    assert 'source install/setup.bash' in capsys.readouterr().err
    assert not (output / 'aligned').exists()
    assert not (output / 'matrix_summary.json').exists()


def test_domain_guard_allows_read_only_viewers_but_rejects_existing_clock_or_controller():
    from types import SimpleNamespace
    class Graph:
        publishers = {}
        services = []
        def get_publishers_info_by_topic(self, topic):
            return [SimpleNamespace(node_name=name) for name in self.publishers.get(topic, [])]
        def get_service_names_and_types(self):
            return self.services
    graph = Graph()
    # A viewer and plot only subscribe; they must not block an experiment.
    assert runner.runtime_conflicts(graph) == []
    graph.publishers = {'/clock': ['old_mujoco_bridge'], '/joint_states': ['old_mujoco_bridge']}
    graph.services = [('/get_planning_scene', ['moveit_msgs/srv/GetPlanningScene'])]
    assert runner.runtime_conflicts(graph) == ['/clock published by old_mujoco_bridge',
        '/get_planning_scene', '/joint_states published by old_mujoco_bridge']


def test_busy_domain_stops_before_starting_a_second_plant(tmp_path, monkeypatch, capsys):
    output = tmp_path / 'busy_domain'
    monkeypatch.setattr(runner.sys, 'argv', ['runner', '--cases', 'aligned', '--output', str(output)])
    import fr10_matched_run_manifest
    monkeypatch.setattr(fr10_matched_run_manifest, 'runtime_binary_inventory', lambda _: ({}, []))
    def busy(domain):
        raise ValueError(f'ROS domain {domain} already has an active runtime')
    monkeypatch.setattr(runner, 'require_unused_domain', busy)
    def forbidden(*args, **kwargs):
        raise AssertionError('Busy domain must not launch another plant')
    monkeypatch.setattr(runner.subprocess, 'run', forbidden)
    with pytest.raises(SystemExit) as error:
        runner.main()
    assert error.value.code == 2
    assert 'already has an active runtime' in capsys.readouterr().err
    assert not (output / 'aligned').exists()


def test_timeout_and_shutdown_failure_remain_in_success_denominator():
    result = matrix_outcomes([
        {'run_summary_received': True, 'pass': True, 'insert_result': {'success': True}},
        {'run_summary_received': True, 'pass': False, 'error': 'timeout'},
        {'run_summary_received': True, 'pass': False, 'error': 'shutdown failed'},
        {'run_summary_received': False, 'pass': False},
    ])
    assert result['requested_cases'] == 4
    assert result['completed_cases'] == 3
    assert result['action_results_received'] == 1
    assert result['passed_cases'] == 1
    assert result['observed_success_fraction'] == pytest.approx(1. / 3.)
    assert not result['all_pass']
    assert matrix_outcomes([])['observed_success_fraction'] is None


def test_matched_replay_uses_measured_positions_and_wall_time(tmp_path):
    rows = [{'wall': 100. + i * .5, 'position': [float(i)] * 6} for i in range(3)]
    path = tmp_path / 'joint_feedback.json'
    path.write_text(json.dumps(rows))
    frames = load_run_frames(tmp_path)
    assert [f['time'] for f in frames] == [0., .5, 1.]
    assert list(frames[-1]['q']) == [2.] * 6
    rows[-1]['wall'] = 99.
    path.write_text(json.dumps(rows))
    with pytest.raises(ValueError):
        load_run_frames(tmp_path)
