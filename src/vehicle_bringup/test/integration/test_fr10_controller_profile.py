import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location('controller_profile', ROOT /
    'src/fr10_arm_control/fr10_pose_controller/launch/controller_profile.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_uncertainty_profile_preserves_safety_and_budgets_validated_handover():
    folder = ROOT / 'src/fr10_arm_control/fr10_pose_controller/config'
    legacy = MODULE.load_controller_profile(folder / 'matched_socket_commissioning.yaml')
    new = MODULE.load_controller_profile(folder / 'pose_uncertainty_commissioning.yaml')
    # Both matched profiles share the same exact worker/handover pipeline.
    # A35 ms check plus two8 ms handovers does not fit the old48 ms window.
    for profile in (legacy, new):
        assert .035 + 2 * .008 <= profile['force.target_validity_s'] <= .064
        assert profile['force.target_validity_s'] + 3 * .008 <= profile['force.command_lease_s']
        assert profile['force.command_lease_s'] <= 2 * profile['force.target_validity_s']
    assert new['force.target_validity_s'] == legacy['force.target_validity_s']
    assert new['force.command_lease_s'] == legacy['force.command_lease_s']
    defaults = MODULE.load_controller_profile(folder / 'controller.yaml')
    for key, baseline in (
        ('controller.feedback_timeout_s', defaults['controller']['feedback_timeout_s']),
        ('force.safety.stale_timeout_s', defaults['force']['safety']['stale_timeout_s']),
        ('force.goal.max_force_n', [50., 50., 50.]),
        ('force.goal.max_torque_nm', [10., 10., 10.]),
    ):
        assert new.get(key, baseline) == legacy.get(key, baseline)
    assert new['force.contact_links'] == legacy['force.contact_links']
    assert new['force.admittance.stiffness'][0] > 0
    assert new['force.admittance.stiffness'][1] > 0
    assert new['force.search.radius_m'] == .0025
    assert new['force.admittance.stiffness'][2] == 0
    assert new['force.search.force_n'] > new['force.contact.enter_force_n'] + new['force.admittance.drive_deadband'][2]
    assert new['force.search.yaw_probe_torque_nm'] > new['force.search.yaw_contact_torque_nm'] > 0
    assert new['force.search.xy_probe_force_n'] > new['force.search.xy_contact_force_n'] > 2.0*.4
    assert 'force.search.xy_probe_force_n' not in legacy
    assert new['force.pre_stop_force_error_tolerance_n'] == .6
    assert new['force.admittance.damping'][3:] == [20.,20.,20.]


def test_profile_cycle_and_invalid_parameters_fail_before_launch(tmp_path):
    first = tmp_path / 'first.yaml'
    second = tmp_path / 'second.yaml'
    first.write_text('extends: second.yaml\n')
    second.write_text('extends: first.yaml\n')
    with pytest.raises(ValueError, match='Cyclic'):
        MODULE.load_controller_profile(first)
    second.write_text('fr10_pose_controller:\n  ros__parameters: []\n')
    with pytest.raises(ValueError, match='mapping'):
        MODULE.load_controller_profile(second)


def test_external_contact_source_compensates_gravity_once_and_inherits_uncertainty_guards():
    folder = ROOT / 'src/fr10_arm_control/fr10_pose_controller/config'
    legacy = MODULE.load_controller_profile(folder / 'pose_uncertainty_commissioning.yaml')
    contact = MODULE.load_controller_profile(folder / 'gazebo_contact_wrench_commissioning.yaml')
    assert contact['force.gravity_compensation.owner'] == 'source'
    assert not contact['force.gravity_compensation.enabled']
    assert not contact['force.gravity_compensation.calibrated']
    assert contact['force.wrench_source_frame'] == contact['force.wrench_target_frame'] == 'charging_tcp'
    assert contact['force.wrench_transform.target_to_source_m'] == [0., 0., 0.]
    assert contact['force.wrench_source_sign'] == 1.
    assert contact['force.wrench_topic'] == '/fr10_simulation/contact_wrench'
    assert all(contact[key] == value for key,value in legacy.items()
               if not key.startswith(('force.gravity_compensation.', 'force.wrench_')))
