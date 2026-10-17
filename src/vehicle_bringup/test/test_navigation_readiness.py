import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gate = load('navigation_gate', ROOT / 'scripts/wait_for_navigation.py')
bringup = load('navigation_bringup', ROOT / 'launch/bringup.launch.py')


def test_ready_requires_fresh_inputs_controller_and_both_transforms():
    data = dict(clock=10., scan=10., odom=10.)
    controllers = {'joint_state_broadcaster', 'swerve_controller'}
    assert gate.inputs_ready(11., data, controllers, True, True)
    assert not gate.inputs_ready(13., data, controllers, True, True)
    assert not gate.inputs_ready(11., data, set(), True, True)
    assert not gate.inputs_ready(11., data, controllers, False, True)
    assert not gate.inputs_ready(11., data, controllers, True, False)
    for key in data:
        assert not gate.inputs_ready(11., {k: v for k, v in data.items() if k != key}, controllers, True, True)


@pytest.mark.parametrize('overrides', [
    {'use_vehicle': 'false'}, {'use_gazebo': 'false'}, {'use_force': 'true'},
])
def test_unsupported_navigation_combinations_fail_before_backend_lookup(overrides):
    context = LaunchContext()
    context.launch_configurations.update(navigation_mode='navigation', **overrides)
    for action in bringup.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    with pytest.raises(ValueError, match='Navigation requires'):
        bringup.setup(context)
