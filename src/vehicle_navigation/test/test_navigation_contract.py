import importlib.util
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('navigation_launch', ROOT / 'launch/navigation.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def stack(**overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    with patch.object(module, 'get_package_share_directory', return_value=str(ROOT)), \
            patch.object(module, 'Node', side_effect=lambda **kwargs: SimpleNamespace(**kwargs)):
        nodes = module.setup(context)
    return context, nodes


def names(nodes, context):
    return [node.executable for node in nodes]


def test_disabled_has_no_nodes():
    assert stack()[1] == []


@pytest.mark.parametrize('mode,expected,absent', [
    ('mapping', 'async_slam_toolbox_node', 'amcl'),
    ('localization', 'amcl', 'controller_server'),
    ('navigation', 'controller_server', 'async_slam_toolbox_node'),
])
def test_localization_owners_are_exclusive(mode, expected, absent):
    context, nodes = stack(navigation_mode=mode)
    executables = names(nodes, context)
    assert expected in executables
    assert absent not in executables
    assert 'robot_state_publisher' not in executables
    assert 'rviz2' not in executables


def test_missing_map_fails_before_nodes_start(tmp_path):
    with pytest.raises(ValueError, match='Map YAML does not exist'):
        stack(navigation_mode='navigation', map=str(tmp_path / 'missing.yaml'))
    filename = tmp_path / 'map.yaml'
    filename.write_text('image: missing.pgm\n')
    with pytest.raises(ValueError, match='Map image does not exist'):
        module.validate_map(filename)


@pytest.mark.parametrize('topic', ['/cmd_vel_nav', 'cmd_vel_nav', '/navigation/cmd_vel'])
def test_internal_topics_cannot_be_used_as_output(topic):
    with pytest.raises(ValueError, match='internal velocity'):
        stack(navigation_mode='navigation', cmd_vel_topic=topic)


def test_swerve_parameters_and_rewrites():
    context, nodes = stack(navigation_mode='navigation', scan_topic='/test_scan', odom_topic='/test_odom')
    server = nodes[names(nodes, context).index('controller_server')]
    parameter_file = server.parameters[0].evaluate(context)
    params = yaml.safe_load(Path(parameter_file).read_text())
    assert params['amcl']['ros__parameters']['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
    assert params['amcl']['ros__parameters']['scan_topic'] == '/test_scan'
    assert params['controller_server']['ros__parameters']['FollowPath']['motion_model'] == 'Omni'
    for name in ['local_costmap', 'global_costmap']:
        costmap = params[name][name]['ros__parameters']
        assert costmap['robot_base_frame'] == 'base_footprint'
        assert costmap['obstacle_layer']['scan']['topic'] == '/test_scan'
    assert params['velocity_smoother']['ros__parameters']['odom_topic'] == '/test_odom'
    assert params['velocity_smoother']['ros__parameters']['max_velocity'][1] > 0
    assert params['amcl']['ros__parameters']['set_initial_pose'] is False


def test_velocity_pipeline_has_distinct_message_types():
    context, nodes = stack(navigation_mode='navigation', cmd_vel_topic='/test_cmd_vel')
    by_name = dict(zip(names(nodes, context), nodes))
    def remappings(node):
        return node.remappings
    assert ('cmd_vel', '/navigation/cmd_vel') in remappings(by_name['controller_server'])
    assert ('cmd_vel', '/navigation/cmd_vel') in remappings(by_name['behavior_server'])
    assert ('cmd_vel_smoothed', '/test_cmd_vel') in remappings(by_name['velocity_smoother'])
    assert all(target != '/cmd_vel_nav' for node in nodes for _, target in remappings(node))


def test_packaged_map_is_portable():
    assert module.validate_map(ROOT / 'maps/vinfast.yaml')
