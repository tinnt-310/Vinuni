"""ROS Python children must not resolve an unrelated interpreter from PATH."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


SOURCE = Path(__file__).resolve().parents[2]


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def interpreter_probe(tmp_path, monkeypatch):
    shadow = tmp_path / 'bin'
    shadow.mkdir()
    python = shadow / 'python3'
    python.write_text('#!/bin/sh\necho "Wrong Python from PATH" >&2\nexit 73\n')
    python.chmod(0o755)
    monkeypatch.setenv('PATH', str(shadow) + os.pathsep + os.environ['PATH'])
    probe = tmp_path / 'probe.py'
    probe.write_text('#!/usr/bin/env python3\nimport sys\nimport rclpy\nprint(sys.executable)\n')
    probe.chmod(0o755)
    # Establish that an unqualified env-python3 script really fails here.
    assert subprocess.run([str(probe)], capture_output=True).returncode == 73
    return probe


def assert_ros_interpreter(node, context, probe):
    # Ask launch to resolve the actual process command and environment. Replace
    # only the node body to avoid starting DDS/Gazebo during this unit test.
    executable = perform_substitutions(context, node.process_description.cmd[0])
    node.process_description.prepare(context, node)
    command = list(node.process_description.final_cmd)
    command[command.index(executable)] = str(probe)
    result = subprocess.run(command, env=node.process_description.final_env,
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, (executable, result.stderr)
    assert result.stdout.strip() == sys.executable


@pytest.mark.parametrize('overrides', [
    {'enable_arm': 'false', 'navigation_mode': 'mapping'},
    {'enable_vehicle': 'false', 'enable_force_control': 'true'},
    {'enable_force_control': 'true', 'station': 'front_mid'},
])
def test_simulation_python_nodes_with_shadowed_path(tmp_path, interpreter_probe, overrides):
    simulation = load(SOURCE / 'vehicle_bringup/launch/simulation.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(arm_controller_profile='pose', gui='false',
                                         rviz='false', **overrides)
    for action in simulation.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = []

    def capture_node(**kwargs):
        node = Node(**kwargs)
        if kwargs['executable'].endswith('.py'):
            nodes.append(node)
        return node

    with patch.object(simulation.socket, 'socket') as port_probe, patch.object(
            simulation.tempfile, 'mkdtemp', return_value=str(tmp_path)), patch.object(
            simulation, 'Node', side_effect=capture_node):
        port_probe.return_value.__enter__.return_value.connect_ex.return_value = 1
        simulation.launch_setup(context)
    assert nodes
    for node in nodes:
        assert_ros_interpreter(node, context, interpreter_probe)


@pytest.mark.parametrize('filename', ['swerve_controller.launch.py', 'drive.launch.py'])
def test_velocity_normalizer_with_shadowed_path(interpreter_probe, filename):
    module = load(SOURCE / 'swerve_vehicle_control/launch' / filename)
    context = LaunchContext()
    nodes = []
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        if isinstance(action, Node) and perform_substitutions(
                context, normalize_to_list_of_substitutions(action.node_executable)).endswith('.py'):
            nodes.append(action)
    assert len(nodes) == 1
    assert_ros_interpreter(nodes[0], context, interpreter_probe)
