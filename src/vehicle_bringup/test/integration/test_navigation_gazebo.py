"""Opt-in Gazebo mapping -> saved map -> AMCL/Nav2 integration acceptance.

RUN_GAZEBO_NAVIGATION_TEST=1 python3 -m pytest -s <this file>
Run in a sourced workspace. Never connects to physical robot hardware.
"""
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('RUN_GAZEBO_NAVIGATION_TEST') != '1',
    reason='Opt-in Gazebo integration test')


def test_mapping_save_localize_navigate(tmp_path):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    from tf2_ros import Buffer, TransformListener

    os.environ['ROS_DOMAIN_ID'] = os.environ.get('NAVIGATION_TEST_DOMAIN', '169')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(tmp_path / 'ros_log')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        gazebo_port = str(probe.getsockname()[1])
    rclpy.init()
    node = rclpy.create_node('navigation_acceptance', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    observations = {}
    map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map',
                             lambda msg: observations.update(map=msg), map_qos)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                             lambda msg: observations.update(pose=msg), map_qos)
    node.create_subscription(TwistStamped, '/cmd_vel_nav',
                             lambda msg: observations.update(command=msg), 10)
    initial_pose = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    process = None
    log = None

    def wait(predicate, seconds=90, label='condition'):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if process is not None and process.poll() is not None:
                pytest.fail(f'Bringup exited with {process.returncode}; see {log.name}')
            if predicate():
                return
        pytest.fail(f'Timed out waiting for {label}; artifacts: {tmp_path}')

    def start(mode, **arguments):
        nonlocal process, log
        log = (tmp_path / f'{mode}.log').open('w')
        command = ['ros2', 'launch', 'vehicle_bringup', 'bringup.launch.py',
                   'use_arm:=false', 'use_gazebo:=true', 'gui:=false', 'rviz:=false',
                   f'gazebo_port:={gazebo_port}', f'navigation_mode:={mode}']
        command += [f'{key}:={value}' for key, value in arguments.items()]
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)

    def stop():
        nonlocal process, log
        if process is not None:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=10)
            process = None
        if log:
            log.close()
            log = None

    def initialize_localization():
        wait(lambda: initial_pose.get_subscription_count() > 0, label='AMCL initial pose subscriber')
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685
        last_initial_pose = 0.0

        def localized():
            nonlocal last_initial_pose
            if time.monotonic() - last_initial_pose > 1.0:
                pose.header.stamp = node.get_clock().now().to_msg()
                initial_pose.publish(pose)
                last_initial_pose = time.monotonic()
            return 'pose' in observations and buffer.can_transform('map', 'base_footprint', rclpy.time.Time())

        wait(localized, label='AMCL pose and map TF')

    try:
        start('mapping')
        wait(lambda: 'map' in observations and observations['map'].info.width > 0,
             label='SLAM map')
        assert any(cell == 0 for cell in observations['map'].data)
        prefix = tmp_path / 'acceptance_map'
        with (tmp_path / 'save_map.log').open('w') as saver_log:
            result = subprocess.run(['ros2', 'launch', 'vehicle_bringup', 'save_map.launch.py',
                                     f'map_name:={prefix}'], stdout=saver_log,
                                    stderr=subprocess.STDOUT, timeout=30)
        assert result.returncode == 0
        assert prefix.with_suffix('.yaml').is_file()
        assert prefix.with_suffix('.pgm').is_file()
        stop()
        observations.clear()
        buffer.clear()
        start('localization', map=prefix.with_suffix('.yaml'))
        initialize_localization()
        wait(lambda: 'slam_toolbox' not in node.get_node_names(), label='exclusive AMCL ownership')
        assert 'controller_server' not in node.get_node_names()
        assert 'bt_navigator' not in node.get_node_names()
        for name in ['amcl', 'map_server']:
            client = node.create_client(GetState, f'/{name}/get_state')
            wait(client.service_is_ready, label=f'{name} localization lifecycle')
            future = client.call_async(GetState.Request())
            wait(future.done, label=f'{name} localization state')
            assert future.result().current_state.id == 3
            node.destroy_client(client)
        stop()
        observations.clear()
        buffer.clear()
        start('navigation', map=prefix.with_suffix('.yaml'))
        initialize_localization()
        for name in ['amcl', 'map_server', 'controller_server', 'planner_server',
                     'smoother_server', 'behavior_server', 'bt_navigator',
                     'waypoint_follower', 'velocity_smoother']:
            client = node.create_client(GetState, f'/{name}/get_state')
            wait(client.service_is_ready, label=f'{name} lifecycle service')
            active = False
            deadline = time.monotonic() + 45
            while not active and time.monotonic() < deadline:
                future = client.call_async(GetState.Request())
                wait(future.done, seconds=10, label=f'{name} lifecycle response')
                active = future.result().current_state.id == 3
                if not active:
                    rclpy.spin_once(node, timeout_sec=0.2)
            assert active, f'{name} did not become active'
            node.destroy_client(client)
        navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        wait(navigator.server_is_ready, label='NavigateToPose action server')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose = observations['pose'].pose.pose
        goal.pose.pose.position.x += 0.4
        future = navigator.send_goal_async(goal)
        wait(future.done, label='goal acceptance')
        handle = future.result()
        assert handle.accepted
        result = handle.get_result_async()
        wait(result.done, seconds=90, label='navigation goal result')
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        assert 'command' in observations
        assert observations['command'].header.frame_id == 'base_footprint'
        print(f'PASS: mapping, map saving, AMCL-only, Nav2 lifecycle, TF, NavigateToPose; {tmp_path}')
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()
