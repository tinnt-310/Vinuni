"""Opt-in acceptance of the operational launch profiles documented in README_COMMANDS."""
import json
import os
import signal
import socket
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('RUN_GAZEBO_PROFILE_TESTS') != '1',
    reason='Opt-in Gazebo operational profile tests')


@pytest.mark.parametrize('profile,arm,vehicle,force,station', [
    ('vehicle', False, True, False, ''),
    ('arm', True, False, False, ''),
    ('combined', True, True, False, ''),
    ('force_arm', True, False, True, ''),
    ('force_mounted', True, True, True, 'front_mid'),
])
def test_bringup_profile(tmp_path, profile, arm, vehicle, force, station):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from controller_manager_msgs.srv import ListControllers
    from fr10_motion_interfaces.action import Insert
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState, LaserScan
    from tf2_ros import Buffer, TransformListener

    domain = 180 + ['vehicle', 'arm', 'combined', 'force_arm', 'force_mounted'].index(profile)
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
                      ROS_LOG_DIR=str(tmp_path / 'ros_log'))
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    command = ['ros2', 'launch', 'vehicle_bringup', 'bringup.launch.py',
               f'use_arm:={str(arm).lower()}', f'use_vehicle:={str(vehicle).lower()}',
               f'use_force:={str(force).lower()}', 'use_gazebo:=true',
               'gui:=false', 'rviz:=false', f'gazebo_port:={port}']
    if station:
        command += [f'station:={station}', 'initial_state:=home', 'demo:=false']
    rclpy.init()
    node = rclpy.create_node('bringup_profile_acceptance')
    counts, latest, clock_samples = {}, {}, []

    def observe(name, message):
        counts[name] = counts.get(name, 0) + 1
        latest[name] = message
        if name == 'clock':
            clock_samples.append(message.clock.sec + message.clock.nanosec * 1.e-9)

    for name, topic, message_type in [
        ('clock', '/clock', Clock), ('joint', '/joint_states', JointState),
        ('scan', '/scan', LaserScan), ('odom', '/odom', Odometry),
        ('tcp', '/fr10_motion/tcp_pose', PoseStamped),
    ]:
        node.create_subscription(message_type, topic,
                                 lambda msg, name=name: observe(name, msg), qos_profile_sensor_data)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    expected = {'joint_state_broadcaster'}
    if arm:
        expected.add('fairino10_controller')
    if vehicle:
        expected.add('swerve_controller')
    log = (tmp_path / 'runtime.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    report = dict(profile=profile, command=command, domain=domain, passed=False)

    def wait(predicate, timeout=90, label='condition'):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, f'Bringup exited; {tmp_path}'
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        pytest.fail(f'{profile}: timed out waiting for {label}; {tmp_path}')

    def spin_for(seconds, twist):
        deadline = time.monotonic() + seconds
        next_publish = 0.0
        while time.monotonic() < deadline:
            assert process.poll() is None
            if time.monotonic() >= next_publish:
                publisher.publish(twist)
                next_publish = time.monotonic() + 0.05
            rclpy.spin_once(node, timeout_sec=0.01)

    try:
        wait(client.service_is_ready, label='controller manager')
        active = set()
        deadline = time.monotonic() + 60
        while active != expected and time.monotonic() < deadline:
            future = client.call_async(ListControllers.Request())
            wait(future.done, timeout=10, label='controller state')
            response = future.result()
            active = {item.name for item in response.controller if item.state == 'active'}
        assert active == expected
        report['controllers'] = sorted(active)
        wait(lambda: counts.get('clock', 0) >= 20 and counts.get('joint', 0) >= 20,
             label='clock and joint feedback')
        assert clock_samples[-1] > clock_samples[0]
        joint_names = set(latest['joint'].name)
        if arm:
            assert {f'j{i}' for i in range(1, 7)} <= joint_names
            wait(lambda: counts.get('tcp', 0) >= 5, label='FR10 TCP feedback')
        if vehicle:
            assert {f'Wheel{i}_to_Revolute{i}' for i in range(1, 5)} <= joint_names
            wait(lambda: counts.get('scan', 0) >= 5 and counts.get('odom', 0) >= 5,
                 label='laser and odometry')
        frame = 'world' if station else 'odom' if vehicle else 'fr10_base_link'
        target = 'charging_tcp' if arm else 'laser_frame'
        wait(lambda: buffer.can_transform(frame, target, rclpy.time.Time()), label='model TF')
        report['tf'] = [frame, target]
        if force:
            insertion = ActionClient(node, Insert, '/fr10_force/insert')
            wait(insertion.server_is_ready, label='force insertion action server')
            report['force_action_ready'] = True
        if profile == 'vehicle':
            wait(lambda: publisher.get_subscription_count() > 0, label='Twist input')
            before = latest['odom'].pose.pose.position
            twist = Twist()
            twist.linear.x = 0.1
            spin_for(2.0, twist)
            spin_for(0.7, Twist())
            after_forward = latest['odom'].pose.pose.position
            assert after_forward.x - before.x > 0.03
            twist = Twist()
            twist.linear.y = 0.1
            spin_for(2.5, twist)
            spin_for(0.7, Twist())
            after_strafe = latest['odom'].pose.pose.position
            assert after_strafe.y - after_forward.y > 0.03
            report['forward_odom_m'] = after_forward.x - before.x
            report['strafe_odom_m'] = after_strafe.y - after_forward.y
        report.update(passed=True, samples=counts)
        print(json.dumps(report, ensure_ascii=False))
    finally:
        publisher.publish(Twist())
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=10)
        log.close()
        report['launch_exit'] = process.returncode
        (tmp_path / 'summary.json').write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
