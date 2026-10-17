"""Gazebo-only mounted test: arm goal at rest, then arm goal while driving."""

import json
import math
import os
from pathlib import Path
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


assert os.environ.get('ROS_DOMAIN_ID') == '93'
rclpy.init()
node = rclpy.create_node('ode_mounted_coupled_client')
joint_samples = []
odom_samples = []
subscriptions = [
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: joint_samples.append({
                                 'names': list(msg.name), 'p': list(msg.position),
                                 'v': list(msg.velocity), 'e': list(msg.effort)}), 10),
    node.create_subscription(Odometry, '/odom',
                             lambda msg: odom_samples.append([
                                 msg.pose.pose.position.x,
                                 msg.pose.pose.position.y]), 10),
]
publisher = node.create_publisher(Twist, '/cmd_vel', 10)
action = ActionClient(node, FollowJointTrajectory,
                      '/joint_trajectory_controller/follow_joint_trajectory')
drive_command = Twist()
drive_command.linear.x = 0.15
next_drive_publish = 0.0


def step(drive=False):
    global next_drive_publish
    now = time.monotonic()
    if drive and now >= next_drive_publish:
        publisher.publish(drive_command)
        next_drive_publish = now + 0.02
    rclpy.spin_once(node, timeout_sec=0.03)


def spin_for(seconds, drive=False):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        step(drive)


def await_future(future, timeout, drive=False):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        step(drive)
    if not future.done():
        raise TimeoutError('mounted action future timed out')
    return future.result()


def send_joint_goal(target, drive=False):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f'j{i}' for i in range(1, 7)]
    point = JointTrajectoryPoint()
    point.positions = target
    point.velocities = [0.0] * 6
    point.time_from_start.sec = 4
    goal.trajectory.points = [point]
    sent = time.monotonic()
    handle = await_future(action.send_goal_async(goal), 10, drive)
    if not handle.accepted:
        raise RuntimeError('mounted arm goal was rejected')
    response = await_future(handle.get_result_async(), 20, drive)
    return {'sent_wall': sent, 'result_wall': time.monotonic(),
            'status': response.status, 'error_code': response.result.error_code}


home = [0.0, -math.pi / 2, math.pi / 2,
        -math.pi / 2, -math.pi / 2, 0.0]
rest_target = [0.15, *home[1:5], 0.30]
moving_target = [0.30, *home[1:5], 0.45]
arm_result = {'home': home, 'rest_target': rest_target,
              'target': moving_target}
vehicle_result = {}
drive_start = None
drive_end = None

try:
    if not action.wait_for_server(timeout_sec=25):
        raise TimeoutError('mounted arm action server unavailable')
    spin_for(2.0)
    arm_result['initial'] = joint_samples[-1] if joint_samples else None
    rest_goal = send_joint_goal(rest_target)
    arm_result['rest_goal'] = rest_goal
    if rest_goal['status'] != 4 or rest_goal['error_code'] != 0:
        raise RuntimeError('stationary arm goal failed')

    odom_before = odom_samples[-1] if odom_samples else None
    drive_start = time.monotonic()
    spin_for(0.5, drive=True)
    moving_goal = send_joint_goal(moving_target, drive=True)
    arm_result['moving_goal'] = moving_goal
    remaining_drive = max(0.0, drive_start + 8.0 - time.monotonic())
    spin_for(remaining_drive, drive=True)
    drive_end = time.monotonic()
    publisher.publish(Twist())
    spin_for(3.0)

    arm_result.update(status=moving_goal['status'],
                      error_code=moving_goal['error_code'],
                      final=joint_samples[-1] if joint_samples else None)
    overlap = max(0.0, min(moving_goal['result_wall'], drive_end) -
                  max(moving_goal['sent_wall'], drive_start))
    arm_result['moving_goal_drive_overlap_s'] = overlap
    arm_result['rest_goal_completed_before_drive'] = (
        rest_goal['result_wall'] < drive_start)
    arm_joint_names = {f'j{i}' for i in range(1, 7)}
    arm_result['finite'] = bool(joint_samples) and all(
        arm_joint_names.issubset(set(sample['names'])) and
        all(index < len(sample[field]) and math.isfinite(sample[field][index])
            for index, name in enumerate(sample['names'])
            if name in arm_joint_names for field in ('p', 'v', 'e'))
        for sample in joint_samples)

    odom_after = odom_samples[-1] if odom_samples else None
    wheel_names = {f'Wheel{i}_to_Revolute{i}' for i in range(1, 5)}
    wheel_present = bool(joint_samples) and wheel_names.issubset(
        set(joint_samples[-1]['names']))
    odom_distance = math.dist(odom_before, odom_after) if (
        odom_before is not None and odom_after is not None) else 0.0
    vehicle_result = {
        'sample_count': len(joint_samples), 'odom_count': len(odom_samples),
        'wheel_names_present': wheel_present,
        'finite': all(math.isfinite(value) for sample in joint_samples
                      for field in ('p', 'v') for value in sample[field]),
        'odom_distance_m': odom_distance,
        'drive_start_wall': drive_start, 'drive_end_wall': drive_end,
        'pass': bool(wheel_present and odom_distance > 0.5 and
                     overlap >= 2.0 and arm_result['rest_goal_completed_before_drive']),
    }
except Exception as error:
    arm_result['failure'] = str(error)
    arm_result['finite'] = False
    vehicle_result = {'pass': False, 'failure': str(error)}
finally:
    publisher.publish(Twist())
    directory = Path(os.environ['ODE_OUT'])
    (directory / 'result.json').write_text(json.dumps(arm_result, indent=2))
    (directory / 'vehicle_result.json').write_text(json.dumps(vehicle_result, indent=2))
    print(json.dumps({'arm': arm_result, 'vehicle': vehicle_result}))
    node.destroy_node()
    rclpy.shutdown()
