"""Observe Gazebo's actual model pose through a diagnostic state plugin."""

import json
import math
import os
from pathlib import Path
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy


def quaternion_rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                      1 - 2 * (q.x * q.x + q.y * q.y))
    pitch_sin = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(pitch_sin)
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return [roll, pitch, yaw]


rclpy.init()
node = rclpy.create_node('ode_model_state_observer')
directory = Path(os.environ['ODE_OUT'])
stream = (directory / 'transport_state.jsonl').open('w', buffering=1)
clock = {'seconds': 0.0}


def receive_clock(message):
    clock['seconds'] = message.clock.sec + message.clock.nanosec * 1e-9


def receive_models(message):
    try:
        index = message.name.index('my_mobile_manipulator')
    except ValueError:
        return
    pose = message.pose[index]
    twist = message.twist[index]
    model = {
        'name': 'my_mobile_manipulator',
        'position': [pose.position.x, pose.position.y, pose.position.z],
        'rpy': quaternion_rpy(pose.orientation),
        'linear': [twist.linear.x, twist.linear.y, twist.linear.z],
        'angular': [twist.angular.x, twist.angular.y, twist.angular.z],
        'joints': [],
    }
    stream.write(json.dumps({'wall': time.monotonic(),
                             'sim': clock['seconds'],
                             'models': [model], 'contacts': []}) + '\n')


clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Clock, '/clock', receive_clock, clock_qos)
node.create_subscription(ModelStates, '/ode_probe/model_states', receive_models, 10)
try:
    rclpy.spin(node)
finally:
    stream.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
