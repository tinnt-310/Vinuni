"""Exercise vehicle wheels in an isolated Gazebo ROS domain."""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

assert os.environ.get('ROS_DOMAIN_ID') == '93'
rclpy.init()
node = rclpy.create_node('ode_vehicle_client')
samples = []
odom = []
node.create_subscription(JointState, '/joint_states',
                         lambda m: samples.append({'name': list(m.name),
                                                   'position': list(m.position),
                                                   'velocity': list(m.velocity)}), 10)
node.create_subscription(Odometry, '/odom',
                         lambda m: odom.append([m.pose.pose.position.x,
                                                m.pose.pose.position.y]), 10)
pub = node.create_publisher(Twist, '/cmd_vel', 10)
controller_manager = node.create_client(
    ListControllers, '/controller_manager/list_controllers')

def wait_for_active_controllers(timeout_s):
    deadline = time.monotonic() + timeout_s
    required = {'swerve_controller'}
    while time.monotonic() < deadline:
        if not controller_manager.wait_for_service(timeout_sec=0.25):
            continue
        future = controller_manager.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        if future.done() and future.result() is not None:
            active = {controller.name for controller in future.result().controller
                      if controller.state == 'active'}
            if required.issubset(active) and not ({'steering_controller', 'drive_controller'} & active):
                return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False

def spin_for(seconds, command=None):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if command is not None:
            pub.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)

try:
    if not wait_for_active_controllers(30.0):
        raise RuntimeError('sole swerve controller did not become active')
    spin_for(1.0)
    first = odom[-1] if odom else None
    command = Twist()
    command.linear.x = 0.15
    spin_for(5.0, command)
    pub.publish(Twist())
    spin_for(3.0)
    last = odom[-1] if odom else None
    names = set(samples[-1]['name']) if samples else set()
    wheel_names = {f'Wheel{i}_to_Revolute{i}' for i in range(1, 5)}
    distance = math.dist(first, last) if first and last else 0.0
    result = {
        'sample_count': len(samples), 'odom_count': len(odom),
        'wheel_names_present': wheel_names.issubset(names),
        'finite': all(math.isfinite(value) for sample in samples
                      for field in ('position', 'velocity') for value in sample[field]),
        'odom_distance_m': distance,
        'pass': bool(samples and odom and wheel_names.issubset(names)
                     and distance > 0.03),
    }
except Exception as error:
    result = {'pass': False, 'failure': str(error)}
Path(os.environ['ODE_OUT'], 'vehicle_result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result))
node.destroy_node()
rclpy.shutdown()
