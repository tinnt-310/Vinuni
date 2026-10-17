#!/usr/bin/env python3
"""Wait for live simulation data and controller ownership before navigation."""
import math
import time


def inputs_ready(now, received, controllers, odom_tf, laser_tf):
    return (all(0 <= now - received.get(key, -math.inf) < 2.0
                for key in ('clock', 'scan', 'odom'))
            and {'joint_state_broadcaster', 'swerve_controller'} <= controllers
            and odom_tf and laser_tf)


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from controller_manager_msgs.srv import ListControllers
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('wait_for_navigation')
    timeout = node.declare_parameter('timeout', 90.0).value
    if not math.isfinite(timeout) or timeout <= 0:
        node.get_logger().error('timeout must be a positive finite number')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    received, controllers = {}, set()
    previous_clock = None
    laser_frame = ''

    def clock(message):
        nonlocal previous_clock
        stamp = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if previous_clock is not None and stamp > previous_clock:
            received['clock'] = time.monotonic()
        elif previous_clock is not None and stamp < previous_clock:
            received.clear()
        previous_clock = stamp

    def scan(message):
        nonlocal laser_frame
        if message.header.frame_id and message.ranges:
            received['scan'] = time.monotonic()
            laser_frame = message.header.frame_id

    def odom(message):
        if message.header.frame_id == 'odom' and message.child_frame_id == 'base_footprint':
            received['odom'] = time.monotonic()

    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    node.create_subscription(LaserScan, node.declare_parameter('scan_topic', '/scan').value,
                             scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, node.declare_parameter('odom_topic', '/odom').value,
                             odom, qos_profile_sensor_data)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    request = None
    next_request = 0.0
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if request is not None and request.done():
                response = request.result()
                controllers = {c.name for c in response.controller if c.state == 'active'} if response else set()
                request = None
            if request is None and now >= next_request and client.service_is_ready():
                request = client.call_async(ListControllers.Request())
                next_request = now + 0.5
            if inputs_ready(now, received, controllers,
                            buffer.can_transform('odom', 'base_footprint', rclpy.time.Time()),
                            bool(laser_frame) and buffer.can_transform(
                                'base_footprint', laser_frame, rclpy.time.Time())):
                node.get_logger().info('Vehicle controller, clock, scan, odometry and TF are ready')
                return 0
        node.get_logger().error(f'Navigation startup timed out; received={list(received)}, active={sorted(controllers)}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
