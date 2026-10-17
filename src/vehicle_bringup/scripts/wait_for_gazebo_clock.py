#!/usr/bin/env python3
"""Exit successfully only after the Gazebo world clock advances."""

import time


class ClockReadiness:
    """Pure startup gate: a service alone does not prove physics is stepping."""

    def __init__(self):
        self.previous_ns = None
        self.ready = False

    def observe(self, sec, nanosec):
        if sec < 0 or not 0 <= nanosec < 1_000_000_000:
            return False
        current_ns = sec * 1_000_000_000 + nanosec
        if self.previous_ns is not None:
            if current_ns < self.previous_ns:
                self.ready = False
            elif current_ns > self.previous_ns:
                self.ready = True
        self.previous_ns = current_ns
        return self.ready


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node('wait_for_gazebo_clock')
    gate = ClockReadiness()

    def on_clock(message):
        gate.observe(message.clock.sec, message.clock.nanosec)

    subscription = node.create_subscription(Clock, '/clock', on_clock,
                                            qos_profile_sensor_data)
    deadline = time.monotonic() + 30.0
    try:
        while rclpy.ok() and not gate.ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        if gate.ready:
            node.get_logger().info('Gazebo /clock is advancing; world is ready for spawn')
            return 0
        node.get_logger().error('Gazebo /clock did not advance within 30 seconds')
        return 1
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
