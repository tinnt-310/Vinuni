#!/usr/bin/env python3
"""Convert the legacy Nav2 Twist topic to the swerve controller's TwistStamped input."""

import math
import signal

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


def should_emit_command(source_ns, values, previous_source_ns, previous_values):
    """Coalesce only identical commands stamped by the same /clock sample.

    A changed command, especially zero/stop, still reaches the controller;
    its duplicate stamp makes the controller fail closed until a new /clock
    sample can carry the changed command.
    """
    return (previous_source_ns is None or source_ns != previous_source_ns
            or values != previous_values)


class CmdVelNormalizer(Node):
    def __init__(self):
        super().__init__('cmd_vel_normalizer')
        input_topic = self.declare_parameter('input_topic', '/cmd_vel').value
        output_topic = self.declare_parameter('output_topic', '/cmd_vel_nav').value
        if input_topic == output_topic:
            raise ValueError('Input and output command topics must differ')
        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self.subscription = self.create_subscription(Twist, input_topic, self.on_command, 10)
        self._last_emitted_source_ns = None
        self._last_emitted_values = None

    def on_command(self, command):
        values = (command.linear.x, command.linear.y, command.linear.z,
                  command.angular.x, command.angular.y, command.angular.z)
        output = TwistStamped()
        source_time = self.get_clock().now()
        output.header.stamp = source_time.to_msg()
        output.header.frame_id = 'base_footprint'
        if all(math.isfinite(value) for value in values):
            output.twist = command
            emitted_values = values
        else:
            self.get_logger().warning('Rejected nonfinite Twist; publishing zero command')
            emitted_values = (0.0,) * 6
        if not should_emit_command(source_time.nanoseconds, emitted_values,
                                   self._last_emitted_source_ns,
                                   self._last_emitted_values):
            return
        self.publisher.publish(output)
        self._last_emitted_source_ns = source_time.nanoseconds
        self._last_emitted_values = emitted_values


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelNormalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A launch timeout may deliver SIGINT to both launch and this child.
        # Ignore the second one while releasing ROS resources.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
