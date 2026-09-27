#!/usr/bin/env python3
"""Convert the legacy Nav2 Twist topic to the swerve controller's TwistStamped input."""

import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelNormalizer(Node):
    def __init__(self):
        super().__init__('cmd_vel_normalizer')
        input_topic = self.declare_parameter('input_topic', '/cmd_vel').value
        output_topic = self.declare_parameter('output_topic', '/cmd_vel_nav').value
        if input_topic == output_topic:
            raise ValueError('Input and output command topics must differ')
        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self.subscription = self.create_subscription(Twist, input_topic, self.on_command, 10)

    def on_command(self, command):
        values = (command.linear.x, command.linear.y, command.linear.z,
                  command.angular.x, command.angular.y, command.angular.z)
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'base_footprint'
        if all(math.isfinite(value) for value in values):
            output.twist = command
        else:
            self.get_logger().warning('Rejected nonfinite Twist; publishing zero command')
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelNormalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
