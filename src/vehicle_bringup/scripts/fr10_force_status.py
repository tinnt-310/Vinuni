#!/usr/bin/env python3
"""Read-only force/clock display; imports action feedback directly for Humble."""
from collections import deque
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from fr10_motion_interfaces.action import Insert


class ForceStatus(Node):
    def __init__(self):
        super().__init__('fr10_force_status')
        self.feedback = None
        self.received = 0.
        self.clocks = deque(maxlen=1000)
        self.depths = deque(maxlen=1000)
        self.tcp_positions = deque(maxlen=1000)
        self.tcp_frame = None
        self.create_subscription(Insert.Feedback, '/fr10_force/feedback',
                                 self.on_feedback, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/fr10_motion/tcp_pose',
                                 self.on_tcp, qos_profile_sensor_data)
        self.create_subscription(Insert.Result, '/fr10_force/result', self.on_result,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1., self.report)

    def on_feedback(self, message):
        now = time.monotonic()
        if self.feedback is None or message.generation != self.feedback.generation:
            self.depths.clear()
        self.feedback, self.received = message, now
        self.depths.append((now, message.depth))

    def on_clock(self, message):
        now = time.monotonic()
        source = message.clock.sec + message.clock.nanosec * 1e-9
        if self.clocks and source < self.clocks[-1][1]:
            self.clocks.clear()
        if not self.clocks or now - self.clocks[-1][0] >= .02:
            self.clocks.append((now, source))

    def on_tcp(self, message):
        now = time.monotonic()
        p = message.pose.position
        position = (p.x, p.y, p.z)
        if not all(math.isfinite(value) for value in position):
            self.tcp_positions.clear()
            return
        if message.header.frame_id != self.tcp_frame:
            self.tcp_positions.clear()
            self.tcp_frame = message.header.frame_id
        self.tcp_positions.append((now, position))

    def tcp_speed_wall_mm_s(self, now):
        while len(self.tcp_positions) > 2 and self.tcp_positions[1][0] < now - 5.:
            self.tcp_positions.popleft()
        rows = self.tcp_positions
        if len(rows) < 2 or now - rows[-1][0] > 1. or rows[-1][0] <= rows[0][0]:
            return float('nan')
        if any(b[0] - a[0] > 1. for a, b in zip(rows, list(rows)[1:])):
            return float('nan')
        distance = sum(math.dist(a[1], b[1]) for a, b in zip(rows, list(rows)[1:]))
        return distance * 1000. / (rows[-1][0] - rows[0][0])

    def on_result(self, message):
        self.get_logger().info(
            f'RESULT success={message.success} code={message.code} '
            f'depth={message.final_depth*1000:.3f} mm '
            f'controlled_stop={message.controlled_stop_completed}: {message.message}')

    def report(self):
        now = time.monotonic()
        if self.feedback is None:
            self.get_logger().info('Waiting for force feedback; no active session observed.')
            return
        age = now - self.received
        if age > 1.:
            self.get_logger().info(f'No fresh force feedback for {age:.1f} s; last phase={self.feedback.detail}')
            return
        for rows in (self.clocks, self.depths):
            while len(rows) > 2 and rows[1][0] < now - 5.:
                rows.popleft()
        rtf = ((self.clocks[-1][1] - self.clocks[0][1]) /
               (self.clocks[-1][0] - self.clocks[0][0])) if len(self.clocks) > 1 else 0.
        speed = ((self.depths[-1][1] - self.depths[0][1]) * 1000 /
                 (self.depths[-1][0] - self.depths[0][0])) if len(self.depths) > 1 else 0.
        feedback = self.feedback
        measured = feedback.measured_wrench.force.z
        desired = measured + feedback.force_error.force.z
        preparing = feedback.state in (Insert.Feedback.IDLE, Insert.Feedback.MOVE_PREINSERT,
                                       Insert.Feedback.HOLD_PREINSERT, Insert.Feedback.FT_ZERO)
        reference = 'inactive' if preparing else f'{desired:.3f} N'
        self.get_logger().info(
            f'FORCE {feedback.detail} | axial_gap={max(0., -feedback.depth)*1000:.2f} mm '
            f'| depth={feedback.depth*1000:.2f} mm | Fz={measured:.3f} N | Fz_ref={reference} '
            f'| axial_speed_wall={speed:.3f} mm/s '
            f'| tcp_speed_wall={self.tcp_speed_wall_mm_s(now):.3f} mm/s | RTF={rtf:.3f} '
            f'| wrench_age={feedback.source_age*1000:.1f} ms')


def main():
    rclpy.init()
    node = ForceStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
