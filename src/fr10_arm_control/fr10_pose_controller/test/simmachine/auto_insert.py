#!/usr/bin/env python3
"""SimMachine helper: approach a pre-insert pose, then insert along tool +Z."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class AutoInsert(Node):
    def __init__(self):
        super().__init__("fr10_simmachine_auto_insert")
        self.distance = float(self.declare_parameter("pre_distance_m", 0.20).value)
        self.position_tolerance = 0.003
        self.angular_tolerance = math.radians(2.0)
        self.phase, self.status = "idle", ""
        self.pre_pose = self.final_pose = None

        self.command_pub = self.create_publisher(
            PoseStamped, "/charging_port/raw_pose", qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/charging_port/insert_pose", self.target_cb, 10)
        self.create_subscription(
            PoseStamped, "/fr10_motion/tcp_pose", self.tcp_cb,
            qos_profile_sensor_data)
        self.create_subscription(String, "/controller_status", self.status_cb, 10)
        self.get_logger().info(
            "Ready: publish the final socket pose to /charging_port/insert_pose")

    @staticmethod
    def make_pose(position, quaternion):
        result = PoseStamped()
        result.header.frame_id = "base_link"
        result.pose.position.x, result.pose.position.y, result.pose.position.z = position
        (result.pose.orientation.x, result.pose.orientation.y,
         result.pose.orientation.z, result.pose.orientation.w) = quaternion
        return result

    def target_cb(self, message):
        if self.phase != "idle":
            self.get_logger().warning("Insertion already running; target ignored")
            return
        if message.header.frame_id != "base_link":
            self.get_logger().error("Target frame must be base_link")
            return

        q = [message.pose.orientation.x, message.pose.orientation.y,
             message.pose.orientation.z, message.pose.orientation.w]
        norm = math.sqrt(sum(value * value for value in q))
        if not math.isfinite(norm) or norm < 1e-9:
            self.get_logger().error("Target quaternion is invalid")
            return
        q = [value / norm for value in q]
        x, y, z, w = q
        axis = [2.0 * (x * z + y * w),
                2.0 * (y * z - x * w),
                1.0 - 2.0 * (x * x + y * y)]
        final = [message.pose.position.x, message.pose.position.y,
                 message.pose.position.z]
        if not all(math.isfinite(value) for value in final):
            self.get_logger().error("Target position is invalid")
            return
        pre = [final[i] - self.distance * axis[i] for i in range(3)]

        self.pre_pose = self.make_pose(pre, q)
        self.final_pose = self.make_pose(final, q)
        self.phase = "approach"
        self.send(self.pre_pose)
        self.get_logger().info(
            f"APPROACH -> pre-insert [{pre[0]:.4f}, {pre[1]:.4f}, {pre[2]:.4f}]")

    def status_cb(self, message):
        self.status = message.data
        if self.phase != "idle" and ("state=FAULT" in self.status or
                                     "state=STOPPED" in self.status):
            self.get_logger().error("Controller stopped; insertion aborted")
            self.phase = "idle"

    def tcp_cb(self, actual):
        if "state=HOLDING" not in self.status:
            return
        if self.phase == "approach" and self.reached(actual, self.pre_pose):
            self.phase = "insert"
            self.send(self.final_pose)
            self.get_logger().info("PRE-INSERT reached -> straight INSERT")
        elif self.phase == "insert" and self.reached(actual, self.final_pose):
            self.phase = "idle"
            self.get_logger().info("INSERT complete -> HOLDING")

    def send(self, target):
        target.header.stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(target)

    def reached(self, actual, target):
        if target is None:
            return False
        pa, pt = actual.pose.position, target.pose.position
        position_error = math.sqrt(
            (pa.x - pt.x) ** 2 + (pa.y - pt.y) ** 2 + (pa.z - pt.z) ** 2)
        qa = actual.pose.orientation
        qt = target.pose.orientation
        dot = abs(qa.x * qt.x + qa.y * qt.y + qa.z * qt.z + qa.w * qt.w)
        angular_error = 2.0 * math.acos(min(1.0, dot))
        return (position_error <= self.position_tolerance and
                angular_error <= self.angular_tolerance)


def main():
    rclpy.init()
    node = AutoInsert()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
