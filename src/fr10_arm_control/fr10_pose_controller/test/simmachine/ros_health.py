#!/usr/bin/env python3
"""rclpy collector/publisher used by the progressive HIL runner."""

from __future__ import annotations

import csv
import math
import threading
import time
from typing import Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, String
from tf2_ros import Buffer, TransformListener

from metrics import max_abs_error, position_error, quaternion_error


class RuntimeProbe(Node):
    def __init__(self):
        super().__init__("fr10_automated_test_probe")
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        transient = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        sensor = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, reliable)
        self.create_subscription(PoseStamped, "/fr10_motion/tcp_pose", self._tcp_cb, sensor)
        self.create_subscription(String, "/controller_status", self._status_cb, transient)
        self.create_subscription(
            Float64MultiArray,
            "/fairino10_controller/commands",
            self._command_cb,
            reliable,
        )
        self.joint_pub = self.create_publisher(JointState, "/joint_command", reliable)
        self.pose_pub = self.create_publisher(PoseStamped, "/charging_port/raw_pose", 10)
        self.retract_pub = self.create_publisher(
            Empty, "/fr10_motion/retract", reliable)
        self.controlled_stop_pub = self.create_publisher(
            Empty, "/fr10_motion/controlled_stop", reliable)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.joint_stamps = []
        self.joint_history = []
        self.tcp_history = []
        self.status_history = []
        self.latest_joint: Optional[JointState] = None
        self.latest_tcp: Optional[PoseStamped] = None
        self.latest_status = ""
        self.latest_command: Optional[Float64MultiArray] = None
        self.command_count = 0
        self.command_history = []
        self._joint_layout = ()
        self._joint_index_map = ()
        self.joint_velocity_source = "unavailable"

    def _joint_cb(self, message):
        layout = tuple(message.name)
        if layout != self._joint_layout:
            try:
                self._joint_index_map = tuple(
                    layout.index(name)
                    for name in ("j1", "j2", "j3", "j4", "j5", "j6")
                )
            except ValueError:
                return
            self._joint_layout = layout
        if any(index >= len(message.position)
               for index in self._joint_index_map):
            return

        position = [
            message.position[index] for index in self._joint_index_map
        ]
        stamp = time.monotonic()
        reported_velocity = all(
            index < len(message.velocity) and
            math.isfinite(message.velocity[index])
            for index in self._joint_index_map)
        with self.lock:
            previous = self.joint_history[-1] if self.joint_history else None
        if reported_velocity:
            velocity = [message.velocity[index]
                        for index in self._joint_index_map]
            velocity_source = "reported"
        elif previous is not None and stamp > previous[0]:
            dt = stamp - previous[0]
            velocity = [
                (position[index] - previous[1][index]) / dt
                for index in range(6)
            ]
            velocity_source = "finite_difference"
        else:
            velocity = [0.0] * 6
            velocity_source = "unavailable"

        canonical = JointState()
        canonical.header = message.header
        canonical.name = ["j1", "j2", "j3", "j4", "j5", "j6"]
        canonical.position = position
        canonical.velocity = velocity
        canonical.effort = [
            message.effort[index]
            if index < len(message.effort) and math.isfinite(message.effort[index])
            else 0.0
            for index in self._joint_index_map
        ]
        with self.lock:
            self.joint_velocity_source = velocity_source
            self.latest_joint = canonical
            self.joint_stamps.append(stamp)
            self.joint_history.append((stamp, list(canonical.position),
                                       list(canonical.velocity)))

    def _tcp_cb(self, message):
        stamp = time.monotonic()
        pose = message.pose
        with self.lock:
            self.latest_tcp = message
            self.tcp_history.append((stamp,
                                     [pose.position.x, pose.position.y, pose.position.z],
                                     [pose.orientation.x, pose.orientation.y,
                                      pose.orientation.z, pose.orientation.w]))

    def _status_cb(self, message):
        with self.lock:
            self.latest_status = message.data
            self.status_history.append((time.monotonic(), message.data))

    def _command_cb(self, message):
        with self.lock:
            self.latest_command = message
            self.command_count += 1
            if len(message.data) == 6:
                self.command_history.append(
                    (time.monotonic(), list(message.data)))

    def wait_ready(self, timeout: float = 15.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if (self.latest_joint is not None and self.latest_tcp is not None
                        and "backend_ready=true" in self.latest_status):
                    return True
            time.sleep(0.02)
        return False

    def joint_position(self):
        with self.lock:
            return list(self.latest_joint.position) if self.latest_joint else None

    def joint_velocity(self):
        with self.lock:
            return list(self.latest_joint.velocity) if self.latest_joint else None

    def tcp_pose(self):
        with self.lock:
            if not self.latest_tcp:
                return None
            pose = self.latest_tcp.pose
            return ([pose.position.x, pose.position.y, pose.position.z],
                    [pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w])

    def status(self):
        with self.lock:
            return self.latest_status

    def command_state(self):
        with self.lock:
            if self.latest_command is None:
                return None
            return {
                "count": self.command_count,
                "positions": list(self.latest_command.data),
            }

    def publish_joint(self, positions: Sequence[float]):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["j1", "j2", "j3", "j4", "j5", "j6"]
        # The production configuration accepts explicit degree input while
        # /joint_states and all test calculations remain REP-103 radians.
        message.position = [math.degrees(value) for value in positions]
        self.joint_pub.publish(message)

    def publish_pose(self, position: Sequence[float], quaternion: Sequence[float]):
        self.publish_pose_message(position, quaternion, "base_link", True)

    def publish_pose_message(self, position: Sequence[float],
                             quaternion: Sequence[float], frame_id: str,
                             stamp_now: bool = True):
        message = PoseStamped()
        if stamp_now:
            message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.pose.position.x, message.pose.position.y, message.pose.position.z = position
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = quaternion
        self.pose_pub.publish(message)

    def publish_controlled_stop(self):
        self.controlled_stop_pub.publish(Empty())

    def publish_retract(self):
        self.retract_pub.publish(Empty())

    def wait_joint(self, target, tolerance=0.002, timeout=30.0):
        deadline = time.monotonic() + timeout
        stable = 0
        observed_count = -1
        while time.monotonic() < deadline:
            with self.lock:
                count = len(self.joint_history)
                actual = list(self.latest_joint.position) if self.latest_joint else None
            if count == observed_count:
                time.sleep(0.005)
                continue
            observed_count = count
            if actual and max_abs_error(actual, target) <= tolerance:
                stable += 1
                if stable >= 5:
                    return True
            else:
                stable = 0
            time.sleep(0.02)
        return False

    def wait_pose(self, position, quaternion, position_tolerance=0.002,
                  angular_tolerance=0.02, timeout=30.0,
                  republish_period_s=None):
        deadline = time.monotonic() + timeout
        next_publish = time.monotonic()
        stable = 0
        observed_count = -1
        while time.monotonic() < deadline:
            if (republish_period_s is not None
                    and time.monotonic() >= next_publish):
                self.publish_pose(position, quaternion)
                next_publish = time.monotonic() + republish_period_s
            with self.lock:
                count = len(self.tcp_history)
                if self.latest_tcp:
                    pose = self.latest_tcp.pose
                    actual = ([pose.position.x, pose.position.y, pose.position.z],
                              [pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w])
                else:
                    actual = None
            if count == observed_count:
                time.sleep(0.005)
                continue
            observed_count = count
            if actual and position_error(actual[0], position) <= position_tolerance and \
                    quaternion_error(actual[1], quaternion) <= angular_tolerance:
                stable += 1
                if stable >= 5:
                    return True
            else:
                stable = 0
            time.sleep(0.02)
        return False

    def tf_connected(self) -> bool:
        try:
            self.tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
            return True
        except Exception:
            return False

    def write_joint_csv(self, path, start_index=0):
        with self.lock:
            rows = list(self.joint_history[start_index:])
        with open(path, "w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["monotonic_s"] + [f"q{i+1}_rad" for i in range(6)] +
                            [f"qd{i+1}_rad_s" for i in range(6)])
            for stamp, positions, velocities in rows:
                writer.writerow([stamp] + positions + velocities)


class ProbeContext:
    def __init__(self, domain_id: int):
        self.domain_id = domain_id
        self.node = None
        self.executor = None
        self.thread = None

    def __enter__(self):
        import os
        os.environ["ROS_DOMAIN_ID"] = str(self.domain_id)
        rclpy.init()
        self.node = RuntimeProbe()
        from rclpy.executors import MultiThreadedExecutor
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        return self.node

    def __exit__(self, exc_type, exc, tb):
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=3.0)
        if self.thread is not None:
            self.thread.join(timeout=3.0)
        if self.executor is not None and self.node is not None:
            self.executor.remove_node(self.node)
        if self.node is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
