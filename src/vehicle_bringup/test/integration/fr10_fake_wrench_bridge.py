#!/usr/bin/env python3
"""Deterministic spring wrench on FakeSystem joints; FK only, no physics step.

Uses the installed MuJoCo model solely as a canonical kinematic evaluator.
Does not publish robot commands or simulate physical contact.
"""
import copy
import math
import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
from fr10_mujoco_bridge import Fr10MujocoPlant, JOINT_NAMES, _matrix_to_quaternion


class FakeWrenchBridge(Node):
    def __init__(self):
        super().__init__('fr10_fake_spring_wrench')
        self.plant = Fr10MujocoPlant()
        self.have_joints = False
        self.contact = None
        self.axis = None
        self.contact_message = None
        self.previous_depth = None
        self.previous_time = None
        self.last_wrench = None
        self.fault = 'normal'
        self.sequence = 0
        self.wrench_pub = self.create_publisher(WrenchStamped, '/ft_sensor/wrench', qos_profile_sensor_data)
        self.actual_pub = self.create_publisher(PoseStamped, '/fr10_fake/tcp_pose', qos_profile_sensor_data)
        self.contact_pub = self.create_publisher(PoseStamped, '/fr10_fake/contact_pose', 1)
        self.health_pub = self.create_publisher(Float64MultiArray, '/fairino_driver/health', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(String, '/fr10_mujoco/wrench_fault', self.on_fault, 1)
        self.create_timer(0.008, self.update)

    def on_fault(self, message):
        self.fault = message.data

    def on_joints(self, message):
        index = {name:i for i,name in enumerate(message.name)}
        if not all(name in index for name in JOINT_NAMES): return
        q = [message.position[index[name]] for name in JOINT_NAMES]
        if not all(math.isfinite(value) for value in q): return
        self.plant.data.qpos[:6] = q
        mujoco.mj_forward(self.plant.model, self.plant.data) # No mj_step/contact physics.
        self.have_joints = True

    def update(self):
        self.sequence += 1
        health = Float64MultiArray()
        health.data = [0.0]*16
        health.data[0], health.data[1], health.data[6] = 1.0, float(self.sequence), 1.0
        self.health_pub.publish(health) # Explicit test-only FakeSystem heartbeat.
        if not self.have_joints: return
        position, rotation = self.plant.tcp_pose()
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'fr10_base_link'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
        qw,qx,qy,qz = _matrix_to_quaternion(rotation)
        pose.pose.orientation.x, pose.pose.orientation.y = qx,qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz,qw
        self.actual_pub.publish(pose)
        if self.contact is None:
            self.axis = rotation[:,2].copy()
            self.contact = position + 0.005*self.axis
            self.contact_message = copy.deepcopy(pose)
            p = self.contact_message.pose.position
            p.x,p.y,p.z = map(float,self.contact)
        self.contact_message.header.stamp = stamp
        self.contact_pub.publish(self.contact_message)
        depth = float(np.dot(position-self.contact,self.axis))
        time_s = stamp.sec + stamp.nanosec*1e-9
        velocity = 0.0 if self.previous_time is None else (depth-self.previous_depth)/max(1e-6,time_s-self.previous_time)
        self.previous_depth,self.previous_time = depth,time_s
        force = max(0.0,2000.0*max(0.0,depth)+80.0*velocity) if depth>0 else 0.0
        tcp_force = rotation.T @ (self.axis*force)
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = 'charging_tcp'
        wrench.wrench.force.x,wrench.wrench.force.y,wrench.wrench.force.z = map(float,tcp_force)
        if self.fault == 'dropout': return
        if self.fault == 'duplicate' and self.last_wrench is not None:
            self.wrench_pub.publish(self.last_wrench)
            return
        if self.fault == 'nan': wrench.wrench.force.z = math.nan
        if self.fault == 'clock_jump':
            wrench.header.stamp.sec = 1
            wrench.header.stamp.nanosec = 0
        if self.fault == 'overload': wrench.wrench.force.z = 1000.0
        if self.fault == 'torque': wrench.wrench.torque.z = 1000.0
        if self.fault == 'lost_contact': wrench.wrench.force.x=wrench.wrench.force.y=wrench.wrench.force.z=0.0
        self.last_wrench = wrench
        self.wrench_pub.publish(wrench)


def main():
    rclpy.init()
    node = FakeWrenchBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
