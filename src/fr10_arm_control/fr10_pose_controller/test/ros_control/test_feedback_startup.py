#!/usr/bin/env python3
"""ROS transport regression, isolated DDS domain, no actuator or SDK process.

Clock discovery precedes delayed mock joint feedback. Initialization must wait;
loss of that same feedback after initialization must still stop the runtime.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray


class FeedbackStartupTest(unittest.TestCase):
    def test_delayed_initial_feedback_then_initialized_dropout(self):
        os.environ['ROS_DOMAIN_ID']='195'
        os.environ['RMW_LOCALHOST_ONLY']='1'
        rclpy.init()
        node=rclpy.create_node('delayed_initial_feedback_fixture')
        clock_pub=node.create_publisher(Clock,'/clock',10)
        joint_pub=node.create_publisher(JointState,'/joint_states',10)
        statuses=[]
        node.create_subscription(String,'/controller_status',lambda msg:statuses.append(msg.data),10)
        node.create_subscription(Float64MultiArray,'/fairino10_controller/commands',lambda msg:None,10)
        enabled=False;last_joint_ns=0;anchor=time.monotonic()

        def publish_measurement():
            nonlocal last_joint_ns
            source_ns=1_000_000_000+int((time.monotonic()-anchor)/.008)*8_000_000
            clock=Clock();clock.clock.sec,clock.clock.nanosec=divmod(source_ns,1_000_000_000)
            clock_pub.publish(clock)
            if enabled and source_ns>last_joint_ns:
                sample=JointState();sample.header.stamp=clock.clock
                sample.name=[f'j{i}' for i in range(1,7)]
                sample.position=[0.,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2,0.]
                sample.velocity=[0.]*6;sample.effort=[0.]*6
                joint_pub.publish(sample);last_joint_ns=source_ns

        node.create_timer(.002,publish_measurement)
        with tempfile.TemporaryDirectory(prefix='fr10_feedback_startup_') as directory:
            log=Path(directory)/'runtime.log'
            env=dict(os.environ,ROS_LOG_DIR=str(Path(directory)/'ros_log'))
            with log.open('w') as stream:
                process=subprocess.Popen(['ros2','launch','fr10_pose_controller','fr10_pose_controller.launch.py',
                    'start_robot_stack:=false','use_gazebo:=true','use_rviz:=false'],
                    env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
                def wait_until(predicate,timeout):
                    deadline=time.monotonic()+timeout
                    while time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.002)
                        if predicate():return True
                    return False
                try:
                    self.assertTrue(wait_until(lambda:'Waiting for the first complete ros_control' in log.read_text(),10),log.read_text())
                    wait_until(lambda:False,.25) # Five times the configured 50 ms timeout.
                    self.assertIsNone(process.poll(),log.read_text())
                    self.assertNotIn('SHUTDOWN: COMM_FAILURE',log.read_text())
                    enabled=True
                    self.assertTrue(wait_until(lambda:any('backend_ready=true' in s for s in statuses),5),log.read_text())
                    self.assertNotIn('SHUTDOWN: COMM_FAILURE',log.read_text())
                    enabled=False
                    self.assertTrue(wait_until(lambda:'SHUTDOWN: COMM_FAILURE' in log.read_text(),3),
                                    'Initialized feedback watchdog failed to stop\n'+log.read_text())
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid,signal.SIGINT)
                        try:process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
                    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
