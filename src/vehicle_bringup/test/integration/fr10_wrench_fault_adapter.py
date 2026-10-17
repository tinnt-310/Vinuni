#!/usr/bin/env python3
"""Test-only, backend-independent WrenchStamped relay; never commands joints."""
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String, UInt8


class FaultAdapter(Node):
    def __init__(self):
        super().__init__('fr10_test_wrench_fault_adapter')
        self.fault = 'normal'
        self.last = None
        self.baseline = None
        self.state = 0
        self.pub = self.create_publisher(WrenchStamped, '/ft_sensor/wrench', qos_profile_sensor_data)
        self.create_subscription(WrenchStamped, '/ft_sensor/raw_wrench', self.on_sample, qos_profile_sensor_data)
        self.create_subscription(String, '/fr10_mujoco/wrench_fault', self.on_fault, 1)
        self.create_subscription(UInt8, '/fr10_force/state', self.on_state, 1)

    def on_fault(self, message):
        self.fault = message.data

    def on_state(self, message):
        self.state = message.data

    def on_sample(self, sample):
        if self.state == 3: # FT_ZERO: fixed reference from the actual source.
            self.baseline = copy.deepcopy(sample.wrench)
        if self.fault == 'dropout':
            return
        if self.fault == 'duplicate' and self.last is not None:
            self.pub.publish(self.last)
            return
        message = copy.deepcopy(sample)
        if self.fault == 'nan':
            message.wrench.force.z = math.nan
        elif self.fault == 'overload':
            message.wrench.force.z = 1000.0
        elif self.fault == 'torque':
            message.wrench.torque.z = 1000.0
        elif self.fault == 'lost_contact' and self.baseline is not None:
            message.wrench = copy.deepcopy(self.baseline)
        self.last = message
        self.pub.publish(message)


def main():
    rclpy.init()
    node = FaultAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # Humble may invalidate a pending take_message handle during SIGINT.
        # Suppress only after ROS shutdown, never a fault in a live relay.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
