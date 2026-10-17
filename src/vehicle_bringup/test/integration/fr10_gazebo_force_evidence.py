#!/usr/bin/env python3
"""Independent Gazebo link-state TCP evidence; no command publisher."""
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import LinkStates
from rosgraph_msgs.msg import Clock
import numpy as np
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from fr10_model_evidence import fixed_link_transform


def main():
    rclpy.init()
    node = rclpy.create_node('fr10_gazebo_force_evidence')
    actual_pub = node.create_publisher(PoseStamped, '/fr10_gazebo/tcp_pose', qos_profile_sensor_data)
    contact_pub = node.create_publisher(PoseStamped, '/fr10_gazebo/contact_pose', 1)
    stamp = [None]
    tool = [None]
    contact = [None]
    node.create_subscription(Clock, '/clock', lambda m: stamp.__setitem__(0, m.clock), qos_profile_sensor_data)
    description_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def description_callback(message):
        import xml.etree.ElementTree as ET
        tool[0] = fixed_link_transform(ET.fromstring(message.data), 'wrist3_link', 'charging_tcp')

    node.create_subscription(String, '/robot_description', description_callback, description_qos)

    def callback(message):
        if stamp[0] is None or tool[0] is None: return
        index = next((i for i, name in enumerate(message.name) if name.endswith('::wrist3_link')), None)
        if index is None: return
        pose = message.pose[index]
        q = pose.orientation
        x,y,z,w = q.x,q.y,q.z,q.w
        rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                             [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                             [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        position = np.array([pose.position.x,pose.position.y,pose.position.z]) + rotation @ tool[0][:3, 3]
        orientation = Rotation.from_matrix(rotation @ tool[0][:3, :3]).as_quat()
        out = PoseStamped()
        out.header.stamp = stamp[0]
        out.header.frame_id = 'fr10_base_link'
        out.pose.position.x, out.pose.position.y, out.pose.position.z = position.tolist()
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = orientation.tolist()
        actual_pub.publish(out)
        if contact[0] is None:
            plate_index = next((i for i, name in enumerate(message.name)
                                if name.endswith('force_contact_proxy::plate')), None)
            if plate_index is not None:
                plate = message.pose[plate_index]
                pq = plate.orientation
                pr = Rotation.from_quat([pq.x, pq.y, pq.z, pq.w])
                front = np.array([plate.position.x, plate.position.y, plate.position.z]) + pr.apply([0, 0, -.005])
                target = PoseStamped()
                target.pose.position.x, target.pose.position.y, target.pose.position.z = front.tolist()
                target.pose.orientation = plate.orientation
                contact[0] = target
        if contact[0] is not None:
            contact[0].header = out.header
            contact_pub.publish(contact[0])
    node.create_subscription(LinkStates, '/gazebo/link_states', callback, qos_profile_sensor_data)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
