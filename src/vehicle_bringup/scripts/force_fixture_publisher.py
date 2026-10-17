#!/usr/bin/env python3
"""Publish the planning-scene proxy for the standalone Gazebo force fixture.

The real collision geometry remains in the selected force-test world.  This
small launch-owned publisher gives the offline planning worker the same rear
plate proxy used by the deterministic force tests; it is never started by the
normal vehicle or hardware bringup.
"""

import signal

import rclpy
from moveit_msgs.msg import CollisionObject
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class ForceFixturePublisher(Node):
    def __init__(self):
        super().__init__('fr10_force_fixture_publisher')
        self.declare_parameter('fixture_profile', 'home')
        profile = str(self.get_parameter('fixture_profile').value).lower()
        if profile == 'socket':
            # Grounded mating frame z=0.0938. The rear collision plate is
            # 55 mm along local +Z, which points down in the base frame.
            self.proxy_position = (0.55, 0.0, 0.0388)
            # Keep the planning proxy in the same charging frame as the
            # socket visual and the FR10 tool (rpy=-pi,0,-pi/2).
            self.proxy_orientation = (-0.7071067811865476,
                                      0.7071067811865476,
                                      0.0, 0.0)
            self.proxy_dimensions = (0.18, 0.20, 0.01)
        else:
            # fr10_force_home_test.world keeps the home XY/orientation but
            # lowers the CAD rear face to ground: mating-frame z=0.093799987.
            # The plate is 5 mm along connector +Z.  At the canonical home
            # orientation +Z points down in the base frame.
            self.proxy_position = (-0.789999032108, -0.159000854021, 0.088799987)
            self.proxy_orientation = (-0.707105281178, 0.707108281186,
                                      -1.66533e-16, 2.99999483e-6)
            self.proxy_dimensions = (0.30, 0.30, 0.01)
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        # Every CollisionObject update increments the planning-scene revision.
        # Re-publishing the same proxy during insertion would invalidate the
        # validated corridor forever.  Wait for MoveIt's subscriber, publish
        # one immutable proxy, then stop the timer.
        self.timer = self.create_timer(0.25, self.publish_when_ready)

    def publish_when_ready(self):
        if self.publisher.get_subscription_count() == 0:
            return
        self.publish_fixture()
        self.timer.cancel()

    def publish_fixture(self):
        object_message = CollisionObject()
        object_message.header.frame_id = 'fr10_base_link'
        object_message.id = 'force_contact_proxy'
        object_message.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # Keep the planning proxy identical to the selected fixture profile;
        # the home profile intentionally uses the 300 mm deterministic plate.
        box.dimensions = list(self.proxy_dimensions)
        pose = Pose()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = self.proxy_orientation
        pose.position.x, pose.position.y, pose.position.z = self.proxy_position
        object_message.primitives = [box]
        object_message.primitive_poses = [pose]
        self.publisher.publish(object_message)


def main():
    rclpy.init()
    node = ForceFixturePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
