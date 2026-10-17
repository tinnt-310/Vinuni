#!/usr/bin/env python3
"""Exact CAD scene/frames from the same launch-resolved station as Gazebo."""
import json
import math
import signal
from pathlib import Path
import struct
import sys

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from geometry_msgs.msg import Point, Pose, TransformStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformBroadcaster
from nav_msgs.msg import Odometry
from fr10_motion_interfaces.msg import VehicleParkingState

sys.path.insert(0, str(Path(get_package_share_directory('vehicle_bringup'))/'launch'))
from charging_station import rotate, multiply, rpy_matrix, matrix_rpy, rpy_quaternion


def quaternion_matrix(q):
    x,y,z,w = q.x,q.y,q.z,q.w
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if norm < 1e-12: raise ValueError('Invalid frame quaternion')
    x,y,z,w = x/norm,y/norm,z/norm,w/norm
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def cad_mesh(path):
    data = Path(path).read_bytes(); count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84+50*count: raise ValueError(f'Invalid binary CAD mesh {path}')
    mesh = Mesh()
    for index in range(count):
        vertices = struct.unpack_from('<9f', data, 84+50*index+12)
        face = MeshTriangle(); face.vertex_indices = [len(mesh.vertices)+i for i in range(3)]
        mesh.triangles.append(face)
        for i in range(3):
            point = Point(); point.x,point.y,point.z = vertices[3*i:3*i+3]; mesh.vertices.append(point)
    return mesh


class MatchedStationPublisher(Node):
    def __init__(self):
        super().__init__('matched_station_publisher')
        self.declare_parameter('station_file', '')
        self.station = json.loads(Path(self.get_parameter('station_file').value).read_text())
        self.parent = 'world'
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.base_broadcaster = TransformBroadcaster(self)
        self.last_base_ns = 0
        if self.station['mounted']:
            self.create_subscription(Odometry, '/fr10_sim/base_odometry', self.base_callback, 10)
        self.initial_published = False; self.park_refresh_published = False
        model = Path(self.station['model_path'])
        self.meshes = [('charging_port_mating_surfaces', cad_mesh(model/'socket_contact_surface.stl')),
                       ('charging_port_protected_rear', cad_mesh(model/'socket_protected_rear.stl'))]
        self.create_subscription(VehicleParkingState, '/vehicle/parking_state', self.parking_callback, 10)
        self.timer = self.create_timer(.25, self.publish_when_ready)
        self.publish_frames()

    def base_callback(self, message):
        if message.header.frame_id != 'world' or message.child_frame_id != 'base_footprint': return
        ns=message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        # Broadcast at the arm's source rate, with the actual sensor timestamp.
        if ns <= self.last_base_ns or ns-self.last_base_ns < 8000000: return
        pose=message.pose.pose
        values=[pose.position.x,pose.position.y,pose.position.z,
                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[3:])<1e-12: return
        transform=TransformStamped();transform.header=message.header
        transform.child_frame_id=message.child_frame_id
        transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=values[:3]
        transform.transform.rotation=pose.orientation
        self.base_broadcaster.sendTransform(transform)
        self.last_base_ns=ns

    def publish_frames(self):
        position = self.station['world_mouth_position_m']; rotation = rpy_matrix(self.station['world_mouth_rpy'])
        quaternion = rpy_quaternion(self.station['world_mouth_rpy']); axis = rotate(rotation, [0,0,1])
        frames = []
        for child, depth in [('charging_port_mating', 0),
                             ('charging_port_preinsert', -self.station['preinsert_distance_m']),
                             ('charging_port_seated', self.station['insertion_depth_m'])]:
            transform = TransformStamped(); transform.header.frame_id = self.parent
            transform.header.stamp = self.get_clock().now().to_msg(); transform.child_frame_id = child
            xyz = [position[i]+axis[i]*depth for i in range(3)]
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
            transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = quaternion
            frames.append(transform)
        self.broadcaster.sendTransform(frames)

    def publish_when_ready(self):
        if not self.initial_published and self.publisher.get_subscription_count():
            self.initial_published = self.publish_scene()

    def parking_callback(self, message):
        if message.owner_id == 0: self.park_refresh_published = False
        elif message.state == VehicleParkingState.STATIONARY and not self.park_refresh_published:
            # Snapshot the actual TF boundary only when the vehicle controller
            # acknowledges its hold; never republish/revise a running corridor.
            self.park_refresh_published = self.publish_scene()

    def publish_scene(self):
        if not self.tf_buffer.can_transform('fr10_base_link',self.parent,Time()): return False
        position=self.station['world_mouth_position_m']
        quaternion=rpy_quaternion(self.station['world_mouth_rpy'])
        for name, mesh in self.meshes:
            message = CollisionObject(); message.header.frame_id = self.parent
            message.header.stamp = self.get_clock().now().to_msg(); message.id = name; message.operation = CollisionObject.ADD
            pose = Pose(); pose.position.x,pose.position.y,pose.position.z = position
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = quaternion
            message.pose.orientation.w=1.; message.meshes = [mesh]; message.mesh_poses = [pose]; self.publisher.publish(message)
        for primitive in self.station['world_primitives']:
            message = CollisionObject(); message.header.frame_id = self.parent
            message.header.stamp = self.get_clock().now().to_msg()
            message.id = primitive['id']; message.operation = CollisionObject.ADD
            shape = SolidPrimitive(); shape.type = SolidPrimitive.BOX; shape.dimensions = primitive['size_m']
            xyz = primitive['position_world_m']
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = xyz
            pose.orientation.w = 1.
            message.pose.orientation.w=1.; message.primitives = [shape]; message.primitive_poses = [pose]; self.publisher.publish(message)
        self.get_logger().info('CAD collision snapshot published: mating surfaces + independently protected rear')
        return True


def main():
    rclpy.init(); node = MatchedStationPublisher()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        # launch can deliver SIGINT twice (parent propagation + direct child).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
