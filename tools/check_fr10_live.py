"""Read-only assertions against the isolated Gazebo verification session."""
import os
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from tf2_ros import Buffer, TransformListener

assert os.environ.get('ROS_DOMAIN_ID') == '77', 'Use isolated test ROS_DOMAIN_ID=77'
rclpy.init()
node=rclpy.create_node('check_fr10_live')
buf=Buffer()
listener=TransformListener(buf,node)
seen={}
subs=[]
for topic,typ in [('/joint_states',JointState),('/arm_camera/arm_camera_sensor/image_raw',Image),('/arm_camera/arm_camera_sensor/depth/image_raw',Image)]:
    subs.append(node.create_subscription(typ,topic,lambda msg,t=topic: seen.__setitem__(t,msg),qos_profile_sensor_data))
deadline=time.monotonic()+15
pairs=[('wrist3_link','plate_frame'),('plate_frame','charging_connector_link'),('plate_frame','camera_link'),('plate_frame','camera_depth_optical_frame')]
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.1)
    if len(seen)==3 and all(buf.can_transform(a,b,rclpy.time.Time()) for a,b in pairs):
        break
assert len(seen)==3, 'Missing streams: '+str(seen.keys())
for a,b in pairs:
    t=buf.lookup_transform(a,b,rclpy.time.Time()).transform
    p=np.array([t.translation.x,t.translation.y,t.translation.z])
    print(a,'->',b,p,'quaternion',t.rotation)
    if b=='plate_frame': np.testing.assert_allclose(p,[0,0,.106],atol=1e-8)
    if b=='charging_connector_link':
        np.testing.assert_allclose(p,[0,.09,.09],atol=1e-8)
        np.testing.assert_allclose(t.rotation.z,0,atol=1e-8)
        np.testing.assert_allclose(abs(t.rotation.w),1,atol=1e-8)
    if b=='camera_link': np.testing.assert_allclose(p,[.01745705,-.0125,.0507547],atol=1e-8)
    if b=='camera_depth_optical_frame':
        x,y,z,w=t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w
        np.testing.assert_allclose([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)],[0,0,1],atol=1e-8)
joint=seen['/joint_states']
positions=dict(zip(joint.name,joint.position))
assert all('j'+str(i) in positions for i in range(1,7))
print('FR10 actual positions:',{k:positions[k] for k in ['j1','j2','j3','j4','j5','j6']})
for topic,msg in seen.items():
    if isinstance(msg,Image):
        assert msg.width==1280 and msg.height==720 and len(msg.data)>0
        assert msg.header.frame_id=='camera_depth_optical_frame'
        print(topic,msg.width,msg.height,msg.encoding,'bytes',len(msg.data))
print('PASS: live TF and RGB/depth streams')
node.destroy_node()
rclpy.shutdown()
