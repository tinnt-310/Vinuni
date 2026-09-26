"""Render a red-over-blue target to verify the D435 image is upright in Gazebo."""
import os,time,math,subprocess,json
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SpawnEntity,DeleteEntity
from tf2_ros import Buffer,TransformListener
from rclpy.qos import qos_profile_sensor_data
assert os.environ.get('ROS_DOMAIN_ID')=='77'
assert os.environ.get('GAZEBO_MASTER_URI')=='http://127.0.0.1:11370'
def rotation(roll,pitch,yaw):
    x,y,z=roll,pitch,yaw
    rx=np.array([[1,0,0],[0,math.cos(x),-math.sin(x)],[0,math.sin(x),math.cos(x)]])
    ry=np.array([[math.cos(y),0,math.sin(y)],[0,1,0],[-math.sin(y),0,math.cos(y)]])
    rz=np.array([[math.cos(z),-math.sin(z),0],[math.sin(z),math.cos(z),0],[0,0,1]])
    return rz@ry@rx
rclpy.init();node=rclpy.create_node('test_d435_upright_render');buf=Buffer();listener=TransformListener(buf,node);images=[]
sub=node.create_subscription(Image,'/arm_camera/arm_camera_sensor/image_raw',lambda m:images.append(m),qos_profile_sensor_data)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
        if len(images)>2:del images[:-2]
spin(2)
t=buf.lookup_transform('base_footprint','camera_depth_optical_frame',rclpy.time.Time()).transform
p=np.array([t.translation.x,t.translation.y,t.translation.z]);q=t.rotation;x,y,z,w=q.x,q.y,q.z,q.w
rt=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
base=np.array([float(v) for v in subprocess.check_output(['gz','model','-m','my_mobile_manipulator','-p'],text=True,timeout=10).split()]);rb=rotation(*base[3:]);rw=rb@rt;world_p=base[:3]+rb@p
assert rw[2,1]<-.98, 'Optical image-down must point down in world at home'
# Target uses WORLD up, independent of optical orientation, so inverted images fail.
forward=rw[:,2];center=world_p+.65*forward
roll=math.atan2(rw[2,1],rw[2,2]);pitch=math.asin(-rw[2,0]);yaw=math.atan2(rw[1,0],rw[0,0])
parts=[]
for name,offset,color in [('red_top',.09,'1 0 0 1'),('blue_bottom',-.09,'0 0 1 1')]:
    at=center+np.array([0,0,offset]);pose=' '.join(map(str,[*at,roll,pitch,yaw]))
    parts.append(f'<link name="{name}"><pose>{pose}</pose><visual name="board"><geometry><box><size>0.28 0.15 0.01</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse><emissive>{color}</emissive></material></visual></link>')
spawn=node.create_client(SpawnEntity,'/spawn_entity');delete=node.create_client(DeleteEntity,'/delete_entity')
assert spawn.wait_for_service(timeout_sec=5)
request=SpawnEntity.Request();request.name='d435_orientation_test';request.xml='<sdf version="1.6"><model name="target"><static>true</static>'+''.join(parts)+'</model></sdf>'
future=spawn.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=10);assert future.result().success
try:
    spin(3);msg=images[-1];assert msg.encoding=='rgb8'
    rgb=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
    red=(rgb[:,:,0]>150)&(rgb[:,:,1]<80)&(rgb[:,:,2]<80)
    blue=(rgb[:,:,2]>150)&(rgb[:,:,0]<80)&(rgb[:,:,1]<80)
    assert red.sum()>500 and blue.sum()>500, 'Target not visible'
    red_y=float(np.nonzero(red)[0].mean());blue_y=float(np.nonzero(blue)[0].mean())
    assert red_y+50<blue_y, 'Image vertically inverted'
    print('PASS: actual RGB upright; red-top mean row',red_y,'blue-bottom mean row',blue_y,flush=True)
    with open('/tmp/d435-upright-result.json','w') as f:json.dump({'red_top_row':red_y,'blue_bottom_row':blue_y,'width':msg.width,'height':msg.height},f)
finally:
    req=DeleteEntity.Request();req.name=request.name
    future=delete.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
    node.destroy_node();rclpy.shutdown()
