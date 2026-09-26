"""Isolated simulation acceptance test; never send commands to the user's domain."""
import os, time, math, json, subprocess, sys
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
assert os.environ.get('ROS_DOMAIN_ID')=='77'
assert os.environ.get('GAZEBO_MASTER_URI') in ['http://127.0.0.1:11370','http://127.0.0.1:11371']
base_only='--base-only' in sys.argv
rclpy.init();node=rclpy.create_node('vehicle_acceptance_test')
buffer=Buffer();listener=TransformListener(buffer,node)
seen={};subs=[]
for topic,typ in [('/joint_states',JointState),('/odom',Odometry),('/scan',LaserScan),('/body_camera/d455/image_raw',Image),('/body_camera/d455/depth/image_raw',Image),('/steering_controller/commands',Float64MultiArray),('/drive_controller/commands',Float64MultiArray)]:
    subs.append(node.create_subscription(typ,topic,lambda msg,t=topic:seen.__setitem__(t,msg),qos_profile_sensor_data))
pub=node.create_publisher(Twist,'/cmd_vel',10)
def tick(seconds,cmd=None):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if cmd is not None:pub.publish(cmd)
        rclpy.spin_once(node,timeout_sec=.02)
def pose():
    text=subprocess.check_output(['gz','model','-m','my_mobile_manipulator','-p'],text=True,timeout=10)
    return np.array([float(x) for x in text.strip().split()])
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def odom():
    p=seen['/odom'].pose.pose
    return np.array([p.position.x,p.position.y,yaw(p.orientation)])
def local_delta(before,after):
    d=after-before;c=math.cos(before[2]);s=math.sin(before[2])
    return np.array([c*d[0]+s*d[1],-s*d[0]+c*d[1],math.remainder(d[2],2*math.pi)])
zero=Twist();results=[]
try:
    tick(3,zero)
    assert len(seen)==7,str(seen.keys())
    pairs=[('base_link','roof_frame',[0,0,1]),('roof_frame','lidar_link',[(.4+math.hypot(.21,.05)+.020)/2,0,.0415])]
    if not base_only:pairs += [('roof_frame','robot_base',[0,0,0]),('wrist3_link','plate_frame',[0,0,.106])]
    for a,b,p in pairs:
        t=buffer.lookup_transform(a,b,rclpy.time.Time()).transform
        np.testing.assert_allclose([t.translation.x,t.translation.y,t.translation.z],p,atol=1e-8)
    for topic in ['/body_camera/d455/image_raw','/body_camera/d455/depth/image_raw']:
        msg=seen[topic];assert msg.width==640 and msg.height==480 and len(msg.data)>0
        assert msg.header.frame_id=='d455_depth_optical_frame'
    p=pose();print('Initial Gazebo pose:',p,flush=True)
    # Contact compliance allows small mm-level departure from nominal ground.
    assert abs(p[2])<.01 and max(abs(p[3]),abs(p[4]))<.02
    for label,v in [('forward',[.15,0,0]),('sideways',[0,.15,0]),('rotation',[0,0,.25]),('diagonal',[.10,.10,0])]:
        cmd=Twist();cmd.linear.x,cmd.linear.y,cmd.angular.z=map(float,v)
        tick(2,cmd)  # allow steering alignment before measuring steady motion
        before=odom();g=pose();gb=np.array([g[0],g[1],g[5]])
        tick(3,cmd)
        after=odom();g=pose();ga=np.array([g[0],g[1],g[5]])
        state=seen['/joint_states'];pos=dict(zip(state.name,state.position));vel=dict(zip(state.name,state.velocity))
        vectors=[]
        for i,(x,y) in enumerate([(.4,.35),(.4,-.35),(-.4,.35),(-.4,-.35)],1):
            angle=pos[f'Revolute{i}_to_base_link'];rate=vel[f'Wheel{i}_to_Revolute{i}']
            actual=.210*rate*np.array([math.cos(angle),math.sin(angle)])
            expected=np.array([v[0]-v[2]*y,v[1]+v[2]*x])
            np.testing.assert_allclose(actual,expected,atol=.015,err_msg=label+str(i))
            vectors.append({'angle':angle,'rate':rate,'actual_xy':actual.tolist()})
        od=local_delta(before,after);gd=local_delta(gb,ga)
        print(label,'odom delta',od,'Gazebo delta',gd,flush=True)
        for axis,requested in enumerate(v):
            if requested:assert gd[axis]*requested>0, label+' wrong Gazebo direction'
        # Physics slip is measured, never hidden by command-based odometry.
        assert np.linalg.norm(od[:2]-gd[:2])<.12,label+' translation mismatch'
        assert abs(od[2]-gd[2])<.15,label+' yaw mismatch'
        tw=seen['/odom'].twist.twist
        np.testing.assert_allclose([tw.linear.x,tw.linear.y,tw.angular.z],v,atol=.025)
        results.append({'test':label,'odom_delta':od.tolist(),'gazebo_delta':gd.tolist(),'wheels':vectors})
        tick(1,zero)
    # No further publishes: timeout must stop wheel commands.
    cmd=Twist();cmd.linear.x=.1;tick(.5,cmd);tick(1)
    np.testing.assert_allclose(seen['/drive_controller/commands'].data,0,atol=1e-8)
    print('PASS: four motions, measured wheel vectors/odom/ground truth, timeout, TF, D455',flush=True)
finally:
    tick(.3,zero)
    with open('/tmp/vehicle420-'+('base' if base_only else 'arm')+'-results.json','w') as f:json.dump(results,f,indent=2)
    node.destroy_node();rclpy.shutdown()
