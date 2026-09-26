"""Offline Gazebo-only regression client. Launch in dedicated ROS_DOMAIN_ID=93."""
import os,time,json,math
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
assert os.environ.get('ROS_DOMAIN_ID')=='93'
rclpy.init();n=rclpy.create_node('ode_repro_client');samples=[]
sub=n.create_subscription(JointState,'/joint_states',lambda m:samples.append({'t':time.monotonic(),'names':list(m.name),'p':list(m.position),'v':list(m.velocity),'e':list(m.effort)}),10)
c=ActionClient(n,FollowJointTrajectory,'/joint_trajectory_controller/follow_joint_trajectory')
q=[0.,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2,0.]
result={'home':q,'target':[.15,*q[1:5],.3]}
def wait(f,timeout):
 end=time.monotonic()+timeout
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if not f.done():raise TimeoutError('action timeout')
 return f.result()
try:
 assert c.wait_for_server(timeout_sec=25),'no action server'
 end=time.monotonic()+2
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 result['initial']=samples[-1] if samples else None
 goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=[f'j{i}' for i in range(1,7)]
 p=JointTrajectoryPoint();p.positions=result['target'];p.velocities=[0.]*6;p.time_from_start.sec=4;goal.trajectory.points=[p]
 handle=wait(c.send_goal_async(goal),10);assert handle.accepted,'rejected'
 res=wait(handle.get_result_async(),20);result.update(status=res.status,error_code=res.result.error_code)
 end=time.monotonic()+3
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 result['final']=samples[-1] if samples else None
except Exception as e:result['failure']=str(e)
result['finite']=all(math.isfinite(x) for s in samples for k in ['p','v','e'] for x in s[k])
Path(os.environ['ODE_OUT']+'/result.json').write_text(json.dumps(result,indent=2))
Path(os.environ['ODE_OUT']+'/joints.json').write_text(json.dumps(samples))
print(json.dumps(result));n.destroy_node();rclpy.shutdown()
