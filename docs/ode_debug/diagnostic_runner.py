import subprocess,os,sys,time,signal,json
from pathlib import Path
case=sys.argv[1];out=Path('/home/tin/Vinuni/docs/ode_debug')/case;out.mkdir(exist_ok=True)
e=os.environ.copy();e.update(ROS_DOMAIN_ID='93',GAZEBO_MASTER_URI='http://127.0.0.1:11393',ODE_CASE=case,ODE_OUT=str(out))
with (out/'gazebo.log').open('w') as log:
 p=subprocess.Popen(['ros2','launch','/tmp/odecase.launch.py','enable_arm:=true','enable_vehicle:='+os.environ.get('ODE_VEHICLE','false'),'gui:=false','rviz:=false','gazebo_port:=11393'],env=e,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  r=subprocess.run(['python3','tools/ode_repro_client.py'],env=e,capture_output=True,text=True,timeout=65)
  (out/'client.log').write_text(r.stdout+r.stderr)
 finally:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
s=(out/'gazebo.log').read_text();print(case,'LCP',s.count('LCP internal error'),(out/'result.json').read_text() if (out/'result.json').exists() else 'NO RESULT')
