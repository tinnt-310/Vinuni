"""Run after sourcing install/setup.bash. No real hardware.
Usage: python3 src/vehicle_bringup/test/integration/run_ode_regression.py LABEL; ODE_VEHICLE=true for vehicle.
"""
import subprocess,os,sys,time,signal,json
from pathlib import Path
workspace_root=Path(__file__).resolve().parents[4];os.chdir(workspace_root)
case=sys.argv[1];out=workspace_root/'docs'/'ode_debug'/case;out.mkdir(parents=True,exist_ok=True)
source=Path('src/vehicle_bringup/launch/simulation.launch.py').read_text()
source=source.replace('    # Source Xacros retain package://.', "    from pathlib import Path\n    Path(os.environ['ODE_OUT']+'/model.urdf').write_text(doc.toxml())\n    # Source Xacros retain package://.")
launch_path=out/'repro.launch.py';launch_path.write_text(source)
e=os.environ.copy();e.update(ROS_DOMAIN_ID='93',GAZEBO_MASTER_URI='http://127.0.0.1:11393',ODE_CASE=case,ODE_OUT=str(out))
with (out/'gazebo.log').open('w') as log:
 p=subprocess.Popen(['ros2','launch',str(launch_path),'enable_arm:=true','enable_vehicle:='+os.environ.get('ODE_VEHICLE','false'),'gui:=false','rviz:=false','gazebo_port:=11393'],env=e,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  r=subprocess.run(['python3','src/vehicle_bringup/test/integration/ode_repro_client.py'],env=e,capture_output=True,text=True,timeout=65)
  (out/'client.log').write_text(r.stdout+r.stderr)
 finally:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
s=(out/'gazebo.log').read_text();print(case,'LCP',s.count('LCP internal error'),(out/'result.json').read_text() if (out/'result.json').exists() else 'NO RESULT')

# Explicit acceptance gate: action success alone is insufficient.
import re,math
result=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {}
actual=dict(zip(result.get('final',{}).get('names',[]),result.get('final',{}).get('p',[])))
error=max((abs(actual.get(f'j{i}',float('inf'))-target) for i,target in enumerate(result.get('target',[]),1)),default=float('inf'))
summary={'lcp_errors':s.count('LCP internal error'),'nonfinite_log_tokens':len(re.findall(r'(?i)\b(?:nan|[-+]?inf(?:inity)?)\b',s)),
         'max_final_error_rad':error,'finite_joint_states':result.get('finite',False),'action_status':result.get('status'),
         'warnings':[line for line in s.splitlines() if 'WARN' in line or 'ERROR' in line]}
summary['pass']=summary['lcp_errors']==0 and summary['nonfinite_log_tokens']==0 and summary['finite_joint_states'] and result.get('status')==4 and result.get('error_code')==0 and error<0.01
(out/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
if not summary['pass']:sys.exit(1)
