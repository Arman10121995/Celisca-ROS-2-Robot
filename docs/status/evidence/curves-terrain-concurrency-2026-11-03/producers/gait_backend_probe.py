import json,math,os,signal,subprocess,time
from pathlib import Path
import psutil,rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
root=Path(os.environ['GAIT_CASE_DIRECTORY']);root.mkdir(parents=True,exist_ok=True)
backend=os.environ.get('GAIT_BACKEND','mujoco');robot=os.environ.get('GAIT_ROBOT','lab_octopod');terrain=os.environ.get('GAIT_TERRAIN','true')=='true'
import hashlib
workspace=Path('/workspace/molar/ros_ws/bumperbot_ws')
files=list((workspace/'src/robot_lab_utils/robot_lab_utils').glob('gait*.py'))
for package in ('robot_lab_mujoco','robot_lab_pybullet','robot_lab_isaac'):
 files+=list((workspace/'src'/package/'python'/package).glob('*.py'))
files+=list((workspace/'src/robot_lab_maps/vendor/gait_physics').glob('*'))
files+=list((workspace/'src/robot_lab_maps/maps/gait_low_relief').rglob('*'))
files+=list((workspace/'src/robot_lab_robots'/robot).rglob('*.yaml'))+list((workspace/'src/robot_lab_robots'/robot).rglob('*.urdf'))
(root/'source-sha256.json').write_text(json.dumps({str(f.relative_to(workspace)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.is_file()},indent=2))
log=(root/'plant.log').open('w');command=['ros2','launch','robot_lab_bringup','gait_simulator.launch.py','robot_model:='+robot,'simulator:='+backend,'map_name:='+('gait_low_relief' if terrain else 'nav_empty'),'gui:=false','rviz:=false']
(root/'command.json').write_text(json.dumps(command));plant=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,cwd=root,start_new_session=True)
rclpy.init();node=rclpy.create_node('terrain_gait_probe');state={};states=[]
node.create_subscription(String,'/gait/state',lambda m:state.update(json.loads(m.data)),10)
hb=node.create_publisher(String,'/gait/operator_heartbeat',10);control=node.create_publisher(String,'/gait/command',10);drive=node.create_publisher(Twist,'/key_vel',10)
start=time.monotonic();phase='waiting';passed=False;reason='deadline';last=0.;initial=None;held=None
def send(op,**kwargs):control.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id='terrain-probe',op=op,**kwargs))))
try:
 while time.monotonic()-start<180.:
  rclpy.spin_once(node,timeout_sec=.015);now=time.monotonic()
  if plant.poll() is not None:raise RuntimeError('Owned plant exited')
  if log.tell()>64*1024*1024:raise RuntimeError('Log budget exceeded')
  if not state:continue
  hb.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id='terrain-probe'))))
  if now-last>.15:states.append(dict(wall=now,phase=phase,state=dict(state)));last=now
  if phase=='waiting' and state['ready'] and state.get('operator_id')=='terrain-probe' and state.get('sim_time',0.)>1. and state.get('support_fraction',0.)>=.5:
   if terrain:
    if not state.get('terrain_observation'):continue
    send('terrain',enabled=True);phase='terrain'
   else:send('arm');phase='arming'
  elif phase=='terrain' and state.get('terrain_enabled'):send('arm');phase='arming'
  elif phase=='arming':
   if state.get('enabled'):initial=state['position'];begin=state['sim_time'];phase='walking'
   elif state.get('reason','').startswith('Rejected:'):raise RuntimeError(state['reason'])
  elif phase=='walking':
   if not state.get('enabled'):raise RuntimeError('Gait interrupted: '+state['reason'])
   msg=Twist();msg.linear.x=.02;drive.publish(msg)
   if state['sim_time']-begin>10.:
    send('stop');held=state['position'];stop_time=state['sim_time'];phase='holding'
  elif phase=='holding' and state['sim_time']-stop_time>2.:
   distance=math.hypot(state['position']['x']-initial['x'],state['position']['y']-initial['y'])
   drift=math.dist(list(held.values()),list(state['position'].values()))
   passed=not state['enabled'] and distance>.05 and drift<.03 and state['tilt_rad']<.3
   reason=f'Actual source gait progress {distance:.4f} m; held drift {drift:.4f} m';break
except (Exception,KeyboardInterrupt) as exc:reason=type(exc).__name__+': '+str(exc)
finally:
 if state and rclpy.ok():send('stop');rclpy.spin_once(node,timeout_sec=.1)
 result=dict(passed=passed,reason=reason,backend=backend,robot=robot,terrain=terrain,wall_seconds=time.monotonic()-start,phase=phase,state=state)
 (root/'result.json').write_text(json.dumps(result,indent=2));(root/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in states));print(json.dumps({k:v for k,v in result.items() if k!='state'},indent=2),flush=True)
 children=psutil.Process(plant.pid).children(recursive=True) if plant.poll() is None else []
 if plant.poll() is None:
  os.killpg(plant.pid,signal.SIGINT)
  try:plant.wait(timeout=8.)
  except subprocess.TimeoutExpired:os.killpg(plant.pid,signal.SIGKILL)
 for child in children:
  try:
   if child.is_running():child.kill()
  except psutil.NoSuchProcess:pass
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 log.close()
