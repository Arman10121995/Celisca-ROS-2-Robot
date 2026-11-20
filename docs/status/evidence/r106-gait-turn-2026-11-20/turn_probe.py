"""R10.6: in-place turn probe (2026-11-20), from the 11-20 push probe. Flat nav_empty, no terrain mode;
GAIT_YAW (rad/s) and GAIT_SPEED (m/s, body forward) are commanded for GAIT_WALK_S of simulated time. Truth
(/odom/ground_truth) gives the yaw turned and the body translation, split along the initial heading
(forward_yaw applied) and across it.
"""

import json,math,os,signal,subprocess,time
from pathlib import Path
import psutil,rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
root=Path(os.environ['GAIT_CASE_DIRECTORY']);root.mkdir(parents=True,exist_ok=True)
backend=os.environ.get('GAIT_BACKEND','mujoco');robot=os.environ.get('GAIT_ROBOT','lab_octopod');terrain=False
import hashlib
workspace=Path('/workspace/molar/ros_ws/bumperbot_ws')
files=list((workspace/'src/robot_lab_utils/robot_lab_utils').glob('gait*.py'))
for package in ('robot_lab_mujoco','robot_lab_pybullet','robot_lab_isaac'):
 files+=list((workspace/'src'/package/'python'/package).glob('*.py'))
files+=list((workspace/'src/robot_lab_maps/vendor/gait_physics').glob('*'))
files+=list((workspace/'src/robot_lab_maps/maps'/os.environ.get('GAIT_MAP','gait_low_relief')).rglob('*'))
files+=list((workspace/'src/robot_lab_robots'/robot).rglob('*.yaml'))+list((workspace/'src/robot_lab_robots'/robot).rglob('*.urdf'))
(root/'source-sha256.json').write_text(json.dumps({str(f.relative_to(workspace)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.is_file()},indent=2))
budget=float(os.environ.get('GAIT_WALL_BUDGET','180'))
if not math.isfinite(budget) or not 0<budget<=600:raise ValueError('Trial wall budget must be finite and at most 600 s')
log=(root/'plant.log').open('w');command=['ros2','launch','robot_lab_bringup','gait_simulator.launch.py','robot_model:='+robot,'simulator:='+backend,'map_name:='+(os.environ.get('GAIT_MAP','gait_low_relief') if terrain else 'nav_empty'),'gui:=false','rviz:=false']
push=os.environ.get('GAIT_PUSH','')
if push:
 force,start_s,duration,axis=push.split(',');command+=['perturbation_force_n:='+force,'perturbation_start_s:='+start_s,'perturbation_duration_s:='+duration,'perturbation_axis:='+axis]
(root/'command.json').write_text(json.dumps(command));plant=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,cwd=root,start_new_session=True)
rclpy.init();node=rclpy.create_node('terrain_gait_probe');state={};states=[]
node.create_subscription(String,'/gait/state',lambda m:state.update(json.loads(m.data)),10)
truth={}
def observe_truth(m):
 q=m.pose.pose.orientation;truth.update(x=m.pose.pose.position.x,y=m.pose.pose.position.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
node.create_subscription(Odometry,'/odom/ground_truth',observe_truth,10)
turned=0.;last_yaw=None;samples=[]
hb=node.create_publisher(String,'/gait/operator_heartbeat',10);control=node.create_publisher(String,'/gait/command',10);drive=node.create_publisher(Twist,'/key_vel',10)
recoveries=set();history=[];start=time.monotonic();phase='waiting';passed=False;reason='deadline';last=0.;initial=None;held=None
def send(op,**kwargs):control.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id='terrain-probe',op=op,**kwargs))))
try:
 while time.monotonic()-start<budget:
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
   if state.get('enabled') and truth:initial=dict(truth);last_yaw=truth['yaw'];begin=state['sim_time'];phase='walking'
   elif state.get('reason','').startswith('Rejected:'):raise RuntimeError(state['reason'])
  elif phase=='walking':
   if not state.get('enabled'):raise RuntimeError('Gait interrupted: '+state['reason'])
   recoveries.add(json.dumps((state.get('slip_recovery') or {}).get('reason')))
   history.append(dict(sim=state.get('sim_time'),recovery=state.get('slip_recovery'),position=state.get('position'),tilt=state.get('tilt_rad'))) if state.get('slip_recovery')!=(history[-1]['recovery'] if history else None) else None
   msg=Twist();msg.linear.x=float(os.environ.get('GAIT_SPEED','0'));msg.angular.z=float(os.environ.get('GAIT_YAW','.1'));drive.publish(msg)
   turned+=math.remainder(truth['yaw']-last_yaw,2*math.pi);last_yaw=truth['yaw']
   samples.append(dict(sim=state['sim_time'],x=truth['x'],y=truth['y'],turned=turned))
   if state['sim_time']-begin>float(os.environ.get('GAIT_WALK_S','10')):
    send('stop');held=state['position'];stop_time=state['sim_time'];phase='holding'
  elif phase=='holding' and state['sim_time']-stop_time>2.:
   heading=initial['yaw']-float(state.get('drive_forward_yaw_rad',0.))
   dx,dy=truth['x']-initial['x'],truth['y']-initial['y']
   along=dx*math.cos(heading)+dy*math.sin(heading);across=-dx*math.sin(heading)+dy*math.cos(heading)
   walk=state_walk=stop_time-begin
   metrics=dict(walk_sim_s=walk,turned_rad=turned,mean_yaw_rate=turned/walk if walk>0 else None,
                displacement_m=math.hypot(dx,dy),along_initial_heading_m=along,across_m=across,
                drift_per_rad_m=math.hypot(dx,dy)/abs(turned) if abs(turned)>1e-3 else None)
   passed=not state['enabled'] and state['tilt_rad']<.3
   reason='turned %.3f rad in %.1f s; moved %.4f m (along %.4f, across %.4f)'%(turned,walk,math.hypot(dx,dy),along,across);break
except (Exception,KeyboardInterrupt) as exc:reason=type(exc).__name__+': '+str(exc)
finally:
 if state and rclpy.ok():send('stop');rclpy.spin_once(node,timeout_sec=.1)
 result=dict(metrics=locals().get('metrics'),command=dict(yaw=os.environ.get('GAIT_YAW','.1'),speed=os.environ.get('GAIT_SPEED','0')),recovery_reasons=sorted(recoveries),recovery_history=history,push=push,passed=passed,reason=reason,backend=backend,robot=robot,terrain=terrain,wall_seconds=time.monotonic()-start,phase=phase,wall_budget_s=budget,state_publishers=[dict(node=e.node_name,namespace=e.node_namespace) for e in node.get_publishers_info_by_topic('/gait/state')],state=state)
 (root/'result.json').write_text(json.dumps(result,indent=2));(root/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in states));(root/'turn.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in samples[::10]));print(json.dumps({k:v for k,v in result.items() if k!='state'},indent=2),flush=True)
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
