import json,math,os,signal,subprocess,time
from pathlib import Path
import psutil,rclpy
from std_msgs.msg import String
root=Path(os.environ['FLEET_CASE_DIRECTORY']);root.mkdir(parents=True,exist_ok=True)
command=['ros2','launch','robot_lab_bringup','shared_world.launch.py','session:='+str(Path(__file__).parent/'fleet_three.yaml'),'navigation:=true','gui:=false','rviz:=false']
(root/'command.json').write_text(json.dumps(command));log=(root/'plant.log').open('w');plant=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,cwd=root,start_new_session=True)
rclpy.init();node=rclpy.create_node('concurrent_missions_probe');fleet={};node.create_subscription(String,'/fleet/state',lambda m:fleet.update(json.loads(m.data)),10)
hb=node.create_publisher(String,'/fleet/operator_heartbeat',10);control=node.create_publisher(String,'/fleet/command',10)
start=time.monotonic();phase='waiting';passed=False;reason='deadline';last=0.;overlap=[];trace=(root/'trace.jsonl').open('w',buffering=1)
goals={'alpha':dict(x=2.3,y=0.,yaw=0.),'gamma':dict(x=-2.,y=1.,yaw=0.)}
def send(op,**kwargs):control.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],operator_id='planar-probe',op=op,**kwargs))))
try:
 while time.monotonic()-start<240.:
  rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
  if plant.poll() is not None:raise RuntimeError('Owned plant exited')
  if log.tell()>128*1024*1024:raise RuntimeError('Log budget exceeded')
  if not fleet:continue
  hb.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],operator_id='planar-probe'))))
  if now-last>.2:trace.write(json.dumps(dict(wall=now,phase=phase,fleet=fleet))+'\n');last=now
  members={m['id']:m for m in fleet['members']}
  if phase=='waiting' and all(m.get('navigation_ready') and m['mode']=='idle' for m in members.values()) and fleet.get('operator_id')=='planar-probe':
   send('traffic',serial=False,targets=[]);phase='traffic'
  elif phase=='traffic' and fleet.get('traffic',{}).get('serial') is False:
   for name,goal in goals.items():send('route',targets=[name],request_id='concurrent-'+name,goals=[dict(goal,timeout=180.)])
   phase='running'
  if all(members[name]['mode'] in ('dispatching','navigation','settling') for name in goals):overlap.append(now)
  outcomes=[item for item in fleet.get('task_history',[]) if item.get('source') in ['route:concurrent-'+n for n in goals]]
  if any(o['state']!='succeeded' for o in outcomes):raise RuntimeError('Native task failed: '+str(outcomes))
  if len(outcomes)==2:
   for name,goal in goals.items():
    b=members[name]['truth']
    if math.hypot(b['x']-goal['x'],b['y']-goal['y'])>=.15 or abs(math.atan2(math.sin(b['yaw']-goal['yaw']),math.cos(b['yaw']-goal['yaw'])))>=.15 or abs(b['linear'])>=.03 or abs(b['angular'])>=.05:raise RuntimeError('Independent endpoint did not settle')
   passed=bool(overlap and overlap[-1]-overlap[0]>1.);reason='Two independent actual Nav2 missions and settled body endpoints in one three-base world';break
except (Exception,KeyboardInterrupt) as exc:reason=type(exc).__name__+': '+str(exc)
finally:
 if fleet and rclpy.ok():send('all_stop');rclpy.spin_once(node,timeout_sec=.1)
 result=dict(passed=passed,reason=reason,phase=phase,wall_seconds=time.monotonic()-start,overlap_s=(overlap[-1]-overlap[0]) if overlap else 0.,goals=goals,fleet=fleet)
 (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='fleet'},indent=2),flush=True)
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
 trace.close();log.close()
