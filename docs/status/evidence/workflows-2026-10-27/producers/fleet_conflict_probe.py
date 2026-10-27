import hashlib,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

base=Path(os.environ['LAB_EXTENSION_RUN'])
folder=base/('fleet-conflict-'+time.strftime('%H%M%S'));folder.mkdir()
root=Path('/workspace/molar/ros_ws/bumperbot_ws')
(folder/'sources.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (root/'src/robot_lab_utils/robot_lab_utils').glob('fleet*.py')},indent=2))
command=['ros2','launch','robot_lab_bringup','shared_world.launch.py','navigation:=true','rmf:=false',
    'map_name:=nav_empty','gui:=false','rviz:=false']
(folder/'command.json').write_text(json.dumps(command))
log=(folder/'launch.log').open('w')
process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('extension_fleet_diagnostic')
state=None;rmf=None;states=[];rmf_states=[]
def observe(msg):
 global state
 state=json.loads(msg.data);states.append(dict(wall=time.time(),**state))
def observe_rmf(msg):
 global rmf
 rmf=json.loads(msg.data);rmf_states.append(rmf)
node.create_subscription(String,'/fleet/state',observe,10)
node.create_subscription(String,'/fleet/rmf/state',observe_rmf,10)
hb=node.create_publisher(String,'/fleet/operator_heartbeat',10)
pub=node.create_publisher(String,'/fleet/command',10)
rmf_control=node.create_publisher(String,'/fleet/rmf/control',10)
operator='extension-fleet-diagnostic';phase='waiting';started=time.monotonic();last_report=0.;handed=False
try:
 while time.monotonic()-started<145 and process.poll() is None:
  rclpy.spin_once(node,timeout_sec=.05);now=time.monotonic()
  if state:
   common=dict(session_id=state['session_id'],operator_id=operator)
   if not handed:hb.publish(String(data=json.dumps(common)))
   if phase=='waiting' and all(m['ready'] and m['navigation_ready'] for m in state['members']) and state['operator_id']==operator:
    pub.publish(String(data=json.dumps(dict(common,op='traffic',targets=[],serial=False))))
    phase='traffic'
   elif phase=='traffic' and not state['traffic']['serial']:
    rows=state['members']
    for index, member in enumerate(rows):
     p=rows[1-index]['estimate']
     pub.publish(String(data=json.dumps(dict(common,op='dispatch',targets=[member['id']],
         x=p['x'],y=p['y'],yaw=0.,timeout=100.))))
    phase='conflict';sent=now
   elif phase=='conflict':
    outcomes=[r for r in state.get('task_history',[]) if r['source']=='operator']
    if any(r['state']=='failed' for r in outcomes):
     phase='failed';break
    if len([r for r in outcomes if r['state']=='succeeded'])==2:
     phase='done';break
  if now-last_report>10:
   last_report=now;print(json.dumps(dict(seconds=round(now-started,1),phase=phase,
    members=[dict(id=m['id'],mode=m['mode'],reason=m['reason'],queued=m.get('queued')) for m in state['members']] if state else None,
    rmf_robots=rmf.get('robots') if rmf else None)),flush=True)
finally:
 if state:pub.publish(String(data=json.dumps(dict(session_id=state['session_id'],op='all_stop',targets=[]))))
 for _ in range(4):rclpy.spin_once(node,timeout_sec=.05)
 node.destroy_node();rclpy.shutdown()
 if process.poll() is None:
  os.killpg(process.pid,signal.SIGINT)
  try:process.wait(timeout=20)
  except subprocess.TimeoutExpired:
   os.killpg(process.pid,signal.SIGTERM)
   try:process.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
 log.close()
 report=dict(phase=phase,state=state,rmf=rmf,state_samples=len(states),launch_returncode=process.returncode)
 (folder/'report.json').write_text(json.dumps(report,indent=2));(folder/'states.json').write_text(json.dumps(states))
 print(json.dumps(dict(folder=str(folder),phase=phase,samples=len(states),returncode=process.returncode)),flush=True)
