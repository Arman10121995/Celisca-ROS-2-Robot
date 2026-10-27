import hashlib,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
base=Path(os.environ['LAB_EXTENSION_RUN'])
folder=base/('aerial-native-'+time.strftime('%H%M%S'));folder.mkdir()
root=Path('/workspace/molar/ros_ws/bumperbot_ws')
files=list((root/'src/robot_lab_adapter/robot_lab_adapter').glob('*swarm*.py'))
(folder/'sources.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
command=['ros2','launch','robot_lab_bringup','aerial_swarm.launch.py','map_name:=nav_empty','gui:=false']
(folder/'command.json').write_text(json.dumps(command))
log=(folder/'launch.log').open('w')
proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('actual_aerial_swarm_diagnostic')
state=None;rows=[]
def observe(msg):
 global state
 state=json.loads(msg.data);rows.append(dict(wall=time.time(),**state))
node.create_subscription(String,'/aerial_swarm/state',observe,10)
hb=node.create_publisher(String,'/aerial_swarm/operator_heartbeat',10)
pub=node.create_publisher(String,'/aerial_swarm/command',10)
vel=node.create_publisher(Twist,'/aerial_swarm/key_vel',10)
operator='aerial-diagnostic';phase='waiting';started=time.monotonic();sent=started;last=0.;before=None;last_hb=0.
def send(op):
 pub.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id=operator,op=op,targets=[m['id'] for m in state['members']]))))
try:
 while time.monotonic()-started<180 and proc.poll() is None:
  rclpy.spin_once(node,timeout_sec=.04);now=time.monotonic()
  if state:
   if now-last_hb>.15:
    hb.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id=operator))));last_hb=now
   members=state['members']
   if phase=='waiting' and all(m['ready'] for m in members) and state['operator_id']==operator:
    send('check');phase='checking';sent=now
   elif phase=='checking' and all((m.get('last_service') or {}).get('operation')=='check' for m in members):
    if not all(m['last_service']['accepted'] for m in members):
     phase='clearance_rejected';break
    send('takeoff');phase='ascending';sent=now
   elif phase=='ascending' and any((m.get('last_service') or {}).get('operation')=='takeoff' and not m['last_service']['accepted'] for m in members):
    phase='takeoff_rejected';break
   elif phase=='ascending' and now-sent>3. and not any((m.get('last_service') or {}).get('operation')=='takeoff' for m in members):
    phase='group_takeoff_rejected';break
   elif phase=='ascending' and all((m.get('status') or {}).get('phase')=='flying' for m in members):
    before={m['id']:m['truth'] for m in members};send('manual');phase='manual';sent=now
   elif phase=='manual' and state['mode']=='manual':
    msg=Twist();msg.linear.x=.15 if now-sent<3. else 0.;vel.publish(msg)
    if now-sent>4.:
     send('hold');phase='holding';sent=now
   elif phase=='holding' and now-sent>3.:
    send('formation');phase='formation';sent=now
   elif phase=='formation' and now-sent>10.:
    send('land');phase='landing';sent=now
   elif phase=='landing' and all(not (m.get('status') or {}).get('armed',True) for m in members):
    phase='landed';break
  if now-last>10.:
   last=now;print(json.dumps(dict(seconds=round(now-started,1),phase=phase,members=[dict(id=m['id'],ready=m['ready'],phase=(m.get('status') or {}).get('phase'),truth=m['truth'],reply=m.get('last_service')) for m in state['members']] if state else None)),flush=True)
finally:
 if state:
  send('land')
  for _ in range(5):rclpy.spin_once(node,timeout_sec=.04)
 node.destroy_node();rclpy.shutdown()
 from robot_lab_gui.process_control import stop_owned_launch
 stop_owned_launch(proc,grace_timeout=25.)
 log.close()
 report=dict(phase=phase,state=state,before_manual_truth=before,state_samples=len(rows),launch_returncode=proc.returncode)
 (folder/'report.json').write_text(json.dumps(report,indent=2))
 (folder/'states.json').write_text(json.dumps(rows))
 print(json.dumps(dict(folder=str(folder),phase=phase,samples=len(rows),returncode=proc.returncode)),flush=True)
