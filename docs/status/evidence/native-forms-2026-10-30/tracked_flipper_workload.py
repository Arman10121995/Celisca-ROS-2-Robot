import json,os,time,math,hashlib
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

folder=Path(os.environ['ROBOT_LAB_EXPERIMENT_OUTPUT']);trace=(folder/'flipper-trace.jsonl').open('w',buffering=1)
rclpy.init();node=Node('measured_articulated_tracks');native=None;body=None;phase='waiting';sent=None;first=None;last=None;settled=None
target=[-.3,.3,-.3,.3];result={};previous=None;rates=None
publisher=node.create_publisher(String,'/tracked/flipper_command',10);drive=node.create_publisher(Twist,'/key_vel',10)
def state(msg):
 global native,previous,rates
 data=json.loads(msg.data);positions=data['flippers']['positions'];stamp=data.get('sim_time')
 if positions is not None and stamp is not None and previous is not None and stamp>previous[0]:
  rates=[(a-b)/(stamp-previous[0]) for a,b in zip(positions,previous[1])]
 if positions is not None and stamp is not None:previous=(stamp,positions)
 native=data;trace.write(json.dumps(dict(kind='state',state=data))+'\n')
def truth(msg):
 global body
 p=msg.pose.pose.position;body=[p.x,p.y,p.z];trace.write(json.dumps(dict(kind='physical_body',position=body))+'\n')
node.create_subscription(String,'/tracked/state',state,10)
node.create_subscription(Odometry,'/odom/ground_truth',truth,qos_profile_sensor_data)
started=time.monotonic();success=False;reason='Timed out'
try:
 while time.monotonic()-started<50:
  rclpy.spin_once(node,timeout_sec=.02)
  if native is None or body is None or not native['ready'] or not native['flippers']['feedback_fresh']:continue
  stamp=native['sim_time']
  if phase=='waiting':
   publisher.publish(String(data=json.dumps(dict(session_id=native['session_id'],robot_id=native['robot_id'],op='target',positions=target))))
   phase='targeting';sent=stamp
  elif phase=='targeting':
   positions=native['flippers']['positions']
   if rates is not None and max(abs(a-b) for a,b in zip(positions,target))<.05 and max(map(abs,rates))<.02:
    settled=stamp if settled is None else settled
    if stamp-settled>=1:
     result['angles']=dict(requested_rad=target,measured_rad=positions,measured_rates_rad_s=rates,settled_sim_s=stamp-settled)
     first=body[:];sent=stamp;phase='driving'
   else:settled=None
  elif phase=='driving':
   if stamp-sent<3:
    m=Twist();m.linear.x=.1;drive.publish(m)
   else:drive.publish(Twist());sent=stamp;phase='braking';last=None
  elif phase=='braking':
   drive.publish(Twist())
   if stamp-sent>2:
    last=body[:];success=last[0]-first[0]>.15;reason='Actual four-joint settling and forward contact-track travel' if success else 'Physical forward travel below the required 0.15 m'
    result['drive']=dict(initial=first,final=last,forward_m=last[0]-first[0]);break
finally:
 drive.publish(Twist());trace.close();node.destroy_node();rclpy.shutdown()
 result.update(success=success,reason=reason,run_id=os.environ['ROBOT_LAB_EXPERIMENT_ID'],
  producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),trace_sha256=hashlib.sha256((folder/'flipper-trace.jsonl').read_bytes()).hexdigest(),
  scope='Native four-flipper angle settling and flat contact-track travel only; terrain/navigation/contact clearance separate')
 (folder/'flipper-result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
raise SystemExit(0 if success else 1)
