import os,json,time,math,hashlib
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
root=Path(os.environ['ROBOT_LAB_EXPERIMENT_OUTPUT']);source=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();trace=(root/'transition-trace.jsonl').open('w',buffering=1)
rclpy.init();node=Node('actual_amphibious_transition');state=None;body=None;start=None;phase='waiting';since=None;settled=None;measurements=[];success=False;reason='Timed out';operator='bounded-amphibious-trial';last_hb=0.;first=None;last_drive=0.
command=node.create_publisher(String,'/marine/command',10);hb=node.create_publisher(String,'/marine/operator_heartbeat',10);drive=node.create_publisher(Twist,'/marine/key_vel',10)
def observe(m):
 global state
 state=json.loads(m.data);trace.write(json.dumps(dict(kind='native_state',state=state))+'\n')
def truth(m):
 global body
 p=m.pose.pose.position;body=dict(position=[p.x,p.y,p.z],sim_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9);trace.write(json.dumps(dict(kind='actual_body',**body))+'\n')
node.create_subscription(String,'/marine/state',observe,10);node.create_subscription(Odometry,'/marine/odometry',truth,qos_profile_sensor_data)
def op(name):command.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id=operator,op=name))))
started=time.monotonic()
try:
 while time.monotonic()-started<210:
  rclpy.spin_once(node,timeout_sec=.01);now=time.monotonic()
  if not state or not body or not state['ready'] or not state.get('native_plant'):continue
  stamp=body['sim_time'];position=body['position'];plant=state['native_plant']
  if now-last_hb>.15:
   hb.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id=operator))));last_hb=now
  if phase=='waiting':
   if state.get('operator_id')!=operator:continue
   first=position[:];start=position[:];since=stamp;phase='neutral'
  elif phase=='neutral':
   if max(abs(v) for v in plant['applied_thrust_N'])>1e-6:raise RuntimeError('Neutral enablement applies propeller thrust')
   if math.dist(position,start)>.06:raise RuntimeError('Uncommanded neutral displacement')
   if stamp-since>1:
    measurements.append(dict(phase='neutral',start=start,end=position[:],sim_s=stamp-since));op('arm');phase='outbound'
  elif phase in ('outbound','inbound'):
   if not state['enabled']:
    if stamp-since>3:raise RuntimeError('Controller disarmed: '+state['reason'])
    continue
   if now-last_drive>.05:
    m=Twist();m.linear.x=.3 if phase=='outbound' else -.3;drive.publish(m);last_drive=now
   arrived=position[0]>6.3 if phase=='outbound' else position[0]<-5.8
   if arrived:
    measurements.append(dict(phase=phase,position=position[:],wet_fraction=plant['wet_fraction'],thrust=plant['applied_thrust_N'],sim_time=stamp));op('stop');since=stamp;phase='water_stop' if phase=='outbound' else 'land_stop';settled=None
  elif phase in ('water_stop','land_stop'):
   v=state.get('velocity_world');rates=state.get('rpy')
   speed=math.sqrt(sum(x*x for x in v)) if v else math.inf
   if speed<.04:
    settled=stamp if settled is None else settled
    if stamp-settled>=1:
     measurements.append(dict(phase=phase,position=position[:],speed_m_s=speed,settled_sim_s=stamp-settled,wet_fraction=plant['wet_fraction']))
     if phase=='land_stop':success=True;reason='Physical bank-water-bank traversal and settled stops';break
     op('arm');phase='inbound';since=stamp
   else:settled=None
except Exception as exc:reason=str(exc)
finally:
 if state:op('stop')
 trace.close();node.destroy_node();rclpy.shutdown();result=dict(success=success,reason=reason,measurements=measurements,wall_s=time.monotonic()-started,producer_startup_sha256=source,trace_sha256=hashlib.sha256((root/'transition-trace.jsonl').read_bytes()).hexdigest(),scope='Actual native amphibious travel/Stop only; no autonomy, clearance/contact, independent map or hardware acceptance')
 (root/'transition-result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
raise SystemExit(0 if success else 1)
