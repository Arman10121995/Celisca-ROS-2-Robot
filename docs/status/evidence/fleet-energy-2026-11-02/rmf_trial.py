import rclpy,json,time,os,math
from pathlib import Path
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('native_rmf_door_dock_trial');root=Path(os.environ['CASE_OUTPUT']);root.mkdir(exist_ok=True)
out=(root/'trace.jsonl').open('w',buffering=1);fleet={};rmf={};infra={};phase='wait';start=time.monotonic();last=0.;sent=False;passed=False;reason='deadline';crossed=None
n.create_subscription(String,'/fleet/state',lambda m:fleet.update(json.loads(m.data)),10)
n.create_subscription(String,'/fleet/rmf/state',lambda m:rmf.update(json.loads(m.data)),10)
n.create_subscription(String,'/fleet/infrastructure/state',lambda m:infra.update(json.loads(m.data)),10)
p=n.create_publisher(String,'/fleet/rmf/control',10)
def send(op,**v):p.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],op=op,**v))))
try:
 while time.monotonic()-start<190:
  rclpy.spin_once(n,timeout_sec=.05);now=time.monotonic()
  if not fleet or not rmf or not infra:continue
  member=fleet['members'][0];body=member.get('truth') or {};sid=fleet['session_id']
  if now-last>.2:out.write(json.dumps(dict(phase=phase,wall=now,t=fleet.get('sim_time'),body=body,rmf=rmf,facility=infra,member=member),allow_nan=False)+'\n');last=now
  if phase=='wait' and member.get('navigation_ready') and rmf.get('robots'):
   send('enable');phase='enabled'
  elif phase=='enabled' and rmf.get('enabled') and fleet.get('operator_id')==rmf.get('operator_id'):
   send('patrol',robot='alpha',waypoint='door_main_1');phase='cross'
  elif phase=='cross' and math.hypot(body.get('x',-100)-1.2,body.get('y',100))<.18 and abs(body.get('linear',1))<.05:
   crossed=dict(body);send('patrol',robot='alpha',waypoint='west_charger');phase='dock'
  elif phase=='dock' and infra.get('docking',{}).get('alpha',{}).get('docked'):
   passed=True;reason='Actual RMF patrol crossed native door, then invoked native independently settled dock event';break
except Exception as e:reason=str(e)
finally:
 if fleet:send('disable')
 result=dict(passed=passed,reason=reason,phase=phase,crossed=crossed,wall_seconds=time.monotonic()-start,final=fleet,rmf=rmf,facility=infra)
 (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('final','rmf','facility')},indent=2));out.close();n.destroy_node();rclpy.shutdown()
