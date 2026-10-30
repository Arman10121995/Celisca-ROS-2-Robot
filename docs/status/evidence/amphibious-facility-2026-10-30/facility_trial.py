import os,time,json,hashlib,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rmf_door_msgs.msg import DoorRequest
root=Path(os.environ['ROBOT_LAB_EXPERIMENT_OUTPUT']);source=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();trace=(root/'facility-trace.jsonl').open('w',buffering=1)
rclpy.init();node=Node('physical_facility_trial');fleet=None;infra=None;phase='waiting';started=time.monotonic();operator='actual-facility-trial';last_hb=0.;since=None;success=False;reason='Timed out';rows=[]
hb=node.create_publisher(String,'/fleet/operator_heartbeat',10);request=node.create_publisher(String,'/fleet/infrastructure/command',10);door=node.create_publisher(DoorRequest,'/door_requests',10)
def fs(m):
 global fleet
 fleet=json.loads(m.data);trace.write(json.dumps(dict(kind='fleet',state=fleet))+'\n')
def ins(m):
 global infra
 infra=json.loads(m.data);trace.write(json.dumps(dict(kind='infrastructure',state=infra))+'\n')
node.create_subscription(String,'/fleet/state',fs,10);node.create_subscription(String,'/fleet/infrastructure/state',ins,10)
def cmd(op,**extra):request.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],operator_id=operator,op=op,**extra))))
def doorcmd(opened):
 m=DoorRequest();m.door_name='main';m.requester_id='actual-rmf-door-trial';m.requested_mode.value=2 if opened else 0;door.publish(m)
try:
 while time.monotonic()-started<235:
  rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
  if fleet and now-last_hb>.15:
   hb.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],operator_id=operator))));last_hb=now
  if not fleet or not infra or not infra['ready'] or fleet.get('operator_id')!=operator:continue
  if phase=='waiting':
   assert all(e['fraction'] is None for e in infra['energy'].values()),'Unqualified motor power must remain unknown'
   since=fleet['sim_time'];phase='neutral'
  elif phase=='neutral':
   assert all(abs(d['position_m'])<.04 for d in infra['doors'].values()),'Infrastructure moved before an explicit request'
   if fleet['sim_time']-since>1:
    rows.append(dict(phase='neutral',actual=infra['doors']));doorcmd(True);phase='door_open'
  elif phase=='door_open' and infra['doors']['main']['mode']==2:
   rows.append(dict(phase=phase,actual=infra['doors']['main']));doorcmd(False);phase='door_closed'
  elif phase=='door_closed' and infra['doors']['main']['mode']==0:
   rows.append(dict(phase=phase,actual=infra['doors']['main']));cmd('lift',floor='L1');phase='lift_up'
  elif phase=='lift_up' and infra['lift']['current_floor']=='L1' and infra['doors']['lift']['mode']==2:
   rows.append(dict(phase=phase,actual=infra['lift'],door=infra['doors']['lift']));cmd('lift',floor='L0');phase='lift_down'
  elif phase=='lift_down' and infra['lift']['current_floor']=='L0' and infra['doors']['lift']['mode']==2:
   rows.append(dict(phase=phase,actual=infra['lift'],door=infra['doors']['lift']));cmd('dock',robot='alpha',dock='west_charger',request_id='actual-dock-1');phase='dock'
  elif phase=='dock' and infra['docking'].get('alpha',{}).get('docked'):
   rows.append(dict(phase=phase,actual=infra['docking']['alpha'],fleet=fleet));success=True;reason='Physical door open/close, lift L0-L1-L0 with car door and independently settled dock';break
except Exception as exc:reason=str(exc)
finally:
 if fleet:
  stop=node.create_publisher(String,'/fleet/command',10);stop.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],op='all_stop',targets=[]))))
 trace.close();node.destroy_node();rclpy.shutdown();result=dict(success=success,reason=reason,phase=phase,measurements=rows,wall_s=time.monotonic()-started,producer_startup_sha256=source,trace_sha256=hashlib.sha256((root/'facility-trace.jsonl').read_bytes()).hexdigest(),scope='Actual physical infrastructure joints and observed docking; no robot lift transport, RMF conflict/negotiation or verified motor-energy acceptance')
 (root/'facility-result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
raise SystemExit(0 if success else 1)
