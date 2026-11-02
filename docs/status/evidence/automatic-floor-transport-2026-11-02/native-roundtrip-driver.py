import json,time,os
from pathlib import Path
import rclpy
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('automatic_floor_transport_trial');r=Path(os.environ['CASE_OUTPUT']);r.mkdir(exist_ok=True);out=(r/'trace.jsonl').open('w',buffering=1)
fleet={};facility={};phase='wait';start=time.monotonic();last=0.;passed=False;reason='deadline';landings=[]
n.create_subscription(String,'/fleet/state',lambda m:fleet.update(json.loads(m.data)),10);n.create_subscription(String,'/fleet/infrastructure/state',lambda m:facility.update(json.loads(m.data)),10)
h=n.create_publisher(String,'/fleet/operator_heartbeat',10);c=n.create_publisher(String,'/fleet/infrastructure/command',10);stop=n.create_publisher(String,'/fleet/command',10)
def send(floor):c.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],operator_id='transport-trial',op='transport',robot='alpha',floor=floor))))
try:
 while time.monotonic()-start<360:
  rclpy.spin_once(n,timeout_sec=.03);now=time.monotonic()
  if not fleet or not facility:continue
  h.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],operator_id='transport-trial'))));member=fleet['members'][0];task=facility.get('transports',{}).get('alpha',{})
  if now-last>.2:out.write(json.dumps(dict(phase=phase,wall=now,fleet=fleet,facility=facility),allow_nan=False)+'\n');last=now
  if phase=='wait' and member.get('navigation_ready') and member['mode']=='idle' and fleet.get('operator_id')=='transport-trial' and facility.get('operator_id')=='transport-trial' and facility.get('operator_fresh'):send('L1');phase='up'
  elif task.get('phase') in ('failed','canceled'):raise RuntimeError(task['reason'])
  elif phase=='up' and task.get('phase')=='succeeded' and facility['robot_floors']['alpha']=='L1':
   landings.append(dict(floor='L1',body=member['truth'],map=facility['active_robot_maps']['alpha']));send('L0');phase='down'
  elif phase=='down' and task.get('phase')=='succeeded' and task.get('destination')=='L0' and facility['robot_floors']['alpha']=='L0':
   landings.append(dict(floor='L0',body=member['truth'],map=facility['active_robot_maps']['alpha']));passed=True;reason='Actual autonomous approach/boarding/ride/floor-map/settled exits L0-L1-L0';break
except (Exception, KeyboardInterrupt) as e:reason=type(e).__name__+': '+str(e)
finally:
 if fleet and rclpy.ok():stop.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],operator_id='transport-trial',op='all_stop'))))
 result=dict(passed=passed,reason=reason,phase=phase,landings=landings,wall_seconds=time.monotonic()-start,fleet=fleet,facility=facility);(r/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('fleet','facility')},indent=2));out.close();n.destroy_node();
 if rclpy.ok():rclpy.shutdown()
