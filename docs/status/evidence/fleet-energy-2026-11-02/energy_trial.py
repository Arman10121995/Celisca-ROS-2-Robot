import json,time,os,math
from pathlib import Path
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController
rclpy.init();n=rclpy.create_node('measured_energy_dock_trial');root=Path(os.environ['CASE_OUTPUT']);root.mkdir(exist_ok=True)
out=(root/'trace.jsonl').open('w',buffering=1);state={};infra={};phase='wait';phase_at=time.monotonic();initial=None;charged=None;departed=None;future=None;success=False;reason='deadline'
n.create_subscription(String,'/fleet/state',lambda m:state.update(json.loads(m.data)),10)
n.create_subscription(String,'/fleet/infrastructure/state',lambda m:infra.update(json.loads(m.data)),10)
h=n.create_publisher(String,'/fleet/operator_heartbeat',10);c=n.create_publisher(String,'/fleet/command',10);f=n.create_publisher(String,'/fleet/infrastructure/command',10);d=n.create_publisher(Twist,'/alpha/key_vel',10)
client=n.create_client(SwitchController,'/alpha/controller_manager/switch_controller')
def cmd(op,**v):c.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id='energy-trial',op=op,**v))))
def switch(active):
 q=SwitchController.Request();q.strictness=2
 if active:q.activate_controllers=['joint_state_broadcaster']
 else:q.deactivate_controllers=['joint_state_broadcaster']
 return client.call_async(q)
start=time.monotonic();last=0.
try:
 while time.monotonic()-start<100:
  rclpy.spin_once(n,timeout_sec=.03);now=time.monotonic()
  if not state or not infra:continue
  h.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id='energy-trial'))))
  e=infra['energy']['alpha'];a=next(m for m in state['members'] if m['id']=='alpha')
  if now-last>.1:out.write(json.dumps(dict(phase=phase,t=state.get('sim_time'),energy=infra['energy'],docking=infra.get('docking'),body=a.get('truth'),mode=a.get('mode')),allow_nan=False)+'\n');last=now
  if phase=='wait' and state.get('operator_id')=='energy-trial' and a.get('navigation_ready') and all(v.get('fraction') is not None for v in infra['energy'].values()):
   initial=e['fraction'];f.publish(String(data=json.dumps(dict(session_id=state['session_id'],operator_id='energy-trial',op='dock',robot='alpha',dock='west_charger',request_id='measured-charge'))));phase='dock';phase_at=now
  elif phase=='dock' and infra.get('docking',{}).get('alpha',{}).get('docked') and e.get('charging_W')==120. and e['fraction']>initial+.001:
   charged=e['fraction'];cmd('manual',targets=['alpha']);phase='depart';phase_at=now
  elif phase=='depart':
   t=Twist();t.linear.x=.1;d.publish(t)
   if now-phase_at>3:
    d.publish(Twist());cmd('stop',targets=['alpha']);departed=e['fraction'];phase='stop';phase_at=now
  elif phase=='stop' and now-phase_at>2:
   if e.get('charging_W')!=0 or e['fraction']>=departed:raise RuntimeError('Charging failed to disengage or load failed to drain')
   future=switch(False);phase='dropout';phase_at=now
  elif phase=='dropout' and future.done() and future.result().ok and e.get('fraction') is None and now-phase_at>1:
   future=switch(True);phase='resume';phase_at=now
  elif phase=='resume' and future.done() and future.result().ok and now-phase_at>2 and e.get('power_W') is not None:
   if e.get('fraction') is not None:raise RuntimeError('Unknown gap was incorrectly integrated')
   success=True;reason='Actual dock/energy increase, departure/discharge, broadcaster loss and unknown-coverage recovery observed';break
except Exception as error:reason=str(error)
finally:
 if state:cmd('all_stop')
 result=dict(passed=success,reason=reason,phase=phase,initial_fraction=initial,charged_fraction=charged,departed_fraction=departed,final=infra,fleet=state,wall_seconds=time.monotonic()-start)
 (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('final','fleet')},indent=2));out.close();n.destroy_node();rclpy.shutdown()
