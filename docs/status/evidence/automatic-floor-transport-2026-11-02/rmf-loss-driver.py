import json, math, os, signal, time
from pathlib import Path
import rclpy
from std_msgs.msg import String

rclpy.init(); node=rclpy.create_node('actual_rmf_owner_loss')
root=Path(os.environ['CASE_OUTPUT']);root.mkdir(exist_ok=True)
trace=(root/'trace.jsonl').open('w',buffering=1)
pid=int(os.environ['CASE_RMF_PID'])
if b'fleet_rmf_adapter.py' not in Path(f'/proc/{pid}/cmdline').read_bytes():
    raise RuntimeError('Expected the owned native SDK bridge process')
fleet, facility, rmf={}, {}, {}
for topic,state in [('/fleet/state',fleet),('/fleet/infrastructure/state',facility),('/fleet/rmf/state',rmf)]:
    node.create_subscription(String,topic,lambda m,state=state:state.update(json.loads(m.data)),10)
pub=node.create_publisher(String,'/fleet/rmf/control',10)
stop=node.create_publisher(String,'/fleet/command',10)
start=time.monotonic();phase='wait';paused=False;passed=False;reason='deadline';lost_at=None;enabled_at=None;held=[];resume_at=None
def send(op,**kw):pub.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],op=op,**kw))))
try:
    while time.monotonic()-start<200:
        rclpy.spin_once(node,timeout_sec=.05);now=time.monotonic()
        if not fleet or not facility or not rmf:continue
        member=fleet['members'][0]
        trace.write(json.dumps(dict(wall=now,phase=phase,fleet=fleet,facility=facility,rmf=rmf),allow_nan=False)+'\n')
        if phase=='wait' and member.get('navigation_ready') and member['mode']=='idle':
            send('enable');phase='enable'
        elif phase=='enable' and facility.get('rmf_control_ready'):
            send('patrol',robot='alpha',waypoint='L1_landing');phase='ride'
        elif phase=='ride' and .4<facility['lift']['position_m']<2.5:
            enabled_at=rmf['enabled_at'];os.kill(pid,signal.SIGSTOP);paused=True;lost_at=now;phase='paused'
        elif phase=='paused':
            if now-lost_at>4.:
                if not facility.get('operator_hold') or facility.get('rmf_control_ready'):
                    raise RuntimeError('Facility did not invalidate active RMF ownership')
                held.append(dict(position=facility['lift']['position_m'],command=facility['lift']['command_m']))
            if now-lost_at>7.:
                os.kill(pid,signal.SIGCONT);paused=False;resume_at=now;phase='resumed'
        elif phase=='resumed' and now-resume_at>4.:
            if rmf.get('enabled') or facility.get('rmf_control_ready') or not facility.get('operator_hold'):
                raise RuntimeError('Feedback resumption re-enabled actuation without an explicit control request')
            if not facility.get('rmf_stopped_at',0.)>enabled_at:
                raise RuntimeError('Stop epoch did not advance')
            excursion=max(x['position'] for x in held)-min(x['position'] for x in held)
            if excursion>.01:raise RuntimeError('Lift did not settle within the predeclared 10 mm held window')
            body=member['truth']
            if abs(body['linear'])>.03 or abs(body['angular'])>.05:
                raise RuntimeError('Body did not settle after ownership loss')
            passed=True;reason='Actual SDK process suspension holds the car; resumption remains disabled until explicit re-enable';break
except (Exception,KeyboardInterrupt) as error:reason=type(error).__name__+': '+str(error)
finally:
    if paused:os.kill(pid,signal.SIGCONT)
    if fleet and rclpy.ok():stop.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],op='all_stop',targets=[]))))
    result=dict(passed=passed,reason=reason,phase=phase,wall_seconds=time.monotonic()-start,held_observations=held,fleet=fleet,facility=facility,rmf=rmf)
    (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('fleet','facility','rmf','held_observations')},indent=2))
    trace.close();node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
