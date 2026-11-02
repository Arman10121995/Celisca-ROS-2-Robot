import json, math, os, time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from rmf_lift_msgs.msg import LiftRequest

rclpy.init()
node = rclpy.create_node('actual_rmf_lift_roundtrip')
output = Path(os.environ['CASE_OUTPUT'])
output.mkdir(exist_ok=True)
trace = (output / 'trace.jsonl').open('w', buffering=1)
fleet, facility, rmf = {}, {}, {}
events, arrivals = [], []
for topic, state in [('/fleet/state', fleet), ('/fleet/infrastructure/state', facility), ('/fleet/rmf/state', rmf)]:
    node.create_subscription(String, topic, lambda m, state=state: state.update(json.loads(m.data)), 10)
node.create_subscription(LiftRequest, '/lift_requests', lambda m: events.append(dict(at=time.monotonic(), floor=m.destination_floor, door=m.door_state, type=m.request_type, session=m.session_id)), 100)
control = node.create_publisher(String, '/fleet/rmf/control', 10)
stop = node.create_publisher(String, '/fleet/command', 10)
start = time.monotonic()
goal_yaws = {'L0': None, 'L1': None}
phase, passed, reason, last, settled = 'wait', False, 'deadline', 0., None
def send(op, **data):
    control.publish(String(data=json.dumps(dict(session_id=fleet['session_id'], op=op, **data))))
try:
    while time.monotonic() - start < 500:
        rclpy.spin_once(node, timeout_sec=.03)
        now = time.monotonic()
        if not fleet or not facility or not rmf:
            continue
        member = fleet['members'][0]
        route = rmf.get('routes', {}).get('alpha')
        if route:
            target = route['positions'][-1]
            for level, x in [('L0', 2.3), ('L1', 2.2)]:
                if abs(target[0]-x)<.001 and abs(target[1])<.001:
                    goal_yaws[level]=target[2]
        if now-last > .3:
            trace.write(json.dumps(dict(wall=now, phase=phase, fleet=fleet, facility=facility, rmf=rmf, lift_events=len(events)), allow_nan=False)+'\n')
            last = now
        if phase == 'wait' and member.get('navigation_ready') and member['mode'] == 'idle' and 'alpha' in rmf.get('robots', []):
            send('enable'); phase = 'enable'
        elif phase == 'enable' and facility.get('rmf_control_ready') and not fleet.get('traffic', {}).get('serial', True):
            send('patrol', robot='alpha', waypoint='L1_landing'); phase = 'up'
        elif phase in ('up', 'down'):
            floor, target_x = ('L1', 2.2) if phase == 'up' else ('L0', 2.3)
            body = member.get('truth') or {}
            good = (facility.get('robot_floors', {}).get('alpha') == floor
                    and abs(body.get('x', math.inf)-target_x) < .15
                    and abs(body.get('y', math.inf)) < .15
                    and goal_yaws[floor] is not None
                    and abs(math.atan2(math.sin(body.get('yaw', math.inf)-goal_yaws[floor]), math.cos(body.get('yaw', math.inf)-goal_yaws[floor]))) < .15
                    and abs(body.get('linear', math.inf)) < .03
                    and abs(body.get('angular', math.inf)) < .05
                    and member['mode'] == 'idle'
                    and not rmf.get('routes', {}).get('alpha'))
            settled = (settled or now) if good else None
            if settled and now-settled > 2.:
                arrivals.append(dict(floor=floor, body=body, map=facility['active_robot_maps']['alpha'], target_yaw=goal_yaws[floor]))
                if phase == 'up':
                    send('patrol', robot='alpha', waypoint='L0_landing'); phase='down'; settled=None
                else:
                    passed=True; reason='Real SDK lift sessions and independently settled L0-L1-L0'; break
            if not rmf.get('enabled'):
                raise RuntimeError('RMF ownership lost; explicit re-enable required')
except (Exception, KeyboardInterrupt) as error:
    reason=type(error).__name__+': '+str(error)
finally:
    if fleet and rclpy.ok():
        stop.publish(String(data=json.dumps(dict(session_id=fleet['session_id'], op='all_stop', targets=[]))))
    result=dict(passed=passed, reason=reason, phase=phase, arrivals=arrivals, wall_seconds=time.monotonic()-start, lift_requests=events, fleet=fleet, facility=facility, rmf=rmf)
    (output/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('fleet','facility','rmf','lift_requests')}, indent=2))
    trace.close(); node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
