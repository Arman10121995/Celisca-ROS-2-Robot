"""Two RMF patrols cross the main door at once with calibrated RMF energy (R10.2)."""
import json, math, os, time
from pathlib import Path
import rclpy
from std_msgs.msg import String
TARGETS = dict(alpha=(2.0, 2.5), beta=(-3.0, -2.0))
root = Path(os.environ['CASE_OUTPUT']); root.mkdir(parents=True, exist_ok=True)
out = (root / 'trace.jsonl').open('w', buffering=1)
rclpy.init(); n = rclpy.create_node('rmf_crossing_probe')
fleet, rmf, infra = {}, {}, {}
n.create_subscription(String, '/fleet/state', lambda m: fleet.update(json.loads(m.data)), 10)
n.create_subscription(String, '/fleet/rmf/state', lambda m: rmf.update(json.loads(m.data)), 10)
n.create_subscription(String, '/fleet/infrastructure/state', lambda m: infra.update(json.loads(m.data)), 10)
control = n.create_publisher(String, '/fleet/rmf/control', 10)
def send(op, **v): control.publish(String(data=json.dumps(dict(session_id=fleet['session_id'], op=op, **v))))
phase, start, last, sent_at = 'wait', time.monotonic(), 0., None
chosen, closest, reason, passed, settled = {}, dict(distance=math.inf), 'deadline', False, {}
try:
    while time.monotonic() - start < 480:
        rclpy.spin_once(n, timeout_sec=.05); now = time.monotonic()
        if not fleet or not rmf or not infra: continue
        members = {m['id']: m for m in fleet.get('members', [])}
        if now - last > .25:
            out.write(json.dumps(dict(phase=phase, wall=now - start, truth={k: m.get('truth') for k, m in members.items()},
                modes={k: [m['mode'], m['reason'], m.get('zone_wait')] for k, m in members.items()},
                routes=rmf.get('routes'), battery=rmf.get('battery'), zones={k: v for k, v in fleet.get('zones', {}).items() if k != 'declared'}),
                default=str) + '\n'); last = now
        if phase == 'wait' and len(rmf.get('robots', [])) == 2 and all(m.get('navigation_ready') for m in members.values()):
            send('enable'); phase = 'enabled'
        elif phase == 'enabled' and rmf.get('enabled') and fleet.get('operator_id') == rmf.get('operator_id'):
            selectable = [v for v in rmf['waypoints'] if v.get('selectable', True)]
            for name, (x, y) in TARGETS.items():
                chosen[name] = min(selectable, key=lambda v: math.hypot(v['x'] - x, v['y'] - y))
                send('patrol', robot=name, waypoint=chosen[name]['name'])
            phase, sent_at = 'running', now
        elif phase == 'running':
            truth = {k: m.get('truth') for k, m in members.items()}
            if all(truth.values()):
                d = math.hypot(truth['alpha']['x'] - truth['beta']['x'], truth['alpha']['y'] - truth['beta']['y'])
                if d < closest['distance']:
                    closest = dict(distance=d, wall=now - sent_at, modes={k: m['mode'] for k, m in members.items()})
                for name, body in truth.items():
                    goal = chosen[name]
                    if (math.hypot(body['x'] - goal['x'], body['y'] - goal['y']) < .25 and abs(body.get('linear', 1)) < .03
                            and not (rmf.get('routes') or {}).get(name) and members[name]['mode'] in ('idle', 'waiting')):
                        settled.setdefault(name, dict(body, wall=now - sent_at))
            if len(settled) == 2:
                passed, reason = True, 'Both RMF patrols ended with independently settled bodies at their waypoints'
                break
            if any(e.get('kind') == 'contact_detected' for e in fleet.get('contacts', {}).get('events', [])):
                reason = 'contact stop'; break
except Exception as e:
    reason = repr(e)
finally:
    if fleet: send('disable')
    events = []
    try:
        for line in (Path(fleet['directory']) / 'events.jsonl').read_text().splitlines():
            row = json.loads(line)
            if row.get('event') in ('zone_wait', 'zone_granted', 'contact_detected', 'traffic_yield_planned', 'conflict_resolved', 'task_finished', 'route_queued'):
                events.append(row)
    except Exception as exc:
        events.append(dict(error=repr(exc)))
    result = dict(passed=passed and closest['distance'] > .35, reason=reason, chosen=chosen, settled=settled, closest_approach=closest,
                  elapsed_s=time.monotonic() - sent_at if sent_at else None, energy_calibration=rmf.get('energy_calibration'),
                  battery=rmf.get('battery'), rmf_scope=rmf.get('scope'), events=events, session_directory=fleet.get('directory'))
    (root / 'result.json').write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps({k: v for k, v in result.items() if k != 'events'}, indent=2, default=str))
    out.close(); n.destroy_node(); rclpy.shutdown()
