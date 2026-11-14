"""Two members cross the shared main door at the same time (R10.2 zone reservations).

Dispatches alpha west->east and beta east->west concurrently, records truth
separation, zone state and controller events, and passes only when both tasks
succeed with measured settling, bodies never come within radii + 5 cm and no
contact stop occurred.
"""
import json, math, os, sys, time
from pathlib import Path
import rclpy
from std_msgs.msg import String
GOALS = dict(alpha=dict(x=1.5, y=2.5, yaw=0.), beta=dict(x=-3.0, y=-2.0, yaw=math.pi))
root = Path(os.environ['CASE_OUTPUT']); root.mkdir(parents=True, exist_ok=True)
trace = (root / 'trace.jsonl').open('w', buffering=1)
rclpy.init(); node = rclpy.create_node('zone_conflict_probe')
state = {}
node.create_subscription(String, '/fleet/state', lambda m: state.update(json.loads(m.data)), 10)
heartbeat = node.create_publisher(String, '/fleet/operator_heartbeat', 10)
command = node.create_publisher(String, '/fleet/command', 10)
operator = 'zone-conflict-probe'
phase, started, sent_at = 'wait', time.monotonic(), None
closest = dict(distance=math.inf)
budget = float(os.environ.get('CASE_BUDGET_S', '420'))
def send(**c):
    command.publish(String(data=json.dumps(dict(session_id=state['session_id'], operator_id=operator, **c))))
while time.monotonic() - started < budget:
    rclpy.spin_once(node, timeout_sec=.05)
    if not state: continue
    heartbeat.publish(String(data=json.dumps(dict(session_id=state['session_id'], operator_id=operator))))
    members = {m['id']: m for m in state.get('members', [])}
    now = time.monotonic()
    if phase == 'wait':
        if state.get('operator_id') == operator and len(members) == 2 and all(m.get('navigation_ready') and m.get('ready') for m in members.values()):
            send(op='traffic', targets=[], serial=False); phase = 'traffic'; phase_at = now
    elif phase == 'traffic' and now - phase_at > 1.:
        if state.get('traffic', {}).get('serial'):
            send(op='traffic', targets=[], serial=False); phase_at = now; continue
        for name, goal in GOALS.items():
            send(op='dispatch', targets=[name], timeout=240., **goal)
        phase, sent_at = 'running', now
        trace.write(json.dumps(dict(kind='dispatched', wall=now, goals=GOALS, state=state)) + '\n')
    elif phase == 'running':
        truth = {n: m.get('truth') for n, m in members.items()}
        if all(truth.values()):
            a, b = truth['alpha'], truth['beta']
            d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            if d < closest['distance']:
                closest = dict(distance=d, wall=now - sent_at, alpha=[a['x'], a['y']], beta=[b['x'], b['y']],
                               modes={n: m['mode'] for n, m in members.items()}, zones=state.get('zones', {}).get('grants'))
        trace.write(json.dumps(dict(kind='sample', wall=now - sent_at, truth=truth,
            modes={n: [m['mode'], m['reason'], m.get('zone_wait')] for n, m in members.items()},
            zones={k: v for k, v in state.get('zones', {}).items() if k != 'declared'})) + '\n')
        history = [row for row in state.get('task_history', []) if row.get('source') == 'operator']
        terminal = [row for row in history if row.get('state') in ('succeeded', 'failed', 'canceled')]
        if len(terminal) >= 2:
            phase = 'done'; break
    if phase == 'running' and now - sent_at > budget - 20: break
if state:
    send(op='all_stop', targets=[])
events = []
try:
    for line in (Path(state['directory']) / 'events.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row.get('event') in ('zone_wait', 'zone_granted', 'contact_detected', 'traffic_yield_planned', 'conflict_resolved', 'task_finished'):
            events.append(row)
except Exception as exc:
    events.append(dict(error=str(exc)))
members = {m['id']: m for m in state.get('members', [])}
radii = sum(m['radius_m'] for m in members.values()) if members else math.nan
outcomes = [dict(member=e.get('member'), state=e.get('state'), reason=e.get('reason'), source=e.get('source'),
                 observation=e.get('observation')) for e in events if e.get('event') == 'task_finished']
succeeded = sorted(row['member'] for row in outcomes if row['state'] == 'succeeded' and row['source'] == 'operator')
contacts = [e for e in events if e.get('event') == 'contact_detected']
waits = [e for e in events if e.get('event') == 'zone_wait']
result = dict(passed=phase == 'done' and succeeded == ['alpha', 'beta'] and closest['distance'] > radii + .05 and not contacts,
              outcomes=outcomes, closest_approach=closest, radii_sum_m=radii, zone_waits=waits,
              zone_grants=[e for e in events if e.get('event') == 'zone_granted'], contacts=contacts,
              elapsed_s=time.monotonic() - sent_at if sent_at else None, declared_zones=state.get('zones', {}).get('declared'),
              session_directory=state.get('directory'), events=events)
(root / 'result.json').write_text(json.dumps(result, indent=2, default=str))
print(json.dumps({k: v for k, v in result.items() if k not in ('events', 'declared_zones')}, indent=2, default=str))
trace.close(); node.destroy_node(); rclpy.shutdown()
