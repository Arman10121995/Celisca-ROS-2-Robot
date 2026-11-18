"""fleet_ward (no lift): two members cross the door at once, then each docks and charges (R10.2).

Phase 1 dispatches alpha west->east and beta east->west through the one
sliding door, records truth separation, door states and zone events, and
needs both tasks to succeed without contact or a near pass (radii + 5 cm).
Phase 2 docks alpha at east_charger and beta at west_charger through the
infrastructure's dock command, and needs each to be docked (pose within
tolerance, still for 1 s) with its modeled energy rising.
"""
import json, math, os, time
from pathlib import Path
import rclpy
from std_msgs.msg import String
CROSS = dict(alpha=dict(x=2.8, y=1.0, yaw=0.), beta=dict(x=-2.8, y=-1.0, yaw=math.pi))
DOCKS = dict(alpha='east_charger', beta='west_charger')
root = Path(os.environ['CASE_OUTPUT']); root.mkdir(parents=True, exist_ok=True)
trace = (root / 'trace.jsonl').open('w', buffering=1)
rclpy.init(); node = rclpy.create_node('ward_probe')
state, infra = {}, {}
node.create_subscription(String, '/fleet/state', lambda m: state.update(json.loads(m.data)), 10)
node.create_subscription(String, '/fleet/infrastructure/state', lambda m: infra.update(json.loads(m.data)), 10)
heartbeat = node.create_publisher(String, '/fleet/operator_heartbeat', 10)
command = node.create_publisher(String, '/fleet/command', 10)
facility = node.create_publisher(String, '/fleet/infrastructure/command', 10)
operator = 'ward-probe'
phase, started, sent_at, dock_at = 'wait', time.monotonic(), None, None
closest = dict(distance=math.inf)
door_modes, energy_at_dock, docked_at = set(), {}, {}
budget = float(os.environ.get('CASE_BUDGET_S', '900'))
def send(publisher=None, **c):
    (publisher or command).publish(String(data=json.dumps(dict(session_id=state['session_id'], operator_id=operator, **c))))
def finished():
    history = [row for row in state.get('task_history', []) if row.get('source') == 'operator']
    return [row for row in history if row.get('state') in ('succeeded', 'failed', 'canceled')]
while time.monotonic() - started < budget:
    rclpy.spin_once(node, timeout_sec=.05)
    if not state: continue
    heartbeat.publish(String(data=json.dumps(dict(session_id=state['session_id'], operator_id=operator))))
    members = {m['id']: m for m in state.get('members', [])}
    now = time.monotonic()
    for name, row in (infra.get('doors') or {}).items():
        door_modes.add((name, row.get('mode')))
    if phase == 'wait':
        if (state.get('operator_id') == operator and len(members) == 2 and infra.get('ready')
                and all(m.get('navigation_ready') and m.get('ready') for m in members.values())):
            send(op='traffic', targets=[], serial=False); phase = 'traffic'; phase_at = now
    elif phase == 'traffic' and now - phase_at > 1.:
        for name, goal in CROSS.items():
            send(op='dispatch', targets=[name], timeout=300., **goal)
        phase, sent_at = 'crossing', now
        trace.write(json.dumps(dict(kind='dispatched', wall=0., goals=CROSS)) + '\n')
    elif phase in ('crossing', 'docking'):
        truth = {n: m.get('truth') for n, m in members.items()}
        if phase == 'crossing' and all(truth.values()):
            a, b = truth['alpha'], truth['beta']
            d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            if d < closest['distance']:
                closest = dict(distance=d, wall=now - sent_at, alpha=[a['x'], a['y']], beta=[b['x'], b['y']])
        trace.write(json.dumps(dict(kind='sample', phase=phase, wall=now - sent_at, truth=truth,
            modes={n: [m['mode'], m['reason'], m.get('zone_wait')] for n, m in members.items()},
            doors=infra.get('doors'), docking=infra.get('docking'), energy=infra.get('energy'))) + '\n')
        if phase == 'crossing' and len(finished()) >= 2:
            if not all(row['state'] == 'succeeded' for row in finished()):
                break
            for name, dock in DOCKS.items():
                send(publisher=facility, op='dock', robot=name, dock=dock, request_id='ward-' + name)
            phase, dock_at = 'docking', now
        elif phase == 'docking':
            for name, item in (infra.get('docking') or {}).items():
                if item.get('docked') and name not in docked_at:
                    docked_at[name] = now - dock_at
                    energy_at_dock[name] = dict(first=(infra.get('energy') or {}).get(name), at=now)
            if set(docked_at) == set(DOCKS) and all(now - row['at'] > 10. for row in energy_at_dock.values()):
                for name in DOCKS:
                    energy_at_dock[name]['after_10s'] = (infra.get('energy') or {}).get(name)
                phase = 'done'; break
            if now - dock_at > 400: break
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
outcomes = [dict(member=e.get('member'), state=e.get('state'), reason=e.get('reason'), source=e.get('source')) for e in events if e.get('event') == 'task_finished']
contacts = [e for e in events if e.get('event') == 'contact_detected']
def charging(row):
    # fleet_energy.EnergyEstimate.state: the modeled fraction rises while docked.
    first, after = (row or {}).get('first') or {}, (row or {}).get('after_10s') or {}
    return (first.get('fraction') is not None and after.get('fraction') is not None
            and after['fraction'] > first['fraction'] and (after.get('charging_W') or 0) > 0)
# The dock commands dispatch operator tasks too: every operator task must
# succeed, and each member must have crossed (its first task).
operator_tasks = [r for r in outcomes if r['source'] == 'operator']
crossed = sorted({r['member'] for r in operator_tasks[:2] if r['state'] == 'succeeded'})
result = dict(passed=phase == 'done' and crossed == ['alpha', 'beta'] and all(r['state'] == 'succeeded' for r in operator_tasks)
              and closest['distance'] > radii + .05 and not contacts
              and all(charging(energy_at_dock.get(n)) for n in DOCKS),
              crossing=dict(outcomes=outcomes, closest_approach=closest, radii_sum_m=radii, contacts=contacts,
                            zone_waits=[e for e in events if e.get('event') == 'zone_wait'],
                            door_modes_seen=sorted(door_modes)),
              docking=dict(docked_after_s=docked_at, energy=energy_at_dock, charging={n: charging(energy_at_dock.get(n)) for n in DOCKS}),
              phase=phase, session_directory=state.get('directory'), events=events)
(root / 'result.json').write_text(json.dumps(result, indent=2, default=str))
print(json.dumps({k: v for k, v in result.items() if k != 'events'}, indent=1, default=str)[:3000])
