"""R10.3: distributed dispersion and aggregation, then sensed coverage with an actual member loss.

Loss injection: SIGINT to gamma's own AMCL process (exact /proc cmdline match),
so its localization transform stops and the controller sees a stale member.
"""
import itertools, json, math, os, signal, time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
root = Path(os.environ['CASE_OUTPUT']); root.mkdir(parents=True, exist_ok=True)
trace = (root / 'trace.jsonl').open('w', buffering=1)
rclpy.init(); node = rclpy.create_node('swarm_distributed_probe')
state = {}
node.create_subscription(String, '/fleet/state', lambda m: state.update(json.loads(m.data)), 10)
scan_min = {}
for _name in ('alpha', 'beta', 'gamma'):
    node.create_subscription(LaserScan, '/%s/scan' % _name, lambda m, k=_name: scan_min.__setitem__(
        k, [min((v for v in m.ranges if math.isfinite(v)), default=None), sum(1 for v in m.ranges if v == -math.inf)]),
        rclpy.qos.qos_profile_sensor_data)
heartbeat = node.create_publisher(String, '/fleet/operator_heartbeat', 10)
command = node.create_publisher(String, '/fleet/command', 10)
operator = 'swarm-distributed-probe'
names = ('alpha', 'beta', 'gamma')
def send(**c): command.publish(String(data=json.dumps(dict(session_id=state['session_id'], operator_id=operator, **c))))
def spin(seconds, phase, until=None):
    end = time.monotonic() + seconds
    rows = []
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
        if not state: continue
        heartbeat.publish(String(data=json.dumps(dict(session_id=state['session_id'], operator_id=operator))))
        truth = {m['id']: m.get('truth') for m in state.get('members', [])}
        if all(truth.get(n) for n in names) and (not rows or time.monotonic() - rows[-1]['wall'] > .25):
            pairs = {a + '-' + b: math.hypot(truth[a]['x'] - truth[b]['x'], truth[a]['y'] - truth[b]['y'])
                     for a, b in itertools.combinations(names, 2)}
            row = dict(phase=phase, wall=time.monotonic(), sim=state.get('sim_time'), pairs=pairs,
                       truth={n: [truth[n]['x'], truth[n]['y']] for n in names},
                       modes={m['id']: [m['mode'], m['reason']] for m in state['members']},
                       coverage=(state.get('sensed_coverage') or {}).get('fraction'), lost=state.get('lost_members'),
                       scan_min=dict(scan_min), swarm_state={m['id']: m.get('swarm_state') for m in state['members']})
            rows.append(row); trace.write(json.dumps(row) + '\n')
        if until and until(): break
    return rows
def summary(rows):
    if not rows: return None
    first, last = rows[0]['pairs'], rows[-1]['pairs']
    return dict(samples=len(rows), mean_start_m=sum(first.values()) / 3, mean_end_m=sum(last.values()) / 3,
                min_pair_m=min(min(r['pairs'].values()) for r in rows), end_pairs=last,
                duration_s=rows[-1]['wall'] - rows[0]['wall'])
def kill_amcl(member):
    for pid in os.listdir('/proc'):
        if not pid.isdigit(): continue
        try: cmd = Path('/proc', pid, 'cmdline').read_bytes().split(b'\0')
        except OSError: continue
        if any(c.endswith(b'/amcl') for c in cmd) and b'__ns:=/' + member.encode() in cmd:
            os.kill(int(pid), signal.SIGINT); return int(pid)
    return None
result = dict(phases={})
start = time.monotonic()
while time.monotonic() - start < 240:
    spin(.5, 'wait')
    members = state.get('members', [])
    if state.get('operator_id') == operator and len(members) == 3 and all(m.get('ready') and m.get('navigation_ready') for m in members):
        break
result['ready_s'] = time.monotonic() - start
phases = [(p.split(':')[0], float(p.split(':')[1])) for p in os.environ.get('CASE_PHASES', 'local_dispersion:35,local_aggregation:70').split(',')]
for behaviour, seconds in phases:
    send(op='swarm', targets=list(names), controller=behaviour, loss_policy='stop')
    rows = spin(seconds, behaviour)
    send(op='stop', targets=list(names))
    spin(3., 'stop')
    result['phases'][behaviour] = dict(summary(rows), states=sorted({r['modes'][n][1] for r in rows for n in names})[:20])
if os.environ.get('CASE_SKIP_COVERAGE'):
    (root / 'result.json').write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result['phases'], indent=2, default=str)); raise SystemExit(0)
send(op='traffic', targets=[], serial=False); spin(1.5, 'traffic')
send(op='coverage', targets=list(names), sensed=True, x=0., y=0., width=6., height=6., goal_fraction=.9,
     sensor_range=2., loss_policy='replan')
cover_start = time.monotonic()
rows = spin(40., 'coverage')
killed = kill_amcl('gamma'); result['killed_amcl_pid'] = killed; killed_at = time.monotonic()
rows += spin(420., 'coverage_after_loss', until=lambda: bool((state.get('sensed_coverage') or {}).get('finished')))
send(op='all_stop', targets=[])
sensed = state.get('sensed_coverage') or {}
events = []
for line in (Path(state['directory']) / 'events.jsonl').read_text().splitlines():
    row = json.loads(line)
    if row.get('event') in ('member_lost_replanned', 'sensed_coverage_started', 'sensed_coverage_finished', 'contact_detected',
                            'all_stop', 'command_rejected', 'zone_wait', 'traffic_yield_planned'):
        events.append(row)
tasks = [json.loads(l) for l in (Path(state['directory']) / 'events.jsonl').read_text().splitlines() if '"task_finished"' in l]
result['phases']['sensed_coverage'] = dict(summary(rows), final=sensed, loss_at_s=killed_at - cover_start,
    coverage_at_loss=next((r['coverage'] for r in reversed(rows) if r['wall'] <= killed_at), None),
    elapsed_s=time.monotonic() - cover_start,
    tasks={s: sum(1 for t in tasks if t.get('state') == s and t.get('source') == 'sensed_coverage') for s in ('succeeded', 'failed', 'canceled')})
result['events'] = events
result['contacts'] = [e for e in events if e.get('event') == 'contact_detected']
result['session_directory'] = state.get('directory')
d, a, c = (result['phases'][k] for k in ('local_dispersion', 'local_aggregation', 'sensed_coverage'))
radii = .118 + .18
result['checks'] = dict(
    dispersion_spread=d['mean_end_m'] - d['mean_start_m'], aggregation_change=a['mean_end_m'] - a['mean_start_m'],
    min_separation_m=min(d['min_pair_m'], a['min_pair_m']), loss_replanned=any(e.get('event') == 'member_lost_replanned' for e in events),
    coverage_finished=sensed.get('finished'), coverage_fraction=sensed.get('fraction'), contacts=len(result['contacts']))
(root / 'result.json').write_text(json.dumps(result, indent=2, default=str))
print(json.dumps(dict(checks=result['checks'], phases={k: {kk: vv for kk, vv in v.items() if kk not in ('states', 'final')} for k, v in result['phases'].items()}), indent=2, default=str))
trace.close(); node.destroy_node(); rclpy.shutdown()
