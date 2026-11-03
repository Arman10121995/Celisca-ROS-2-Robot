"""Two-member L0->L1->L0 parking wave: occupied car + source-clear peer.

Baseline `8b958e97`, Jetson AGX Orin, ROS 2 Humble. This probe verifies a
two-member floor parking wave where:
  * beta is parked at L1 (occupied car — the landing peer stays put while
    the second member walks through the carousel),
  * alpha is a source-clear parked peer at L0 (seated, unobstructed),
  * alpha goes L0 -> L1 -> L0 with beta already at L1 and alpha already
    at L0, each endpoint settled and independently observed.

This is not a concurrent-proxy race: alpha and beta keep independent
ownership; beta does not move during alpha's round trip.

Artifacts written under `<experiment_output>/two_member_floor/`:
  result.json, trace.jsonl, plant.log, probe.log
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import psutil
import rclpy
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1] / 'docs/status/evidence/two-member-floor-parking-2026-11-04'
OUT = Path(os.environ.get('ROBOT_LAB_EXPERIMENT_OUTPUT', '/workspace/molar/robot_lab_runtime'))
OUT.mkdir(parents=True, exist_ok=True)
EXP_ID = os.environ.get('ROBOT_LAB_EXPERIMENT_ID', 'two-member-floor-parking-2026-11-04')
DOMAIN = os.environ.get('ROS_DOMAIN_ID', '194')
PARTITION = 'robotlab-two-member-floor-parking-2026-11-04'

root = OUT / 'two_member_floor'
root.mkdir(exist_ok=True)
plant_log = (root / 'plant.log').open('w')
plant = subprocess.Popen(['ros2', 'launch', 'robot_lab_bringup',
    'shared_world.launch.py', f'session:={ROOT / "session.yaml"}',
    f'ros_domain_id:={DOMAIN}', f'partition:={PARTITION}',
    'gui:=false', 'rviz:=false', 'navigation:=true'],
    stdout=plant_log, stderr=subprocess.STDOUT, start_new_session=True, cwd=root)
rclpy.init()
node = rclpy.create_node('two_member_floor_probe')
fleet, facility = {}, {}
node.create_subscription(String, '/fleet/state', lambda msg: fleet.update(json.loads(msg.data)), 10)
node.create_subscription(String, '/fleet/infrastructure/state',
    lambda msg: facility.update(json.loads(msg.data)), 10)
heartbeat = node.create_publisher(String, '/fleet/operator_heartbeat', 10)
command = node.create_publisher(String, '/fleet/infrastructure/command', 10)
stop = node.create_publisher(String, '/fleet/command', 10)
trace = (root / 'trace.jsonl').open('w', buffering=1)
started = time.monotonic()
last_trace = 0.0
sent, passed, reason = False, False, 'deadline'
landings = []
try:
    while time.monotonic() - started < 480.:
        rclpy.spin_once(node, timeout_sec=.03)
        now = time.monotonic()
        if plant.poll() is not None:
            raise RuntimeError('Owned plant exited')
        if plant_log.tell() > 64 * 1024 * 1024:
            raise RuntimeError('Bounded log exceeded')
        if not fleet or not facility:
            continue
        heartbeat.publish(String(data=json.dumps(
            dict(session_id=fleet.get('session_id'), operator_id='two-member-floor-probe'))))
        if now - last_trace > .25:
            trace.write(json.dumps(dict(wall=now, fleet=fleet, facility=facility), allow_nan=False) + '\n')
            last_trace = now
        alpha = fleet.get('members', {}).get('alpha', {})
        for row in history:
            if row['phase'] in ('succeeded', 'failed', 'canceled') and not any(r['id'] == row['id'] for r in landings):
                landings.append(row)
        if len(landings) >= 2:
            first, second = landings[-2], landings[-1]
            beta_at_L1 = (beta.get('floor') == 'L1'
                          and beta.get('depth_z', 0.) > 2.9)
            alpha_cleared = (alpha.get('mode') == 'idle' or alpha.get('action') in ('stand', 'stop'))
            if first['floor'] == 'L1' and second['floor'] == 'L0':
                body1 = alpha.get('truth') or alpha
                good1 = (body1 and math.dist([body1['x'], body1['y'], body1['z']],
                            (2.2, 0., 3.0)) < .15 and abs(body1['yaw']) < .15
                        and abs(body1['linear']) < .03 and abs(body1['angular']) < .05
                        and beta_at_L1 and alpha_cleared)
                if not good1:
                    raise RuntimeError(
                        f'Endpoint 1 (L1) failed: {first}; beta_at_L1={beta_at_L1}, alpha_cleared={alpha_cleared}, body={body1}')
                body2 = alpha.get('truth') or alpha
                good2 = (body2 and math.dist([body2['x'], body2['y'], body2['z']],
                            (2.3, 0., 0.0)) < .15 and abs(body2['yaw']) < .15
                        and abs(body2['linear']) < .03 and abs(body2['angular']) < .05
                        and beta_at_L1)
                if not good2:
                    raise RuntimeError(
                        f'Endpoint 2 (L0) failed: {second}; body={body2}, beta_at_L1={beta_at_L1}')
            break
        if facility.get('last_rejection'):
            raise RuntimeError('Facility rejection: ' + str(facility['last_rejection']))
except (Exception, KeyboardInterrupt) as exc:
    reason = type(exc).__name__ + ': ' + str(exc)
finally:
    if fleet and rclpy.ok():
        stop.publish(String(data=json.dumps(dict(session_id=fleet.get('session_id'),
            op='all_stop'))))
        rclpy.spin_once(node, timeout_sec=.1)
    result = dict(passed=passed, reason=reason,
                  wall_seconds=time.monotonic() - started,
                  landings=landings, final_fleet=fleet, final_facility=facility)
    (root / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps({k: v for k, v in result.items() if k not in ('final_fleet', 'final_facility')},
                     indent=2), flush=True)
    children = psutil.Process(plant.pid).children(recursive=True) if plant.poll() is None else []
    if plant.poll() is None:
        os.killpg(plant.pid, signal.SIGINT)
        try:
            plant.wait(timeout=8.)
        except subprocess.TimeoutExpired:
            os.killpg(plant.pid, signal.SIGKILL)
    for child in children:
        try:
            if child.is_running():
                child.kill()
        except psutil.NoSuchProcess:
            pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    trace.close()
    plant_log.close()
alpha = fleet.get('members', {}).get('alpha', {})
beta = fleet.get('members', {}).get('beta', {})
if (not sent and alpha.get('mode') == 'idle' and beta.get('mode') == 'idle'
        and facility.get('ready', False) and facility.get('operator_id') == 'two-member-floor-probe'
        and facility.get('operator_fresh', False)):
    command.publish(String(
        data=json.dumps(dict(session_id=fleet['session_id'],
            operator_id='two-member-floor-probe',
            op='transport_queue',
            requests=[dict(robot='alpha', floor='L1', parking='auto'),
                      dict(robot='alpha', floor='L0', parking='auto')]))));
    sent = True
history = (facility.get('transport_queue') or {}).get('history', [])
if any(row['phase'] in ('failed', 'canceled') for row in history):
    raise RuntimeError('Queue interrupted: ' + str(history))
