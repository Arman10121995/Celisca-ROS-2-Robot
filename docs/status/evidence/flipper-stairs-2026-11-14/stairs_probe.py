"""R10.7: drive the articulated tracked vehicle at the terrain_stairs flight (0.18 m rises,
1.1 m treads) with fixed or automatic flippers. Truth from /odom/ground_truth."""
import json, math, os, time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
root = Path(os.environ['CASE_OUTPUT']); root.mkdir(parents=True, exist_ok=True)
trace = (root / 'trace.jsonl').open('w', buffering=1)
rclpy.init(); node = rclpy.create_node('flipper_stairs_probe')
state, body = {}, {}
def truth(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    body.update(x=p.x, y=p.y, z=p.z, t=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                pitch=math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))),
                roll=math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
node.create_subscription(String, '/tracked/state', lambda m: state.update(json.loads(m.data)), 10)
node.create_subscription(Odometry, '/odom/ground_truth', truth, qos_profile_sensor_data)
drive = node.create_publisher(Twist, '/key_vel', 10)
flip = node.create_publisher(String, '/tracked/flipper_command', 10)
mode = os.environ.get('CASE_FLIPPERS', 'fixed'); duration = float(os.environ.get('CASE_DURATION', '70'))
start = time.monotonic(); began = None; rows = []
while time.monotonic() - start < duration + 120:
    rclpy.spin_once(node, timeout_sec=.05)
    if not state or not body: continue
    if began is None and not state.get('ready'): continue
    if began is None:
        if mode == 'auto':
            flip.publish(String(data=json.dumps(dict(session_id=state['session_id'], robot_id=state['robot_id'], op='auto', enabled=True))))
            if not state['flippers'].get('auto'): continue
        began, t0, start_body = time.monotonic(), body['t'], dict(body)
    t = Twist(); t.linear.x = .15; drive.publish(t)
    row = dict(t=body['t'] - t0, x=body['x'], y=body['y'], z=body['z'], ready=state.get('ready'), pitch=body['pitch'], roll=body['roll'],
               flippers=state['flippers'].get('positions'), auto=state['flippers'].get('auto_state'), command=state.get('command'))
    rows.append(row); trace.write(json.dumps(row) + '\n')
    if body['t'] - t0 >= duration: break
drive.publish(Twist())
top = max(rows, key=lambda r: r['z']) if rows else {}
result = dict(mode=mode, duration_s=rows[-1]['t'] if rows else None, start=dict(x=start_body['x'], y=start_body['y'], z=start_body['z']) if began else None,
              end=dict(x=rows[-1]['x'], y=rows[-1]['y'], z=rows[-1]['z']) if rows else None, max_z=top.get('z'), max_pitch=max(r['pitch'] for r in rows) if rows else None,
              min_pitch=min(r['pitch'] for r in rows) if rows else None, max_roll=max(abs(r['roll']) for r in rows) if rows else None,
              steps_climbed=round(((rows[-1]['z'] - start_body['z']) / .18)) if rows else None,
              states=sorted({(r['auto'] or {}).get('state') for r in rows} - {None}),
              not_ready_samples=sum(1 for r in rows if not r['ready']),
              on_tread=bool(rows) and rows[-1]['z'] > start_body['z'] + .15 and abs(rows[-1]['pitch']) < .1)
(root / 'result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=1))
trace.close(); node.destroy_node(); rclpy.shutdown()
