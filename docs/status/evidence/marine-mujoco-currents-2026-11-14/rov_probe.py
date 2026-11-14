"""R10.4: same lab_rov controller on a backend: hold, surge, yaw, current disturbance, operator loss.

Pose comes from /marine/odometry (the plant observation both backends publish);
thrust from /marine/state. Phases are timed on the simulation clock.
"""
import json, math, os, time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
root = Path(os.environ['CASE_OUTPUT']); root.mkdir(parents=True, exist_ok=True)
trace = (root / 'trace.jsonl').open('w', buffering=1)
rclpy.init(); node = rclpy.create_node('rov_backend_probe')
state, body = {}, {}
def pose(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    body.update(xyz=[p.x, p.y, p.z], yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                t=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
node.create_subscription(String, '/marine/state', lambda m: state.update(json.loads(m.data)), 10)
node.create_subscription(Odometry, '/marine/odometry', pose, qos_profile_sensor_data)
command = node.create_publisher(String, '/marine/command', 10)
hb = node.create_publisher(String, '/marine/operator_heartbeat', 10)
drive = node.create_publisher(Twist, '/marine/key_vel', 10)
current = node.create_publisher(Vector3, '/marine/current', 10)
operator = 'rov-backend-probe'
def op(name, **extra): command.publish(String(data=json.dumps(dict(session_id=state['session_id'], operator_id=operator, op=name, **extra))))
PHASES = [('hold', 12., (0, 0, 0, 0), (0., 0.)), ('surge', 10., (.2, 0, 0, 0), (0., 0.)), ('settle', 6., (0, 0, 0, 0), (0., 0.)),
          ('yaw', 6., (0, 0, 0, .3), (0., 0.)), ('hold2', 10., (0, 0, 0, 0), (0., 0.)),
          ('current', 15., (0, 0, 0, 0), (0., .1)), ('loss', 6., None, (0., 0.))]
rows, phase, index, start_t, armed_at = [], 'wait', -1, None, None
summary = {}
wall = time.monotonic()
def sample():
    rows.append(dict(phase=phase, t=body['t'], xyz=body['xyz'], yaw=body['yaw'], enabled=state.get('enabled'),
                     reason=state.get('reason'), forces=state.get('forces') or state.get('thrust_N')))
    trace.write(json.dumps(rows[-1]) + '\n')
last_hb = last_drive = 0.
while time.monotonic() - wall < 400:
    rclpy.spin_once(node, timeout_sec=.01)
    now = time.monotonic()
    if not state or not body: continue
    heartbeat = phase != 'loss'
    if heartbeat and now - last_hb > .15:
        hb.publish(String(data=json.dumps(dict(session_id=state['session_id'], operator_id=operator)))); last_hb = now
    if phase == 'wait':
        if state.get('operator_id') == operator and state.get('ready'):
            op('arm'); phase, armed_at = 'arming', now
        continue
    if phase == 'arming':
        if state.get('enabled'):
            index = 0; phase = PHASES[0][0]; start_t = body['t']; start_xyz, start_yaw = body['xyz'][:], body['yaw']
        elif now - armed_at > 3:
            op('arm'); armed_at = now
        continue
    name, duration, vel, cur = PHASES[index]
    if now - last_drive > .05:
        last_drive = now
        if vel is not None:
            t = Twist(); t.linear.x, t.linear.y, t.linear.z, t.angular.z = map(float, vel); drive.publish(t)
        v = Vector3(); v.x, v.y = cur; current.publish(v)
        sample()
    if body['t'] - start_t >= duration:
        seg = [r for r in rows if r['phase'] == name]
        if seg:
            d = [r['xyz'] for r in seg]
            summary[name] = dict(duration_s=seg[-1]['t'] - seg[0]['t'], start=d[0], end=d[-1],
                displacement_m=math.dist(d[0][:2], d[-1][:2]), depth_change_m=d[-1][2] - d[0][2],
                max_depth_dev_m=max(abs(x[2] - d[0][2]) for x in d), max_xy_drift_m=max(math.dist(d[0][:2], x[:2]) for x in d),
                yaw_change_rad=math.atan2(math.sin(seg[-1]['yaw'] - seg[0]['yaw']), math.cos(seg[-1]['yaw'] - seg[0]['yaw'])),
                speed_m_s=math.dist(d[0][:2], d[-1][:2]) / max(seg[-1]['t'] - seg[0]['t'], 1e-6),
                enabled_end=seg[-1]['enabled'], reason_end=seg[-1]['reason'])
        index += 1
        if index >= len(PHASES): break
        phase = PHASES[index][0]; start_t = body['t']
op('stop') if state else None
result = dict(backend=os.environ.get('CASE_BACKEND'), phases=summary, samples=len(rows),
              loss=dict(disarmed=summary.get('loss', {}).get('enabled_end') is False, reason=summary.get('loss', {}).get('reason_end')))
(root / 'result.json').write_text(json.dumps(result, indent=2, default=str))
print(json.dumps(result, indent=1, default=str))
trace.close(); node.destroy_node(); rclpy.shutdown()
