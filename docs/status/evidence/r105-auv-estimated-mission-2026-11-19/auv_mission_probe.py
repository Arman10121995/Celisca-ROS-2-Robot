"""R10.5: a Tethys mission planned and flown from the sensor estimate, judged by simulator truth.

The estimator (marine_sensors estimate: IMU, pressure, DVL) feeds the mission
controller (marine_navigation); simulator truth (/marine/odometry) is only
recorded here. Arms, plans to TARGET from the estimate, executes, and
reports the estimate error and the truth distance to the goal at the end.
"""
import json, math, os, time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
TARGET = [float(v) for v in os.environ.get('CASE_TARGET', '1.5,0.0,-2.0').split(',')]
root = Path(os.environ['CASE_OUTPUT']); root.mkdir(parents=True, exist_ok=True)
trace = (root / 'trace.jsonl').open('w', buffering=1)
rclpy.init(); node = rclpy.create_node('auv_mission_probe')
native, mission, truth, estimate = {}, {}, {}, {}
def odom(store):
    def receive(m):
        p = m.pose.pose.position
        store.update(xyz=[p.x, p.y, p.z], t=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return receive
node.create_subscription(String, '/marine/state', lambda m: native.update(json.loads(m.data)), 10)
node.create_subscription(String, '/marine/mission/state', lambda m: mission.update(json.loads(m.data)), 10)
node.create_subscription(Odometry, '/marine/odometry', odom(truth), qos_profile_sensor_data)
node.create_subscription(Odometry, '/marine/estimated_odometry', odom(estimate), qos_profile_sensor_data)
command = node.create_publisher(String, '/marine/command', 10)
mission_cmd = node.create_publisher(String, '/marine/mission/command', 10)
hb = node.create_publisher(String, '/marine/operator_heartbeat', 10)
operator = 'auv-mission-probe'
def send(pub, **c): pub.publish(String(data=json.dumps(dict(session_id=native['session_id'], operator_id=operator, **c))))
phase, started, last_hb, sent_at, errors, statuses = 'wait', time.monotonic(), 0., None, [], []
result = dict(target=TARGET)
while time.monotonic() - started < float(os.environ.get('CASE_BUDGET_S', '600')):
    rclpy.spin_once(node, timeout_sec=.02)
    now = time.monotonic()
    if not native: continue
    if now - last_hb > .15:
        hb.publish(String(data=json.dumps(dict(session_id=native['session_id'], operator_id=operator)))); last_hb = now
    if truth and estimate:
        errors.append(math.dist(truth['xyz'], estimate['xyz']))
    if mission.get('status') and (not statuses or statuses[-1] != mission['status']):
        statuses.append(mission['status'])
        trace.write(json.dumps(dict(kind='status', wall=now - started, status=mission['status'], reason=mission.get('reason'))) + '\n')
    if phase == 'wait':
        if native.get('operator_id') == operator and native.get('ready') and mission.get('ready'):
            send(command, op='arm'); phase, sent_at = 'arming', now
    elif phase == 'arming':
        if native.get('enabled'):
            result['start_truth'], result['start_estimate'] = truth.get('xyz'), estimate.get('xyz')
            send(mission_cmd, op='plan', x=TARGET[0], y=TARGET[1], z=TARGET[2]); phase, sent_at = 'planning', now
        elif now - sent_at > 3: send(command, op='arm'); sent_at = now
    elif phase == 'planning':
        if mission.get('status') == 'planned':
            result['route'] = mission.get('route'); send(mission_cmd, op='execute'); phase, sent_at = 'executing', now
        elif mission.get('status') in ('idle', 'failed', 'canceled') and now - sent_at > 2:
            result['plan_failed'] = mission.get('reason'); break
    elif phase == 'executing':
        if int(now * 2) != int((now - .02) * 2):
            trace.write(json.dumps(dict(kind='sample', wall=now - started, truth=truth.get('xyz'), estimate=estimate.get('xyz'),
                                        status=mission.get('status'), std=mission.get('position_std_m'))) + '\n')
        if mission.get('status') in ('completed', 'failed', 'canceled', 'idle') and now - sent_at > 3:
            break
result.update(statuses=statuses, final_status=mission.get('status'), final_reason=mission.get('reason'),
              end_truth=truth.get('xyz'), end_estimate=estimate.get('xyz'),
              truth_to_goal_m=math.dist(truth['xyz'], TARGET) if truth else None,
              estimate_to_goal_m=math.dist(estimate['xyz'], TARGET) if estimate else None,
              estimate_error_end_m=errors[-1] if errors else None, estimate_error_max_m=max(errors) if errors else None,
              position_std_end_m=mission.get('position_std_m'), localization_mode=mission.get('localization_mode'),
              mission_seconds=time.monotonic() - sent_at if sent_at else None)
send(command, op='stop')
(root / 'result.json').write_text(json.dumps(result, indent=2, default=str))
print(json.dumps({k: v for k, v in result.items() if k != 'route'}, indent=1, default=str))
