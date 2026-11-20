"""R10.5: a Tethys mission planned and flown from the sensor estimate, judged by simulator truth.

2026-11-20 copy for the long-route work: samples also record the native
attitude, fin targets/joints, thrust and mission command at 4 Hz.

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
# Optional further legs, flown one after another from where the last ended
# ("x,y,z;x,y,z"); each leg is planned from the estimate and must complete.
LEGS = [[float(v) for v in leg.split(',')] for leg in os.environ.get('CASE_TARGETS', '').split(';') if leg]
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
localized_seen = False
result = dict(target=TARGET)
while time.monotonic() - started < float(os.environ.get('CASE_BUDGET_S', '600')):
    rclpy.spin_once(node, timeout_sec=.02)
    now = time.monotonic()
    if not native: continue
    if now - last_hb > .15:
        hb.publish(String(data=json.dumps(dict(session_id=native['session_id'], operator_id=operator)))); last_hb = now
    localized_seen = localized_seen or bool(mission.get('localized'))
    if truth and estimate:
        errors.append(math.dist(truth['xyz'], estimate['xyz']))
    native_key = (native.get('enabled'), native.get('reason'), native.get('input_owner'))
    if native_key != result.get('_native_key'):
        result['_native_key'] = native_key
        trace.write(json.dumps(dict(kind='native', wall=now - started, enabled=native_key[0], reason=native_key[1],
                                    input_owner=native_key[2])) + '\n')
    if mission.get('status') and (not statuses or statuses[-1] != mission['status']):
        statuses.append(mission['status'])
        trace.write(json.dumps(dict(kind='status', wall=now - started, status=mission['status'], reason=mission.get('reason'))) + '\n')
    if phase == 'wait':
        if native.get('operator_id') == operator and native.get('ready') and mission.get('ready'):
            send(command, op='arm'); phase, sent_at = 'arming', now
    elif phase == 'arming':
        if native.get('enabled') and mission.get('status') not in ('executing',):
            result.setdefault('start_truth', truth.get('xyz')); result.setdefault('start_estimate', estimate.get('xyz'))
            send(mission_cmd, op='plan', x=TARGET[0], y=TARGET[1], z=TARGET[2]); phase, sent_at = 'planning', now
        elif now - sent_at > 3: send(command, op='arm'); sent_at = now
    elif phase == 'planning':
        if mission.get('status') == 'planned':
            result['route'] = mission.get('route'); send(mission_cmd, op='execute'); phase, sent_at = 'executing', now
        elif mission.get('status') in ('idle', 'failed', 'canceled') and now - sent_at > 2:
            result['plan_failed'] = mission.get('reason'); break
    elif phase == 'executing':
        if int(now * 4) != int((now - .02) * 4):
            trace.write(json.dumps(dict(kind='sample', wall=now - started, truth=truth.get('xyz'), estimate=estimate.get('xyz'),
                                        status=mission.get('status'), std=mission.get('position_std_m'),
                                        rpy=native.get('rpy'), fins=native.get('fin_targets_rad'), joints=native.get('actual_joints_rad'),
                                        thrust=native.get('commanded_thruster_forces_N'), surge=native.get('measured_forward_surge_m_s'),
                                        guidance=native.get('cruise_guidance_active'), target_depth=native.get('target_depth'),
                                        command=mission.get('command'), cross_track=mission.get('cross_track_m'))) + '\n')
        if mission.get('status') in ('completed', 'failed', 'canceled', 'idle') and now - sent_at > 3:
            if LEGS and mission.get('status') == 'completed':
                result.setdefault('legs', []).append(dict(target=TARGET, end_truth=truth.get('xyz'), end_estimate=estimate.get('xyz'),
                    truth_to_goal_m=math.dist(truth['xyz'], TARGET), status=mission.get('status'), reason=mission.get('reason'),
                    route_length_m=sum(math.dist(a[:2], b[:2]) for a, b in zip(result['route'], result['route'][1:]))))
                # Braking ends with the native controller disarmed: rearm, then plan
                # the next leg from the estimate (the arming phase sends the plan).
                # The completed mission then stops the native controller; wait for
                # that Stop before rearming, or the arm lands first and is undone.
                TARGET = LEGS.pop(0); result['target'] = TARGET
                phase, sent_at = 'between_legs', now
            elif os.environ.get('CASE_SAVE_MAP') and mission.get('status') == 'completed':
                result['mission_status'] = mission.get('status')
                send(mission_cmd, op='save_map'); phase, saved_at = 'saving', now
            else:
                break
    elif phase == 'between_legs':
        if not native.get('enabled') or now - sent_at > 3:
            send(command, op='arm'); phase, sent_at = 'arming', now
    elif phase == 'saving':
        if mission.get('mapping_status') in ('saved', 'failed') or now - saved_at > 120:
            result['saved_map'], result['mapping_status'] = mission.get('saved_map'), mission.get('mapping_status')
            break
result.update(statuses=statuses, final_status=result.get('mission_status', mission.get('status')), final_reason=mission.get('reason'),
              end_truth=truth.get('xyz'), end_estimate=estimate.get('xyz'),
              truth_to_goal_m=math.dist(truth['xyz'], TARGET) if truth else None,
              estimate_to_goal_m=math.dist(estimate['xyz'], TARGET) if estimate else None,
              estimate_error_end_m=errors[-1] if errors else None, estimate_error_max_m=max(errors) if errors else None,
              position_std_end_m=mission.get('position_std_m'), localization_mode=mission.get('localization_mode'),
              localized_end=mission.get('localized'), localized_seen=localized_seen,
              mission_seconds=time.monotonic() - sent_at if sent_at else None)
send(command, op='stop')
(root / 'result.json').write_text(json.dumps(result, indent=2, default=str))
print(json.dumps({k: v for k, v in result.items() if k != 'route'}, indent=1, default=str))
