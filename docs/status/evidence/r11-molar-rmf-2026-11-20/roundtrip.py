"""Drive a simulated MOLAR round trip through the normal RMF control topic."""
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rmf_lift_msgs.msg import LiftRequest
from std_msgs.msg import String


def run(output, budget):
    rclpy.init()
    node = rclpy.create_node('molar_rmf_roundtrip_probe')
    states, lift_events, arrivals = {}, [], []
    for name, topic in (('fleet', '/fleet/state'), ('facility', '/fleet/infrastructure/state'),
                        ('rmf', '/fleet/rmf/state')):
        node.create_subscription(String, topic,
                                 lambda message, key=name: states.update({key: json.loads(message.data)}), 10)
    node.create_subscription(LiftRequest, '/lift_requests',
        lambda m: lift_events.append(dict(at=time.monotonic(), floor=m.destination_floor,
            door=m.door_state, type=m.request_type, session=m.session_id)), 100)
    control = node.create_publisher(String, '/fleet/rmf/control', 10)
    stop = node.create_publisher(String, '/fleet/command', 10)
    started = time.monotonic()
    phase, settled, last = 'wait', None, 0.
    reason = 'Deadline expired'

    def send(op, **data):
        control.publish(String(data=json.dumps(dict(session_id=states['fleet']['session_id'], op=op, **data))))

    try:
        with (output/'trace.jsonl').open('w', buffering=1) as trace:
            while time.monotonic()-started < budget:
                rclpy.spin_once(node, timeout_sec=.05)
                if len(states) != 3:
                    continue
                now = time.monotonic()
                fleet, facility, rmf = (states[key] for key in ('fleet', 'facility', 'rmf'))
                member = next((m for m in fleet['members'] if m['id'] == 'molar'), None)
                if member is None:
                    continue
                if now-last >= .3:
                    trace.write(json.dumps(dict(wall=now-started, phase=phase, **states))+'\n')
                    last = now
                if phase == 'wait' and member.get('navigation_ready') and 'molar' in rmf.get('robots', []):
                    send('enable')
                    phase = 'enable'
                elif phase == 'enable' and facility.get('rmf_control_ready') and not fleet['traffic']['serial']:
                    send('patrol', robot='molar', waypoint='L1_landing')
                    phase = 'L1'
                elif phase in ('L1', 'L0'):
                    if not rmf.get('enabled'):
                        raise RuntimeError('RMF ownership was lost')
                    body = member.get('truth') or {}
                    map_path = facility.get('active_robot_maps', {}).get('molar') or ''
                    expected_map = 'map_L1.yaml' if phase == 'L1' else 'map.yaml'
                    heading_error = math.remainder(body.get('yaw', 0.)-math.pi, 2*math.pi)
                    arrived = (facility.get('robot_floors', {}).get('molar') == phase
                        and Path(map_path).name == expected_map
                        and math.hypot(body.get('x', math.inf)-2.1, body.get('y', math.inf)) < .15
                        and abs(heading_error) < .15
                        and abs(body.get('linear', math.inf)) < .03
                        and abs(body.get('angular', math.inf)) < .05
                        and member['mode'] == 'idle' and not rmf.get('routes', {}).get('molar'))
                    settled = (settled or now) if arrived else None
                    if settled is not None and now-settled >= 1.:
                        arrivals.append(dict(floor=phase, body=body, map=map_path, wall_s=now-started))
                        if phase == 'L0':
                            phase, reason = 'completed', 'Independent pose, motion and map checks passed on both floors'
                            break
                        send('patrol', robot='molar', waypoint='L0_landing')
                        phase, settled = 'L0', None
    except (Exception, KeyboardInterrupt) as exc:
        reason = repr(exc)
    finally:
        if rclpy.ok() and states.get('fleet'):
            stop.publish(String(data=json.dumps(dict(session_id=states['fleet']['session_id'],
                                                      op='all_stop', targets=[]))))
        report = dict(phase=phase, reason=reason, arrivals=arrivals, lift_events=lift_events, **states)
        (output/'mission.json').write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return phase == 'completed'


if __name__ == '__main__':
    raise SystemExit(0 if run(Path(sys.argv[1]), float(sys.argv[2])) else 1)
