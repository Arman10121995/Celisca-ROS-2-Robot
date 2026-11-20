#!/usr/bin/env python3
"""Native fleet lift transport L0 -> L1 -> L0 for one member (R11 MOLAR, 2026-11-20).

The November 2 Bumperbot driver (automatic-floor-transport-2026-11-02/
native-roundtrip-driver.py) with the member id as CASE_ROBOT. The operator only
sends Facility ``transport`` requests; the infrastructure's LiftTransport drives
the Nav2 approach, boarding, ride, floor/map change and settled exit, and this
driver records each landing by truth. CASE_OUTPUT is a new directory.
"""
import json, os, time
from pathlib import Path
import rclpy
from std_msgs.msg import String

ROBOT = os.environ.get('CASE_ROBOT', 'molar')
OPERATOR = 'transport-trial'
DEADLINE = float(os.environ.get('CASE_DEADLINE_S', '900'))


def main():
    rclpy.init()
    node = rclpy.create_node('molar_transport_trial')
    root = Path(os.environ['CASE_OUTPUT'])
    root.mkdir(parents=True, exist_ok=True)
    trace = (root/'trace.jsonl').open('w', buffering=1)
    fleet, facility = {}, {}
    node.create_subscription(String, '/fleet/state', lambda m: fleet.update(json.loads(m.data)), 10)
    node.create_subscription(String, '/fleet/infrastructure/state', lambda m: facility.update(json.loads(m.data)), 10)
    heartbeat = node.create_publisher(String, '/fleet/operator_heartbeat', 10)
    command = node.create_publisher(String, '/fleet/infrastructure/command', 10)
    stop = node.create_publisher(String, '/fleet/command', 10)

    def send(floor):
        command.publish(String(data=json.dumps(dict(session_id=fleet['session_id'], operator_id=OPERATOR,
                                                    op='transport', robot=ROBOT, floor=floor))))

    phase, start, last, passed, reason, landings, phases = 'wait', time.monotonic(), 0., False, 'deadline', [], []
    try:
        while time.monotonic() - start < DEADLINE:
            rclpy.spin_once(node, timeout_sec=.03)
            now = time.monotonic()
            if not fleet or not facility:
                continue
            heartbeat.publish(String(data=json.dumps(dict(session_id=fleet['session_id'], operator_id=OPERATOR))))
            member = next(m for m in fleet['members'] if m['id'] == ROBOT)
            task = facility.get('transports', {}).get(ROBOT, {})
            if not phases or phases[-1][1:] != [task.get('destination'), task.get('phase')]:
                phases.append([round(now - start, 1), task.get('destination'), task.get('phase')])
                print(phases[-1], task.get('reason'), flush=True)
            if now - last > .2:
                trace.write(json.dumps(dict(phase=phase, wall=now, fleet=fleet, facility=facility), allow_nan=False)+'\n')
                last = now
            if (phase == 'wait' and member.get('navigation_ready') and member['mode'] == 'idle'
                    and fleet.get('operator_id') == OPERATOR and facility.get('operator_id') == OPERATOR
                    and facility.get('operator_fresh')):
                send('L1')
                phase = 'up'
            elif task.get('phase') in ('failed', 'canceled'):
                raise RuntimeError(task['reason'])
            elif phase == 'up' and task.get('phase') == 'succeeded' and facility['robot_floors'][ROBOT] == 'L1':
                landings.append(dict(floor='L1', wall_s=now-start, body=member['truth'], map=facility['active_robot_maps'][ROBOT]))
                send('L0')
                phase = 'down'
            elif (phase == 'down' and task.get('phase') == 'succeeded' and task.get('destination') == 'L0'
                    and facility['robot_floors'][ROBOT] == 'L0'):
                landings.append(dict(floor='L0', wall_s=now-start, body=member['truth'], map=facility['active_robot_maps'][ROBOT]))
                passed, reason = True, 'Actual autonomous approach/boarding/ride/floor-map/settled exits L0-L1-L0'
                break
    except (Exception, KeyboardInterrupt) as error:
        reason = type(error).__name__ + ': ' + str(error)
    finally:
        if fleet and rclpy.ok():
            stop.publish(String(data=json.dumps(dict(session_id=fleet['session_id'], operator_id=OPERATOR, op='all_stop'))))
        result = dict(robot=ROBOT, passed=passed, reason=reason, phase=phase, landings=landings, phases=phases,
                      wall_seconds=time.monotonic()-start, fleet=fleet, facility=facility)
        (root/'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps({k: v for k, v in result.items() if k not in ('fleet', 'facility')}, indent=1))
        trace.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
