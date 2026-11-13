#!/usr/bin/env python3
"""Arm screen for a running robot on the URDF articulation contract
(robot_lab_utils.urdf_articulation: /molar/articulation/{state,command,heartbeat,stop}).

On an already running launch it:
1. moves each position joint by ``--delta`` rad (clamped to its limits) and
   back, recording settle time, steady error and whether the base was inhibited
   while the joint moved;
2. sends a long move and calls Stop part-way: the joint must hold;
3. sends a long move and withholds the heartbeat: the plant must hold.
Prints a JSON report; exit 0 when every check passed.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--delta', type=float, default=0.5)
    ap.add_argument('--tolerance', type=float, default=0.03, help='rad, settled error')
    ap.add_argument('--settle-timeout', type=float, default=20.0)
    ap.add_argument('--ready-timeout', type=float, default=300.0)
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('urdf_arm_screen')
    state = {}
    node.create_subscription(String, '/molar/articulation/state', lambda m: state.update(json.loads(m.data)), 10)
    command = node.create_publisher(String, '/molar/articulation/command', 10)
    heartbeat = node.create_publisher(Empty, '/molar/articulation/heartbeat', 10)
    stop = node.create_client(Trigger, '/molar/articulation/stop')
    beating = [True]
    last_beat = [0.0]

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if beating[0] and time.monotonic() - last_beat[0] > 0.1:
                heartbeat.publish(Empty())
                last_beat[0] = time.monotonic()

    def channels():
        return {c['name']: c for c in state.get('channels', [])}

    def send(targets):
        command.publish(String(data=json.dumps({'plant_id': state['plant_id'], 'targets': targets})))

    report = {'phases': [], 'passed': False}
    deadline = time.monotonic() + args.ready_timeout
    while not (state.get('initialized') and channels()) and time.monotonic() < deadline:
        spin(0.2)
    if not state.get('initialized'):
        report['error'] = 'no initialized articulation state'
        print(json.dumps(report, indent=1))
        return 1
    joints = [n for n, c in channels().items() if c.get('mode') == 'position' and not c.get('coupled')]
    home = {n: channels()[n]['measured'] for n in joints}
    report['joints'] = joints
    report['initial'] = {n: round(v, 4) for n, v in home.items()}
    ok = True

    def move(name, target):
        send({name: target})
        start, inhibited, settled_at, errors = time.monotonic(), False, None, []
        while time.monotonic() - start < args.settle_timeout:
            spin(0.05)
            c = channels()[name]
            inhibited |= bool(state.get('base_inhibited'))
            if abs(c['measured'] - target) < args.tolerance and settled_at is None:
                settled_at = time.monotonic() - start
            if settled_at is not None:
                errors.append(abs(c['measured'] - target))
                if time.monotonic() - start > settled_at + 1.0:
                    break
        return dict(target=round(target, 4), settled_s=None if settled_at is None else round(settled_at, 2),
                    steady_error_rad=round(max(errors), 4) if errors else None, base_inhibited_seen=inhibited,
                    final=round(channels()[name]['measured'], 4))

    for name in joints:
        lo, hi = channels()[name]['limits']
        target = max(lo + 0.05, min(hi - 0.05, home[name] + args.delta))
        out = move(name, target)
        back = move(name, home[name])
        passed = (out['settled_s'] is not None and back['settled_s'] is not None and out['base_inhibited_seen'])
        ok &= passed
        report['phases'].append(dict(joint=name, out=out, back=back, passed=passed))

    # Stop part-way through a long move: the joint must hold where it is.
    name = joints[0]
    lo, hi = channels()[name]['limits']
    far = max(lo + 0.05, min(hi - 0.05, home[name] + 1.5))
    send({name: far})
    spin(0.6)
    stop.wait_for_service(timeout_sec=5.0)
    future = stop.call_async(Trigger.Request())
    spin(0.5)
    held = channels()[name]['measured']
    spin(2.0)
    drift = abs(channels()[name]['measured'] - held)
    stop_ok = bool(future.done() and drift < 0.02 and abs(held - far) > 0.1)
    report['stop'] = dict(joint=name, held_at=round(held, 4), target_was=round(far, 4),
                          drift_after_rad=round(drift, 4), passed=stop_ok)
    ok &= stop_ok
    # Withhold the heartbeat during a long move: the plant must hold short of the target.
    spin(0.5)
    start = channels()[name]['measured']
    goal = lo + 0.05 if start - lo > hi - start else hi - 0.05     # the far end of the range
    send({name: goal})
    spin(0.4)
    beating[0] = False
    spin(1.0)
    held = channels()[name]['measured']
    spin(2.0)
    drift = abs(channels()[name]['measured'] - held)
    watchdog_ok = drift < 0.02 and abs(held - goal) > 0.3
    report['watchdog'] = dict(joint=name, held_at=round(held, 4), target_was=round(goal, 4), drift_after_rad=round(drift, 4),
                              status=state.get('status'), passed=watchdog_ok)
    ok &= watchdog_ok
    beating[0] = True
    spin(0.5)
    for n in joints:          # leave the arm at its start pose
        send({n: home[n]})
        spin(0.2)
    spin(5.0)
    report['final_error_rad'] = round(max(abs(channels()[n]['measured'] - home[n]) for n in joints), 4)
    report['passed'] = bool(ok)
    print(json.dumps(report, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
