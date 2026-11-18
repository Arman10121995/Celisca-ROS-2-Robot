#!/usr/bin/env python3
"""Stairs traversal screen for a legged robot on a running launch (R5.2).

Drives ``--speed`` m/s forward on ``/cmd_vel`` for ``--seconds`` of sim time
and judges from simulator truth (``/odom/ground_truth``). With
``--hold-line`` it steers like an operator would, holding the start line
and heading (yaw rate -2 (y - y0) - 1.5 yaw, at most 0.6 rad/s): a policy
that drifts 0.6 m sideways walks round a 1.1 m wide stair instead of up it.

- **climbed:** the base's rise above its start, and the number of whole
  ``--rise`` steps that makes (1 cm tolerance: front feet on a step pitch the
  base up most of a rise without climbing it);
- **fell:** tilt past 60° (body z axis ``up_z`` < 0.5) at any time.

It also records, at 10 Hz of sim time, the measured joint efforts
(``/joint_states``) and, when published, the foot contact forces
(``/go2/foot_contact_forces``, MuJoCo Go2). Passed when the robot climbed
``--steps`` steps without falling. ``terrain_stairs`` has five 0.18 m
steps with 1.1 m treads along y = -4 from x = -7.2:

  python3 scripts/run_with_launch.py --log run.log --warmup 20 --launch \\
    "robot_model:=unitree_go2 simulator:=mujoco mode:=loc map_name:=terrain_stairs legged_policy:=<dir> \\
     spawn_x:=-7.5 spawn_y:=-4.0" -- python3 scripts/screen_stairs.py --out stairs.json
"""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--seconds', type=float, default=40.0, help='sim seconds of driving')
    ap.add_argument('--settle', type=float, default=2.0, help='sim seconds standing before driving')
    ap.add_argument('--rise', type=float, default=0.18, help='step height, m')
    ap.add_argument('--steps', type=int, default=5, help='steps to climb to pass')
    ap.add_argument('--timeout', type=float, default=900.0, help='wall seconds')
    ap.add_argument('--hold-line', action='store_true', help='steer to keep the start line and heading')
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('stairs_screen')
    truth, efforts, contacts, samples = {}, {}, {}, []

    def on_odom(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        truth.update(t=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9, x=p.x, y=p.y, z=p.z,
                     up_z=1.0 - 2.0 * (q.x * q.x + q.y * q.y),
                     yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_joints(message):
        if len(message.effort) == len(message.name):
            efforts.update(zip(message.name, message.effort))

    node.create_subscription(Odometry, '/odom/ground_truth', on_odom, 10)
    node.create_subscription(JointState, '/joint_states', on_joints, 10)
    node.create_subscription(Float64MultiArray, '/go2/foot_contact_forces',
                             lambda m: contacts.update(forces=list(m.data)), 10)
    command = node.create_publisher(Twist, '/cmd_vel', 1)
    deadline = time.monotonic() + args.timeout
    while 't' not in truth and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    report = dict(speed_mps=args.speed, rise_m=args.rise, steps_to_pass=args.steps, hold_line=args.hold_line,
                  passed=False)
    if 't' not in truth:
        report['error'] = 'no /odom/ground_truth'
    else:
        begin = truth['t']
        while time.monotonic() < deadline and truth['t'] - begin < args.settle:
            rclpy.spin_once(node, timeout_sec=.05)
        start = dict(truth)
        while time.monotonic() < deadline and truth['t'] - start['t'] < args.seconds:
            message = Twist()
            message.linear.x = args.speed
            if args.hold_line:
                turn = -2.0 * (truth['y'] - start['y']) - 1.5 * math.atan2(math.sin(truth['yaw'] - start['yaw']),
                                                                         math.cos(truth['yaw'] - start['yaw']))
                message.angular.z = max(-.6, min(.6, turn))
            command.publish(message)
            rclpy.spin_once(node, timeout_sec=.05)
            if not samples or truth['t'] - samples[-1]['t'] >= .1:
                samples.append(dict(t=round(truth['t'] - start['t'], 2), x=round(truth['x'], 3), y=round(truth['y'], 3),
                                    z=round(truth['z'], 3), up_z=round(truth['up_z'], 3),
                                    effort={k: round(v, 2) for k, v in efforts.items()},
                                    **({'feet_n': [round(v, 1) for v in contacts['forces']]} if 'forces' in contacts else {})))
        command.publish(Twist())
        rise = max(s['z'] for s in samples) - start['z'] if samples else 0.
        fell = next((s for s in samples if s['up_z'] < .5), None)
        peak = {}
        for s in samples:
            for k, v in s['effort'].items():
                peak[k] = max(peak.get(k, 0.), abs(v))
        feet = [s['feet_n'] for s in samples if 'feet_n' in s]
        report.update(start=dict(x=round(start['x'], 3), y=round(start['y'], 3), z=round(start['z'], 3)),
                      end=samples[-1] if samples else None, forward_m=round(samples[-1]['x'] - start['x'], 3) if samples else 0.,
                      max_rise_m=round(rise, 3), steps_climbed=int(math.floor((rise + .01) / args.rise)),
                      fell=fell is not None, fell_at_s=fell['t'] if fell else None,
                      min_up_z=round(min(s['up_z'] for s in samples), 3) if samples else None,
                      peak_effort_nm=peak,
                      foot_contact_fraction=[round(sum(f[i] > 1. for f in feet) / len(feet), 3)
                                             for i in range(len(feet[0]))] if feet else None)
        report['passed'] = report['steps_climbed'] >= args.steps and not report['fell']
        report['trace'] = samples
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({k: v for k, v in report.items() if k != 'trace'}, indent=1))
    if args.out:
        with open(args.out, 'w') as stream:
            json.dump(report, stream, indent=0)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
