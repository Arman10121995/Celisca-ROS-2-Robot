#!/usr/bin/env python3
"""Get-up screen for a legged robot on a running launch (R5.2).

Judged from simulator truth (``/odom/ground_truth``, sim-time stamps):

- **upright** means the body's z axis points up (``up_z`` > 0.9, i.e. tilt
  under 26°) and the base is above ``0.8 x --stand-height``;
- **fallen** means tilt past 60° (``up_z`` < 0.5) or the base below
  ``0.6 x --stand-height``.

The screen records ``up_z`` and height at 10 Hz of sim time for
``--seconds``. It reports whether the robot fell, whether it then got up and
stayed upright for ``--hold`` s, and how long that took. Use it with a
launch that knocks the robot over (``go2_perturbation_force_n:=60``) or
spawns it fallen (``spawn_roll:=1.57`` on MuJoCo):

  python3 scripts/run_with_launch.py --log run.log --warmup 30 --launch \\
    "robot_model:=unitree_go2 simulator:=mujoco mode:=loc map_name:=nav_empty legged_policy:=<dir> \\
     go2_perturbation_force_n:=60 go2_perturbation_start_s:=3 go2_perturbation_duration_s:=0.2" \\
    -- python3 scripts/screen_get_up.py --stand-height 0.32

Prints a JSON report; exit 0 when the robot got up (or, with
``--expect-no-fall``, when it never fell).
"""
import argparse
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--stand-height', type=float, required=True, help='standing base height, m')
    ap.add_argument('--seconds', type=float, default=20.0, help='sim seconds to observe')
    ap.add_argument('--hold', type=float, default=1.0, help='upright for this long counts as up, sim s')
    ap.add_argument('--timeout', type=float, default=600.0, help='wall seconds')
    ap.add_argument('--expect-no-fall', action='store_true')
    ap.add_argument('--out', default='', help='write the full report with its trace here')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('get_up_screen')
    samples = []

    def on_odom(message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if samples and stamp - samples[-1]['t'] < .1:
            return
        q = message.pose.pose.orientation
        up_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)     # world z of the body z axis
        samples.append(dict(t=round(stamp, 3), up_z=round(up_z, 4), height=round(message.pose.pose.position.z, 4)))

    node.create_subscription(Odometry, '/odom/ground_truth', on_odom, 10)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and (not samples or samples[-1]['t'] - samples[0]['t'] < args.seconds):
        rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node()
    rclpy.shutdown()

    def upright(s):
        return s['up_z'] > .9 and s['height'] > .8 * args.stand_height

    def fallen(s):
        return s['up_z'] < .5 or s['height'] < .6 * args.stand_height

    report = dict(stand_height_m=args.stand_height, samples=len(samples), passed=False, trace=samples)
    if not samples:
        report['error'] = 'no /odom/ground_truth'
        print(json.dumps(report))
        if args.out:
            with open(args.out, 'w') as stream:
                json.dump(report, stream)
        return 1
    fell = next((s for s in samples if fallen(s)), None)
    report.update(start=samples[0], end=samples[-1], fell=fell is not None,
                  min_up_z=min(s['up_z'] for s in samples), min_height_m=min(s['height'] for s in samples))
    if fell is None:
        report['passed'] = args.expect_no_fall
        report['note'] = 'never fell'
    else:
        report['fell_at_s'] = fell['t']
        run_start = None
        for s in samples[samples.index(fell):]:
            if upright(s):
                run_start = run_start if run_start is not None else s
                if s['t'] - run_start['t'] >= args.hold:
                    report.update(got_up=True, upright_at_s=run_start['t'],
                                  time_to_get_up_s=round(run_start['t'] - fell['t'], 2))
                    break
            else:
                run_start = None
        report.setdefault('got_up', False)
        report['passed'] = report['got_up'] and not args.expect_no_fall
        # Must still be upright at the end, not just once.
        report['upright_at_end'] = upright(samples[-1])
        report['passed'] = report['passed'] and report['upright_at_end']
    print(json.dumps({k: v for k, v in report.items() if k != 'trace'}, indent=1))
    if args.out:
        with open(args.out, 'w') as stream:
            json.dump(report, stream, indent=0)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
