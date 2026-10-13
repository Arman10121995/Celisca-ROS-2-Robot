#!/usr/bin/env python3
"""Drive a walking robot and compare leg odometry with simulator truth.

Publishes /cmd_vel phases (forward, arcs, stop) and records three
odometries: the bridge's truth (/odom/ground_truth), leg kinematic
odometry (/leg_odometry) and the EKF output (/odometry/filtered).  For
each phase it reports mean forward speed and yaw rate from each source;
over the whole run it reports the displacement error of the leg and EKF
estimates against truth (drift), as a distance and as a fraction of the
distance walked.

Prints one JSON object.
"""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

PHASES = [
    ('stand', 0.0, 0.0, 3.0),
    ('straight', 0.4, 0.0, 8.0),
    ('left_arc', 0.4, 0.5, 6.0),
    ('right_arc', 0.4, -0.5, 6.0),
    ('straight_2', 0.3, 0.0, 6.0),
    ('stop', 0.0, 0.0, 3.0),
]
SOURCES = {'truth': '/odom/ground_truth', 'leg': '/leg_odometry', 'ekf': '/odometry/filtered'}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self):
        super().__init__('leg_odometry_check')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.samples = {name: [] for name in SOURCES}
        for name, topic in SOURCES.items():
            self.create_subscription(Odometry, topic, lambda msg, n=name: self.on_odom(n, msg),
                                     qos_profile_sensor_data)

    def on_odom(self, name, msg):
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples[name].append((stamp, p.position.x, p.position.y, yaw_of(p.orientation),
                                   msg.twist.twist.linear.x, msg.twist.twist.angular.z))

    def sim_now(self):
        truth = self.samples['truth']
        return truth[-1][0] if truth else None


def window(samples, start, end):
    return [s for s in samples if start <= s[0] <= end]


def displacement(samples, start, end):
    """Planar displacement of a source between two sim times, in its own frame."""
    a = min(samples, key=lambda s: abs(s[0] - start))
    b = min(samples, key=lambda s: abs(s[0] - end))
    return (b[1] - a[1], b[2] - a[2]), a, b


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--timeout', type=float, default=300.0)
    parser.add_argument('--warmup', type=float, default=8.0, help='sim seconds before the first phase')
    args = parser.parse_args()
    rclpy.init()
    node = Check()
    deadline = time.monotonic() + args.timeout

    def spin_until(sim_time):
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = node.sim_now()
            if now is not None and now >= sim_time:
                return True
        return False

    while rclpy.ok() and time.monotonic() < deadline and (
            node.sim_now() is None or not node.samples['leg']):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.sim_now() is None or not node.samples['leg']:
        print(json.dumps({'error': 'no truth or no /leg_odometry',
                          'counts': {k: len(v) for k, v in node.samples.items()}}))
        return 1
    spin_until(node.sim_now() + args.warmup)
    phases = []
    for name, vx, wz, duration in PHASES:
        start = node.sim_now()
        command = Twist()
        command.linear.x, command.angular.z = vx, wz
        end = start + duration
        while rclpy.ok() and time.monotonic() < deadline:
            node.pub.publish(command)
            if spin_until(min(end, node.sim_now() + 0.05)) and node.sim_now() >= end:
                break
        phases.append((name, vx, wz, start, end))
    node.pub.publish(Twist())
    result = {'phases': [], 'counts': {k: len(v) for k, v in node.samples.items()}}
    for name, vx, wz, start, end in phases:
        row = {'phase': name, 'command': [vx, wz]}
        for source, samples in node.samples.items():
            w = window(samples, start + 1.0, end)  # skip the first second
            if len(w) > 1:
                row[source] = {'vx': round(sum(s[4] for s in w) / len(w), 3),
                               'wz': round(sum(s[5] for s in w) / len(w), 3)}
        result['phases'].append(row)
    t0, t1 = phases[0][3], phases[-1][4]
    (tx, ty), _, _ = displacement(node.samples['truth'], t0, t1)
    truth = node.samples['truth']
    walked = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(truth, truth[1:])
                 if t0 <= a[0] <= t1)
    result['truth_displacement_m'] = [round(tx, 3), round(ty, 3)]
    result['walked_m'] = round(walked, 3)
    for source in ('leg', 'ekf'):
        if len(node.samples[source]) < 2:
            continue
        (dx, dy), _, _ = displacement(node.samples[source], t0, t1)
        error = math.hypot(dx - tx, dy - ty)
        result[source + '_displacement_m'] = [round(dx, 3), round(dy, 3)]
        result[source + '_drift_m'] = round(error, 3)
        result[source + '_drift_fraction'] = round(error / walked, 4) if walked > 0 else None
    print(json.dumps(result, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
