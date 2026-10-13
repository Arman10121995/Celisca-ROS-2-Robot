#!/usr/bin/env python3
"""Mecanum strafe probe: do the passive rollers spin as they should?

Commands a pure strafe on /cmd_vel and records /joint_states and simulator
truth.  For ideal 45-degree mecanum wheels, a strafe at v turns each hub at
v / R and the rollers touching the ground at about sqrt(2) v / r (r the
roller radius).  The report gives the measured lateral speed, each hub's
mean |rate| and, per wheel, the fastest roller's mean |rate|, next to those
ideal values: rollers far below ideal mean they are being held, hubs at
ideal with a slow body mean the wheels slip.

Prints one JSON object.
"""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

WHEEL_RADIUS = 0.05
ROLLER_RADIUS = 0.0063


class Probe(Node):
    def __init__(self, odom_topic):
        super().__init__('roller_spin_probe')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joints = None
        self.truth = None
        self.recording = False
        self.samples = []
        self.truth_samples = []
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.on_truth, qos_profile_sensor_data)

    def on_joints(self, msg):
        self.joints = msg
        if self.recording and msg.velocity:
            self.samples.append(dict(zip(msg.name, msg.velocity)))

    def on_truth(self, msg):
        self.truth = msg
        if self.recording:
            p = msg.pose.pose
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.truth_samples.append((stamp, p.position.x, p.position.y, yaw))

    def drive(self, vy, seconds):
        cmd = Twist()
        cmd.linear.y = float(vy)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--speed', type=float, default=0.3)
    parser.add_argument('--duration', type=float, default=6.0)
    parser.add_argument('--settle', type=float, default=2.0, help='seconds of strafe ignored at the start')
    parser.add_argument('--warmup', type=float, default=8.0)
    parser.add_argument('--timeout', type=float, default=300.0)
    parser.add_argument('--odom', default='/odom/ground_truth', help='truth odometry topic')
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args.odom)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and (node.joints is None or node.truth is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.joints is None or node.truth is None:
        print(json.dumps({'error': 'no /joint_states or %s' % args.odom}))
        return 1
    node.drive(0.0, args.warmup)
    node.drive(args.speed, args.settle)
    node.recording = True
    node.drive(args.speed, args.duration)
    node.recording = False
    node.drive(0.0, 1.0)

    t = node.truth_samples
    lateral = None
    if len(t) > 2:
        dt = t[-1][0] - t[0][0]
        heading = t[0][3]
        dx, dy = t[-1][1] - t[0][1], t[-1][2] - t[0][2]
        lateral = (-dx * math.sin(heading) + dy * math.cos(heading)) / dt if dt > 0 else None
        forward = (dx * math.cos(heading) + dy * math.sin(heading)) / dt if dt > 0 else None
    names = sorted(node.samples[0]) if node.samples else []
    hubs = [n for n in names if n.endswith('_wheel_joint') and 'roller' not in n]
    wheels = {}
    for hub in hubs:
        prefix = hub[:-len('_joint')]
        rollers = [n for n in names if n.startswith(prefix + '_roller')]
        hub_rate = float(np.mean([abs(s.get(hub, 0.0)) for s in node.samples]))
        fastest = [max(abs(s.get(r, 0.0)) for r in rollers) for s in node.samples] if rollers else [0.0]
        wheels[hub] = {'hub_rate_mean': round(hub_rate, 3), 'rollers': len(rollers),
                       'fastest_roller_rate_mean': round(float(np.mean(fastest)), 2),
                       'fastest_roller_rate_p90': round(float(np.percentile(fastest, 90)), 2)}
    report = {
        'command_vy': args.speed, 'samples': len(node.samples),
        'measured_lateral_mps': None if lateral is None else round(lateral, 3),
        'measured_forward_mps': None if lateral is None else round(forward, 3),
        'ideal_hub_rate': round(args.speed / WHEEL_RADIUS, 2),
        'ideal_roller_rate': round(math.sqrt(2.0) * args.speed / ROLLER_RADIUS, 1),
        'wheels': wheels,
    }
    print(json.dumps(report, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
