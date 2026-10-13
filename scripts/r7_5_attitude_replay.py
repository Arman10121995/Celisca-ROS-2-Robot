#!/usr/bin/env python3
"""Replay one synthetic IMU stream to every attitude node over ROS and score them.

Publishes the R7.5 benchmark's attitude data (seed 1, 100 Hz) on /imu/out
in real time with wall-clock stamps, and records the orientation each
running node publishes: /imu/complementary, /imu/mahony, /imu/madgwick,
/imu/eskf (robot_lab, at their node defaults) and /imu/madgwick_upstream
(imu_tools).  Each output is matched to the truth sample with the same
stamp.  Prints JSON: tilt RMS after 10 s, RMS during the 2 m/s^2 phase,
and message counts.

Start the nodes first (scripts/r7_5_attitude_replay.sh does both).
"""
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

from r7_5_fusion_benchmark import attitude_data  # noqa: E402
from robot_lab_algorithms.fusion_filters import q_to_euler, wrap  # noqa: E402

OUTPUTS = ['complementary', 'mahony', 'madgwick', 'eskf', 'madgwick_upstream']


def main():
    samples, dt = attitude_data(1)
    rclpy.init()
    node = Node('attitude_replay')
    pub = node.create_publisher(Imu, '/imu/out', 200)
    received = {name: {} for name in OUTPUTS}

    def callback(name):
        def on(msg):
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            q = msg.orientation
            received[name][key] = (q.w, q.x, q.y, q.z)
        return on
    for name in OUTPUTS:
        node.create_subscription(Imu, '/imu/' + name, callback(name), 500)
    time.sleep(2.0)
    stamps = []
    start = time.monotonic()
    for i, (t, gyro, accel, q) in enumerate(samples):
        while time.monotonic() < start + i * dt:
            rclpy.spin_once(node, timeout_sec=0.0005)
        msg = Imu()
        now = node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'imu_link'
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (float(v) for v in gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (float(v) for v in accel)
        pub.publish(msg)
        stamps.append(((now.sec, now.nanosec), t, q))
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    result = {'samples_published': len(stamps), 'wall_seconds': round(time.monotonic() - start, 1)}
    for name in OUTPUTS:
        steady, disturbed = [], []
        for key, t, q in stamps:
            if key not in received[name] or t < 10.0:
                continue
            r, p, _ = q_to_euler(received[name][key])
            rt, pt, _ = q_to_euler(q)
            errors = (math.degrees(abs(wrap(r - rt))), math.degrees(abs(wrap(p - pt))))
            (disturbed if 60.0 <= t < 80.0 else steady).extend(errors)
        everything = steady + disturbed
        result[name] = {
            'messages': len(received[name]),
            'overall_rms_deg': round(float(np.sqrt(np.mean(np.square(everything)))), 3) if everything else None,
            'disturbed_rms_deg': round(float(np.sqrt(np.mean(np.square(disturbed)))), 3) if disturbed else None,
        }
    print(json.dumps(result, indent=1))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
