#!/usr/bin/env python3
"""Measure a running simulation's real-time factor from /clock.

Waits for /clock, discards a warm-up, then reports simulated seconds per
wall second over the window (first/last /clock arrival), plus the clock
message rate.  Read-only: publishes nothing.
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', type=float, default=20.0)
    parser.add_argument('--warmup', type=float, default=5.0)
    parser.add_argument('--timeout', type=float, default=300.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node('sim_rtf_probe')
    samples = []
    node.create_subscription(Clock, '/clock', lambda m: samples.append(
        (time.monotonic(), m.clock.sec+m.clock.nanosec*1e-9)), qos_profile_sensor_data)
    deadline = time.monotonic()+args.timeout
    while not samples and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    result = {'error': 'no /clock'} if not samples else None
    if samples:
        end = time.monotonic()+args.warmup
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        start = len(samples)
        end = time.monotonic()+args.window
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        window = samples[start:]
        if len(window) < 2:
            result = {'error': 'clock stalled', 'messages': len(window)}
        else:
            wall = window[-1][0]-window[0][0]
            sim = window[-1][1]-window[0][1]
            result = {'rtf': round(sim/wall, 3), 'sim_s': round(sim, 3), 'wall_s': round(wall, 3),
                      'clock_hz': round((len(window)-1)/wall, 1)}
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
