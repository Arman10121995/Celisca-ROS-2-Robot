#!/usr/bin/env python3
"""Check that localization runs: map -> base transform versus ground truth.

Run against a launch in ``mode:=loc`` (or nav).  Waits for the ``map`` ->
base transform published by the localization stack, samples it with
``/odom/ground_truth`` for ``--window`` seconds, and reports the position
error between them (the maps share the world origin in this lab).

Usage: sim_loc_check.py [--base base_footprint] [--window 10] [--timeout 300]
"""
import argparse
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", default="base_footprint")
    parser.add_argument("--window", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args(argv)
    rclpy.init()
    node = Node("sim_loc_check")
    truth = []
    node.create_subscription(Odometry, "/odom/ground_truth", truth.append, 10)
    buffer = Buffer()
    TransformListener(buffer, node)

    def estimate():
        try:
            t = buffer.lookup_transform("map", args.base, rclpy.time.Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y

    result = {}
    started = time.monotonic()
    while time.monotonic() - started < args.timeout and (estimate() is None or not truth):
        rclpy.spin_once(node, timeout_sec=0.1)
    if estimate() is None or not truth:
        result["error"] = "no %s" % ("map->%s transform" % args.base if estimate() is None else "ground truth")
        print(json.dumps(result))
        return 1
    result["ready_wall_s"] = round(time.monotonic() - started, 1)
    errors = []
    end = time.monotonic() + args.window
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        e = estimate()
        p = truth[-1].pose.pose.position
        if e is not None:
            errors.append(math.hypot(e[0] - p.x, e[1] - p.y))
    result.update(samples=len(errors), mean_error_m=round(sum(errors) / len(errors), 3),
                  max_error_m=round(max(errors), 3), final_truth=[round(truth[-1].pose.pose.position.x, 2),
                                                                   round(truth[-1].pose.pose.position.y, 2)])
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
