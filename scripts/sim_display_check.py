#!/usr/bin/env python3
"""Check what RViz would show for a display-mode run, from the ROS graph.

Run against a robot already launched with ``mode:=display`` (see
``sim_display_check.sh``).  Reports, as one JSON object:

* whether ``/joint_states`` covers every movable joint of the published
  ``/robot_description`` (robot_state_publisher only places links whose
  joints it hears about, so a missing name is a missing limb in RViz);
* whether the joint positions change while the simulator runs, and how far
  (a frozen robot publishes the same values forever);
* whether ``/tf`` carries the robot's moving links;
* the base height and tilt from ``/odom/ground_truth`` over the window, so a
  robot spawned inside the floor, falling through it or jumping shows up.

Usage: sim_display_check.py [--window 6] [--timeout 300]
"""
import argparse
import json
import math
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def movable_joints(urdf_text):
    root = ET.fromstring(urdf_text)
    return sorted(j.get("name") for j in root.findall("joint")
                  if j.get("type") not in ("fixed", None) and j.find("mimic") is None)


class Check(Node):
    def __init__(self):
        super().__init__("sim_display_check")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.description = None
        self.joints, self.odom, self.tf_children = [], [], set()
        self.create_subscription(String, "/robot_description",
                                 lambda m: setattr(self, "description", m.data), latched)
        self.create_subscription(JointState, "/joint_states", self.joints.append, 50)
        self.create_subscription(Odometry, "/odom/ground_truth", self.odom.append, 50)
        self.create_subscription(TFMessage, "/tf", self._tf, 50)

    def _tf(self, msg):
        for t in msg.transforms:
            self.tf_children.add(t.child_frame_id)

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--window", type=float, default=6.0, help="sim seconds observed")
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = Check()
    result = {}
    started = time.monotonic()
    if not node.spin_until(lambda: node.description and node.joints, args.timeout):
        result["error"] = "no %s" % ("robot_description" if not node.description else "joint_states")
        print(json.dumps(result))
        return 1
    result["startup_wall_s"] = round(time.monotonic() - started, 1)
    node.joints.clear()
    node.odom.clear()
    node.tf_children.clear()
    node.spin_until(lambda: node.joints and _stamp(node.joints[-1]) - _stamp(node.joints[0]) >= args.window,
                    max(60.0, args.window * 40))
    expected = movable_joints(node.description)
    published = sorted(set(n for m in node.joints for n in m.name))
    first, last = node.joints[0], node.joints[-1]
    start = dict(zip(first.name, first.position))
    excursion = {}
    for msg in node.joints:
        for name, position in zip(msg.name, msg.position):
            if name in start:
                excursion[name] = max(excursion.get(name, 0.0), abs(position - start[name]))
    moving = sorted(n for n, e in excursion.items() if e > 1e-3)
    result.update({
        "joint_state_msgs": len(node.joints),
        "sim_window_s": round(_stamp(last) - _stamp(first), 2),
        "movable_joints": len(expected),
        "missing_joint_names": [n for n in expected if n not in published],
        "unknown_joint_names": [n for n in published if n not in expected],
        "joints_that_moved": len(moving),
        "max_joint_excursion": round(max(excursion.values(), default=0.0), 4),
        "tf_children": len(node.tf_children),
    })
    if node.odom:
        zs = [m.pose.pose.position.z for m in node.odom]

        def tilt(m):
            q = m.pose.pose.orientation
            return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
        result.update({
            "base_z_first_last_min_max": [round(zs[0], 3), round(zs[-1], 3),
                                          round(min(zs), 3), round(max(zs), 3)],
            "base_tilt_deg_last_max": [round(tilt(node.odom[-1]), 1),
                                       round(max(tilt(m) for m in node.odom), 1)],
            "base_speed_max": round(max(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y,
                                                   m.twist.twist.linear.z) for m in node.odom), 3),
        })
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
