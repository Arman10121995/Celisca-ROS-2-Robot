#!/usr/bin/env python3
"""Drive a robot through /cmd_vel phases and measure what it really did.

Each phase holds one (vx, wz) command; the forward speed and yaw rate the
robot achieved are taken from an odometry topic (the bridges' ground truth
by default) over the phase, after a settling time.  A car cannot turn on
the spot, so the last phase (vx = 0, wz != 0) is expected to leave it still.

Prints one JSON object with every phase.
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
    ("straight", 0.4, 0.0, 4.0),
    ("left_arc", 0.4, 0.5, 5.0),
    ("right_arc", 0.4, -0.5, 5.0),
    ("reverse_left", -0.3, 0.3, 4.0),
    ("tight_left", 0.3, 1.5, 4.0),   # beyond a 0.49 m radius: clamped for cars
    ("spin_in_place", 0.0, 1.0, 3.0),
    ("stop", 0.0, 0.0, 2.0),
]


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveCheck(Node):
    def __init__(self, odom_topic):
        super().__init__("sim_drive_check")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.samples = []
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.samples.append((time.monotonic(), p.position.x, p.position.y, _yaw(p.orientation)))

    def wait_for_odom(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not self.samples:
            rclpy.spin_once(self, timeout_sec=0.2)
        return bool(self.samples)

    def run_phase(self, vx, wz, duration, settle=1.0):
        twist = Twist()
        twist.linear.x, twist.angular.z = vx, wz
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        window = [s for s in self.samples if start + settle <= s[0] <= start + duration]
        if len(window) < 2:
            return None
        (t0, x0, y0, a0), (t1, x1, y1, a1) = window[0], window[-1]
        dt = t1 - t0
        yaw_change = 0.0
        for (_, _, _, a), (_, _, _, b) in zip(window, window[1:]):
            yaw_change += math.atan2(math.sin(b - a), math.cos(b - a))
        # Forward speed: displacement projected on the mean heading.
        heading = a0 + yaw_change / 2.0
        forward = ((x1 - x0) * math.cos(heading) + (y1 - y0) * math.sin(heading)) / dt
        return {"vx": round(forward, 3), "wz": round(yaw_change / dt, 3),
                "distance": round(math.hypot(x1 - x0, y1 - y0), 3)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom", default="/odom/ground_truth")
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--warmup", type=float, default=3.0)
    args = parser.parse_args()
    rclpy.init()
    node = DriveCheck(args.odom)
    result = {"odom": args.odom, "phases": []}
    if not node.wait_for_odom(args.timeout):
        result["error"] = "no odometry on %s" % args.odom
    else:
        node.run_phase(0.0, 0.0, args.warmup)
        for name, vx, wz, duration in PHASES:
            measured = node.run_phase(vx, wz, duration)
            result["phases"].append({"phase": name, "command": [vx, wz],
                                     "measured": measured})
    print(json.dumps(result, indent=1))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
