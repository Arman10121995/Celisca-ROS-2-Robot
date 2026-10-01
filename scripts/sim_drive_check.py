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
from sensor_msgs.msg import JointState

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
    def __init__(self, odom_topic, cmd_topic="/cmd_vel"):
        super().__init__("sim_drive_check")
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.samples = []
        self.joint_positions = {}
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

    def _on_odom(self, msg):
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # (arrival, sim stamp, x, y, yaw).  The bridges stamp odometry with
        # sim time; on a simulator slower than real time, arrival-time
        # velocities read low (measured 0.28 m/s for a true 0.40 m/s on
        # MuJoCo at RTF ~0.7, 2026-10-01), so speeds use the sim stamp.
        self.samples.append((time.monotonic(), stamp,
                             p.position.x, p.position.y, _yaw(p.orientation)))

    @staticmethod
    def _dt(a, b):
        """Sim-time delta from header stamps, falling back to arrival time."""
        if b[1] > a[1]:
            return b[1] - a[1]
        return b[0] - a[0]

    def _on_joints(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))

    def _steering_positions(self):
        return {
            name: round(position, 4)
            for name, position in sorted(self.joint_positions.items())
            if "steer_joint" in name
        }

    def wait_for_odom(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not self.samples:
            rclpy.spin_once(self, timeout_sec=0.2)
        return bool(self.samples)

    def run_phase(self, vx, wz, duration, settle=1.0, vy=0.0):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = vx, vy, wz
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.pub.publish(twist)
            # Match the GUI's 10 Hz button repeat.  Spinning between sends
            # also lets odometry callbacks run when the mux is active.
            next_send = time.monotonic() + 0.1
            while time.monotonic() < next_send:
                rclpy.spin_once(self, timeout_sec=min(
                    0.02, max(0.0, next_send - time.monotonic())))
        window = [s for s in self.samples if start + settle <= s[0] <= start + duration]
        if len(window) < 2:
            return None
        (_, _, x0, y0, a0), (_, _, x1, y1, a1) = window[0], window[-1]
        dt = self._dt(window[0], window[-1])
        yaw_change = 0.0
        for (_, _, _, _, a), (_, _, _, _, b) in zip(window, window[1:]):
            yaw_change += math.atan2(math.sin(b - a), math.cos(b - a))
        # Forward speed: displacement projected on the mean heading.
        heading = a0 + yaw_change / 2.0
        forward = ((x1 - x0) * math.cos(heading) + (y1 - y0) * math.sin(heading)) / dt
        lateral = (-(x1 - x0) * math.sin(heading) + (y1 - y0) * math.cos(heading)) / dt
        return {"vx": round(forward, 3), "vy": round(lateral, 3),
            "wz": round(yaw_change / dt, 3),
            "distance": round(math.hypot(x1 - x0, y1 - y0), 3),
            "steering_positions_rad": self._steering_positions()}

    def run_silence(self, duration):
        """Stop publishing and measure coast plus final odometry speed."""
        if not self.samples:
            return None
        before = self.samples[-1]
        start = time.monotonic()
        while time.monotonic() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.02)
        window = [sample for sample in self.samples if sample[0] >= start]
        if not window:
            return None
        points = [before] + window
        first, last = points[0], points[-1]
        yaw_change = sum(
            math.atan2(math.sin(b[4] - a[4]), math.cos(b[4] - a[4]))
            for a, b in zip(points, points[1:]))
        coast = math.hypot(last[2] - first[2], last[3] - first[3])
        tail_start = last[0] - min(0.5, duration)
        tail = [sample for sample in points if sample[0] >= tail_start]
        if len(tail) < 2:
            tail_vx = tail_wz = None
        else:
            a, b = tail[0], tail[-1]
            dt = self._dt(a, b)
            heading = a[4] + math.atan2(
                math.sin(b[4] - a[4]), math.cos(b[4] - a[4])) / 2.0
            tail_vx = (((b[2] - a[2]) * math.cos(heading)
                        + (b[3] - a[3]) * math.sin(heading)) / dt)
            tail_wz = sum(
                math.atan2(math.sin(nxt[4] - prev[4]),
                           math.cos(nxt[4] - prev[4]))
                for prev, nxt in zip(tail, tail[1:])) / dt
        return {
            "duration_s": round(self._dt(first, last), 3),
            "coast_distance_m": round(coast, 3),
            "coast_yaw_change_rad": round(yaw_change, 3),
            "tail_vx_mps": None if tail_vx is None else round(tail_vx, 3),
            "tail_wz_rps": None if tail_wz is None else round(tail_wz, 3),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom", default="/odom/ground_truth")
    parser.add_argument("--cmd-topic", default="/cmd_vel",
                        help="Use /key_vel to exercise the GUI's drive path")
    parser.add_argument("--quick", action="store_true",
                        help="Run the straight and stop phases only")
    parser.add_argument("--command-loss", action="store_true",
                        help="Stop publishing after the final moving phase")
    parser.add_argument("--command-loss-duration", type=float, default=3.0,
                        help="Seconds to measure motion after publisher loss")
    parser.add_argument("--strafe", action="store_true",
                        help="Also command 0.3 m/s lateral velocity")
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--warmup", type=float, default=3.0)
    args = parser.parse_args()
    rclpy.init()
    node = DriveCheck(args.odom, args.cmd_topic)
    result = {"odom": args.odom, "cmd_topic": args.cmd_topic, "phases": []}
    if not node.wait_for_odom(args.timeout):
        result["error"] = "no odometry on %s" % args.odom
    else:
        node.run_phase(0.0, 0.0, args.warmup)
        if args.quick:
            phases = [PHASES[0]] if args.command_loss else [PHASES[0], PHASES[-1]]
        else:
            phases = PHASES[:-2] if args.command_loss else PHASES
        if args.strafe:
            phases.insert(-1, ("strafe", 0.0, 0.0, 5.0, 0.3))
        for phase in phases:
            name, vx, wz, duration = phase[:4]
            vy = phase[4] if len(phase) > 4 else 0.0
            measured = node.run_phase(vx, wz, duration, vy=vy)
            result["phases"].append({"phase": name, "command": [vx, wz],
                                     "lateral_command": vy,
                                     "measured": measured})
        if args.command_loss:
            result["publisher_loss"] = node.run_silence(
                args.command_loss_duration)
    print(json.dumps(result, indent=1))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
