#!/usr/bin/env python3
"""Measure Go2 ROS stance from physics truth, joint state and effort topics."""

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node


def tilt(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                      1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
    return max(abs(roll), abs(pitch))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--wall-timeout", type=float, default=35.0)
    parser.add_argument("--trace-interval", type=float, default=0.1)
    parser.add_argument("--trace-joints", action="store_true")
    parser.add_argument("--drive-vx", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.0)
    parser.add_argument("--drive-start", type=float, default=1.0)
    parser.add_argument("--drive-end", type=float, default=4.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node("go2_stance_probe")
    state = {"sim": None, "truth": [], "joint_messages": 0,
             "effort_messages": 0, "max_command_nm": 0.0,
             "joint_names": [], "latest_q": {}, "latest_v": {},
             "latest_efforts": {}, "latest_tau": 0.0,
             "trace": [], "yaw": []}
    cmd_pub = node.create_publisher(
        Twist, "/robot_lab_controller/cmd_vel_unstamped", 10)

    def clock(msg):
        state["sim"] = msg.clock.sec + msg.clock.nanosec * 1e-9

    def truth(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if state["sim"] is not None:
            state["truth"].append((state["sim"], p.x, p.y, p.z, tilt(q)))
            state["yaw"].append(math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)))
            if not state["trace"] or state["sim"] - state["trace"][-1]["sim_s"] >= args.trace_interval:
                point = {"sim_s": round(state["sim"], 3),
                                       "xyz": [round(p.x, 3), round(p.y, 3), round(p.z, 3)],
                                       "tilt_rad": round(tilt(q), 3),
                                       "max_effort_nm": round(state["latest_tau"], 2),
                                       "rr_calf_rad": round(state["latest_q"].get(
                                           "RR_calf_joint", 0.0), 3)}
                if args.trace_joints:
                    point["q"] = {k: round(v, 3) for k, v in state["latest_q"].items()}
                    point["v"] = {k: round(v, 3) for k, v in state["latest_v"].items()}
                    point["effort"] = {k: round(v, 2) for k, v in state["latest_efforts"].items()}
                state["trace"].append(point)

    def joints(msg):
        state["joint_messages"] += 1
        state["joint_names"] = list(msg.name)
        state["latest_q"] = dict(zip(msg.name, msg.position))
        state["latest_v"] = dict(zip(msg.name, msg.velocity))

    def efforts(msg):
        state["effort_messages"] += 1
        state["max_command_nm"] = max(
            state["max_command_nm"], *(abs(x) for x in msg.data))
        state["latest_tau"] = max((abs(x) for x in msg.data), default=0.0)
        state["latest_efforts"] = dict(zip(
            [f"{leg}_{kind}_joint" for leg in ("FL", "FR", "RL", "RR")
             for kind in ("hip", "thigh", "calf")], msg.data))

    node.create_subscription(Clock, "/clock", clock, 10)
    node.create_subscription(Odometry, "/odom/ground_truth", truth, 10)
    node.create_subscription(JointState, "/joint_states", joints, 10)
    node.create_subscription(Float64MultiArray,
                             "/go2_group_effort_controller/commands", efforts, 10)
    start = None
    deadline = time.monotonic() + args.wall_timeout
    last_command_at = float("-inf")
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if start is None and state["truth"]:
            start = state["truth"][0][0]
        if start is not None and time.monotonic() - last_command_at >= 0.05:
            elapsed = state["sim"] - start
            cmd = Twist()
            if args.drive_start <= elapsed < args.drive_end:
                cmd.linear.x = args.drive_vx
                cmd.angular.z = args.drive_wz
            cmd_pub.publish(cmd)
            last_command_at = time.monotonic()
        if start is not None and state["sim"] - start >= args.duration:
            break
    samples = state["truth"]
    result = {"sim_duration_s": 0.0 if not samples else samples[-1][0] - samples[0][0],
              "truth_count": len(samples), "joint_messages": state["joint_messages"],
              "joint_count": len(state["joint_names"]),
              "effort_messages": state["effort_messages"],
              "max_command_nm": state["max_command_nm"]}
    result["trace"] = state["trace"]
    if samples:
        result.update(min_height_m=min(s[3] for s in samples),
                      final_height_m=samples[-1][3],
                      max_tilt_rad=max(s[4] for s in samples),
                      xy_drift_m=math.hypot(samples[-1][1] - samples[0][1],
                                            samples[-1][2] - samples[0][2]))
        result["yaw_change_rad"] = math.atan2(
            math.sin(state["yaw"][-1] - state["yaw"][0]),
            math.cos(state["yaw"][-1] - state["yaw"][0]))
    result["passed"] = (result["sim_duration_s"] >= args.duration - 0.02
                        and result.get("min_height_m", 0) > 0.15
                        and result.get("max_tilt_rad", 99) < 0.35
                        and result.get("xy_drift_m", 99) < 0.1
                        and result["joint_count"] >= 12
                        and result["effort_messages"] > 20)
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
