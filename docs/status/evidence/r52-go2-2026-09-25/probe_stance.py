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
from std_msgs.msg import Float64MultiArray, String
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
    parser.add_argument("--drop-command-after-drive", action="store_true")
    parser.add_argument("--perturbation-start", type=float, default=None)
    parser.add_argument("--perturbation-duration", type=float, default=None)
    parser.add_argument("--perturbation-force-n", type=float, default=None)
    parser.add_argument("--perturbation-axis", type=int, default=1)
    parser.add_argument("--recovery-window", type=float, default=2.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node("go2_stance_probe")
    state = {"sim": None, "truth": [], "joint_messages": 0,
             "effort_messages": 0, "max_command_nm": 0.0,
             "contact_messages": 0, "latest_forces": None,
             "peak_foot_force_n": {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")},
             "joint_names": [], "latest_q": {}, "latest_v": {},
             "latest_efforts": {}, "latest_tau": 0.0,
             "trace": [], "yaw": [], "safety_states": [],
             "safety_transitions": [], "safety_messages": 0,
             "latest_safety_state": "unavailable"}
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
                                       "yaw_rad": round(state["yaw"][-1], 3),
                                       "tilt_rad": round(tilt(q), 3),
                                       "safety_state": state["latest_safety_state"],
                                       "max_effort_nm": round(state["latest_tau"], 2),
                                       "rr_calf_rad": round(state["latest_q"].get(
                                           "RR_calf_joint", 0.0), 3)}
                if state["latest_forces"] is not None:
                    point["foot_force_n"] = {
                        leg: round(force, 2)
                        for leg, force in state["latest_forces"].items()}
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

    def safety(msg):
        state["safety_messages"] += 1
        if state["sim"] is not None and msg.data != state["latest_safety_state"]:
            state["safety_transitions"].append({
                "sim_s": round(state["sim"], 3), "state": msg.data})
        state["latest_safety_state"] = msg.data
        if state["sim"] is not None:
            state["safety_states"].append((state["sim"], msg.data))

    def contacts(msg):
        if len(msg.data) != 4:
            return
        state["contact_messages"] += 1
        state["latest_forces"] = dict(zip(("FL", "FR", "RL", "RR"), msg.data))
        for leg, force in state["latest_forces"].items():
            state["peak_foot_force_n"][leg] = max(
                state["peak_foot_force_n"][leg], force)

    node.create_subscription(Clock, "/clock", clock, 10)
    node.create_subscription(Odometry, "/odom/ground_truth", truth, 10)
    node.create_subscription(JointState, "/joint_states", joints, 10)
    node.create_subscription(Float64MultiArray,
                             "/go2_group_effort_controller/commands", efforts, 10)
    node.create_subscription(Float64MultiArray,
                             "/go2/foot_contact_forces", contacts, 10)
    node.create_subscription(String, "/go2/safety_state", safety, 10)
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
            if not (args.drop_command_after_drive and elapsed >= args.drive_end):
                cmd_pub.publish(cmd)
            last_command_at = time.monotonic()
        if start is not None and state["sim"] - start >= args.duration:
            break
    samples = state["truth"]
    result = {"sim_duration_s": 0.0 if not samples else samples[-1][0] - samples[0][0],
              "truth_count": len(samples), "joint_messages": state["joint_messages"],
              "joint_count": len(state["joint_names"]),
              "effort_messages": state["effort_messages"],
              "contact_messages": state["contact_messages"],
              "peak_foot_force_n": state["peak_foot_force_n"],
              "max_command_nm": state["max_command_nm"],
              "safety_messages": state["safety_messages"],
              "safety_transitions": state["safety_transitions"],
              "final_safety_state": state["latest_safety_state"]}
    result["trace"] = state["trace"]
    if args.perturbation_start is not None and samples:
        origin = samples[0][0]
        start = origin + args.perturbation_start
        end = start + (args.perturbation_duration or 0.0)
        recovery_end = min(samples[-1][0], end + args.recovery_window)
        def window(lo, hi):
            values = [sample for sample in samples if lo <= sample[0] <= hi]
            return {
                "sample_count": len(values),
                "max_tilt_rad": max((sample[4] for sample in values), default=None),
                "min_height_m": min((sample[3] for sample in values), default=None),
            }
        def safety_at(sim_s):
            current = "unavailable"
            for transition in state["safety_transitions"]:
                if transition["sim_s"] <= sim_s:
                    current = transition["state"]
            return current
        first_failure = next(({
            "sim_s": round(sample[0], 3),
            "tilt_rad": round(sample[4], 4),
            "height_m": round(sample[3], 4),
            "safety_state": safety_at(sample[0]),
        } for sample in samples
            if sample[0] >= start and sample[4] >= 0.35), None)
        safe_stop_seen = any(
            transition["state"].lower() == "safe_stop"
            and transition["sim_s"] >= start
            for transition in state["safety_transitions"])
        pulse = window(start, end)
        recovery = window(end, recovery_end)
        result["perturbation"] = {
            "requested_force_n": args.perturbation_force_n,
            "axis": args.perturbation_axis,
            "start_sim_s": round(start, 3),
            "end_sim_s": round(end, 3),
            "recovery_end_sim_s": round(recovery_end, 3),
            "pre": window(start - 0.5, start),
            "pulse": pulse,
            "recovery": recovery,
            "first_failure": first_failure,
            "safe_stop_seen": safe_stop_seen,
            "recovery_screening_pass": (
                not safe_stop_seen
                and recovery["max_tilt_rad"] is not None
                and recovery["max_tilt_rad"] < 0.35
                and recovery["min_height_m"] is not None
                and recovery["min_height_m"] > 0.15
            ),
        }

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
    if samples and (args.drive_vx or args.drive_wz):
        trace = result["trace"]
        def point_at(offset):
            return min(trace, key=lambda item: abs(
                item["sim_s"] - samples[0][0] - offset))
        before = point_at(args.drive_start)
        during = point_at(args.drive_end)
        stopped = point_at(min(args.duration, args.drive_end + 1.0))
        final = trace[-1]
        result["motion"] = {
            "drive_delta_x_m": round(during["xyz"][0] - before["xyz"][0], 3),
            "drive_delta_yaw_rad": round(during["yaw_rad"] - before["yaw_rad"], 3),
            "stop_delta_xy_m": round(math.hypot(
                final["xyz"][0] - stopped["xyz"][0],
                final["xyz"][1] - stopped["xyz"][1]), 3),
            "stop_delta_yaw_rad": round(final["yaw_rad"] - stopped["yaw_rad"], 3),
        }
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
