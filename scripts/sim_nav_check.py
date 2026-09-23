#!/usr/bin/env python3
"""Send one Nav2 goal to a running navigation launch and report the outcome.

The same request as an RViz "2D Goal Pose": a NavigateToPose goal in the
``map`` frame.  The goal is picked from the published ``/map`` itself, a
free cell about ``--distance`` metres from the robot with ``--clearance``
metres of free space around it, so the check works on every map.

Reports, as one JSON object: whether the navigation stack came up (action
server, map, localization transform), the goal, the action result, the time
taken, and the final distance to the goal from both the localization estimate
(``map`` -> base frame) and simulator ground truth when available.

Usage: sim_nav_check.py [--distance 2.0] [--clearance 0.35] [--timeout 600]
"""
import argparse
import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

STATUS = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
          GoalStatus.STATUS_CANCELED: "canceled", GoalStatus.STATUS_UNKNOWN: "unknown"}


class Check(Node):
    def __init__(self):
        super().__init__("sim_nav_check")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map = None
        self.truth = None
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), latched)
        self.create_subscription(Odometry, "/odom/ground_truth", lambda m: setattr(self, "truth", m), 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def pose(self, base):
        try:
            t = self.tf.lookup_transform("map", base, rclpy.time.Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y


def free_goal(grid, start, distance, clearance):
    """A free map cell about *distance* from *start* with *clearance* around it."""
    info = grid.info
    res, width, height = info.resolution, info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y
    data = grid.data
    radius = int(math.ceil(clearance / res))

    def free(cx, cy):
        if not (radius <= cx < width - radius and radius <= cy < height - radius):
            return False
        for dy in range(-radius, radius + 1):
            row = (cy + dy) * width
            for dx in range(-radius, radius + 1):
                if data[row + cx + dx] != 0:
                    return False
        return True

    for scale in (1.0, 0.75, 0.5, 1.5):
        for k in range(16):
            angle = 2.0 * math.pi * k / 16.0
            gx = start[0] + scale * distance * math.cos(angle)
            gy = start[1] + scale * distance * math.sin(angle)
            cx, cy = int((gx - ox) / res), int((gy - oy) / res)
            # The straight line must be free too, so the goal is reachable
            # without relying on a long detour.
            steps = int(scale * distance / res)
            line_ok = all(free(int((start[0] + (gx - start[0]) * i / steps - ox) / res),
                               int((start[1] + (gy - start[1]) * i / steps - oy) / res))
                          for i in range(steps // 3, steps + 1, 2)) if steps else False
            if free(cx, cy) and line_ok:
                return gx, gy, angle
    return None


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, default=2.0)
    parser.add_argument("--clearance", type=float, default=0.35)
    parser.add_argument("--base", default="base_footprint")
    parser.add_argument("--timeout", type=float, default=600.0)
    args = parser.parse_args(argv)

    rclpy.init()
    node = Check()
    result = {}
    started = time.monotonic()
    stages = (("map", lambda: node.map is not None),
              ("localization_tf", lambda: node.pose(args.base) is not None),
              ("action_server", lambda: node.action.server_is_ready()))
    for name, ready in stages:
        if not node.spin_until(ready, args.timeout):
            result["error"] = "no %s after %.0f s" % (name, time.monotonic() - started)
            print(json.dumps(result))
            return 1
    result["stack_ready_wall_s"] = round(time.monotonic() - started, 1)
    node.spin_until(lambda: False, 5.0)  # let AMCL settle on its initial pose
    start = node.pose(args.base)
    goal_xy = free_goal(node.map, start, args.distance, args.clearance)
    if goal_xy is None:
        result["error"] = "no free goal near %s" % (start,)
        print(json.dumps(result))
        return 1
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = goal_xy[0], goal_xy[1]
    goal.pose.pose.orientation.z = math.sin(goal_xy[2] / 2.0)
    goal.pose.pose.orientation.w = math.cos(goal_xy[2] / 2.0)
    result.update(start=[round(v, 2) for v in start],
                  goal=[round(goal_xy[0], 2), round(goal_xy[1], 2)])
    sent = time.monotonic()
    future = node.action.send_goal_async(goal)
    node.spin_until(future.done, 30.0)
    handle = future.result() if future.done() else None
    if handle is None or not handle.accepted:
        result["outcome"] = "rejected"
        print(json.dumps(result))
        return 1
    done = handle.get_result_async()
    node.spin_until(done.done, args.timeout)
    status = done.result().status if done.done() else None
    result["outcome"] = STATUS.get(status, "timeout" if status is None else str(status))
    result["wall_s"] = round(time.monotonic() - sent, 1)
    final = node.pose(args.base)
    if final:
        result["final_error_estimate_m"] = round(math.hypot(final[0] - goal_xy[0], final[1] - goal_xy[1]), 3)
    if node.truth is not None:
        p = node.truth.pose.pose.position
        result["final_truth_xy"] = [round(p.x, 2), round(p.y, 2)]
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["outcome"] == "succeeded" else 1


if __name__ == "__main__":
    sys.exit(main())
