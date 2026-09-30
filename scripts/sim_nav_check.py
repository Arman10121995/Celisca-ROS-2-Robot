#!/usr/bin/env python3
"""Send one Nav2 goal to a running navigation launch and report the outcome.

The same request as an RViz "2D Goal Pose": a map-frame PoseStamped sent to
``/robot_lab/goal_pose`` with ``--via-topic``, or a direct NavigateToPose
action otherwise.  The goal is picked from the published ``/map`` itself, a
free cell about ``--distance`` metres from the robot with ``--clearance``
metres of free space around it, so the check works on every map.

Reports, as one JSON object: whether the navigation stack came up (action
server, map, localization transform), the goal, the action result, the time
taken, and the final distance to the goal from both the localization estimate
(``map`` -> base frame) and simulator ground truth when available.

Usage: sim_nav_check.py [--distance 2.0] [--clearance 0.35] [--timeout 600]
       sim_nav_check.py --offset-x -1.0 --offset-y 0.0 --goal-yaw-deg 0
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
        self.motion_source = None
        self.monitor_motion = False
        self.previous_odom_pose = None
        self.forward_distance_m = 0.0
        self.reverse_distance_m = 0.0
        self.heading_travel_rad = 0.0
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), latched)
        self.create_subscription(Odometry, "/odom/ground_truth", self.on_truth, 10)
        self.create_subscription(Odometry, "/robot_lab_controller/odom",
                                 lambda m: self.on_motion(m, "/robot_lab_controller/odom"), 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.action = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def on_truth(self, message):
        self.truth = message
        self.on_motion(message, "/odom/ground_truth")

    def on_motion(self, message, source):
        if self.motion_source == "/odom/ground_truth" and source != self.motion_source:
            return
        if source != self.motion_source:
            self.motion_source = source
            self.previous_odom_pose = None
            self.forward_distance_m = self.reverse_distance_m = self.heading_travel_rad = 0.0
        if not self.monitor_motion:
            return
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        current = (pose.position.x, pose.position.y, yaw)
        if self.previous_odom_pose is not None:
            x, y, previous_yaw = self.previous_odom_pose
            dx, dy = current[0] - x, current[1] - y
            signed_distance = dx * math.cos(previous_yaw) + dy * math.sin(previous_yaw)
            if signed_distance > 0:
                self.forward_distance_m += signed_distance
            else:
                self.reverse_distance_m -= signed_distance
            self.heading_travel_rad += abs(math.atan2(math.sin(yaw - previous_yaw),
                                                      math.cos(yaw - previous_yaw)))
        self.previous_odom_pose = current

    def motion_report(self):
        if self.previous_odom_pose is None:
            return {}
        return {
            "motion_source": self.motion_source,
            "motion_forward_m": round(self.forward_distance_m, 3),
            "motion_reverse_m": round(self.reverse_distance_m, 3),
            "motion_heading_travel_rad": round(self.heading_travel_rad, 3),
        }

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


def free_cell(grid, x, y, clearance):
    """Whether a map-frame point has the requested square free-space clearance."""
    info = grid.info
    res, width, height = info.resolution, info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y
    data = grid.data
    radius = int(math.ceil(clearance / res))
    cx, cy = int((x - ox) / res), int((y - oy) / res)
    if not (radius <= cx < width - radius and radius <= cy < height - radius):
        return False
    for dy in range(-radius, radius + 1):
        row = (cy + dy) * width
        for dx in range(-radius, radius + 1):
            if data[row + cx + dx] != 0:
                return False
    return True


def free_goal(grid, start, distance, clearance):
    """A free map cell about *distance* from *start* with *clearance* around it."""
    res = grid.info.resolution

    for scale in (1.0, 0.75, 0.5, 1.5):
        for k in range(16):
            angle = 2.0 * math.pi * k / 16.0
            gx = start[0] + scale * distance * math.cos(angle)
            gy = start[1] + scale * distance * math.sin(angle)
            # The straight line must be free too, so the goal is reachable
            # without relying on a long detour.
            steps = int(scale * distance / res)
            line_ok = all(free_cell(grid, start[0] + (gx - start[0]) * i / steps,
                                    start[1] + (gy - start[1]) * i / steps, clearance)
                          for i in range(steps // 3, steps + 1, 2)) if steps else False
            if free_cell(grid, gx, gy, clearance) and line_ok:
                return gx, gy, angle
    return None


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, default=2.0)
    parser.add_argument("--clearance", type=float, default=0.35)
    parser.add_argument("--base", default="base_footprint")
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--offset-x", type=float, help="map-frame x offset from the localized start")
    parser.add_argument("--offset-y", type=float, default=0.0,
                        help="map-frame y offset; requires --offset-x")
    parser.add_argument("--goal-yaw-deg", type=float,
                        help="goal heading in map frame; defaults to direction of travel")
    parser.add_argument("--via-topic", action="store_true",
                        help="publish the goal on /robot_lab/goal_pose as RViz's 2D Goal Pose does")
    args = parser.parse_args(argv)
    if args.offset_x is None and args.offset_y != 0.0:
        parser.error("--offset-y requires --offset-x")

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
    if args.offset_x is None:
        goal_xy = free_goal(node.map, start, args.distance, args.clearance)
    else:
        gx, gy = start[0] + args.offset_x, start[1] + args.offset_y
        yaw = math.atan2(args.offset_y, args.offset_x)
        goal_xy = (gx, gy, yaw) if free_cell(node.map, gx, gy, args.clearance) else None
    if goal_xy is None:
        result["error"] = "no free goal near %s" % (start,)
        print(json.dumps(result))
        return 1
    goal = NavigateToPose.Goal()
    goal_yaw = (math.radians(args.goal_yaw_deg) if args.goal_yaw_deg is not None
                else goal_xy[2])
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = goal_xy[0], goal_xy[1]
    goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
    result.update(start=[round(v, 2) for v in start],
                  goal=[round(goal_xy[0], 2), round(goal_xy[1], 2)],
                  goal_yaw_deg=round(math.degrees(goal_yaw), 1))
    node.monitor_motion = True
    sent = time.monotonic()
    if args.via_topic:
        from geometry_msgs.msg import PoseStamped
        from action_msgs.msg import GoalStatusArray
        statuses = []
        node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                 lambda m: statuses.append(m), 10)
        publisher = node.create_publisher(PoseStamped, "/robot_lab/goal_pose", 10)
        node.spin_until(lambda: publisher.get_subscription_count() > 0, 30.0)
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(goal.pose)

        terminal = None
        def finished():
            nonlocal terminal
            for message in statuses[-1:]:
                for status in message.status_list:
                    if status.status in (GoalStatus.STATUS_SUCCEEDED,
                                         GoalStatus.STATUS_ABORTED,
                                         GoalStatus.STATUS_CANCELED):
                        terminal = status.status
                        return True
            return False
        node.spin_until(finished, args.timeout)
        last = terminal
        result["outcome"] = STATUS.get(last, "no goal accepted" if last is None else str(last))
        result["wall_s"] = round(time.monotonic() - sent, 1)
        final = node.pose(args.base)
        if final:
            result["final_pose_estimate_xy"] = [round(v, 3) for v in final]
            result["final_error_estimate_m"] = round(math.hypot(final[0] - goal_xy[0], final[1] - goal_xy[1]), 3)
        if node.truth is not None:
            p = node.truth.pose.pose.position
            result["final_truth_xy"] = [round(p.x, 3), round(p.y, 3)]
        result.update(node.motion_report())
        print(json.dumps(result))
        return 0 if result["outcome"] == "succeeded" else 1
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
        result["final_pose_estimate_xy"] = [round(v, 3) for v in final]
        result["final_error_estimate_m"] = round(math.hypot(final[0] - goal_xy[0], final[1] - goal_xy[1]), 3)
    if node.truth is not None:
        p = node.truth.pose.pose.position
        result["final_truth_xy"] = [round(p.x, 2), round(p.y, 2)]
    result.update(node.motion_report())
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["outcome"] == "succeeded" else 1


if __name__ == "__main__":
    sys.exit(main())
