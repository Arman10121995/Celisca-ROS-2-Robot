#!/usr/bin/env python3
"""Live check of a simulator bridge against the map it loaded.

Run against a robot already launched in any backend (see
``sim_sensor_probe.sh``).  It measures, from the published topics only:

* ``/scan`` against ranges ray-cast through the SDF world's boxes from the
  ground-truth pose and the laser link's pose in the robot description;
* ``/odom/ground_truth`` at rest (roll/pitch, so a quaternion-order mix-up
  shows up as a 180 degree roll) and its twist while driving (body frame:
  a forward command must give vx, not vy);
* optionally the OAK-D RGB-D topics, comparing the centre depth pixel with the
  same ray cast from the camera link;
* straight and turning drive against the commanded speeds;
* optionally ``/robot_lab/reset`` (pose restored, clock monotonic).

Only box geometry is ray-cast, so compare on box-built arenas such as
``nav_maze``.  Prints one JSON object.

Usage: sim_sensor_probe.py --map nav_maze [--robot bumperbot]
       [--timeout 300] [--camera] [--reset]
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan

from robot_lab_utils import camera_model, sdf_world, sim_frames

IDENTITY = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _quaternion(msg_orientation):
    o = msg_orientation
    return sim_frames.wxyz_from_xyzw([o.x, o.y, o.z, o.w])


def _box_hit(shape, ox, oy, dx, dy):
    """Entry distance of a horizontal ray into a yaw-only box, or None."""
    cx, cy, _ = shape["position"]
    hx, hy = shape["size"][0] / 2.0, shape["size"][1] / 2.0
    yaw = sim_frames.yaw_of(shape["orientation"])
    c, s = math.cos(-yaw), math.sin(-yaw)
    px, py = c * (ox - cx) - s * (oy - cy), s * (ox - cx) + c * (oy - cy)
    vx, vy = c * dx - s * dy, s * dx + c * dy
    tmin, tmax = -1e18, 1e18
    for p, v, h in ((px, vx, hx), (py, vy, hy)):
        if abs(v) < 1e-12:
            if abs(p) > h:
                return None
            continue
        t1, t2 = (-h - p) / v, (h - p) / v
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0.0) or tmin <= 0.0:
        return None
    return tmin


def expected_ranges(shapes, origin, yaw, samples, range_max):
    """Ranges of a planar scan starting at -pi about *yaw*, from the map."""
    ox, oy, oz = origin
    boxes = [s for s in shapes if s["type"] == "box"
             and abs(s["position"][2] - oz) <= s["size"][2] / 2.0]
    ranges = []
    for i in range(samples):
        angle = yaw - math.pi + i * 2.0 * math.pi / samples
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        for box in boxes:
            t = _box_hit(box, ox, oy, dx, dy)
            if t is not None and t < best:
                best = t
        ranges.append(best if best < range_max else math.inf)
    return ranges


def robot_urdf(robot_id):
    config = os.path.join(get_package_share_directory("robot_lab_robots"),
                          "config", "robots.yaml")
    entry = yaml.safe_load(open(config))["robots"][robot_id]
    xacro = os.path.join(get_package_share_directory("robot_lab_robots"),
                         entry["xacro"])
    return subprocess.run(["xacro", xacro], capture_output=True, text=True,
                          check=True, timeout=60).stdout


class Probe(Node):
    def __init__(self):
        super().__init__("sim_sensor_probe")
        self.odom, self.scans, self.joints = [], [], []
        self.create_subscription(Odometry, "/odom/ground_truth", self.odom.append, 50)
        self.create_subscription(LaserScan, "/scan", self.scans.append, 10)
        self.create_subscription(JointState, "/joint_states", self.joints.append, 50)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def spin_until(self, predicate, wall_timeout, twist=None):
        deadline = time.monotonic() + wall_timeout
        last_cmd = 0.0
        while time.monotonic() < deadline:
            if twist is not None and time.monotonic() - last_cmd > 0.05:
                self.cmd.publish(twist)
                last_cmd = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False


def scan_check(probe, shapes, pose, laser_offset):
    scan = probe.scans[-1]
    n = len(scan.ranges)
    (lx, ly, lz), lq = sim_frames.mounted_pose(
        (pose.position.x, pose.position.y, pose.position.z),
        _quaternion(pose.orientation), laser_offset)
    expected = expected_ranges(shapes, (lx, ly, lz), sim_frames.yaw_of(lq), n,
                               scan.range_max)
    errors = sorted(abs(m - e) for m, e in zip(scan.ranges, expected)
                    if math.isfinite(m) and math.isfinite(e))
    return {
        "samples": n, "frame": scan.header.frame_id,
        "geometry_consistent": round((scan.angle_max - scan.angle_min)
                                     / scan.angle_increment) + 1 == n,
        "finite": sum(1 for r in scan.ranges if math.isfinite(r)),
        "expected_finite": sum(1 for e in expected if math.isfinite(e)),
        "compared": len(errors),
        "median_abs_error_m": round(errors[len(errors) // 2], 4) if errors else None,
        "within_5cm": round(sum(e < 0.05 for e in errors) / len(errors), 3) if errors else None,
    }


def camera_check(probe, shapes, pose, camera_offset, timeout):
    import numpy as np
    from sensor_msgs.msg import CameraInfo, Image
    rgb, depth, info = [], [], []
    probe.create_subscription(Image, camera_model.OAKD["rgb_topic"], rgb.append, 5)
    probe.create_subscription(Image, camera_model.OAKD["depth_topic"], depth.append, 5)
    probe.create_subscription(CameraInfo, camera_model.OAKD["info_topic"], info.append, 5)
    if not probe.spin_until(lambda: len(rgb) >= 2 and depth and info, timeout):
        return {"error": "no camera data (rgb=%d depth=%d info=%d)"
                         % (len(rgb), len(depth), len(info))}
    d, c, k = depth[-1], rgb[-1], info[-1]
    depth_image = np.frombuffer(bytes(d.data), dtype=np.float32).reshape(d.height, d.width)
    colour = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(c.height, c.width, 3)
    (cx, cy, cz), cq = sim_frames.mounted_pose(
        (pose.position.x, pose.position.y, pose.position.z),
        _quaternion(pose.orientation), camera_offset)
    # One-sample "scan" starts at yaw - pi, so add pi to look straight ahead.
    expected = expected_ranges(shapes, (cx, cy, cz), sim_frames.yaw_of(cq) + math.pi,
                               1, camera_model.OAKD["far"])[0]
    return {
        "rgb": [c.width, c.height, c.encoding, c.header.frame_id],
        "depth": [d.width, d.height, d.encoding],
        "k": [round(v, 2) for v in k.k],
        "centre_depth_m": round(float(depth_image[d.height // 2, d.width // 2]), 3),
        "expected_centre_depth_m": round(expected, 3),
        "depth_finite_fraction": round(float(np.isfinite(depth_image).mean()), 3),
        "rgb_mean": round(float(colour.mean()), 1),
        "rgb_nonblack_fraction": round(float((colour.max(axis=2) > 8).mean()), 3),
    }


def drive(probe, linear, angular, sim_seconds, timeout):
    twist = Twist()
    twist.linear.x, twist.angular.z = linear, angular
    probe.odom.clear()
    probe.joints.clear()
    if not probe.spin_until(lambda: probe.odom, 30.0, twist):
        return {"error": "no odometry while driving"}
    start = _stamp(probe.odom[0])
    completed = probe.spin_until(
        lambda: _stamp(probe.odom[-1]) - start >= sim_seconds, timeout, twist)
    for _ in range(5):
        probe.cmd.publish(Twist())
        rclpy.spin_once(probe, timeout_sec=0.05)
    steady = [m for m in probe.odom if _stamp(m) - start >= 0.5] or probe.odom
    first, last = steady[0], steady[-1]
    dt = max(_stamp(last) - _stamp(first), 1e-9)
    dyaw = math.remainder(sim_frames.yaw_of(_quaternion(last.pose.pose.orientation))
                          - sim_frames.yaw_of(_quaternion(first.pose.pose.orientation)),
                          2.0 * math.pi)
    wheels = {}
    for msg in probe.joints[len(probe.joints) // 2:]:
        for name, velocity in zip(msg.name, msg.velocity):
            if "wheel" in name:
                wheels.setdefault(name, []).append(velocity)
    mean = lambda values: sum(values) / len(values)  # noqa: E731
    return {
        "command": [linear, angular], "sim_s": round(_stamp(probe.odom[-1]) - start, 2),
        "completed": completed,
        "twist_vx": round(mean([m.twist.twist.linear.x for m in steady]), 4),
        "twist_abs_vy": round(mean([abs(m.twist.twist.linear.y) for m in steady]), 4),
        "twist_wz": round(mean([m.twist.twist.angular.z for m in steady]), 4),
        "pose_speed": round(math.hypot(last.pose.pose.position.x - first.pose.pose.position.x,
                                       last.pose.pose.position.y - first.pose.pose.position.y) / dt, 4),
        "pose_yaw_rate": round(dyaw / dt, 4),
        "wheel_velocity_mean": {k: round(mean(v), 3) for k, v in wheels.items()},
    }


def reset_check(probe, rest, timeout):
    from std_srvs.srv import Trigger
    client = probe.create_client(Trigger, "/robot_lab/reset")
    if not client.wait_for_service(timeout_sec=10.0):
        return {"error": "no /robot_lab/reset service"}
    before = probe.odom[-1]
    future = client.call_async(Trigger.Request())
    probe.spin_until(future.done, 30.0)
    if not future.done():
        return {"error": "reset call timed out"}
    called_at = _stamp(probe.odom[-1])
    probe.odom.clear()
    probe.spin_until(lambda: probe.odom and _stamp(probe.odom[-1]) - called_at >= 1.0, timeout)
    after = probe.odom[-1] if probe.odom else None
    distance = lambda m: math.hypot(m.pose.pose.position.x - rest.position.x,  # noqa: E731
                                    m.pose.pose.position.y - rest.position.y)
    return {
        "success": future.result().success, "message": future.result().message,
        "distance_from_start_before_m": round(distance(before), 3),
        "distance_from_start_after_m": round(distance(after), 3) if after else None,
        "clock_monotonic": bool(after) and _stamp(after) >= called_at,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", required=True)
    parser.add_argument("--robot", default="bumperbot")
    parser.add_argument("--timeout", type=float, default=300.0,
                        help="wall-clock seconds per phase")
    parser.add_argument("--camera", action="store_true")
    parser.add_argument("--reset", action="store_true")
    args = parser.parse_args(argv)

    urdf = robot_urdf(args.robot)
    laser_offset = sim_frames.offset_from_root(urdf, "laser_link") \
        or ((0.0, 0.0, 0.12), IDENTITY[1])
    maps = get_package_share_directory("robot_lab_maps")
    world = os.path.join(maps, "maps", args.map, "worlds", args.map + ".world")
    shapes, _ = sdf_world.extract_static_shapes(world, sdf_world.uri_resolver(
        [os.path.join(get_package_share_directory("robot_lab_models"), "models")],
        get_package_share_directory))

    rclpy.init()
    probe = Probe()
    result = {"robot": args.robot, "map": args.map}
    if not probe.spin_until(lambda: probe.odom and probe.scans, args.timeout):
        result["error"] = "no odometry/scan (odom=%d scan=%d)" % (len(probe.odom), len(probe.scans))
        print(json.dumps(result))
        return 1
    probe.scans.clear()
    probe.spin_until(lambda: len(probe.scans) >= 2, args.timeout)
    rest = probe.odom[-1].pose.pose
    w, x, y, z = _quaternion(rest.orientation)
    result["rest"] = {
        "position": [round(v, 3) for v in (rest.position.x, rest.position.y, rest.position.z)],
        "roll_pitch": [round(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), 3),
                       round(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))), 3)],
    }
    result["scan"] = scan_check(probe, shapes, rest, laser_offset)
    if args.camera:
        camera_offset = sim_frames.offset_from_root(urdf, camera_model.OAKD["link"])
        result["camera"] = (camera_check(probe, shapes, rest, camera_offset, args.timeout)
                            if camera_offset else {"error": "robot has no camera link"})
    result["straight"] = drive(probe, 0.3, 0.0, 3.0, args.timeout)
    result["turn"] = drive(probe, 0.15, 0.6, 3.0, args.timeout)
    if args.reset:
        result["reset"] = reset_check(probe, rest, args.timeout)
    print(json.dumps(result))
    probe.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
