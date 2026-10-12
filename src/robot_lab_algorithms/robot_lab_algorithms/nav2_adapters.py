"""Nav2 action servers that let the robot_lab planners drive (R7.6/R7.7).

The node planners used to run beside Nav2 and publish to side topics
(``/plan/rrt``, ``/cmd_vel_gap``), so selecting one still ran the plugin
planner.  These adapters serve the actions the Nav2 navigator calls,
under their own names, and the launch points the navigator's behavior tree
at them (``server_name``):

* ``nav2_global_planner``: ``nav2_msgs/action/ComputePathToPose`` with
  RRT, RRT*, PRM or the Voronoi roadmap (robot_lab_algorithms.grid_planners)
  on the inflated Nav2 global costmap.
* ``nav2_local_planner``: ``nav2_msgs/action/FollowPath`` with a pure
  pursuit tracker or follow-the-gap, publishing ``/cmd_vel`` like the
  controller server, with the same arrival, stop-before-collision and
  progress checks for both.

The Nav2 planner/controller servers keep running (lifecycle, costmaps,
recovery behaviours); only their actions go unused.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from robot_lab_algorithms import grid_planners


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def nearest_free(view, point, max_radius=0.6):
    """``point`` if free, else the nearest free point within ``max_radius``."""
    if view.is_free(*point):
        return tuple(point)
    for radius in np.arange(view.resolution, max_radius + 1e-9, view.resolution):
        angles = np.linspace(0, 2 * math.pi, max(8, int(2 * math.pi * radius / view.resolution)),
                             endpoint=False)
        ring = np.stack([point[0] + radius * np.cos(angles), point[1] + radius * np.sin(angles)], 1)
        free = view.points_free(ring)
        if free.any():
            return tuple(ring[int(np.argmax(free))])
    return None


def path_poses(waypoints, goal_yaw, spacing=0.05):
    """(x, y, yaw) poses along the waypoints; the last carries the goal heading."""
    points = grid_planners.densify(waypoints, spacing)
    poses = []
    for i, (x, y) in enumerate(points):
        if i + 1 < len(points):
            yaw = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
        else:
            yaw = goal_yaw
        poses.append((x, y, yaw))
    return poses


# ---------------------------------------------------------------------------
# Local control laws (pure functions, unit-tested without ROS)
# ---------------------------------------------------------------------------

class TrackingLimits:
    def __init__(self, max_linear=0.3, max_angular=1.0, lookahead=0.6, robot_radius=0.12,
                 rotate_to_heading=0.785, min_turn_radius=0.9, approach_distance=0.8):
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.lookahead = lookahead
        self.robot_radius = robot_radius
        self.rotate_to_heading = rotate_to_heading
        self.min_turn_radius = min_turn_radius
        self.approach_distance = approach_distance


def lookahead_point(path, pose, start_index, lookahead):
    """(index of the closest path point at/after start_index, lookahead point)."""
    xy = np.asarray([[p[0], p[1]] for p in path])
    window = xy[start_index:]
    d = np.hypot(window[:, 0] - pose[0], window[:, 1] - pose[1])
    closest = start_index + int(np.argmin(d[:max(1, min(len(d), 200))]))
    ahead = np.hypot(xy[closest:, 0] - pose[0], xy[closest:, 1] - pose[1])
    beyond = np.nonzero(ahead >= lookahead)[0]
    target = xy[closest + int(beyond[0])] if len(beyond) else xy[-1]
    return closest, (float(target[0]), float(target[1]))


def to_robot(pose, point):
    dx, dy = point[0] - pose[0], point[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


def pure_pursuit(pose, target, remaining, limits):
    """Regulated pure pursuit toward ``target`` (map frame) -> (v, w)."""
    x, y = to_robot(pose, target)
    bearing = math.atan2(y, x)
    if abs(bearing) > limits.rotate_to_heading:
        return 0.0, math.copysign(min(limits.max_angular, 1.5 * abs(bearing)), bearing)
    distance_sq = max(x * x + y * y, 1e-6)
    curvature = 2.0 * y / distance_sq
    v = limits.max_linear
    if abs(curvature) > 1e-6 and 1.0 / abs(curvature) < limits.min_turn_radius:
        v *= (1.0 / abs(curvature)) / limits.min_turn_radius
    v *= min(1.0, max(0.25, remaining / limits.approach_distance))
    v = max(0.05, v)
    w = max(-limits.max_angular, min(limits.max_angular, v * curvature))
    return v, w


def follow_the_gap(pose, target, remaining, limits, scan):
    """Steer to the free gap closest to the path direction -> (v, w).

    ``scan``: (angle_min, angle_increment, ranges).  A beam is open when its
    range exceeds a clearance horizon; gaps narrower than the robot at that
    horizon are ignored.  Speed follows the free distance ahead.
    """
    angle_min, increment, ranges = scan
    ranges = np.nan_to_num(np.asarray(ranges, dtype=float), nan=0.0, posinf=30.0)
    angles = angle_min + increment * np.arange(len(ranges))
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    x, y = to_robot(pose, target)
    goal_bearing = math.atan2(y, x)
    horizon = max(0.6, min(1.5, math.hypot(x, y)))
    open_ = ranges > horizon
    best, best_score = None, math.inf
    order = np.argsort(angles)
    a, o = angles[order], open_[order]
    i = 0
    while i < len(a):
        if not o[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(a) and o[j + 1]:
            j += 1
        width = (a[j] - a[i]) * horizon
        if width >= 2.0 * (limits.robot_radius + 0.1):
            centre = min(max(goal_bearing, a[i] + (limits.robot_radius + 0.1) / horizon),
                         a[j] - (limits.robot_radius + 0.1) / horizon)
            score = abs(wrap(centre - goal_bearing))
            if score < best_score:
                best, best_score = centre, score
        i = j + 1
    if best is None:
        return 0.0, math.copysign(limits.max_angular * 0.5, goal_bearing or 1.0)
    if abs(best) > limits.rotate_to_heading:
        return 0.0, math.copysign(min(limits.max_angular, 1.5 * abs(best)), best)
    front = ranges[np.abs(angles) < 0.35]
    clear = float(front.min()) if len(front) else horizon
    v = limits.max_linear * min(1.0, max(0.0, (clear - limits.robot_radius) / 1.0))
    v *= min(1.0, max(0.25, remaining / limits.approach_distance)) * math.cos(best)
    w = max(-limits.max_angular, min(limits.max_angular, 1.5 * best))
    return max(0.0, v), w


def blocked_ahead(scan, v, w, limits, horizon_s=0.8):
    """An obstacle within the swept circle of the next ``horizon_s`` of motion."""
    if scan is None or (abs(v) < 1e-3):
        return False
    angle_min, increment, ranges = scan
    ranges = np.nan_to_num(np.asarray(ranges, dtype=float), nan=30.0, posinf=30.0)
    angles = angle_min + increment * np.arange(len(ranges))
    px, py = ranges * np.cos(angles), ranges * np.sin(angles)
    margin = limits.robot_radius + 0.03
    for t in np.linspace(0.1, horizon_s, 8):
        theta = w * t
        if abs(w) > 1e-6:
            cx, cy = v / w * math.sin(theta), v / w * (1 - math.cos(theta))
        else:
            cx, cy = v * t, 0.0
        if np.any(np.hypot(px - cx, py - cy) < margin):
            return True
    return False


LOCAL_LAWS = {'regulated_pure_pursuit', 'follow_the_gap'}


# ---------------------------------------------------------------------------
# ROS nodes
# ---------------------------------------------------------------------------

def _ros():
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    return rclpy, ActionServer, CancelResponse, GoalResponse, ReentrantCallbackGroup, Node


def _make_global_planner_node():
    rclpy, ActionServer, CancelResponse, GoalResponse, Reentrant, Node = _ros()
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path
    from nav2_msgs.action import ComputePathToPose
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    import tf2_ros

    class GlobalPlannerServer(Node):
        def __init__(self):
            super().__init__('robot_lab_global_planner')
            self.declare_parameter('planner', 'rrt_star_planner')
            self.declare_parameter('action_name', '/robot_lab/compute_path_to_pose')
            self.declare_parameter('costmap_topic', '/global_costmap/costmap')
            self.declare_parameter('global_frame', 'map')
            self.declare_parameter('robot_frame', 'base_footprint')
            self.declare_parameter('seed', 0)
            self.declare_parameter('path_spacing', 0.05)
            # OccupancyGrid value from which a cell is blocked.  99 is the
            # inscribed radius; sampled paths then graze it and the
            # controller's collision check stops on them (2026-10-12
            # comparison: RRT 8/15, RRT* 9/15).  Lower values keep a margin
            # inside the inflation band (80 ~ 0.07 m beyond the footprint
            # at Bumperbot's 0.55 m / factor 3 inflation).
            self.declare_parameter('blocked_at', 99)
            self.name = str(self.get_parameter('planner').value)
            if self.name not in grid_planners.PLANNERS:
                raise ValueError('unknown planner %r; choose from %s'
                                 % (self.name, sorted(grid_planners.PLANNERS)))
            self.costmap = None
            self.lock = threading.Lock()
            self.calls = 0
            group = Reentrant()
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, self.get_parameter('costmap_topic').value,
                                     self.on_costmap, qos, callback_group=group)
            self.tf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf, self)
            self.path_type, self.pose_type = Path, PoseStamped
            self.server = ActionServer(self, ComputePathToPose,
                                       self.get_parameter('action_name').value,
                                       execute_callback=self.execute, callback_group=group,
                                       goal_callback=lambda _: GoalResponse.ACCEPT,
                                       cancel_callback=lambda _: CancelResponse.ACCEPT)
            self.get_logger().info('%s serving %s' % (self.name, self.get_parameter('action_name').value))

        def on_costmap(self, msg):
            with self.lock:
                self.costmap = msg

        def robot_xy(self):
            transform = self.tf.lookup_transform(self.get_parameter('global_frame').value,
                                                 self.get_parameter('robot_frame').value,
                                                 rclpy.time.Time())
            return (transform.transform.translation.x, transform.transform.translation.y)

        def execute(self, goal_handle):
            started = time.monotonic()
            request = goal_handle.request
            result = ComputePathToPose.Result()
            with self.lock:
                costmap = self.costmap
            if costmap is None:
                self.get_logger().warn('no global costmap yet')
                goal_handle.abort()
                return result
            try:
                start = ((request.start.pose.position.x, request.start.pose.position.y)
                         if request.use_start else self.robot_xy())
            except Exception as exc:  # noqa: BLE001 - TF errors vary by distro
                self.get_logger().warn('no robot pose: %s' % exc)
                goal_handle.abort()
                return result
            goal = (request.goal.pose.position.x, request.goal.pose.position.y)
            view = grid_planners.GridView.from_msg(
                costmap, blocked_at=int(self.get_parameter('blocked_at').value))
            entry = nearest_free(view, start)
            if entry is None or not view.is_free(*goal):
                self.get_logger().warn('%s: start or goal blocked in the costmap' % self.name)
                goal_handle.abort()
                return result
            planner = grid_planners.PLANNERS[self.name]
            self.calls += 1
            if self.name == 'voronoi_planner':
                waypoints = planner(view, entry, goal)
            else:
                rng = np.random.default_rng(int(self.get_parameter('seed').value) + self.calls)
                waypoints = planner(view, entry, goal, rng)
            if not waypoints:
                self.get_logger().warn('%s found no path to (%.2f, %.2f)' % ((self.name,) + goal))
                goal_handle.abort()
                return result
            if entry != tuple(start):
                waypoints = [tuple(start)] + list(waypoints)
            path = self.path_type()
            path.header.frame_id = costmap.header.frame_id or self.get_parameter('global_frame').value
            path.header.stamp = self.get_clock().now().to_msg()
            for x, y, yaw in path_poses(waypoints, yaw_of(request.goal.pose.orientation),
                                        float(self.get_parameter('path_spacing').value)):
                pose = self.pose_type()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                path.poses.append(pose)
            result.path = path
            elapsed = time.monotonic() - started
            result.planning_time.sec = int(elapsed)
            result.planning_time.nanosec = int((elapsed % 1.0) * 1e9)
            goal_handle.succeed()
            return result

    return GlobalPlannerServer


def _make_local_planner_node():
    rclpy, ActionServer, CancelResponse, GoalResponse, Reentrant, Node = _ros()
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import FollowPath
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    import tf2_ros

    class LocalPlannerServer(Node):
        def __init__(self):
            super().__init__('robot_lab_local_planner')
            self.declare_parameter('planner', 'regulated_pure_pursuit')
            self.declare_parameter('action_name', '/robot_lab/follow_path')
            self.declare_parameter('cmd_vel_topic', '/cmd_vel')
            self.declare_parameter('scan_topic', '/scan')
            self.declare_parameter('global_frame', 'map')
            self.declare_parameter('robot_frame', 'base_footprint')
            self.declare_parameter('rate', 20.0)
            self.declare_parameter('max_linear', 0.3)
            self.declare_parameter('max_angular', 1.0)
            self.declare_parameter('lookahead', 0.6)
            self.declare_parameter('robot_radius', 0.12)
            self.declare_parameter('xy_goal_tolerance', 0.10)
            self.declare_parameter('yaw_goal_tolerance', 0.15)
            self.declare_parameter('blocked_abort_s', 5.0)
            self.declare_parameter('progress_radius', 0.2)
            self.declare_parameter('progress_time_s', 15.0)
            self.law = str(self.get_parameter('planner').value)
            if self.law not in LOCAL_LAWS:
                raise ValueError('unknown local planner %r; choose from %s' % (self.law, sorted(LOCAL_LAWS)))
            g = lambda name: self.get_parameter(name).value  # noqa: E731
            self.limits = TrackingLimits(max_linear=float(g('max_linear')), max_angular=float(g('max_angular')),
                                         lookahead=float(g('lookahead')), robot_radius=float(g('robot_radius')))
            self.scan = None
            self.active = None
            self.lock = threading.Lock()
            group = Reentrant()
            self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
            self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data,
                                     callback_group=group)
            self.tf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf, self)
            self.server = ActionServer(self, FollowPath, g('action_name'), execute_callback=self.execute,
                                       goal_callback=lambda _: GoalResponse.ACCEPT,
                                       handle_accepted_callback=self.on_accepted,
                                       cancel_callback=lambda _: CancelResponse.ACCEPT,
                                       callback_group=group)
            self.get_logger().info('%s serving %s' % (self.law, g('action_name')))

        def on_scan(self, msg):
            self.scan = (msg.angle_min, msg.angle_increment, list(msg.ranges))

        def on_accepted(self, goal_handle):
            # A new path preempts the running one, as in the Nav2 controller server.
            with self.lock:
                self.active = goal_handle.goal_id.uuid.tobytes()
            goal_handle.execute()

        def pose(self):
            t = self.tf.lookup_transform(self.get_parameter('global_frame').value,
                                         self.get_parameter('robot_frame').value, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

        def stop(self):
            self.cmd_pub.publish(Twist())

        def execute(self, goal_handle):
            g = lambda name: self.get_parameter(name).value  # noqa: E731
            me = goal_handle.goal_id.uuid.tobytes()
            path = [(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation))
                    for p in goal_handle.request.path.poses]
            result = FollowPath.Result()
            if not path:
                goal_handle.abort()
                return result
            period = 1.0 / float(g('rate'))
            index, blocked_since, anchor = 0, None, None
            feedback = FollowPath.Feedback()
            clock = self.get_clock()
            while rclpy.ok():
                with self.lock:
                    preempted = self.active != me
                if preempted:
                    goal_handle.abort()
                    return result
                if goal_handle.is_cancel_requested:
                    self.stop()
                    goal_handle.canceled()
                    return result
                try:
                    pose = self.pose()
                except Exception:  # noqa: BLE001
                    time.sleep(period)
                    continue
                now = clock.now().nanoseconds * 1e-9
                goal = path[-1]
                distance = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
                if distance <= float(g('xy_goal_tolerance')):
                    error = wrap(goal[2] - pose[2])
                    if abs(error) <= float(g('yaw_goal_tolerance')):
                        self.stop()
                        goal_handle.succeed()
                        return result
                    twist = Twist()
                    twist.angular.z = math.copysign(max(0.2, min(self.limits.max_angular, 1.5 * abs(error))), error)
                    self.cmd_pub.publish(twist)
                    anchor = None
                    time.sleep(period)
                    continue
                index, target = lookahead_point(path, pose, index, self.limits.lookahead)
                remaining = distance
                if self.law == 'follow_the_gap' and self.scan is not None:
                    v, w = follow_the_gap(pose, target, remaining, self.limits, self.scan)
                else:
                    v, w = pure_pursuit(pose, target, remaining, self.limits)
                if blocked_ahead(self.scan, v, w, self.limits):
                    v = 0.0
                    blocked_since = blocked_since if blocked_since is not None else now
                    if now - blocked_since > float(g('blocked_abort_s')):
                        self.stop()
                        self.get_logger().warn('%s: blocked ahead; aborting for recovery' % self.law)
                        goal_handle.abort()
                        return result
                else:
                    blocked_since = None
                # Progress check (simulated time), like Nav2's SimpleProgressChecker.
                if anchor is None or math.hypot(pose[0] - anchor[1], pose[1] - anchor[2]) \
                        > float(g('progress_radius')):
                    anchor = (now, pose[0], pose[1])
                elif now - anchor[0] > float(g('progress_time_s')):
                    self.stop()
                    self.get_logger().warn('%s: failed to make progress' % self.law)
                    goal_handle.abort()
                    return result
                twist = Twist()
                twist.linear.x, twist.angular.z = float(v), float(w)
                self.cmd_pub.publish(twist)
                feedback.distance_to_goal, feedback.speed = float(distance), float(abs(v))
                goal_handle.publish_feedback(feedback)
                time.sleep(period)
            return result

    return LocalPlannerServer


def _spin(factory):
    rclpy = _ros()[0]
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init()
    node = factory()()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def global_planner_main():
    _spin(_make_global_planner_node)


def local_planner_main():
    _spin(_make_local_planner_node)
