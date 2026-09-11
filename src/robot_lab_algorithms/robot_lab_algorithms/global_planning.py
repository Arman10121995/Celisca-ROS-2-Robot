"""Global planning algorithms (P5).

Adds two global planners that round out the global_planning category:
  1. rrt_planner     - Rapidly-exploring Random Tree sampling-based planner.
  2. voronoi_planner - grid-based planner following Voronoi-roadmap-like
                       clearance-maximizing corridors (approximated by a
                       distance-transform ridge follower on a cost grid).

Both are pure-Python deterministic implementations that plan a waypoint path
from a start to a goal over an occupancy grid.
"""

from __future__ import annotations

import math
import random
import sys


try:
    import rclpy
    from rclpy.node import Node as _RclpyNode
    Node = _RclpyNode
except Exception:  # pragma: no cover - optional dependency
    rclpy = None

    class Node:
        """Fallback base when rclpy is unavailable (dry/local testing)."""
        def __init__(self, node_name='node'):
            self._node_name = node_name
            self._params = {}
        def declare_parameter(self, name, value=None):
            self._params.setdefault(name, value)
        def get_parameter(self, name):
            class _P:
                value = self._params.get(name, None)
            return _P()
        def get_logger(self):
            name = self._node_name
            class _L:
                def info(self, *a, **k):
                    print(f'[{name}]', *a)
                def warn(self, *a, **k):
                    print(f'[{name}] WARN', *a)
            return _L()
        def get_name(self):
            return self._node_name


def _to_grid(x, y, origin, resolution):
    return int(round((x - origin[0]) / resolution)), int(round((y - origin[1]) / resolution))


def _from_grid(c, r, origin, resolution):
    return origin[0] + c * resolution, origin[1] + r * resolution


class RRTPlanner:
    """Sampling-based RRT planner over a binary occupancy grid."""

    def __init__(self, step=1.0, max_iter=2000, goal_tol=0.6):
        self.step = step
        self.max_iter = max_iter
        self.goal_tol = goal_tol

    def plan(self, start, goal, is_free, bounds, resolution=0.05, seed=0):
        """Return a list of (x, y) waypoints from start to goal.

        is_free(x, y) -> bool; bounds = (min_x, min_y, max_x, max_y).
        """
        rng = random.Random(seed)
        nodes = [start]
        parents = {start: None}
        goal_reached = None
        for _ in range(self.max_iter):
            if rng.random() < 0.2:
                sample = goal
            else:
                sample = (
                    rng.uniform(bounds[0], bounds[2]),
                    rng.uniform(bounds[1], bounds[3]),
                )
            if not is_free(sample[0], sample[1]):
                continue
            nearest = min(nodes, key=lambda n: math.hypot(n[0] - sample[0], n[1] - sample[1]))
            d = math.hypot(sample[0] - nearest[0], sample[1] - nearest[1])
            if d < 1e-6:
                continue
            newx = nearest[0] + (sample[0] - nearest[0]) * min(1.0, self.step / d)
            newy = nearest[1] + (sample[1] - nearest[1]) * min(1.0, self.step / d)
            if not is_free(newx, newy):
                continue
            new = (newx, newy)
            if new in parents:
                continue
            nodes.append(new)
            parents[new] = nearest
            if math.hypot(new[0] - goal[0], new[1] - goal[1]) <= self.goal_tol:
                goal_reached = new
                break
        if goal_reached is None:
            return []
        path = []
        node = goal_reached
        while node is not None:
            path.append(node)
            node = parents[node]
        path.reverse()
        return path


class VoronoiPlanner:
    """Ridge-following global planner that maximizes local clearance."""

    def __init__(self):
        pass

    def plan(self, start, goal, cost_at, step=0.5, max_steps=2000):
        """Greedy walk that ascends cost ridges toward the goal.

        cost_at(x, y) -> float clearance cost (higher is more clear).
        """
        current = start
        path = [current]
        for _ in range(max_steps):
            if math.hypot(current[0] - goal[0], current[1] - goal[1]) < step:
                path.append(goal)
                return path
            best = None
            best_cost = -1.0
            for dx in (-step, 0.0, step):
                for dy in (-step, 0.0, step):
                    if dx == 0.0 and dy == 0.0:
                        continue
                    cand = (current[0] + dx, current[1] + dy)
                    c = cost_at(cand[0], cand[1])
                    if c <= 0.0:
                        continue
                    if c > best_cost:
                        best_cost = c
                        best = cand
            if best is None:
                # no free neighbor; fall back toward goal
                best = (current[0] + math.copysign(step, goal[0] - current[0]),
                        current[1] + math.copysign(step, goal[1] - current[1]))
            path.append(best)
            current = best
        return path


# ---------------------------------------------------------------------------
# ROS 2 node wrappers (what the console entry points run)
# ---------------------------------------------------------------------------

from ._runtime import run as _run, spin_node as _spin_node  # noqa: E402


class _GridPlannerNode(Node):
    """Shared plumbing for grid planners: /map + /goal_pose -> nav_msgs/Path.

    These planners run alongside the Nav2 stack rather than as Nav2 plugins:
    they consume the same occupancy map and goal, and publish their own plan
    so it can be compared against the plugin planner's output.
    """

    def __init__(self, node_name, default_output):
        super().__init__(node_name)
        from nav_msgs.msg import OccupancyGrid, Path
        from geometry_msgs.msg import PoseStamped
        self._path_type = Path
        self._pose_type = PoseStamped
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('path_topic', default_output)
        self.declare_parameter('occupied_threshold', 50)
        self._pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, 10)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal, 10)
        from nav_msgs.msg import Odometry
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, 10)
        self._map = None
        self._start = (0.0, 0.0)
        self.path = []
        self.get_logger().info('%s ready (waiting for /map and a goal)' % node_name)

    def _on_map(self, msg):
        self._map = msg

    def _on_odom(self, msg):
        self._start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _world_is_free(self, x, y):
        grid = self._map
        if grid is None:
            return True
        column = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        if column < 0 or row < 0 or column >= grid.info.width \
                or row >= grid.info.height:
            return False
        value = grid.data[row * grid.info.width + column]
        threshold = int(self.get_parameter('occupied_threshold').value)
        return value >= 0 and value < threshold

    def _bounds(self):
        grid = self._map
        if grid is None:
            return (-10.0, -10.0, 10.0, 10.0)
        return (
            grid.info.origin.position.x,
            grid.info.origin.position.y,
            grid.info.origin.position.x + grid.info.width * grid.info.resolution,
            grid.info.origin.position.y + grid.info.height * grid.info.resolution,
        )

    def _publish_path(self, waypoints, frame_id):
        path = self._path_type()
        path.header.frame_id = frame_id or 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in waypoints:
            pose = self._pose_type()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path = list(waypoints)
        self._pub.publish(path)

    def _on_goal(self, msg):  # pragma: no cover - overridden
        raise NotImplementedError


class RRTPlannerNode(_GridPlannerNode):
    """Sampling-based RRT global planner publishing /plan/rrt."""

    def __init__(self, node_name='rrt_planner'):
        super().__init__(node_name, '/plan/rrt')
        self.declare_parameter('step', 1.0)
        self.declare_parameter('max_iterations', 2000)
        self.declare_parameter('goal_tolerance', 0.6)
        self.declare_parameter('seed', 0)
        self.planner = RRTPlanner(
            step=float(self.get_parameter('step').value),
            max_iter=int(self.get_parameter('max_iterations').value),
            goal_tol=float(self.get_parameter('goal_tolerance').value))

    def _on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        waypoints = self.planner.plan(
            self._start, goal, self._world_is_free, self._bounds(),
            seed=int(self.get_parameter('seed').value))
        if not waypoints:
            self.get_logger().warn('RRT found no path to (%.2f, %.2f)' % goal)
            return
        self._publish_path(waypoints, msg.header.frame_id)


class VoronoiPlannerNode(_GridPlannerNode):
    """Clearance-maximizing global planner publishing /plan/voronoi."""

    def __init__(self, node_name='voronoi_planner'):
        super().__init__(node_name, '/plan/voronoi')
        self.declare_parameter('step', 0.5)
        self.declare_parameter('max_steps', 2000)
        self.planner = VoronoiPlanner()

    def _clearance_at(self, x, y):
        """Distance-to-obstacle proxy: how far a free ring stays free."""
        if not self._world_is_free(x, y):
            return 0.0
        clearance = 0.0
        for radius in (0.2, 0.4, 0.6, 0.8):
            blocked = any(
                not self._world_is_free(x + radius * math.cos(a),
                                        y + radius * math.sin(a))
                for a in (0.0, math.pi / 2, math.pi, 3 * math.pi / 2))
            if blocked:
                break
            clearance = radius
        return clearance + 0.1

    def _on_goal(self, msg):
        goal = (msg.pose.position.x, msg.pose.position.y)
        waypoints = self.planner.plan(
            self._start, goal, self._clearance_at,
            step=float(self.get_parameter('step').value),
            max_steps=int(self.get_parameter('max_steps').value))
        self._publish_path(waypoints, msg.header.frame_id)


def rrt_planner_main(args=None):
    return _run(RRTPlannerNode, 'rrt_planner', args=args)


def voronoi_planner_main(args=None):
    return _run(VoronoiPlannerNode, 'voronoi_planner', args=args)


if __name__ == '__main__':
    sys.exit(rrt_planner_main())
