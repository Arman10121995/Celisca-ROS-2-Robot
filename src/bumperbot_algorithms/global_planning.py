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


def rrt_planner_main(args=None):
    if rclpy is None:
        print('rrt_planner: rclpy unavailable (dry mode)')
        return 1
    rclpy.init(args=args)
    node = Node('rrt_planner')
    node.get_logger().info('rrt_planner: up')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


def voronoi_planner_main(args=None):
    if rclpy is None:
        return 1
    rclpy.init(args=args)
    node = Node('voronoi_planner')
    node.get_logger().info('voronoi_planner: up')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(rrt_planner_main())
