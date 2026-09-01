"""Local planning algorithms (P5).

Adds `follow_the_gap`, a reactive obstacle-aware local planner that computes a
2D steering command by choosing the angular-gap heading that maximizes free
space. Rounds out the local_planning category to five integrated
implementations.
"""

from __future__ import annotations

import math
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
    class FollowTheGap:
    """Select heading toward the widest gap in a laser scan."""

    def __init__(self, safety_radius=0.3, max_range=10.0):
        self.safety_radius = safety_radius
        self.max_range = max_range

    def steer(self, angle_min, angle_increment, ranges, goal_bearing=0.0):
        """Return a (linear, angular) steering command given a scan.

        goal_bearing: desired heading (rad) toward the goal. The planner picks
        the widest gap and steers toward its center, then biases toward goal.
        """
        if not ranges:
            return (0.0, 0.0)
        # Build an array of (angle, range) with obstacles closer than safety set to 0
        beams = []
        for i, r in enumerate(ranges):
            theta = angle_min + i * angle_increment
            if r < self.safety_radius:
                beams.append((theta, 0.0))
            else:
                beams.append((theta, min(r, self.max_range)))
        # Find the widest contiguous gap of free beams
        best_gap = None
        best_width = 0.0
        i = 0
        n = len(beams)
        while i < n:
            if beams[i][1] > self.safety_radius:
                j = i
                while j < n and beams[j][1] > self.safety_radius:
                    j += 1
                width = j - i
                if width > best_width:
                    best_width = width
                    mid = (beams[i][0] + beams[j - 1][0]) / 2.0
                    best_gap = mid
                i = j
            else:
                i += 1
        if best_gap is None:
            return (0.0, 0.0)
        # blend gap heading with goal bearing
        heading = 0.6 * best_gap + 0.4 * goal_bearing
        linear = 0.5
        angular = heading
        return (linear, angular)


def follow_the_gap_main(args=None):
    if rclpy is None:
        print('follow_the_gap: rclpy unavailable (dry mode)')
        return 1
    rclpy.init(args=args)
    node = Node('follow_the_gap')
    node.get_logger().info('follow_the_gap: up')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(follow_the_gap_main())
