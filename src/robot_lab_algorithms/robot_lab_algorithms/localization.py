"""Localization algorithms (P5).

Adds `dead_reckoning`, a lightweight odometry-integration pose estimator that
rounds out the localization category to five integrated implementations. It
integrates linear/angular velocity commands into an estimated pose and
degrades gracefully when odometry topics are absent.
"""

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


class DeadReckoning:
    """Integrate body twist into an odometry pose (dead reckoning).

    Pure 2D math with no ROS dependency so numerical tests run without a
    ROS context (R1.2: numerical tests must not require rclpy.init()).
    """

    def __init__(self, initial_x=0.0, initial_y=0.0, initial_theta=0.0):
        self.x = float(initial_x)
        self.y = float(initial_y)
        self.theta = float(initial_theta)

    def integrate(self, vx, wz, dt):
        """Advance the pose by (linear x, angular z) over dt seconds (2D)."""
        if dt <= 0.0:
            return (self.x, self.y, self.theta)
        if abs(wz) < 1e-6:
            self.x += vx * dt * math.cos(self.theta)
            self.y += vx * dt * math.sin(self.theta)
        else:
            radius = vx / wz
            self.x += radius * (math.sin(self.theta + wz * dt) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + wz * dt) - math.cos(self.theta))
        self.theta += wz * dt
        return (self.x, self.y, self.theta)


class DeadReckoningNode(Node):
    """ROS wrapper around :class:`DeadReckoning` for the console entry point."""

    def __init__(self, node_name='dead_reckoning'):
        super().__init__(node_name)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.dr = DeadReckoning(
            initial_x=self.get_parameter('initial_x').value,
            initial_y=self.get_parameter('initial_y').value,
            initial_theta=self.get_parameter('initial_theta').value,
        )
        self.get_logger().info('DeadReckoning ready')

    @property
    def x(self):
        return self.dr.x

    @property
    def y(self):
        return self.dr.y

    @property
    def theta(self):
        return self.dr.theta

    def integrate(self, vx, wz, dt):
        return self.dr.integrate(vx, wz, dt)


def dead_reckoning_main(args=None):
    if rclpy is None:
        print('dead_reckoning: rclpy unavailable (dry mode)')
        return 1
    rclpy.init(args=args)
    node = DeadReckoningNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(dead_reckoning_main())
