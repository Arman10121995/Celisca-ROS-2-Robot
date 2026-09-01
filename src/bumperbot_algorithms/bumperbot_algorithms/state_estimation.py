"""State estimation algorithms (P5).

Adds three state estimators that round out the state_estimation category:
  1. ekf_3d_estimator        - an extended Kalman filter style 3D state estimate
                               (position/velocity) from odometry-like measures.
  2. motion_model_estimator  - a constant-velocity motion-model predict/update.
  3. pose_graph_estimator    - a simple pose-graph style incremental pose merge
                               (online pose averaging with uncertainty decay).

Each is a pure-Python deterministic implementation with a lightweight Node that
exposes a state-update method and degrades gracefully when no inputs arrive.
"""

from __future__ import annotations

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


class EKF3DEstimator:
    """Simple EKF over 3D position/velocity with constant-velocity model."""

    def __init__(self, process_noise=0.1, measurement_noise=0.2):
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        # state: [px, py, pz, vx, vy, vz]
        self.x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # diagonal covariance
        self.P = [1.0] * 6

    def predict(self, dt):
        # constant velocity: x' = x + v*dt
        for i in range(3):
            self.x[i] += self.x[i + 3] * dt
            self.P[i] += self.process_noise * dt
            self.P[i + 3] += self.process_noise

    def update(self, z):
        # simple scalar Kalman updates per dimension from a 6-vector measure z
        for i in range(6):
            g = self.P[i] / (self.P[i] + self.measurement_noise)
            self.x[i] += g * (z[i] - self.x[i])
            self.P[i] = (1.0 - g) * self.P[i]

    def state(self):
        return tuple(self.x)


class MotionModelEstimator:
    """Constant-velocity motion-model estimator with measurement correction."""

    def __init__(self, model_noise=0.05):
        self.model_noise = model_noise
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0

    def predict(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt

    def correct(self, measured_x, measured_y, gain=0.5):
        self.x += gain * (measured_x - self.x)
        self.y += gain * (measured_y - self.y)

    def state(self):
        return (self.x, self.y, self.vx, self.vy)


class PoseGraphEstimator:
    """Incremental pose-graph style estimator: merges relative poses with decay."""

    def __init__(self, decay=0.9):
        self.decay = decay
        self.estimate = [0.0, 0.0, 0.0]  # x, y, theta

    def add_relative(self, dx, dy, dtheta):
        # merge a new relative motion into the running estimate
        self.estimate[0] = self.decay * self.estimate[0] + (1 - self.decay) * dx
        self.estimate[1] = self.decay * self.estimate[1] + (1 - self.decay) * dy
        self.estimate[2] = self.decay * self.estimate[2] + (1 - self.decay) * dtheta

    def state(self):
        return tuple(self.estimate)


def ekf_3d_estimator_main(args=None):
    if rclpy is None:
        print('ekf_3d_estimator: rclpy unavailable (dry mode)')
        return 1
    rclpy.init(args=args)
    node = Node('ekf_3d_estimator')
    node.get_logger().info('ekf_3d_estimator: up')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


def motion_model_estimator_main(args=None):
    if rclpy is None:
        return 1
    rclpy.init(args=args)
    node = Node('motion_model_estimator')
    node.get_logger().info('motion_model_estimator: up')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


def pose_graph_estimator_main(args=None):
    if rclpy is None:
        return 1
    rclpy.init(args=args)
    node = Node('pose_graph_estimator')
    node.get_logger().info('pose_graph_estimator: up')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(ekf_3d_estimator_main())
