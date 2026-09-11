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


# ---------------------------------------------------------------------------
# ROS 2 node wrappers (what the console entry points run)
# ---------------------------------------------------------------------------

from ._runtime import run as _run, spin_node as _spin_node  # noqa: E402


class _OdometryEstimatorNode(Node):
    """Shared plumbing: consume /odom, publish a filtered Odometry estimate."""

    def __init__(self, node_name, default_output):
        super().__init__(node_name)
        from nav_msgs.msg import Odometry
        self._odometry_type = Odometry
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('output_topic', default_output)
        self._pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, 10)
        self._last_stamp = None
        self.get_logger().info('%s ready' % node_name)

    def _dt(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self._last_stamp is None else max(0.0, stamp - self._last_stamp)
        self._last_stamp = stamp
        return dt

    def _publish(self, msg, x, y, z=0.0):
        out = self._odometry_type()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = float(x)
        out.pose.pose.position.y = float(y)
        out.pose.pose.position.z = float(z)
        out.pose.pose.orientation = msg.pose.pose.orientation
        out.twist = msg.twist
        self._pub.publish(out)

    def _on_odom(self, msg):  # pragma: no cover - overridden
        raise NotImplementedError


class EKF3DEstimatorNode(_OdometryEstimatorNode):
    """Constant-velocity EKF over 3D position/velocity."""

    def __init__(self, node_name='ekf_3d_estimator'):
        super().__init__(node_name, '/odometry/ekf_3d')
        self.declare_parameter('process_noise', 0.1)
        self.declare_parameter('measurement_noise', 0.2)
        self.estimator = EKF3DEstimator(
            process_noise=float(self.get_parameter('process_noise').value),
            measurement_noise=float(self.get_parameter('measurement_noise').value))

    def _on_odom(self, msg):
        dt = self._dt(msg)
        if dt > 0.0:
            self.estimator.predict(dt)
        position = msg.pose.pose.position
        linear = msg.twist.twist.linear
        self.estimator.update([position.x, position.y, position.z,
                               linear.x, linear.y, linear.z])
        state = self.estimator.state()
        self._publish(msg, state[0], state[1], state[2])


class MotionModelEstimatorNode(_OdometryEstimatorNode):
    """Constant-velocity motion model corrected by odometry measurements."""

    def __init__(self, node_name='motion_model_estimator'):
        super().__init__(node_name, '/odometry/motion_model')
        self.declare_parameter('model_noise', 0.05)
        self.declare_parameter('correction_gain', 0.5)
        self.estimator = MotionModelEstimator(
            model_noise=float(self.get_parameter('model_noise').value))

    def _on_odom(self, msg):
        dt = self._dt(msg)
        self.estimator.vx = msg.twist.twist.linear.x
        self.estimator.vy = msg.twist.twist.linear.y
        if dt > 0.0:
            self.estimator.predict(dt)
        self.estimator.correct(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            gain=float(self.get_parameter('correction_gain').value))
        state = self.estimator.state()
        self._publish(msg, state[0], state[1])


class PoseGraphEstimatorNode(_OdometryEstimatorNode):
    """Incremental pose-graph estimator fed with relative odometry motion."""

    def __init__(self, node_name='pose_graph_estimator'):
        super().__init__(node_name, '/odometry/pose_graph')
        self.declare_parameter('decay', 0.9)
        self.estimator = PoseGraphEstimator(
            decay=float(self.get_parameter('decay').value))
        self._previous = None

    def _on_odom(self, msg):
        position = msg.pose.pose.position
        current = (position.x, position.y)
        if self._previous is not None:
            self.estimator.add_relative(current[0] - self._previous[0],
                                        current[1] - self._previous[1], 0.0)
        self._previous = current
        state = self.estimator.state()
        self._publish(msg, state[0], state[1])


def ekf_3d_estimator_main(args=None):
    return _run(EKF3DEstimatorNode, 'ekf_3d_estimator', args=args)


def motion_model_estimator_main(args=None):
    return _run(MotionModelEstimatorNode, 'motion_model_estimator', args=args)


def pose_graph_estimator_main(args=None):
    return _run(PoseGraphEstimatorNode, 'pose_graph_estimator', args=args)


if __name__ == '__main__':
    sys.exit(ekf_3d_estimator_main())
