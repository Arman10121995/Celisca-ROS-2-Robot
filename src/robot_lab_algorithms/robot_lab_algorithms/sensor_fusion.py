"""Sensor fusion algorithms (P5).

Adds three fusion filters that round out the sensor_fusion category:
  1. wheel_imu_fusion  - complementary fusion of wheel odometry and IMU yaw.
  2. gps_odom_fusion   - simple weighted fusion of GPS and odometry position.
  3. complementary_imu - complementary filter for IMU attitude (pitch/roll).
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


class WheelImuFusion:
    """Complementary fusion of wheel odometry and IMU yaw rate."""

    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.yaw = 0.0
        self.vx = 0.0

    def fuse(self, odom_vx, imu_yaw_rate, dt):
        # integrate both, blend yaw
        odom_yaw = self.yaw + odom_vx * dt * 0.0  # no yaw from vx
        gyro_yaw = self.yaw + imu_yaw_rate * dt
        self.yaw = self.alpha * gyro_yaw + (1.0 - self.alpha) * odom_yaw
        self.vx = self.alpha * self.vx + (1.0 - self.alpha) * odom_vx
        return (self.vx, self.yaw)


class GpsOdomFusion:
    """Weighted fusion of GPS and odometry position estimates."""

    def __init__(self, gps_weight=0.7):
        self.gps_weight = gps_weight
        self.x = 0.0
        self.y = 0.0

    def fuse(self, odom_x, odom_y, gps_x, gps_y):
        self.x = self.gps_weight * gps_x + (1 - self.gps_weight) * odom_x
        self.y = self.gps_weight * gps_y + (1 - self.gps_weight) * odom_y
        return (self.x, self.y)


class ComplementaryImu:
    """Complementary filter for IMU pitch/roll from accelerometer + gyro."""

    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.pitch = 0.0
        self.roll = 0.0

    def update(self, ax, ay, az, gx, gy, dt):
        # accelerometer tilt estimate
        acc_pitch = math.atan2(ax, math.sqrt(ay * ay + az * az))
        acc_roll = math.atan2(ay, math.sqrt(ax * ax + az * az))
        self.pitch = self.alpha * (self.pitch + gx * dt) + (1 - self.alpha) * acc_pitch
        self.roll = self.alpha * (self.roll + gy * dt) + (1 - self.alpha) * acc_roll
        return (self.pitch, self.roll)


# ---------------------------------------------------------------------------
# ROS 2 node wrappers (what the console entry points run)
# ---------------------------------------------------------------------------

from ._runtime import run as _run, spin_node as _spin_node  # noqa: E402


class WheelImuFusionNode(Node):
    """Fuse wheel odometry speed with IMU yaw rate onto /odometry/wheel_imu."""

    def __init__(self, node_name='wheel_imu_fusion'):
        super().__init__(node_name)
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        self._odometry_type = Odometry
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('imu_topic', '/imu/out')
        self.declare_parameter('output_topic', '/odometry/wheel_imu')
        self.declare_parameter('alpha', 0.5)
        self.fusion = WheelImuFusion(alpha=float(self.get_parameter('alpha').value))
        self._pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, 10)
        self._yaw_rate = 0.0
        self._last_stamp = None
        self.get_logger().info('wheel_imu_fusion ready')

    def _on_imu(self, msg):
        self._yaw_rate = msg.angular_velocity.z

    def _on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self._last_stamp is None else max(0.0, stamp - self._last_stamp)
        self._last_stamp = stamp
        vx, yaw = self.fusion.fuse(msg.twist.twist.linear.x, self._yaw_rate, dt)
        out = self._odometry_type()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist.twist.linear.x = float(vx)
        out.twist.twist.angular.z = float(self._yaw_rate)
        self._pub.publish(out)
        self.yaw = yaw


class GpsOdomFusionNode(Node):
    """Weighted GPS/odometry position fusion onto /odometry/gps_odom."""

    def __init__(self, node_name='gps_odom_fusion'):
        super().__init__(node_name)
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import NavSatFix
        self._odometry_type = Odometry
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('output_topic', '/odometry/gps_odom')
        self.declare_parameter('gps_weight', 0.7)
        self.fusion = GpsOdomFusion(
            gps_weight=float(self.get_parameter('gps_weight').value))
        self._pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value, self._on_gps, 10)
        self._gps = None
        self._origin = None
        self.get_logger().info('gps_odom_fusion ready')

    def _on_gps(self, msg):
        # Local tangent-plane approximation around the first fix, which is
        # enough for a planar simulation and needs no projection library.
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
        metres_per_degree = 111320.0
        self._gps = (
            (msg.longitude - self._origin[1]) * metres_per_degree
            * math.cos(math.radians(self._origin[0])),
            (msg.latitude - self._origin[0]) * metres_per_degree,
        )

    def _on_odom(self, msg):
        position = msg.pose.pose.position
        # Without a fix the odometry estimate passes through unchanged.
        gps = self._gps if self._gps is not None else (position.x, position.y)
        x, y = self.fusion.fuse(position.x, position.y, gps[0], gps[1])
        out = self._odometry_type()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = float(x)
        out.pose.pose.position.y = float(y)
        out.pose.pose.position.z = position.z
        out.pose.pose.orientation = msg.pose.pose.orientation
        out.twist = msg.twist
        self._pub.publish(out)


class ComplementaryImuNode(Node):
    """Complementary accelerometer/gyro filter republished on /imu/complementary."""

    def __init__(self, node_name='complementary_imu'):
        super().__init__(node_name)
        from sensor_msgs.msg import Imu
        self._imu_type = Imu
        self.declare_parameter('imu_topic', '/imu/out')
        self.declare_parameter('output_topic', '/imu/complementary')
        self.declare_parameter('alpha', 0.98)
        self.filter = ComplementaryImu(
            alpha=float(self.get_parameter('alpha').value))
        self._pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, 10)
        self._last_stamp = None
        self.get_logger().info('complementary_imu ready')

    def _on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self._last_stamp is None else max(0.0, stamp - self._last_stamp)
        self._last_stamp = stamp
        pitch, roll = self.filter.update(
            msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.linear_acceleration.z, msg.angular_velocity.x,
            msg.angular_velocity.y, dt)
        out = self._imu_type()
        out.header = msg.header
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        half_roll, half_pitch = roll / 2.0, pitch / 2.0
        out.orientation.w = math.cos(half_roll) * math.cos(half_pitch)
        out.orientation.x = math.sin(half_roll) * math.cos(half_pitch)
        out.orientation.y = math.cos(half_roll) * math.sin(half_pitch)
        out.orientation.z = -math.sin(half_roll) * math.sin(half_pitch)
        self._pub.publish(out)


def wheel_imu_fusion_main(args=None):
    return _run(WheelImuFusionNode, 'wheel_imu_fusion', args=args)


def gps_odom_fusion_main(args=None):
    return _run(GpsOdomFusionNode, 'gps_odom_fusion', args=args)


def complementary_imu_main(args=None):
    return _run(ComplementaryImuNode, 'complementary_imu', args=args)


if __name__ == '__main__':
    sys.exit(wheel_imu_fusion_main())
