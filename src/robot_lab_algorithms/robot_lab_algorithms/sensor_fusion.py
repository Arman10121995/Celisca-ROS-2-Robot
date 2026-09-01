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


def wheel_imu_fusion_main(args=None):
    if rclpy is None:
        print('wheel_imu_fusion: rclpy unavailable (dry mode)')
        return 1
    rclpy.init(args=args)
    node = Node('wheel_imu_fusion')
    node.get_logger().info('wheel_imu_fusion: up')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


def gps_odom_fusion_main(args=None):
    if rclpy is None:
        return 1
    rclpy.init(args=args)
    node = Node('gps_odom_fusion')
    node.get_logger().info('gps_odom_fusion: up')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


def complementary_imu_main(args=None):
    if rclpy is None:
        return 1
    rclpy.init(args=args)
    node = Node('complementary_imu')
    node.get_logger().info('complementary_imu: up')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(wheel_imu_fusion_main())
