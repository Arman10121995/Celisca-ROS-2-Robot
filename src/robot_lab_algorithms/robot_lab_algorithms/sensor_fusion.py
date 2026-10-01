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


class MahonyFilter:
    """Mahony AHRS filter for IMU attitude estimation."""

    def __init__(self, sample_rate=100.0, kp=1.0, ki=0.0):
        self.sample_rate = sample_rate
        self.kp = kp
        self.ki = ki
        self.q = [1.0, 0.0, 0.0, 0.0]  # quaternion [w, x, y, z]
        self.e_int = [0.0, 0.0, 0.0]  # integral error

    def update(self, ax, ay, az, gx, gy, gz, dt):
        """Update filter with IMU measurements.
        
        Returns estimated quaternion [w, x, y, z].
        """
        if dt <= 0:
            return self.q
        
        # Normalize accelerometer measurement
        if math.sqrt(ax*ax + ay*ay + az*az) == 0:
            return self.q
        
        # Compute feedback terms
        halfvx = self.q[1] * self.q[3] - self.q[0] * self.q[2]
        halfvy = self.q[0] * self.q[1] + self.q[2] * self.q[3]
        halfvz = self.q[0] * self.q[0] - 0.5 + self.q[3] * self.q[3]
        
        # Error is cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy) + self.e_int[0]
        halfey = (az * halfvx - ax * halfvz) + self.e_int[1]
        halfez = (ax * halfvy - ay * halfvx) + self.e_int[2]
        
        # Apply feedback terms
        if self.ki > 0:
            self.e_int[0] += halfex * dt
            self.e_int[1] += halfey * dt
            self.e_int[2] += halfez * dt
        else:
            self.e_int = [0.0, 0.0, 0.0]
        
        # Apply feedback to gyro
        gx += self.kp * halfex + self.ki * self.e_int[0]
        gy += self.kp * halfey + self.ki * self.e_int[1]
        gz += self.kp * halfez + self.ki * self.e_int[2]
        
        # Integrate rate of change of quaternion
        qa = self.q[0]
        qb = self.q[1]
        qc = self.q[2]
        qd = self.q[3]
        
        qdot1 = 0.5 * (-qb * gx - qc * gy - qd * gz)
        qdot2 = 0.5 * (qa * gx + qc * gz - qd * gy)
        qdot3 = 0.5 * (qa * gy - qb * gz + qd * gx)
        qdot4 = 0.5 * (qa * gz + qb * gy - qc * gx)
        
        # Update quaternion
        self.q[0] += qdot1 * dt
        self.q[1] += qdot2 * dt
        self.q[2] += qdot3 * dt
        self.q[3] += qdot4 * dt
        
        # Normalize quaternion
        norm = math.sqrt(self.q[0]*self.q[0] + self.q[1]*self.q[1] + self.q[2]*self.q[2] + self.q[3]*self.q[3])
        if norm > 0:
            self.q = [self.q[i] / norm for i in range(4)]
        
        return self.q

    def get_euler_angles(self):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        w, x, y, z = self.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)


class MadgwickFilter:
    """Madgwick AHRS filter for IMU attitude estimation."""

    def __init__(self, sample_rate=100.0, beta=0.1):
        self.sample_rate = sample_rate
        self.beta = beta  # Filter gain
        self.q = [1.0, 0.0, 0.0, 0.0]  # quaternion [w, x, y, z]

    def update(self, ax, ay, az, gx, gy, gz, dt):
        """Update filter with IMU measurements.
        
        Returns estimated quaternion [w, x, y, z].
        """
        if dt <= 0:
            return self.q
        
        # Normalize accelerometer
        if ax == 0 and ay == 0 and az == 0:
            return self.q
        
        # Normalize accelerometer vector
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        ax /= norm
        ay /= norm
        az /= norm
        
        # Estimated direction of gravity
        halfvx = self.q[1] * self.q[3] - self.q[0] * self.q[2]
        halfvy = self.q[0] * self.q[1] + self.q[2] * self.q[3]
        halfvz = self.q[0] * self.q[0] - 0.5 + self.q[3] * self.q[3]
        
        # Error is cross product between estimated and measured gravity
        halfex = (ay * halfvz - az * halfvy)
        halfey = (az * halfvx - ax * halfvz)
        halfez = (ax * halfvy - ay * halfvx)
        
        # Compute and apply step to the quaternion
        if halfex != 0 or halfey != 0 or halfez != 0:
            # Normalize error vector
            norm = math.sqrt(halfex*halfex + halfey*halfey + halfez*halfez)
            halfex /= norm
            halfey /= norm
            halfez /= norm
            
            # Compute rate of change of quaternion
            qa = self.q[0]
            qb = self.q[1]
            qc = self.q[2]
            qd = self.q[3]
            
            # Gradient descent step
            step = self.beta * dt
            
            qdot1 = -step * (qb * halfex + qc * halfey + qd * halfez)
            qdot2 = step * (qa * halfex - qd * halfey + qc * halfez)
            qdot3 = step * (qa * halfey + qd * halfex - qb * halfez)
            qdot4 = step * (qa * halfez - qc * halfex + qb * halfey)
            
            # Update quaternion
            self.q[0] += qdot1
            self.q[1] += qdot2
            self.q[2] += qdot3
            self.q[3] += qdot4
            
            # Normalize quaternion
            norm = math.sqrt(self.q[0]*self.q[0] + self.q[1]*self.q[1] + self.q[2]*self.q[2] + self.q[3]*self.q[3])
            if norm > 0:
                self.q = [self.q[i] / norm for i in range(4)]
        
        # Integrate gyroscope (angular rate) using quaternion kinematics
        qa = self.q[0]
        qb = self.q[1]
        qc = self.q[2]
        qd = self.q[3]
        
        qdot1 = 0.5 * (-qb * gx - qc * gy - qd * gz)
        qdot2 = 0.5 * (qa * gx + qc * gz - qd * gy)
        qdot3 = 0.5 * (qa * gy - qb * gz + qd * gx)
        qdot4 = 0.5 * (qa * gz + qb * gy - qc * gx)
        
        # Update quaternion with gyro integration
        self.q[0] += qdot1 * dt
        self.q[1] += qdot2 * dt
        self.q[2] += qdot3 * dt
        self.q[3] += qdot4 * dt
        
        # Normalize quaternion again
        norm = math.sqrt(self.q[0]*self.q[0] + self.q[1]*self.q[1] + self.q[2]*self.q[2] + self.q[3]*self.q[3])
        if norm > 0:
            self.q = [self.q[i] / norm for i in range(4)]
        
        return self.q

    def get_euler_angles(self):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        w, x, y, z = self.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)


class WheelIMUGNSSUKF:
    """UKF for wheel odometry + IMU + GNSS sensor fusion."""

    def __init__(self, initial_state=None, initial_covariance=None):
        # State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = initial_state or [0.0] * 9
        
        # Covariance matrix (9x9)
        if initial_covariance is None:
            self.covariance = [[1.0 if i == j else 0.0 for j in range(9)] for i in range(9)]
        else:
            self.covariance = initial_covariance
        
        self.n = 9  # State dimension
        self.lambda_ = 1.0  # UKF scaling parameter
        self.gamma = math.sqrt(self.n + self.lambda_)

    def update_wheel_imu(self, wheel_vx, wheel_vy, imu_gx, imu_gy, imu_gz, dt):
        """Update using wheel odometry and IMU data."""
        # Simplified UKF update for wheel+IMU fusion
        # In a real implementation, this would be more sophisticated
        
        # Predict step with wheel odometry
        x, y = self.state[0], self.state[1]
        new_x = x + wheel_vx * dt
        new_y = y + wheel_vy * dt
        
        # Update orientation with IMU
        roll, pitch, yaw = self.state[6], self.state[7], self.state[8]
        new_roll = roll + imu_gx * dt
        new_pitch = pitch + imu_gy * dt
        new_yaw = yaw + imu_gz * dt
        
        self.state = [new_x, new_y, self.state[2], wheel_vx, wheel_vy, self.state[5], 
                     new_roll, new_pitch, new_yaw]

    def update_gnss(self, gnss_x, gnss_y, gnss_z):
        """Update using GNSS position measurements."""
        # Simplified update - blend GNSS with current estimate
        alpha = 0.7  # GNSS weight
        
        self.state[0] = alpha * gnss_x + (1 - alpha) * self.state[0]
        self.state[1] = alpha * gnss_y + (1 - alpha) * self.state[1]
        self.state[2] = alpha * gnss_z + (1 - alpha) * self.state[2]

    def get_state(self):
        return tuple(self.state)


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


class MahonyFilterNode(Node):
    """ROS wrapper for Mahony AHRS filter."""

    def __init__(self, node_name='mahony_filter'):
        super().__init__(node_name)
        self.declare_parameter('imu_topic', '/imu/out')
        self.declare_parameter('output_topic', '/imu/mahony')
        self.declare_parameter('sample_rate', 100.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        
        self.filter = MahonyFilter(
            sample_rate=float(self.get_parameter('sample_rate').value),
            kp=float(self.get_parameter('kp').value),
            ki=float(self.get_parameter('ki').value))
        
        from sensor_msgs.msg import Imu
        self._imu_type = Imu
        self._pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, 10)
        self._last_stamp = None
        self.get_logger().info('mahony_filter ready')

    def _on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self._last_stamp is None else max(0.0, stamp - self._last_stamp)
        self._last_stamp = stamp
        
        # Update filter
        self.filter.update(
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, dt)
        
        # Publish filtered IMU
        self._publish_imu(msg, self.filter.q)

    def _publish_imu(self, msg, quaternion):
        out = self._imu_type()
        out.header = msg.header
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.orientation.w = quaternion[0]
        out.orientation.x = quaternion[1]
        out.orientation.y = quaternion[2]
        out.orientation.z = quaternion[3]
        self._pub.publish(out)


class MadgwickFilterNode(Node):
    """ROS wrapper for Madgwick AHRS filter."""

    def __init__(self, node_name='madgwick_filter'):
        super().__init__(node_name)
        self.declare_parameter('imu_topic', '/imu/out')
        self.declare_parameter('output_topic', '/imu/madgwick')
        self.declare_parameter('sample_rate', 100.0)
        self.declare_parameter('beta', 0.1)
        
        self.filter = MadgwickFilter(
            sample_rate=float(self.get_parameter('sample_rate').value),
            beta=float(self.get_parameter('beta').value))
        
        from sensor_msgs.msg import Imu
        self._imu_type = Imu
        self._pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, 10)
        self._last_stamp = None
        self.get_logger().info('madgwick_filter ready')

    def _on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self._last_stamp is None else max(0.0, stamp - self._last_stamp)
        self._last_stamp = stamp
        
        # Update filter
        self.filter.update(
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, dt)
        
        # Publish filtered IMU
        self._publish_imu(msg, self.filter.q)

    def _publish_imu(self, msg, quaternion):
        out = self._imu_type()
        out.header = msg.header
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.orientation.w = quaternion[0]
        out.orientation.x = quaternion[1]
        out.orientation.y = quaternion[2]
        out.orientation.z = quaternion[3]
        self._pub.publish(out)


class WheelIMUGNSSUKFNode(Node):
    """ROS wrapper for Wheel+IMU+GNSS UKF fusion."""

    def __init__(self, node_name='wheel_imu_gnss_ukf'):
        super().__init__(node_name)
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('imu_topic', '/imu/out')
        self.declare_parameter('gnss_topic', '/gps/fix')
        self.declare_parameter('output_topic', '/odometry/wheel_imu_gnss')
        
        self.fusion = WheelIMUGNSSUKF()
        
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, NavSatFix
        self._odometry_type = Odometry
        self._pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, 10)
        self.create_subscription(
            NavSatFix, self.get_parameter('gnss_topic').value, self._on_gnss, 10)
        
        self._last_odom = None
        self._last_imu = None
        self._last_gnss = None
        self._last_stamp = None
        self.get_logger().info('wheel_imu_gnss_ukf ready')

    def _on_odom(self, msg):
        self._last_odom = msg
        self._process()

    def _on_imu(self, msg):
        self._last_imu = msg
        self._process()

    def _on_gnss(self, msg):
        self._last_gnss = msg
        self._process()

    def _process(self):
        if self._last_odom is None:
            return
        
        stamp = self._last_odom.header.stamp.sec + self._last_odom.header.stamp.nanosec * 1e-9
        if self._last_stamp is not None:
            dt = stamp - self._last_stamp
            if dt > 0 and self._last_imu is not None:
                # Update with wheel odometry and IMU
                wheel_vx = self._last_odom.twist.twist.linear.x
                wheel_vy = self._last_odom.twist.twist.linear.y
                imu_gx = self._last_imu.angular_velocity.x
                imu_gy = self._last_imu.angular_velocity.y
                imu_gz = self._last_imu.angular_velocity.z
                self.fusion.update_wheel_imu(wheel_vx, wheel_vy, imu_gx, imu_gy, imu_gz, dt)
        
        # Update with GNSS if available
        if self._last_gnss is not None:
            # Simple conversion from lat/lon to local coordinates
            gnss_x = self._last_gnss.longitude * 111320.0  # Approximate meters per degree
            gnss_y = self._last_gnss.latitude * 111320.0
            gnss_z = 0.0  # Assuming flat ground
            self.fusion.update_gnss(gnss_x, gnss_y, gnss_z)
        
        self._last_stamp = stamp
        
        # Publish fused state
        self._publish_odometry(self._last_odom.header)

    def _publish_odometry(self, header):
        state = self.fusion.get_state()
        out = self._odometry_type()
        out.header = header
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = float(state[0])
        out.pose.pose.position.y = float(state[1])
        out.pose.pose.position.z = float(state[2])
        
        # Convert Euler angles to quaternion
        roll, pitch, yaw = state[6], state[7], state[8]
        half_roll, half_pitch, half_yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
        
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cr = math.cos(half_roll)
        sr = math.sin(half_roll)
        
        out.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        out.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        out.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        out.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        out.twist.twist.linear.x = float(state[3])
        out.twist.twist.linear.y = float(state[4])
        out.twist.twist.linear.z = float(state[5])
        
        self._pub.publish(out)


def mahony_filter_main(args=None):
    return _run(MahonyFilterNode, 'mahony_filter', args=args)


def madgwick_filter_main(args=None):
    return _run(MadgwickFilterNode, 'madgwick_filter', args=args)


def wheel_imu_gnss_ukf_main(args=None):
    return _run(WheelIMUGNSSUKFNode, 'wheel_imu_gnss_ukf', args=args)


if __name__ == '__main__':
    sys.exit(wheel_imu_fusion_main())
