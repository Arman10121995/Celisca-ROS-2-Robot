"""Sensor fusion nodes (R7.5).

The console scripts run the filters in robot_lab_algorithms.fusion_filters
(rewritten 2026-10-13; benchmark docs/status/evidence/r75-fusion-benchmark-2026-10-13):

* complementary_imu, mahony_filter, madgwick_filter, attitude_eskf -
  attitude from ``sensor_msgs/Imu`` gyro + accelerometer, republished as
  ``sensor_msgs/Imu`` with the estimated orientation on /imu/<method>.
* wheel_imu_fusion (EKF), gps_odom_fusion (EKF with GNSS) and
  wheel_imu_gnss_ukf (UKF with GNSS) - planar pose from the wheel odometry
  twist, the IMU gyro and optional ``sensor_msgs/NavSatFix``, published as
  ``nav_msgs/Odometry``.

The classes defined directly below (WheelImuFusion, GpsOdomFusion,
ComplementaryImu, MahonyFilter, MadgwickFilter, WheelIMUGNSSUKF) are the
earlier simplified versions, kept for their existing tests only: the
complementary filter swapped the roll/pitch gyro axes, the "Madgwick" filter
applied Mahony's error with a nonstandard step, and the "UKF" was dead
reckoning with a fixed GNSS blend.  No node uses them.
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

from ._runtime import run as _run  # noqa: E402
from . import fusion_filters as _ff  # noqa: E402


def _seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class AttitudeNode(Node):
    """Run one attitude filter on an IMU topic and republish the orientation."""

    GAINS = {  # parameter name and default (tuned on the R7.5 benchmark's tuning seed)
        'complementary': ('tau', 4.0),
        'mahony': ('kp', 0.25),
        'madgwick': ('beta', 0.02),
        # The benchmark picked 16 m/s^2 for a constant gyro bias; 2.0 keeps the
        # accelerometer in charge of tilt when the bias also drifts.
        'eskf': ('linear_accel_std', 2.0),
    }

    def __init__(self, method, node_name=None):
        super().__init__(node_name or method)
        from sensor_msgs.msg import Imu
        self._imu_type = Imu
        parameter, default = self.GAINS[method]
        self.declare_parameter('imu_topic', '/imu/out')
        self.declare_parameter('output_topic', '/imu/%s' % method)
        self.declare_parameter(parameter, default)
        self.filter = _ff.ATTITUDE_FILTERS[method](**{parameter: float(self.get_parameter(parameter).value)})
        self._pub = self.create_publisher(Imu, self.get_parameter('output_topic').value, 20)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self._on_imu, 50)
        self._last = None
        self.get_logger().info('%s attitude filter on %s -> %s' % (
            method, self.get_parameter('imu_topic').value, self.get_parameter('output_topic').value))

    def _on_imu(self, msg):
        stamp = _seconds(msg.header)
        dt = 0.0 if self._last is None else min(0.1, max(0.0, stamp - self._last))
        self._last = stamp
        g, a = msg.angular_velocity, msg.linear_acceleration
        q = self.filter.update((g.x, g.y, g.z), (a.x, a.y, a.z), dt)
        out = self._imu_type()
        out.header = msg.header
        out.orientation.w, out.orientation.x, out.orientation.y, out.orientation.z = (float(v) for v in q)
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub.publish(out)


class PoseFusionNode(Node):
    """Planar pose from wheel odometry twist + IMU gyro (+ GNSS) with an EKF or UKF."""

    def __init__(self, method, node_name, output_topic, use_gnss):
        super().__init__(node_name)
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, NavSatFix
        self._odometry_type = Odometry
        self.declare_parameter('odom_topic', '/robot_lab_controller/odom')
        self.declare_parameter('imu_topic', '/imu/out')
        self.declare_parameter('gnss_topic', '/gps/fix')
        self.declare_parameter('output_topic', output_topic)
        self.declare_parameter('use_gnss', use_gnss)
        self.declare_parameter('gnss_noise', 0.5)
        self.filter = (_ff.WheelImuGnssUkf if method == 'ukf' else _ff.WheelImuEkf)(
            gnss_noise=float(self.get_parameter('gnss_noise').value))
        self.use_gnss = bool(self.get_parameter('use_gnss').value)
        self._pub = self.create_publisher(Odometry, self.get_parameter('output_topic').value, 20)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 50)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self._on_imu, 50)
        if self.use_gnss:
            self.create_subscription(NavSatFix, self.get_parameter('gnss_topic').value, self._on_gnss, 10)
        self._origin = None
        self._last_imu = None
        self._child = 'base_footprint'
        self.get_logger().info('%s pose fusion -> %s (GNSS %s)' % (
            method, self.get_parameter('output_topic').value, 'on' if self.use_gnss else 'off'))

    def _on_imu(self, msg):
        stamp = _seconds(msg.header)
        if self._last_imu is not None and stamp > self._last_imu:
            self.filter.predict(msg.angular_velocity.z, min(0.1, stamp - self._last_imu))
        self._last_imu = stamp

    def _on_odom(self, msg):
        self._child = msg.child_frame_id or self._child
        self.filter.update_wheels(msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self._publish(msg.header)

    def _on_gnss(self, msg):
        # Local tangent plane around the first fix.
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
        metres = 111320.0
        x = (msg.longitude - self._origin[1]) * metres * math.cos(math.radians(self._origin[0]))
        y = (msg.latitude - self._origin[0]) * metres
        self.filter.update_gnss(x, y)

    def _publish(self, header):
        x, y, theta, v, _ = (float(s) for s in self.filter.state)
        out = self._odometry_type()
        out.header.stamp = header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = self._child
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(theta / 2), math.cos(theta / 2)
        covariance = [0.0] * 36
        P = self.filter.P
        covariance[0], covariance[1], covariance[6], covariance[7] = P[0, 0], P[0, 1], P[1, 0], P[1, 1]
        covariance[35] = P[2, 2]
        out.pose.covariance = [float(c) for c in covariance]
        out.twist.twist.linear.x = v
        out.twist.twist.angular.z = float(self.filter.last_gyro - self.filter.state[4])
        self._pub.publish(out)


def complementary_imu_main(args=None):
    return _run(lambda: AttitudeNode('complementary', 'complementary_imu'), 'complementary_imu', args=args)


def mahony_filter_main(args=None):
    return _run(lambda: AttitudeNode('mahony', 'mahony_filter'), 'mahony_filter', args=args)


def madgwick_filter_main(args=None):
    return _run(lambda: AttitudeNode('madgwick', 'madgwick_filter'), 'madgwick_filter', args=args)


def attitude_eskf_main(args=None):
    return _run(lambda: AttitudeNode('eskf', 'attitude_eskf'), 'attitude_eskf', args=args)


def wheel_imu_fusion_main(args=None):
    return _run(lambda: PoseFusionNode('ekf', 'wheel_imu_fusion', '/odometry/wheel_imu', False),
                'wheel_imu_fusion', args=args)


def gps_odom_fusion_main(args=None):
    return _run(lambda: PoseFusionNode('ekf', 'gps_odom_fusion', '/odometry/gps_odom', True),
                'gps_odom_fusion', args=args)


def wheel_imu_gnss_ukf_main(args=None):
    return _run(lambda: PoseFusionNode('ukf', 'wheel_imu_gnss_ukf', '/odometry/wheel_imu_gnss', True),
                'wheel_imu_gnss_ukf', args=args)


if __name__ == '__main__':
    sys.exit(wheel_imu_fusion_main())
