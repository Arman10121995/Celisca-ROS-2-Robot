"""Sensor fusion filters (R7.5): attitude from an IMU, pose from wheels + IMU + GNSS.

Conventions: quaternions are ``[w, x, y, z]`` and rotate body vectors into
the world (z up).  The gyro reads body angular velocity (rad/s); the
accelerometer reads specific force (m/s^2), so a resting IMU reads
``(0, 0, +g)``, as ROS ``sensor_msgs/Imu`` does.

Attitude (gyro + accelerometer; yaw is unobservable without a magnetometer
and drifts with the gyro bias in every method):

* :class:`ComplementaryAttitude` - gyro integration with roll and pitch
  pulled toward the accelerometer tilt, time constant ``tau``.
* :class:`MahonyAhrs` - Mahony, Hamel and Pflimlin (2008) explicit
  complementary filter: PI feedback of ``a_hat x v_hat`` into the gyro.
* :class:`MadgwickAhrs` - Madgwick (2010) gradient-descent filter, IMU form.
* :class:`AttitudeEskf` - error-state Kalman filter on attitude and gyro
  bias, with the accelerometer down-weighted when it is not reading gravity.

Planar pose (wheel odometry speed and yaw rate, gyro, optional GNSS):

* :class:`WheelImuEkf` - EKF on ``[x, y, theta, v, b_g]``: the gyro drives
  heading, wheel odometry measures speed and the bias-free yaw rate (tight
  at standstill, a zero angular rate update); GNSS position updates are
  optional.
* :class:`WheelImuGnssUkf` - unscented Kalman filter on the same model and
  measurements (Julier-Uhlmann / Wan-van der Merwe sigma points).
"""
from __future__ import annotations

import math

import numpy as np

GRAVITY = 9.80665


# ---------------------------------------------------------------------------
# Quaternion helpers
# ---------------------------------------------------------------------------

def q_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def q_multiply(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def q_exp(rotation_vector):
    """Quaternion of a rotation vector (axis * angle)."""
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1e-12:
        return q_normalize(np.concatenate([[1.0], 0.5 * np.asarray(rotation_vector, dtype=float)]))
    axis = np.asarray(rotation_vector, dtype=float) / angle
    return np.concatenate([[math.cos(angle / 2)], math.sin(angle / 2) * axis])


def q_matrix(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def q_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def q_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def nearest_pd(P, floor=1e-9):
    """Symmetric matrix with eigenvalues clipped at ``floor`` (keeps a UKF going
    when round-off makes its covariance indefinite)."""
    P = 0.5 * (P + P.T)
    values, vectors = np.linalg.eigh(P)
    return (vectors * np.maximum(values, floor)) @ vectors.T



# ---------------------------------------------------------------------------
# Attitude filters
# ---------------------------------------------------------------------------

class AttitudeFilter:
    name = 'attitude'

    def __init__(self, q0=None):
        self.q = q_normalize(q0 if q0 is not None else [1.0, 0.0, 0.0, 0.0])

    def update(self, gyro, accel, dt):  # pragma: no cover - abstract
        raise NotImplementedError

    def euler(self):
        return q_to_euler(self.q)


class ComplementaryAttitude(AttitudeFilter):
    """Gyro integration; roll/pitch blended toward the accelerometer tilt.

    alpha = tau / (tau + dt): the accelerometer corrects roll and pitch with
    time constant ``tau``; yaw is the integrated gyro.
    """

    name = 'complementary'

    def __init__(self, tau=0.5, q0=None):
        super().__init__(q0)
        self.tau = tau

    def update(self, gyro, accel, dt):
        if dt <= 0:
            return self.q
        self.q = q_normalize(q_multiply(self.q, q_exp(np.asarray(gyro, dtype=float) * dt)))
        a = np.asarray(accel, dtype=float)
        if np.linalg.norm(a) < 1e-6:
            return self.q
        roll_g, pitch_g, yaw = q_to_euler(self.q)
        roll_a = math.atan2(a[1], a[2])
        pitch_a = math.atan2(-a[0], math.hypot(a[1], a[2]))
        alpha = self.tau / (self.tau + dt)
        roll = roll_g + (1 - alpha) * wrap(roll_a - roll_g)
        pitch = pitch_g + (1 - alpha) * wrap(pitch_a - pitch_g)
        self.q = q_from_euler(roll, pitch, yaw)
        return self.q


class MahonyAhrs(AttitudeFilter):
    """Mahony explicit complementary filter (IMU only).

    v = R(q)' e_z (predicted gravity direction), e = a/|a| x v,
    I += e dt, w_c = w + kp e + ki I, q <- q * exp(w_c dt).
    """

    name = 'mahony'

    def __init__(self, kp=1.0, ki=0.05, q0=None):
        super().__init__(q0)
        self.kp, self.ki = kp, ki
        self.integral = np.zeros(3)

    def update(self, gyro, accel, dt):
        if dt <= 0:
            return self.q
        w = np.asarray(gyro, dtype=float)
        a = np.asarray(accel, dtype=float)
        norm = np.linalg.norm(a)
        if norm > 1e-6:
            v = q_matrix(self.q)[2]          # R' e_z: third row of R
            error = np.cross(a / norm, v)
            if self.ki > 0:
                self.integral += error * dt
            w = w + self.kp * error + self.ki * self.integral
        self.q = q_normalize(q_multiply(self.q, q_exp(w * dt)))
        return self.q


class MadgwickAhrs(AttitudeFilter):
    """Madgwick gradient-descent filter (IMU form).

    f(q) = R(q)' e_z - a/|a|; one normalised gradient step J'f/|J'f| per
    sample with gain beta: q_dot = 0.5 q * (0, w) - beta J'f/|J'f|.
    """

    name = 'madgwick'

    def __init__(self, beta=0.05, q0=None):
        super().__init__(q0)
        self.beta = beta

    def update(self, gyro, accel, dt):
        if dt <= 0:
            return self.q
        q0, q1, q2, q3 = self.q
        w = np.asarray(gyro, dtype=float)
        q_dot = 0.5 * q_multiply(self.q, np.concatenate([[0.0], w]))
        a = np.asarray(accel, dtype=float)
        norm = np.linalg.norm(a)
        if norm > 1e-6:
            ax, ay, az = a / norm
            f = np.array([2 * (q1 * q3 - q0 * q2) - ax,
                          2 * (q0 * q1 + q2 * q3) - ay,
                          2 * (0.5 - q1 * q1 - q2 * q2) - az])
            jacobian = np.array([[-2 * q2, 2 * q3, -2 * q0, 2 * q1],
                                 [2 * q1, 2 * q0, 2 * q3, 2 * q2],
                                 [0.0, -4 * q1, -4 * q2, 0.0]])
            gradient = jacobian.T @ f
            gnorm = np.linalg.norm(gradient)
            if gnorm > 1e-12:
                q_dot = q_dot - self.beta * gradient / gnorm
        self.q = q_normalize(self.q + q_dot * dt)
        return self.q


class AttitudeEskf(AttitudeFilter):
    """Error-state Kalman filter on attitude and gyro bias.

    Nominal state (q, b); error state (dtheta, db) with
    F = [[I - [w x] dt, -I dt], [0, I]].  The accelerometer direction
    z = a/|a| is predicted as h = R(q)' e_z with H = [[h x], 0].  Its noise
    covers sensor noise plus the expected linear acceleration of the
    platform (``linear_accel_std``), and grows further with the departure
    of |a| from g.
    """

    name = 'eskf'

    def __init__(self, gyro_noise=0.005, bias_walk=1e-4, accel_noise=0.05,
                 linear_accel_std=0.5, accel_disturbance=1.0, q0=None):
        super().__init__(q0)
        self.bias = np.zeros(3)
        self.P = np.diag([0.1] * 3 + [1e-3] * 3)
        self.gyro_noise, self.bias_walk = gyro_noise, bias_walk
        self.accel_noise, self.accel_disturbance = accel_noise, accel_disturbance
        self.linear_accel_std = linear_accel_std

    def update(self, gyro, accel, dt):
        if dt <= 0:
            return self.q
        w = np.asarray(gyro, dtype=float) - self.bias
        self.q = q_normalize(q_multiply(self.q, q_exp(w * dt)))
        F = np.eye(6)
        F[:3, :3] = np.eye(3) - skew(w) * dt
        F[:3, 3:] = -np.eye(3) * dt
        Q = np.diag([self.gyro_noise ** 2 * dt] * 3 + [self.bias_walk ** 2 * dt] * 3)
        self.P = F @ self.P @ F.T + Q
        a = np.asarray(accel, dtype=float)
        norm = np.linalg.norm(a)
        if norm > 1e-6:
            h = q_matrix(self.q)[2]
            H = np.zeros((3, 6))
            H[:, :3] = skew(h)
            sigma = ((self.accel_noise ** 2 + self.linear_accel_std ** 2) / GRAVITY ** 2
                     + (self.accel_disturbance * (norm - GRAVITY) / GRAVITY) ** 2)
            S = H @ self.P @ H.T + np.eye(3) * sigma
            K = self.P @ H.T @ np.linalg.inv(S)
            dx = K @ (a / norm - h)
            self.q = q_normalize(q_multiply(self.q, q_exp(dx[:3])))
            self.bias = self.bias + dx[3:]
            self.P = (np.eye(6) - K @ H) @ self.P
            self.P = 0.5 * (self.P + self.P.T)
        return self.q


ATTITUDE_FILTERS = {
    'complementary': ComplementaryAttitude,
    'mahony': MahonyAhrs,
    'madgwick': MadgwickAhrs,
    'eskf': AttitudeEskf,
}


# ---------------------------------------------------------------------------
# Planar pose fusion: wheel odometry + gyro (+ GNSS)
# ---------------------------------------------------------------------------

STANDSTILL_SPEED = 0.01      # m/s and rad/s from the wheels
STANDSTILL_RATE_NOISE = 0.002  # rad/s: a standing robot does not turn


def _rate_noise(fusion, speed, yaw_rate):
    """Wheel yaw-rate noise; tight at standstill (zero angular rate update).

    While moving, wheel yaw rate carries track-width and slip errors, so it
    only weakly informs the gyro bias.  With the wheels stopped the robot
    is not turning, and the gyro reading is the bias.
    """
    if abs(speed) < STANDSTILL_SPEED and abs(yaw_rate) < STANDSTILL_SPEED:
        return STANDSTILL_RATE_NOISE
    return fusion.wheel_rate_noise


def _unicycle(state, gyro_z, dt):
    x, y, theta, v, bias = state
    w = gyro_z - bias
    return np.array([x + v * math.cos(theta) * dt, y + v * math.sin(theta) * dt,
                     wrap(theta + w * dt), v, bias])


class WheelImuEkf:
    """EKF on [x, y, theta, v, b_g].

    Predict with the gyro (heading) and a constant-speed model; correct with
    the wheel speed v_w and the wheel yaw rate w_w = gyro - b_g (which makes
    the gyro bias observable), and optionally GNSS (x, y).
    """

    name = 'wheel_imu_ekf'

    def __init__(self, x0=(0.0, 0.0, 0.0), speed_walk=0.5, gyro_noise=0.01, bias_walk=1e-4,
                 wheel_speed_noise=0.02, wheel_rate_noise=0.2, gnss_noise=0.5):
        self.state = np.array([x0[0], x0[1], x0[2], 0.0, 0.0])
        self.P = np.diag([0.01, 0.01, 0.01, 0.1, 1e-3])
        self.speed_walk, self.gyro_noise, self.bias_walk = speed_walk, gyro_noise, bias_walk
        self.wheel_speed_noise, self.wheel_rate_noise, self.gnss_noise = wheel_speed_noise, wheel_rate_noise, gnss_noise
        self.last_gyro = 0.0

    def predict(self, gyro_z, dt):
        if dt <= 0:
            return
        self.last_gyro = gyro_z
        x, y, theta, v, bias = self.state
        F = np.eye(5)
        F[0, 2], F[0, 3] = -v * math.sin(theta) * dt, math.cos(theta) * dt
        F[1, 2], F[1, 3] = v * math.cos(theta) * dt, math.sin(theta) * dt
        F[2, 4] = -dt
        Q = np.diag([1e-6, 1e-6, (self.gyro_noise ** 2) * dt, (self.speed_walk ** 2) * dt, (self.bias_walk ** 2) * dt])
        self.state = _unicycle(self.state, gyro_z, dt)
        self.P = F @ self.P @ F.T + Q

    def _correct(self, z, h, H, R, angle_rows=()):
        y = np.asarray(z, dtype=float) - h
        for i in angle_rows:
            y[i] = wrap(y[i])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.state[2] = wrap(self.state[2])
        self.P = (np.eye(5) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

    def update_wheels(self, speed, yaw_rate):
        H = np.zeros((2, 5))
        H[0, 3] = 1.0
        H[1, 4] = -1.0  # w_w = gyro - b
        h = np.array([self.state[3], self.last_gyro - self.state[4]])
        self._correct([speed, yaw_rate], h, H, np.diag([self.wheel_speed_noise ** 2,
                                                        _rate_noise(self, speed, yaw_rate) ** 2]))

    def update_gnss(self, x, y):
        H = np.zeros((2, 5))
        H[0, 0] = H[1, 1] = 1.0
        self._correct([x, y], self.state[:2].copy(), H, np.eye(2) * self.gnss_noise ** 2)


class WheelImuGnssUkf:
    """Unscented Kalman filter on [x, y, theta, v, b_g] with the same inputs.

    Sigma points with (alpha, beta, kappa) = (0.3, 2, 0); the heading is
    averaged on the circle.  Wheel and GNSS updates use the unscented
    transform of their measurement functions.
    """

    name = 'wheel_imu_gnss_ukf'

    def __init__(self, x0=(0.0, 0.0, 0.0), speed_walk=0.5, gyro_noise=0.01, bias_walk=1e-4,
                 wheel_speed_noise=0.02, wheel_rate_noise=0.2, gnss_noise=0.5,
                 alpha=0.3, beta=2.0, kappa=0.0):
        self.n = 5
        self.state = np.array([x0[0], x0[1], x0[2], 0.0, 0.0])
        self.P = np.diag([0.01, 0.01, 0.01, 0.1, 1e-3])
        self.speed_walk, self.gyro_noise, self.bias_walk = speed_walk, gyro_noise, bias_walk
        self.wheel_speed_noise, self.wheel_rate_noise, self.gnss_noise = wheel_speed_noise, wheel_rate_noise, gnss_noise
        lam = alpha ** 2 * (self.n + kappa) - self.n
        self.c = self.n + lam
        self.wm = np.full(2 * self.n + 1, 1.0 / (2 * self.c))
        self.wc = self.wm.copy()
        self.wm[0] = lam / self.c
        self.wc[0] = lam / self.c + (1 - alpha ** 2 + beta)
        self.last_gyro = 0.0

    def _sigma(self):
        try:
            root = np.linalg.cholesky(self.c * self.P)
        except np.linalg.LinAlgError:
            self.P = nearest_pd(self.P)
            root = np.linalg.cholesky(self.c * (self.P + np.eye(self.n) * 1e-9))
        points = [self.state]
        for i in range(self.n):
            points.append(self.state + root[:, i])
            points.append(self.state - root[:, i])
        return np.array(points)

    def _mean(self, points, angle_index=2):
        mean = self.wm @ points
        if angle_index is not None:
            mean[angle_index] = math.atan2(self.wm @ np.sin(points[:, angle_index]),
                                           self.wm @ np.cos(points[:, angle_index]))
        return mean

    def predict(self, gyro_z, dt):
        if dt <= 0:
            return
        self.last_gyro = gyro_z
        points = np.array([_unicycle(p, gyro_z, dt) for p in self._sigma()])
        mean = self._mean(points)
        d = points - mean
        d[:, 2] = [wrap(a) for a in d[:, 2]]
        Q = np.diag([1e-6, 1e-6, (self.gyro_noise ** 2) * dt, (self.speed_walk ** 2) * dt, (self.bias_walk ** 2) * dt])
        self.state = mean
        self.P = (self.wc[:, None] * d).T @ d + Q

    def _update(self, z, measure, R):
        points = self._sigma()
        Z = np.array([measure(p) for p in points])
        z_mean = self.wm @ Z
        dz = Z - z_mean
        dx = points - self.state
        dx[:, 2] = [wrap(a) for a in dx[:, 2]]
        S = (self.wc[:, None] * dz).T @ dz + R
        C = (self.wc[:, None] * dx).T @ dz
        K = C @ np.linalg.inv(S)
        self.state = self.state + K @ (np.asarray(z, dtype=float) - z_mean)
        self.state[2] = wrap(self.state[2])
        self.P = self.P - K @ S @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def update_wheels(self, speed, yaw_rate):
        self._update([speed, yaw_rate], lambda p: np.array([p[3], self.last_gyro - p[4]]),
                     np.diag([self.wheel_speed_noise ** 2, _rate_noise(self, speed, yaw_rate) ** 2]))

    def update_gnss(self, x, y):
        self._update([x, y], lambda p: p[:2].copy(), np.eye(2) * self.gnss_noise ** 2)
