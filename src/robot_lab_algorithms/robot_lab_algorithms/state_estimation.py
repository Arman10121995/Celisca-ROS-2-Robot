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


class LinearKalmanFilter:
    """Linear Kalman Filter for state estimation.
    
    Estimates state (position, velocity) using linear dynamics and measurement models.
    """

    def __init__(self, initial_state=None, initial_covariance=None, 
                 process_noise=0.1, measurement_noise=0.2):
        # State vector: [x, y, vx, vy]
        self.state = initial_state or [0.0, 0.0, 0.0, 0.0]
        
        # Covariance matrix (4x4)
        if initial_covariance is None:
            self.covariance = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        else:
            self.covariance = initial_covariance
        
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        
        # Process noise covariance
        self.Q = [[process_noise if i == j else 0.0 for j in range(4)] for i in range(4)]
        
        # Measurement noise covariance
        self.R = [[measurement_noise if i == j else 0.0 for j in range(2)] for i in range(2)]

    def predict(self, dt):
        """Predict state forward using constant velocity model."""
        x, y, vx, vy = self.state
        
        # State transition matrix F
        F = [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]
        
        # Predicted state: x' = F * x
        new_x = F[0][0] * x + F[0][2] * vx
        new_y = F[1][1] * y + F[1][3] * vy
        new_vx = vx
        new_vy = vy
        
        # Predicted covariance: P' = F * P * F^T + Q
        new_cov = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    new_cov[i][j] += F[i][k] * self.covariance[k][j]
                new_cov[i][j] += self.Q[i][j]
        
        self.state = [new_x, new_y, new_vx, new_vy]
        self.covariance = new_cov

    def update(self, measurement_x, measurement_y):
        """Update state with position measurement."""
        x, y, vx, vy = self.state
        
        # Measurement matrix H
        H = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ]
        
        # Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
        # First compute H * P * H^T
        HPHt = [[0.0 for _ in range(2)] for _ in range(2)]
        for i in range(2):
            for j in range(2):
                for k in range(4):
                    for l in range(4):
                        HPHt[i][j] += H[i][k] * self.covariance[k][l] * H[j][l]
        
        # Add R
        S = [[HPHt[i][j] + self.R[i][j] for j in range(2)] for i in range(2)]
        
        # Invert S (2x2)
        det_S = S[0][0] * S[1][1] - S[0][1] * S[1][0]
        if abs(det_S) > 1e-10:
            S_inv = [
                [S[1][1] / det_S, -S[0][1] / det_S],
                [-S[1][0] / det_S, S[0][0] / det_S]
            ]
        else:
            S_inv = [[1.0, 0.0], [0.0, 1.0]]
        
        # Compute P * H^T
        PHt = [[0.0 for _ in range(2)] for _ in range(4)]
        for i in range(4):
            for j in range(2):
                for k in range(4):
                    PHt[i][j] += self.covariance[i][k] * H[j][k]
        
        # Compute K = PHt * S_inv
        K = [[0.0 for _ in range(2)] for _ in range(4)]
        for i in range(4):
            for j in range(2):
                for k in range(2):
                    K[i][j] += PHt[i][k] * S_inv[k][j]
        
        # Measurement residual
        z = [measurement_x, measurement_y]
        residual = [z[i] - self.state[i] for i in range(2)]
        
        # Update state: x = x + K * residual
        state_update = [0.0 for _ in range(4)]
        for i in range(4):
            for j in range(2):
                state_update[i] += K[i][j] * residual[j]
        
        self.state = [self.state[i] + state_update[i] for i in range(4)]
        
        # Update covariance: P = (I - K * H) * P
        I = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        KH = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(2):
                    KH[i][j] += K[i][k] * H[k][j]
        
        new_cov = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    new_cov[i][j] += (I[i][k] - KH[i][k]) * self.covariance[k][j]
        
        self.covariance = new_cov

    def get_state(self):
        return tuple(self.state)


class UnscentedKalmanFilter:
    """Unscented Kalman Filter (UKF) for nonlinear state estimation."""

    def __init__(self, initial_state=None, initial_covariance=None,
                 alpha=1.0, beta=2.0, kappa=0.0):
        self.state = initial_state or [0.0, 0.0, 0.0, 0.0]  # [x, y, vx, vy]
        
        if initial_covariance is None:
            self.covariance = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        else:
            self.covariance = initial_covariance
        
        self.n = 4  # State dimension
        self.lambda_ = alpha**2 * (self.n + kappa) - self.n
        self.gamma = math.sqrt(self.n + self.lambda_)
        
        # UKF parameters
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        
        # Process noise
        self.Q = [[0.1 if i == j else 0.0 for j in range(4)] for i in range(4)]
        
        # Measurement noise
        self.R = [[0.2 if i == j else 0.0 for j in range(2)] for i in range(2)]

    def predict(self, dt):
        """UKF prediction step."""
        # Generate sigma points
        sigma_points = self._generate_sigma_points()
        
        # Predict sigma points through motion model
        predicted_sigmas = []
        for point in sigma_points:
            x, y, vx, vy = point
            # Constant velocity model
            new_x = x + vx * dt
            new_y = y + vy * dt
            predicted_sigmas.append([new_x, new_y, vx, vy])
        
        # Calculate predicted mean and covariance
        weights = self._get_ukf_weights()
        self.state, self.covariance = self._calculate_statistics(predicted_sigmas, weights)
        
        # Add process noise
        for i in range(4):
            self.covariance[i][i] += self.Q[i][i]

    def update(self, measurement_x, measurement_y):
        """UKF update step."""
        # Generate sigma points from current state
        sigma_points = self._generate_sigma_points()
        
        # Predict measurements for each sigma point
        predicted_measurements = []
        for point in sigma_points:
            # Measurement model: h(x) = [x, y]
            predicted_measurements.append([point[0], point[1]])
        
        # Calculate predicted measurement mean
        weights = self._get_ukf_weights()
        z_pred = [0.0, 0.0]
        for i in range(2):
            for j in range(len(predicted_measurements)):
                z_pred[i] += weights[0][j] * predicted_measurements[j][i]
        
        # Calculate measurement covariance
        P_zz = [[0.0, 0.0], [0.0, 0.0]]
        for j in range(len(predicted_measurements)):
            dz = [predicted_measurements[j][i] - z_pred[i] for i in range(2)]
            weight = weights[1][j]
            for i in range(2):
                for k in range(2):
                    P_zz[i][k] += weight * dz[i] * dz[k]
        
        # Add measurement noise
        for i in range(2):
            P_zz[i][i] += self.R[i][i]
        
        # Calculate cross-correlation
        P_xz = [[0.0 for _ in range(2)] for _ in range(4)]
        for j in range(len(sigma_points)):
            dx = [sigma_points[j][i] - self.state[i] for i in range(4)]
            dz = [predicted_measurements[j][i] - z_pred[i] for i in range(2)]
            weight = weights[1][j]
            for i in range(4):
                for k in range(2):
                    P_xz[i][k] += weight * dx[i] * dz[k]
        
        # Kalman gain
        P_zz_inv = self._matrix_inverse_2x2(P_zz)
        K = [[0.0 for _ in range(2)] for _ in range(4)]
        for i in range(4):
            for j in range(2):
                for k in range(2):
                    K[i][j] += P_xz[i][k] * P_zz_inv[k][j]
        
        # Update state
        z = [measurement_x, measurement_y]
        dz = [z[i] - z_pred[i] for i in range(2)]
        state_update = [0.0 for _ in range(4)]
        for i in range(4):
            for j in range(2):
                state_update[i] += K[i][j] * dz[j]
        
        self.state = [self.state[i] + state_update[i] for i in range(4)]
        
        # Update covariance
        for i in range(4):
            for j in range(4):
                for k in range(2):
                    self.covariance[i][j] -= K[i][k] * P_zz[k][j] * K[j][k]

    def _generate_sigma_points(self):
        """Generate UKF sigma points."""
        # Cholesky decomposition of covariance
        L = self._cholesky_decomposition()
        
        sigma_points = [self.state[:]]  # Center point
        
        for i in range(self.n):
            # Positive direction
            sigma_pos = self.state[:]
            for j in range(self.n):
                sigma_pos[j] += self.gamma * L[j][i]
            sigma_points.append(sigma_pos)
            
            # Negative direction
            sigma_neg = self.state[:]
            for j in range(self.n):
                sigma_neg[j] -= self.gamma * L[j][i]
            sigma_points.append(sigma_neg)
        
        return sigma_points

    def _get_ukf_weights(self):
        """Get UKF weights for mean and covariance calculation."""
        Wm = [0.0] * (2 * self.n + 1)
        Wc = [0.0] * (2 * self.n + 1)
        
        Wm[0] = self.lambda_ / (self.n + self.lambda_)
        Wc[0] = Wm[0] + (1 - self.alpha**2 + self.beta)
        
        for i in range(1, 2 * self.n + 1):
            Wm[i] = 1.0 / (2 * (self.n + self.lambda_))
            Wc[i] = Wm[i]
        
        return (Wm, Wc)

    def _calculate_statistics(self, points, weights):
        """Calculate mean and covariance from weighted samples."""
        Wm = weights[0]
        Wc = weights[1]
        
        # Calculate mean
        mean = [0.0 for _ in range(self.n)]
        for i in range(self.n):
            for j in range(len(points)):
                mean[i] += Wm[j] * points[j][i]
        
        # Calculate covariance
        covariance = [[0.0 for _ in range(self.n)] for _ in range(self.n)]
        for j in range(len(points)):
            dx = [points[j][i] - mean[i] for i in range(self.n)]
            weight = Wc[j]
            for i in range(self.n):
                for k in range(self.n):
                    covariance[i][k] += weight * dx[i] * dx[k]
        
        return (mean, covariance)

    def _cholesky_decomposition(self):
        """Cholesky decomposition of covariance matrix."""
        n = self.n
        L = [[0.0 for _ in range(n)] for _ in range(n)]
        
        for i in range(n):
            for j in range(i + 1):
                sum_k = 0.0
                for k in range(j):
                    sum_k += L[i][k] * L[j][k]
                
                if i == j:
                    L[i][j] = math.sqrt(max(0, self.covariance[i][i] - sum_k))
                else:
                    L[i][j] = (self.covariance[i][j] - sum_k) / L[j][j]
        
        return L

    def _matrix_inverse_2x2(self, matrix):
        """Inverse of 2x2 matrix."""
        det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
        if abs(det) < 1e-10:
            return [[1.0, 0.0], [0.0, 1.0]]
        
        inv_det = 1.0 / det
        return [
            [matrix[1][1] * inv_det, -matrix[0][1] * inv_det],
            [-matrix[1][0] * inv_det, matrix[0][0] * inv_det]
        ]

    def get_state(self):
        return tuple(self.state)


class ParticleFilter:
    """Particle Filter for nonlinear/non-Gaussian state estimation."""

    def __init__(self, num_particles=100, initial_state=None, 
                 initial_covariance=None, motion_noise=0.1, measurement_noise=0.2):
        self.num_particles = num_particles
        
        # Initialize particles
        self.particles = []
        self.weights = []
        
        if initial_state is None:
            initial_state = [0.0, 0.0, 0.0, 0.0]  # x, y, vx, vy
        
        if initial_covariance is None:
            initial_covariance = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        
        # Sample particles from initial distribution
        for _ in range(num_particles):
            particle = []
            for i in range(4):
                # Simple Gaussian sampling
                sigma = math.sqrt(max(0, initial_covariance[i][i]))
                particle.append(initial_state[i] + random.gauss(0, sigma))
            self.particles.append(particle)
            self.weights.append(1.0 / num_particles)
        
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise

    def predict(self, dt):
        """Predict particle positions using motion model with noise."""
        for i in range(len(self.particles)):
            particle = self.particles[i]
            x, y, vx, vy = particle
            
            # Apply motion model with noise
            new_x = x + vx * dt + random.gauss(0, self.motion_noise)
            new_y = y + vy * dt + random.gauss(0, self.motion_noise)
            new_vx = vx + random.gauss(0, self.motion_noise)
            new_vy = vy + random.gauss(0, self.motion_noise)
            
            self.particles[i] = [new_x, new_y, new_vx, new_vy]

    def update(self, measurement_x, measurement_y):
        """Update particle weights based on measurement."""
        total_weight = 0.0
        
        for i in range(len(self.particles)):
            particle = self.particles[i]
            x, y = particle[0], particle[1]
            
            # Calculate likelihood (Gaussian around measurement)
            dx = measurement_x - x
            dy = measurement_y - y
            variance = self.measurement_noise**2
            
            # Gaussian likelihood
            likelihood = math.exp(-0.5 * (dx**2 + dy**2) / variance)
            
            self.weights[i] = likelihood
            total_weight += likelihood
        
        # Normalize weights
        if total_weight > 0:
            self.weights = [w / total_weight for w in self.weights]
        else:
            # Reset to uniform if all weights are zero
            self.weights = [1.0 / len(self.weights)] * len(self.weights)

    def resample(self):
        """Systematic resampling of particles."""
        new_particles = []
        cumulative_weights = []
        running_sum = 0.0
        
        for w in self.weights:
            running_sum += w
            cumulative_weights.append(running_sum)
        
        # Systematic resampling
        step = 1.0 / self.num_particles
        u = random.uniform(0, step)
        
        i = 0
        for _ in range(self.num_particles):
            while u > cumulative_weights[i] and i < len(cumulative_weights) - 1:
                i += 1
            new_particles.append(list(self.particles[i]))
            u += step
        
        self.particles = new_particles
        self.weights = [1.0 / self.num_particles] * self.num_particles

    def estimate_state(self):
        """Calculate weighted mean state estimate."""
        state = [0.0, 0.0, 0.0, 0.0]
        
        for i in range(len(self.particles)):
            particle = self.particles[i]
            weight = self.weights[i]
            for j in range(4):
                state[j] += particle[j] * weight
        
        return tuple(state)

    def get_state(self):
        return self.estimate_state()


class ErrorStateEKF:
    """Error-State Extended Kalman Filter for nonlinear systems."""

    def __init__(self, initial_state=None, initial_covariance=None,
                 process_noise=0.1, measurement_noise=0.2):
        # True state (nominal)
        self.true_state = initial_state or [0.0, 0.0, 0.0, 0.0]  # x, y, vx, vy
        
        # Error state (delta from true state)
        self.error_state = [0.0, 0.0, 0.0, 0.0]
        
        # Error state covariance
        if initial_covariance is None:
            self.covariance = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        else:
            self.covariance = initial_covariance
        
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def predict(self, dt):
        """Predict step using error-state EKF."""
        # In error-state EKF, we mainly propagate the error covariance
        # The true state is propagated using the nonlinear model
        
        # Propagate true state (nonlinear motion model)
        x, y, vx, vy = self.true_state
        new_x = x + vx * dt
        new_y = y + vy * dt
        self.true_state = [new_x, new_y, vx, vy]  # Simplified - velocity unchanged
        
        # Propagate error covariance (linear approximation)
        # F is the Jacobian of the error dynamics (simplified to identity for now)
        F = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        
        new_cov = [[0.0 for _ in range(4)] for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    new_cov[i][j] += F[i][k] * self.covariance[k][j]
                new_cov[i][j] += self.process_noise if i == j else 0.0
        
        self.covariance = new_cov

    def update(self, measurement_x, measurement_y):
        """Update step with measurements."""
        # Measurement residual
        z_measured = [measurement_x, measurement_y]
        z_predicted = [self.true_state[0], self.true_state[1]]
        residual = [z_measured[i] - z_predicted[i] for i in range(2)]
        
        # Measurement matrix H (linear approximation)
        H = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ]
        
        # Calculate innovation covariance
        HPHt = [[0.0, 0.0], [0.0, 0.0]]
        for i in range(2):
            for j in range(2):
                for k in range(4):
                    for l in range(4):
                        HPHt[i][j] += H[i][k] * self.covariance[k][l] * H[j][l]
        
        S = [[HPHt[i][j] + (self.measurement_noise**2 if i == j else 0.0) 
              for j in range(2)] for i in range(2)]
        
        # Kalman gain
        S_inv = self._matrix_inverse_2x2(S)
        PHt = [[0.0 for _ in range(2)] for _ in range(4)]
        for i in range(4):
            for j in range(2):
                for k in range(4):
                    PHt[i][j] += self.covariance[i][k] * H[j][k]
        
        K = [[0.0 for _ in range(2)] for _ in range(4)]
        for i in range(4):
            for j in range(2):
                for k in range(2):
                    K[i][j] += PHt[i][k] * S_inv[k][j]
        
        # Update true state
        state_update = [0.0 for _ in range(4)]
        for i in range(4):
            for j in range(2):
                state_update[i] += K[i][j] * residual[j]
        
        self.true_state = [self.true_state[i] + state_update[i] for i in range(4)]
        
        # Update error covariance
        for i in range(4):
            for j in range(4):
                for k in range(2):
                    self.covariance[i][j] -= K[i][k] * S[k][j] * K[j][k]

    def _matrix_inverse_2x2(self, matrix):
        """Inverse of 2x2 matrix."""
        det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
        if abs(det) < 1e-10:
            return [[1.0, 0.0], [0.0, 1.0]]
        
        inv_det = 1.0 / det
        return [
            [matrix[1][1] * inv_det, -matrix[0][1] * inv_det],
            [-matrix[1][0] * inv_det, matrix[0][0] * inv_det]
        ]

    def get_state(self):
        return tuple(self.true_state)


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


class LinearKalmanFilterNode(_OdometryEstimatorNode):
    """Linear Kalman Filter state estimator node."""

    def __init__(self, node_name='linear_kalman_filter'):
        super().__init__(node_name, '/odometry/linear_kf')
        self.declare_parameter('process_noise', 0.1)
        self.declare_parameter('measurement_noise', 0.2)
        self.estimator = LinearKalmanFilter(
            process_noise=float(self.get_parameter('process_noise').value),
            measurement_noise=float(self.get_parameter('measurement_noise').value))

    def _on_odom(self, msg):
        dt = self._dt(msg)
        if dt > 0.0:
            self.estimator.predict(dt)
        position = msg.pose.pose.position
        self.estimator.update(position.x, position.y)
        state = self.estimator.get_state()
        self._publish(msg, state[0], state[1])


class UKFNode(_OdometryEstimatorNode):
    """Unscented Kalman Filter state estimator node."""

    def __init__(self, node_name='ukf_estimator'):
        super().__init__(node_name, '/odometry/ukf')
        self.declare_parameter('alpha', 1.0)
        self.declare_parameter('beta', 2.0)
        self.declare_parameter('kappa', 0.0)
        self.estimator = UnscentedKalmanFilter(
            alpha=float(self.get_parameter('alpha').value),
            beta=float(self.get_parameter('beta').value),
            kappa=float(self.get_parameter('kappa').value))

    def _on_odom(self, msg):
        dt = self._dt(msg)
        if dt > 0.0:
            self.estimator.predict(dt)
        position = msg.pose.pose.position
        self.estimator.update(position.x, position.y)
        state = self.estimator.get_state()
        self._publish(msg, state[0], state[1])


class ParticleFilterNode(_OdometryEstimatorNode):
    """Particle Filter state estimator node."""

    def __init__(self, node_name='particle_filter'):
        super().__init__(node_name, '/odometry/particle_filter')
        self.declare_parameter('num_particles', 100)
        self.declare_parameter('motion_noise', 0.1)
        self.declare_parameter('measurement_noise', 0.2)
        self.estimator = ParticleFilter(
            num_particles=int(self.get_parameter('num_particles').value),
            motion_noise=float(self.get_parameter('motion_noise').value),
            measurement_noise=float(self.get_parameter('measurement_noise').value))

    def _on_odom(self, msg):
        dt = self._dt(msg)
        if dt > 0.0:
            self.estimator.predict(dt)
        position = msg.pose.pose.position
        self.estimator.update(position.x, position.y)
        self.estimator.resample()
        state = self.estimator.get_state()
        self._publish(msg, state[0], state[1])


class ErrorStateEKFNode(_OdometryEstimatorNode):
    """Error-State EKF state estimator node."""

    def __init__(self, node_name='error_state_ekf'):
        super().__init__(node_name, '/odometry/error_state_ekf')
        self.declare_parameter('process_noise', 0.1)
        self.declare_parameter('measurement_noise', 0.2)
        self.estimator = ErrorStateEKF(
            process_noise=float(self.get_parameter('process_noise').value),
            measurement_noise=float(self.get_parameter('measurement_noise').value))

    def _on_odom(self, msg):
        dt = self._dt(msg)
        if dt > 0.0:
            self.estimator.predict(dt)
        position = msg.pose.pose.position
        self.estimator.update(position.x, position.y)
        state = self.estimator.get_state()
        self._publish(msg, state[0], state[1])


def linear_kalman_filter_main(args=None):
    return _run(LinearKalmanFilterNode, 'linear_kalman_filter', args=args)


def ukf_estimator_main(args=None):
    return _run(UKFNode, 'ukf_estimator', args=args)


def particle_filter_main(args=None):
    return _run(ParticleFilterNode, 'particle_filter', args=args)


def error_state_ekf_main(args=None):
    return _run(ErrorStateEKFNode, 'error_state_ekf', args=args)


if __name__ == '__main__':
    sys.exit(ekf_3d_estimator_main())
