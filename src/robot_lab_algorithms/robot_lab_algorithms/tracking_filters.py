"""State estimation from noisy position fixes (R7.4).

The problem: a ground robot moves in the plane; a sensor reports its
position (x, y) with noise - AMCL, a GNSS or UWB fix, or truth with added
noise for benchmarks.  Each filter estimates position, heading, speed and
turn rate from those fixes alone, with a motion model:

* :class:`CvKalman` - linear Kalman filter, constant velocity
  ``[x, y, vx, vy]`` with white-acceleration process noise.
* :class:`CtrvEkf` - extended Kalman filter on the constant turn rate and
  velocity (CTRV) model ``[x, y, theta, v, w]``.
* :class:`CtrvUkf` - unscented Kalman filter on the CTRV model.
* :class:`CtrvParticleFilter` - particle filter on the CTRV model with
  systematic resampling when the effective sample size halves.
* :class:`ImmFilter` - interacting multiple model: a straight-line model and
  a turning model (both CTRV EKFs, the first with the turn rate held near
  zero), mixed by a Markov switching matrix.
* :class:`WindowSmoother` - sliding-window least squares over the last N
  positions (a linear Gaussian pose graph): fix factors plus
  constant-velocity (second-difference) factors; the newest pose is the
  estimate.

All share ``predict(dt)`` / ``update(z)`` / ``estimate() -> (x, y, theta, v, w)``.
"""
from __future__ import annotations

import math

import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def nearest_pd(P, floor=1e-9):
    """Symmetric matrix with eigenvalues clipped at ``floor`` (keeps a UKF going
    when round-off makes its covariance indefinite)."""
    P = 0.5 * (P + P.T)
    values, vectors = np.linalg.eigh(P)
    return (vectors * np.maximum(values, floor)) @ vectors.T



def canonical(state):
    """(x, y, theta, v, w) with v >= 0: the CTRV model cannot tell (v, theta)
    from (-v, theta + pi) from positions alone."""
    x, y, th, v, w = (float(s) for s in state)
    if v < 0:
        th, v = wrap(th + math.pi), -v
    return x, y, th, v, w


def ctrv(state, dt):
    x, y, th, v, w = state
    if abs(w) > 1e-4:
        x += v / w * (math.sin(th + w * dt) - math.sin(th))
        y += v / w * (-math.cos(th + w * dt) + math.cos(th))
    else:
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
    return np.array([x, y, wrap(th + w * dt), v, w])


def ctrv_jacobian(state, dt):
    """Numerical Jacobian of the CTRV transition (heading wrapped)."""
    J = np.zeros((5, 5))
    base = ctrv(state, dt)
    for i in range(5):
        e = np.zeros(5)
        e[i] = 1e-6
        d = (ctrv(state + e, dt) - base) / 1e-6
        d[2] = wrap(d[2] * 1e-6) / 1e-6
        J[:, i] = d
    return J


def ctrv_noise(state, dt, accel_std, yaw_accel_std):
    """Process noise from white longitudinal and yaw accelerations."""
    th = state[2]
    G = np.array([[0.5 * dt * dt * math.cos(th), 0.0], [0.5 * dt * dt * math.sin(th), 0.0],
                  [0.0, 0.5 * dt * dt], [dt, 0.0], [0.0, dt]])
    return G @ np.diag([accel_std ** 2, yaw_accel_std ** 2]) @ G.T + np.eye(5) * 1e-9


class CvKalman:
    name = 'cv_kalman'

    def __init__(self, measurement_std=0.3, accel_std=0.5):
        self.x = None
        self.P = np.diag([1.0, 1.0, 1.0, 1.0])
        self.r, self.q = measurement_std, accel_std

    def predict(self, dt):
        if self.x is None or dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        G = np.array([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * self.q ** 2

    def update(self, z):
        z = np.asarray(z, dtype=float)
        if self.x is None:
            self.x = np.array([z[0], z[1], 0.0, 0.0])
            self.P = np.diag([self.r ** 2, self.r ** 2, 1.0, 1.0])
            return
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        S = H @ self.P @ H.T + np.eye(2) * self.r ** 2
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(4) - K @ H) @ self.P

    def estimate(self):
        if self.x is None:
            return None
        x, y, vx, vy = self.x
        return x, y, math.atan2(vy, vx), math.hypot(vx, vy), 0.0


class CtrvEkf:
    name = 'ctrv_ekf'

    def __init__(self, measurement_std=0.3, accel_std=0.5, yaw_accel_std=0.5):
        self.x = None
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 0.25])
        self.r, self.accel_std, self.yaw_accel_std = measurement_std, accel_std, yaw_accel_std

    def predict(self, dt):
        if self.x is None or dt <= 0:
            return
        F = ctrv_jacobian(self.x, dt)
        self.P = F @ self.P @ F.T + ctrv_noise(self.x, dt, self.accel_std, self.yaw_accel_std)
        self.x = ctrv(self.x, dt)

    def update(self, z):
        z = np.asarray(z, dtype=float)
        if self.x is None:
            self.x = np.array([z[0], z[1], 0.0, 0.0, 0.0])
            self.P = np.diag([self.r ** 2, self.r ** 2, math.pi ** 2, 1.0, 0.25])
            return 1.0
        H = np.zeros((2, 5))
        H[0, 0] = H[1, 1] = 1.0
        y = z - self.x[:2]
        S = H @ self.P @ H.T + np.eye(2) * self.r ** 2
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        self.P = (np.eye(5) - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
        return float(np.exp(-0.5 * y @ np.linalg.solve(S, y)) / (2 * math.pi * math.sqrt(np.linalg.det(S))))

    def estimate(self):
        return None if self.x is None else canonical(self.x)


class CtrvUkf:
    name = 'ctrv_ukf'

    def __init__(self, measurement_std=0.3, accel_std=0.5, yaw_accel_std=0.5, alpha=0.5, beta=2.0, kappa=0.0):
        self.n = 5
        self.x = None
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 0.25])
        self.r, self.accel_std, self.yaw_accel_std = measurement_std, accel_std, yaw_accel_std
        lam = alpha ** 2 * (self.n + kappa) - self.n
        self.c = self.n + lam
        self.wm = np.full(2 * self.n + 1, 1.0 / (2 * self.c))
        self.wc = self.wm.copy()
        self.wm[0] = lam / self.c
        self.wc[0] = lam / self.c + 1 - alpha ** 2 + beta

    def _sigma(self):
        try:
            root = np.linalg.cholesky(self.c * self.P)
        except np.linalg.LinAlgError:
            self.P = nearest_pd(self.P)
            root = np.linalg.cholesky(self.c * (self.P + np.eye(5) * 1e-9))
        return np.array([self.x] + [self.x + root[:, i] for i in range(5)] + [self.x - root[:, i] for i in range(5)])

    def predict(self, dt):
        if self.x is None or dt <= 0:
            return
        points = np.array([ctrv(p, dt) for p in self._sigma()])
        mean = self.wm @ points
        mean[2] = math.atan2(self.wm @ np.sin(points[:, 2]), self.wm @ np.cos(points[:, 2]))
        d = points - mean
        d[:, 2] = [wrap(a) for a in d[:, 2]]
        self.P = (self.wc[:, None] * d).T @ d + ctrv_noise(mean, dt, self.accel_std, self.yaw_accel_std)
        self.x = mean

    def update(self, z):
        z = np.asarray(z, dtype=float)
        if self.x is None:
            self.x = np.array([z[0], z[1], 0.0, 0.0, 0.0])
            self.P = np.diag([self.r ** 2, self.r ** 2, math.pi ** 2, 1.0, 0.25])
            return
        points = self._sigma()
        Z = points[:, :2]
        z_mean = self.wm @ Z
        dz = Z - z_mean
        dx = points - self.x
        dx[:, 2] = [wrap(a) for a in dx[:, 2]]
        S = (self.wc[:, None] * dz).T @ dz + np.eye(2) * self.r ** 2
        C = (self.wc[:, None] * dx).T @ dz
        K = C @ np.linalg.inv(S)
        self.x = self.x + K @ (z - z_mean)
        self.x[2] = wrap(self.x[2])
        self.P = self.P - K @ S @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def estimate(self):
        return None if self.x is None else canonical(self.x)


class CtrvParticleFilter:
    name = 'particle_filter'

    def __init__(self, measurement_std=0.3, accel_std=0.5, yaw_accel_std=0.5, particles=500, seed=0):
        self.N = particles
        self.r, self.accel_std, self.yaw_accel_std = measurement_std, accel_std, yaw_accel_std
        self.rng = np.random.default_rng(seed)
        self.p = None
        self.w = np.full(particles, 1.0 / particles)

    def predict(self, dt):
        if self.p is None or dt <= 0:
            return
        a = self.rng.normal(0, self.accel_std, self.N)
        alpha = self.rng.normal(0, self.yaw_accel_std, self.N)
        x, y, th, v, w = self.p.T
        v_new = np.maximum(v + a * dt, -2.0)
        w_new = w + alpha * dt
        v_mid, w_mid = 0.5 * (v + v_new), 0.5 * (w + w_new)
        turning = np.abs(w_mid) > 1e-4
        safe = np.where(turning, w_mid, 1.0)
        dx = np.where(turning, v_mid / safe * (np.sin(th + w_mid * dt) - np.sin(th)), v_mid * np.cos(th) * dt)
        dy = np.where(turning, v_mid / safe * (-np.cos(th + w_mid * dt) + np.cos(th)), v_mid * np.sin(th) * dt)
        self.p = np.column_stack([x + dx, y + dy, (th + w_mid * dt + np.pi) % (2 * np.pi) - np.pi, v_new, w_new])

    def update(self, z):
        z = np.asarray(z, dtype=float)
        if self.p is None:
            self.p = np.column_stack([
                z[0] + self.rng.normal(0, self.r, self.N), z[1] + self.rng.normal(0, self.r, self.N),
                self.rng.uniform(-np.pi, np.pi, self.N), self.rng.uniform(0.0, 1.0, self.N),
                self.rng.normal(0, 0.3, self.N)])
            return
        d2 = np.sum((self.p[:, :2] - z) ** 2, axis=1)
        # Sensor resetting (as in augmented MCL): when no particle explains the
        # fix, re-seed a fifth of them around it so the filter can recover.
        if d2.min() > (3.0 * self.r) ** 2:
            k = self.N // 5
            pick = self.rng.choice(self.N, k, replace=False)
            self.p[pick, 0] = z[0] + self.rng.normal(0, self.r, k)
            self.p[pick, 1] = z[1] + self.rng.normal(0, self.r, k)
            d2[pick] = np.sum((self.p[pick, :2] - z) ** 2, axis=1)
        # Log-domain weights: no underflow to a uniform set when all are far.
        log_w = np.log(np.maximum(self.w, 1e-300)) - 0.5 * d2 / self.r ** 2
        log_w -= log_w.max()
        self.w = np.exp(log_w)
        self.w /= self.w.sum()
        if 1.0 / np.sum(self.w ** 2) < self.N / 2:
            positions = (self.rng.random() + np.arange(self.N)) / self.N
            index = np.minimum(np.searchsorted(np.cumsum(self.w), positions), self.N - 1)
            self.p = self.p[index]
            self.w = np.full(self.N, 1.0 / self.N)
            # Roughening against sample impoverishment.
            self.p += self.rng.normal(0, 1.0, self.p.shape) * np.array([0.02, 0.02, 0.02, 0.02, 0.02])

    def estimate(self):
        if self.p is None:
            return None
        p = self.p.copy()
        reverse = p[:, 3] < 0  # fold (v, theta) and (-v, theta + pi) onto one branch
        p[reverse, 3] *= -1.0
        p[reverse, 2] = (p[reverse, 2] + 2 * np.pi) % (2 * np.pi) - np.pi
        mean = self.w @ p
        mean[2] = math.atan2(self.w @ np.sin(p[:, 2]), self.w @ np.cos(p[:, 2]))
        return canonical(mean)


class ImmFilter:
    """Interacting multiple model: straight-line and turning CTRV EKFs."""

    name = 'imm'

    def __init__(self, measurement_std=0.3, accel_std=0.5, yaw_accel_std=0.5, switch=0.05):
        straight = CtrvEkf(measurement_std, accel_std, yaw_accel_std=0.01)
        turning = CtrvEkf(measurement_std, accel_std, yaw_accel_std=yaw_accel_std * 2.0)
        self.models = [straight, turning]
        self.mu = np.array([0.5, 0.5])
        self.T = np.array([[1 - switch, switch], [switch, 1 - switch]])

    def predict(self, dt):
        if self.models[0].x is None or dt <= 0:
            return
        c = self.T.T @ self.mu
        mix = (self.T * self.mu[:, None]) / c[None, :]          # mix[i, j] = P(i | j)
        xs = [m.x for m in self.models]
        Ps = [m.P for m in self.models]
        for j, model in enumerate(self.models):
            x0 = sum(mix[i, j] * xs[i] for i in range(2))
            x0[2] = math.atan2(sum(mix[i, j] * math.sin(xs[i][2]) for i in range(2)),
                               sum(mix[i, j] * math.cos(xs[i][2]) for i in range(2)))
            P0 = np.zeros((5, 5))
            for i in range(2):
                d = xs[i] - x0
                d[2] = wrap(d[2])
                P0 += mix[i, j] * (Ps[i] + np.outer(d, d))
            model.x, model.P = x0, P0
        self.models[0].x[4] *= 0.0  # the straight-line model holds no turn rate
        for model in self.models:
            model.predict(dt)
        self.mu = c

    def update(self, z):
        likelihoods = np.array([m.update(z) for m in self.models], dtype=float)
        likelihoods = np.maximum(likelihoods, 1e-300)
        self.mu = self.mu * likelihoods
        self.mu /= self.mu.sum()

    def estimate(self):
        if self.models[0].x is None:
            return None
        xs = [m.x for m in self.models]
        xs = [np.array(canonical(v)) for v in xs]
        x = self.mu[0] * xs[0] + self.mu[1] * xs[1]
        x[2] = math.atan2(self.mu @ [math.sin(v[2]) for v in xs], self.mu @ [math.cos(v[2]) for v in xs])
        return canonical(x)


class WindowSmoother:
    """Fixed-lag smoother: argmin sum |p_i - z_i|^2 / r^2 + sum |p_{i+1} - 2 p_i + p_{i-1}|^2 / (q dt^2)^2."""

    name = 'window_smoother'

    def __init__(self, measurement_std=0.3, accel_std=0.5, window=20):
        self.r, self.q, self.window = measurement_std, accel_std, window
        self.z, self.dts = [], []
        self.pending_dt = 0.0
        self.result = None

    def predict(self, dt):
        if dt > 0:
            self.pending_dt += dt

    def update(self, z):
        if self.z:
            self.dts.append(max(self.pending_dt, 1e-3))
        self.pending_dt = 0.0
        self.z.append(np.asarray(z, dtype=float))
        if len(self.z) > self.window:
            self.z.pop(0)
            self.dts.pop(0)
        n = len(self.z)
        rows, rhs = [], []
        for i, zi in enumerate(self.z):
            row = np.zeros(n)
            row[i] = 1.0 / self.r
            rows.append(row)
            rhs.append(zi / self.r)
        for i in range(1, n - 1):
            dt = 0.5 * (self.dts[i - 1] + self.dts[i])
            weight = 1.0 / (self.q * dt * dt)
            row = np.zeros(n)
            row[i - 1], row[i], row[i + 1] = weight, -2.0 * weight, weight
            rows.append(row)
            rhs.append(np.zeros(2))
        A = np.array(rows)
        p = np.linalg.lstsq(A, np.array(rhs), rcond=None)[0]
        if n >= 3:
            v1 = (p[-1] - p[-2]) / self.dts[-1]
            v0 = (p[-2] - p[-3]) / self.dts[-2]
            heading = math.atan2(v1[1], v1[0])
            w = wrap(heading - math.atan2(v0[1], v0[0])) / self.dts[-1] if np.linalg.norm(v0) > 1e-3 else 0.0
            self.result = (float(p[-1, 0]), float(p[-1, 1]), heading, float(np.linalg.norm(v1)), float(w))
        else:
            self.result = (float(p[-1, 0]), float(p[-1, 1]), 0.0, 0.0, 0.0)

    def estimate(self):
        return self.result


ESTIMATORS = {
    'cv_kalman': CvKalman,
    'ctrv_ekf': CtrvEkf,
    'ctrv_ukf': CtrvUkf,
    'particle_filter': CtrvParticleFilter,
    'imm': ImmFilter,
    'window_smoother': WindowSmoother,
}
