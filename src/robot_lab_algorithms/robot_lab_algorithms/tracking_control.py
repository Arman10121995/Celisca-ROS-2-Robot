"""Path-tracking controllers on one differential-drive plant (R7.8).

Every controller here drives the same plant: a unicycle

    x' = v cos(theta),  y' = v sin(theta),  theta' = w

with inputs (v, w) bounded by :class:`Limits` (Bumperbot: 0.3 m/s,
1.0 rad/s).  A path (a Nav2 ``nav_msgs/Path`` or any polyline) becomes a
:class:`Reference`: arc length s, heading theta_r(s), curvature kappa(s) and
a speed profile v_r(s).  The controllers measure the error to the path at
the robot's projection s* and return (v, w):

* :class:`PidTracker` - PID on a heading error that includes the lateral
  offset, with derivative filtering and conditional-integration anti-windup.
* :class:`LqrTracker` - discrete LQR on the linearised lateral/heading error
  dynamics, gains from the discrete algebraic Riccati equation per speed.
* :class:`LinearMpcTracker` - constrained linear MPC on the same error model,
  time-varying along the previewed path; the box-constrained QP is solved
  exactly as a bounded least-squares problem.
* :class:`NonlinearMpcTracker` - nonlinear MPC on the full unicycle with
  bounded (v, w), single shooting, adjoint gradient and L-BFGS-B.
* :class:`FeedbackLinearizationTracker` - input-output linearisation of a
  point ahead of the axle, where the unicycle map is invertible.
* :class:`BacksteppingTracker` - the Lyapunov/backstepping tracking law of
  Jiang and Nijmeijer (1997).

Each controller reports its last compute time; ``deadline_s`` is the control
period the ROS server runs at (0.05 s), and a step over it is a deadline miss.
:class:`ActuatorPlant` adds the actuator model used for benchmarks: a
first-order lag, acceleration limits and saturation.

The plant is the differential-drive class only.  Car-like robots need the
bicycle model (minimum turning radius, no rotation in place); these
controllers are not offered for them.
"""

import math
import time

import numpy as np

try:
    from scipy.linalg import solve_discrete_are
    from scipy.optimize import lsq_linear, minimize
except ImportError:  # pragma: no cover - scipy is a declared dependency
    solve_discrete_are = lsq_linear = minimize = None


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Limits:
    """Actuator limits shared by every controller and the plant."""

    def __init__(self, v_max=0.3, v_min=0.0, w_max=1.0, a_max=0.5, alpha_max=2.0):
        self.v_max = float(v_max)
        self.v_min = float(v_min)
        self.w_max = float(w_max)
        self.a_max = float(a_max)
        self.alpha_max = float(alpha_max)

    def clamp(self, v, w):
        return (min(self.v_max, max(self.v_min, float(v))),
                min(self.w_max, max(-self.w_max, float(w))))


# ---------------------------------------------------------------------------
# Reference path
# ---------------------------------------------------------------------------

class Reference:
    """Arc-length parameterised path with heading, curvature and speed.

    The polyline is resampled every ``ds`` metres and smoothed with a
    ``smooth_m`` moving average (grid planners return staircase paths whose
    raw curvature is meaningless).  Speed: v_r(s) = min(v_max,
    sqrt(a_lat / |kappa|), sqrt(2 a_dec (L - s))), at least ``v_creep``.
    """

    def __init__(self, points, v_max=0.3, a_lat=0.25, a_dec=0.25, v_creep=0.05,
                 ds=0.05, smooth_m=0.25):
        xy = np.asarray([(float(p[0]), float(p[1])) for p in points], dtype=float)
        if len(xy) == 0:
            raise ValueError('empty path')
        keep = [0]
        for i in range(1, len(xy)):
            if np.hypot(*(xy[i] - xy[keep[-1]])) > 1e-6:
                keep.append(i)
        xy = xy[keep]
        if len(xy) == 1:
            xy = np.vstack([xy, xy + [1e-3, 0.0]])
        seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
        s_raw = np.concatenate([[0.0], np.cumsum(seg)])
        n = max(2, int(math.ceil(s_raw[-1] / ds)) + 1)
        s = np.linspace(0.0, s_raw[-1], n)
        x = np.interp(s, s_raw, xy[:, 0])
        y = np.interp(s, s_raw, xy[:, 1])
        half = int(round(smooth_m / max(ds, 1e-6) / 2.0))
        if half > 0 and n > 2 * half + 2:
            x, y = self._smooth(x, half), self._smooth(y, half)
        dx, dy = np.gradient(x), np.gradient(y)
        theta = np.unwrap(np.arctan2(dy, dx))
        step = max(s[1] - s[0], 1e-6)
        kappa = np.gradient(theta) / step
        if half > 0 and n > 2 * half + 2:
            kappa = self._smooth(kappa, half)
        self.s, self.x, self.y, self.theta, self.kappa = s, x, y, theta, kappa
        self.length = float(s[-1])
        self.v_max, self.a_lat, self.a_dec, self.v_creep = v_max, a_lat, a_dec, v_creep

    @staticmethod
    def _smooth(values, half):
        """Moving average that keeps the end points fixed."""
        out = values.copy()
        kernel = np.ones(2 * half + 1) / (2 * half + 1)
        inner = np.convolve(values, kernel, mode='valid')
        out[half:len(values) - half] = inner
        for i in range(1, half):  # shrink the window towards the ends
            k = np.ones(2 * i + 1) / (2 * i + 1)
            out[i] = float(np.dot(values[:2 * i + 1], k))
            out[-1 - i] = float(np.dot(values[-2 * i - 1:], k))
        return out

    def state(self, s, extend=False):
        """(x, y, theta, kappa) at arc length s.

        Clamped to the path, or with ``extend`` continued straight along the
        final heading past the end.
        """
        s = max(0.0, float(s))
        beyond = s - self.length if extend else 0.0
        s = min(self.length, s)
        x, y = float(np.interp(s, self.s, self.x)), float(np.interp(s, self.s, self.y))
        theta = float(np.interp(s, self.s, self.theta))
        if beyond > 0.0:
            return x + beyond * math.cos(theta), y + beyond * math.sin(theta), theta, 0.0
        return x, y, theta, float(np.interp(s, self.s, self.kappa))

    def speed(self, s):
        _, _, _, kappa = self.state(s)
        v = self.v_max
        if abs(kappa) > 1e-6:
            v = min(v, math.sqrt(self.a_lat / abs(kappa)))
        v = min(v, math.sqrt(2.0 * self.a_dec * max(0.0, self.length - s)))
        return max(self.v_creep, v) if s < self.length else 0.0

    def project(self, x, y, hint=0.0, back=0.3, ahead=1.5):
        """Arc length of the closest path point within [hint-back, hint+ahead]."""
        lo = np.searchsorted(self.s, max(0.0, hint - back))
        hi = np.searchsorted(self.s, min(self.length, hint + ahead), side='right')
        lo, hi = max(0, lo - 1), max(lo + 2, min(len(self.s), hi + 1))
        d = np.hypot(self.x[lo:hi] - x, self.y[lo:hi] - y)
        i = lo + int(np.argmin(d))
        # Refine on the neighbouring segments.
        best_s, best_d = float(self.s[i]), float(d[i - lo])
        for j in (i - 1, i):
            if 0 <= j < len(self.s) - 1:
                ax, ay = self.x[j], self.y[j]
                bx, by = self.x[j + 1], self.y[j + 1]
                vx, vy = bx - ax, by - ay
                length_sq = vx * vx + vy * vy
                if length_sq < 1e-12:
                    continue
                t = min(1.0, max(0.0, ((x - ax) * vx + (y - ay) * vy) / length_sq))
                px, py = ax + t * vx, ay + t * vy
                dist = math.hypot(px - x, py - y)
                if dist < best_d:
                    best_d, best_s = dist, float(self.s[j] + t * math.sqrt(length_sq))
        return best_s

    def preview(self, s0, steps, dt):
        """Reference samples (s, x, y, theta, kappa, v) advancing at v_r."""
        out, s = [], float(s0)
        for _ in range(steps):
            v = self.speed(s)
            s = min(self.length, s + v * dt)
            x, y, theta, kappa = self.state(s)
            out.append((s, x, y, theta, kappa, self.speed(s)))
        return out

    def errors(self, pose, s):
        """Path-frame errors at s: (along, lateral, heading); lateral > 0 left."""
        x, y, theta, _ = self.state(s)
        dx, dy = pose[0] - x, pose[1] - y
        c, si = math.cos(theta), math.sin(theta)
        return c * dx + si * dy, -si * dx + c * dy, wrap(pose[2] - theta)


# ---------------------------------------------------------------------------
# Controllers
# ---------------------------------------------------------------------------

class TrackingController:
    """Common interface: ``command(pose, measured, reference, s, dt) -> (v, w)``."""

    name = 'tracking'
    deadline_s = 0.05

    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.last_compute_s = 0.0
        self.deadline_misses = 0
        self.reset()

    def reset(self):
        pass

    def command(self, pose, measured, reference, s, dt):
        start = time.perf_counter()
        v, w = self._command(pose, measured, reference, s, dt)
        self.last_compute_s = time.perf_counter() - start
        if self.last_compute_s > self.deadline_s:
            self.deadline_misses += 1
        return self.limits.clamp(v, w)

    def _command(self, pose, measured, reference, s, dt):  # pragma: no cover - abstract
        raise NotImplementedError

    def _speed(self, reference, s, heading_error):
        return reference.speed(s) * max(0.0, math.cos(heading_error))


class PidTracker(TrackingController):
    """PID on e = theta_d - theta with theta_d = theta_r - atan(k_lat e_lat).

    w = kappa v_r + Kp e + Ki I + Kd (de/dt filtered).  Anti-windup by
    conditional integration: the integral only grows while the output is
    unsaturated or the error drives it back out of saturation, and is
    clamped to +-i_max.  ``anti_windup=False`` integrates unconditionally
    (for the saturation-recovery comparison).
    """

    name = 'pid'

    def __init__(self, limits=None, kp=2.0, ki=0.3, kd=0.1, k_lat=2.0, i_max=1.0,
                 d_filter=0.3, anti_windup=True):
        self.kp, self.ki, self.kd, self.k_lat = kp, ki, kd, k_lat
        self.i_max, self.d_filter, self.anti_windup = i_max, d_filter, anti_windup
        super().__init__(limits)

    def reset(self):
        self.integral, self.previous, self.derivative = 0.0, None, 0.0

    def _command(self, pose, measured, reference, s, dt):
        _, lateral, heading = reference.errors(pose, s)
        _, _, _, kappa = reference.state(s)
        v_r = reference.speed(s)
        error = wrap(-heading - math.atan(self.k_lat * lateral))
        if self.previous is not None and dt > 0:
            raw = wrap(error - self.previous) / dt
            self.derivative += (raw - self.derivative) * min(1.0, dt / max(self.d_filter, dt))
        self.previous = error
        unsaturated = kappa * v_r + self.kp * error + self.ki * self.integral + self.kd * self.derivative
        saturated = abs(unsaturated) >= self.limits.w_max
        if not self.anti_windup or not saturated or unsaturated * error < 0:
            self.integral += error * dt
            if self.anti_windup:
                self.integral = min(self.i_max, max(-self.i_max, self.integral))
        w = kappa * v_r + self.kp * error + self.ki * self.integral + self.kd * self.derivative
        return self._speed(reference, s, heading), w


def _lateral_model(v, dt):
    """Exact discretisation of e_lat' = v e_theta, e_theta' = u."""
    a = np.array([[1.0, v * dt], [0.0, 1.0]])
    b = np.array([[0.5 * v * dt * dt], [dt]])
    return a, b


class LqrTracker(TrackingController):
    """Discrete LQR on z = [e_lat, e_theta], u = w - kappa v_r.

    Linearised about the path at speed v_r: e_lat' = v_r e_theta,
    e_theta' = u.  K(v_r) = (R + B'PB)^-1 B'PA with P from the discrete
    algebraic Riccati equation; gains are cached per 0.01 m/s.
    """

    name = 'lqr'

    def __init__(self, limits=None, q_lat=6.0, q_heading=1.0, r=0.6, dt=0.05):
        self.q = np.diag([q_lat, q_heading])
        self.r = np.array([[r]])
        self.dt = dt
        self.gains = {}
        super().__init__(limits)

    def gain(self, v):
        key = round(max(0.05, v), 2)
        if key not in self.gains:
            a, b = _lateral_model(key, self.dt)
            p = solve_discrete_are(a, b, self.q, self.r)
            self.gains[key] = np.linalg.solve(self.r + b.T @ p @ b, b.T @ p @ a)
        return self.gains[key]

    def _command(self, pose, measured, reference, s, dt):
        _, lateral, heading = reference.errors(pose, s)
        _, _, _, kappa = reference.state(s)
        v_r = reference.speed(s)
        z = np.array([lateral, heading])
        u = -float((self.gain(v_r) @ z)[0])
        return self._speed(reference, s, heading), kappa * v_r + u


class LinearMpcTracker(TrackingController):
    """Constrained linear MPC on the lateral error model.

    Horizon N steps of dt along the previewed path, with the model
    linearised at each previewed (v_k, kappa_k).  Decision: absolute turn
    rates w_0..w_{N-1} with the hard bound |w_k| <= w_max.  Cost:
    sum z_k'Qz_k + r (w_k - kappa_k v_k)^2 + s_rate (w_k - w_{k-1})^2.
    The condensed QP with box bounds is solved exactly by bounded-variable
    least squares (scipy.optimize.lsq_linear, BVLS).
    """

    name = 'linear_mpc'

    def __init__(self, limits=None, horizon=20, dt=0.1, q_lat=6.0, q_heading=1.0, r=0.6,
                 rate=0.4, q_terminal=4.0):
        self.horizon, self.dt = horizon, dt
        self.q_lat, self.q_heading, self.r, self.rate, self.q_terminal = q_lat, q_heading, r, rate, q_terminal
        super().__init__(limits)

    def reset(self):
        self.previous_w = None

    def _command(self, pose, measured, reference, s, dt):
        _, lateral, heading = reference.errors(pose, s)
        n = self.horizon
        preview = reference.preview(s, n, self.dt)
        _, _, _, kappa0 = reference.state(s)
        speeds = [reference.speed(s)] + [p[5] for p in preview[:-1]]
        kappas = [kappa0] + [p[4] for p in preview[:-1]]
        feedforward = np.array([k * v for k, v in zip(kappas, speeds)])
        # Condensed prediction z_{k+1} = F_k z0 + G_k (w - ff).
        z0 = np.array([lateral, heading])
        f_rows, g_rows = [], []
        phi = np.eye(2)
        gamma = np.zeros((2, n))
        for k in range(n):
            a, b = _lateral_model(max(0.05, speeds[k]), self.dt)
            phi = a @ phi
            gamma = a @ gamma
            gamma[:, k] += b[:, 0]
            f_rows.append(phi.copy())
            g_rows.append(gamma.copy())
        weights = np.array([math.sqrt(self.q_lat), math.sqrt(self.q_heading)])
        blocks_a, blocks_b = [], []
        for k in range(n):
            scale = weights * (math.sqrt(self.q_terminal) if k == n - 1 else 1.0)
            free = f_rows[k] @ z0 - g_rows[k] @ feedforward
            blocks_a.append(scale[:, None] * g_rows[k])
            blocks_b.append(-scale * free)
        blocks_a.append(math.sqrt(self.r) * np.eye(n))
        blocks_b.append(math.sqrt(self.r) * feedforward)
        difference = np.eye(n) - np.eye(n, k=-1)
        previous = self.previous_w if self.previous_w is not None else float(measured[1])
        start = np.zeros(n)
        start[0] = previous
        blocks_a.append(math.sqrt(self.rate) * difference)
        blocks_b.append(math.sqrt(self.rate) * start)
        result = lsq_linear(np.vstack(blocks_a), np.concatenate(blocks_b),
                            bounds=(-self.limits.w_max, self.limits.w_max), method='bvls')
        w = float(result.x[0])
        self.previous_w = w
        return self._speed(reference, s, heading), w


class NonlinearMpcTracker(TrackingController):
    """Nonlinear MPC on the unicycle with bounded (v, w).

    State (x, y, theta), Euler steps of dt over N steps; decision (v_k, w_k)
    with v_min <= v_k <= v_max and |w_k| <= w_max.  Cost:
    sum q_p |p_k - p_rk|^2 + q_th 2(1 - cos(theta_k - theta_rk))
        + r_v (v_k - v_rk)^2 + r_w (w_k - w_rk)^2 + rate penalties.
    The gradient comes from the adjoint (backward) recursion; L-BFGS-B
    solves within ``max_iterations``, warm-started from the shifted
    previous solution.
    """

    name = 'nonlinear_mpc'

    def __init__(self, limits=None, horizon=15, dt=0.1, q_position=8.0, q_heading=1.0,
                 r_v=1.0, r_w=0.2, rate_v=2.0, rate_w=0.4, max_iterations=30):
        self.horizon, self.dt = horizon, dt
        self.q_position, self.q_heading = q_position, q_heading
        self.r_v, self.r_w, self.rate_v, self.rate_w = r_v, r_w, rate_v, rate_w
        self.max_iterations = max_iterations
        super().__init__(limits)

    def reset(self):
        self.warm = None
        self.iterations = 0

    def _rollout(self, state, u):
        n, dt = self.horizon, self.dt
        xs = np.empty((n + 1, 3))
        xs[0] = state
        for k in range(n):
            x, y, th = xs[k]
            v, w = u[k]
            xs[k + 1] = (x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt)
        return xs

    def _cost_and_gradient(self, flat, state, ref, u_ref, u_prev):
        n, dt = self.horizon, self.dt
        u = flat.reshape(n, 2)
        xs = self._rollout(state, u)
        dp = xs[1:, :2] - ref[:, :2]
        dth = xs[1:, 2] - ref[:, 2]
        du = u - u_ref
        rate = np.diff(np.vstack([u_prev, u]), axis=0)
        cost = (self.q_position * float(np.sum(dp * dp))
                + self.q_heading * float(np.sum(2.0 * (1.0 - np.cos(dth))))
                + self.r_v * float(np.sum(du[:, 0] ** 2)) + self.r_w * float(np.sum(du[:, 1] ** 2))
                + self.rate_v * float(np.sum(rate[:, 0] ** 2)) + self.rate_w * float(np.sum(rate[:, 1] ** 2)))
        # Adjoint recursion: lam_k = dJ/dx_k.
        grad = np.zeros((n, 2))
        grad[:, 0] += 2.0 * self.r_v * du[:, 0]
        grad[:, 1] += 2.0 * self.r_w * du[:, 1]
        rate_grad = np.zeros((n, 2))
        rate_grad[:, 0] = 2.0 * self.rate_v * rate[:, 0]
        rate_grad[:, 1] = 2.0 * self.rate_w * rate[:, 1]
        grad += rate_grad
        grad[:-1] -= rate_grad[1:]
        lam = np.zeros(3)
        for k in range(n - 1, -1, -1):
            # Stage cost on x_{k+1}.
            lam = lam + np.array([2.0 * self.q_position * dp[k, 0], 2.0 * self.q_position * dp[k, 1],
                                  2.0 * self.q_heading * math.sin(dth[k])])
            th = xs[k, 2]
            v = u[k, 0]
            c, si = math.cos(th), math.sin(th)
            grad[k, 0] += lam[0] * c * dt + lam[1] * si * dt
            grad[k, 1] += lam[2] * dt
            # Propagate through x_{k+1} = f(x_k, u_k).
            lam = np.array([lam[0], lam[1], lam[2] + lam[0] * (-v * si * dt) + lam[1] * (v * c * dt)])
        return cost, grad.ravel()

    def _command(self, pose, measured, reference, s, dt):
        n = self.horizon
        preview = reference.preview(s, n, self.dt)
        ref = np.array([[p[1], p[2], p[3]] for p in preview])
        # Keep reference headings on the robot's branch of the angle.
        ref[:, 2] = pose[2] + np.array([wrap(t - pose[2]) for t in ref[:, 2]])
        ref[:, 2] = np.unwrap(ref[:, 2])
        u_ref = np.array([[p[5], p[4] * p[5]] for p in preview])
        u_prev = np.array([[float(measured[0]), float(measured[1])]])
        if self.warm is None:
            guess = u_ref.copy()
        else:
            guess = np.vstack([self.warm[1:], self.warm[-1:]])
        bounds = [(self.limits.v_min, self.limits.v_max), (-self.limits.w_max, self.limits.w_max)] * n
        guess = np.clip(guess, [self.limits.v_min, -self.limits.w_max], [self.limits.v_max, self.limits.w_max])
        state = np.array([pose[0], pose[1], pose[2]], dtype=float)
        result = minimize(self._cost_and_gradient, guess.ravel(), args=(state, ref, u_ref, u_prev),
                          jac=True, method='L-BFGS-B', bounds=bounds,
                          options={'maxiter': self.max_iterations})
        u = result.x.reshape(n, 2)
        self.warm = u
        self.iterations = int(result.nit)
        return float(u[0, 0]), float(u[0, 1])


class FeedbackLinearizationTracker(TrackingController):
    """Input-output linearisation of the point P = p + d (cos theta, sin theta).

    P' = M(theta) [v, w] with M = [[cos, -d sin], [sin, d cos]], invertible
    for d != 0.  The linear outer loop P'_des = P'_r + K (P_r - P) tracks the
    path point d ahead of the projection moving at v_r; [v, w] =
    M^-1 P'_des.  Forward-only saturation (v >= v_min) breaks the exact
    linearisation when P'_des points backwards.  Past the path end the
    reference for P continues straight, so the axle (not P) stops at the end.
    """

    name = 'feedback_linearization'

    def __init__(self, limits=None, offset=0.2, gain=1.5):
        self.offset, self.gain = offset, gain
        super().__init__(limits)

    def _command(self, pose, measured, reference, s, dt):
        d = self.offset
        x, y, theta = pose
        px, py = x + d * math.cos(theta), y + d * math.sin(theta)
        rx, ry, rtheta, _ = reference.state(s + d, extend=True)
        v_r = reference.speed(s)
        ux = v_r * math.cos(rtheta) + self.gain * (rx - px)
        uy = v_r * math.sin(rtheta) + self.gain * (ry - py)
        c, si = math.cos(theta), math.sin(theta)
        v = c * ux + si * uy
        w = (-si * ux + c * uy) / d
        return v, w


class BacksteppingTracker(TrackingController):
    """Backstepping tracking law (Jiang and Nijmeijer, Automatica 1997).

    Robot-frame errors to the reference posture at s*:
      e_x = cos(th)(x_r-x) + sin(th)(y_r-y),  e_y = -sin(th)(x_r-x) + cos(th)(y_r-y),
      e_th = th_r - th.
    v = v_r cos(e_th) + c_x e_x
    w = w_r + c_th e_th + gamma v_r e_y sin(e_th)/e_th
    With V = (e_x^2 + e_y^2)/2 + e_th^2/(2 gamma):
      V' = -c_x e_x^2 - (c_th/gamma) e_th^2 <= 0.
    """

    name = 'backstepping'

    def __init__(self, limits=None, c_x=1.0, c_theta=2.0, gamma=8.0):
        self.c_x, self.c_theta, self.gamma = c_x, c_theta, gamma
        super().__init__(limits)

    def _command(self, pose, measured, reference, s, dt):
        x, y, theta = pose
        rx, ry, rtheta, kappa = reference.state(s)
        v_r = reference.speed(s)
        c, si = math.cos(theta), math.sin(theta)
        e_x = c * (rx - x) + si * (ry - y)
        e_y = -si * (rx - x) + c * (ry - y)
        e_theta = wrap(rtheta - theta)
        sinc = math.sin(e_theta) / e_theta if abs(e_theta) > 1e-6 else 1.0
        v = v_r * math.cos(e_theta) + self.c_x * e_x
        w = kappa * v_r + self.c_theta * e_theta + self.gamma * v_r * e_y * sinc
        return v, w


CONTROLLERS = {
    'pid': PidTracker,
    'lqr': LqrTracker,
    'linear_mpc': LinearMpcTracker,
    'nonlinear_mpc': NonlinearMpcTracker,
    'feedback_linearization': FeedbackLinearizationTracker,
    'backstepping': BacksteppingTracker,
}


def make_controller(name, limits=None, **parameters):
    if name not in CONTROLLERS:
        raise ValueError('unknown tracking controller %r; choose from %s' % (name, sorted(CONTROLLERS)))
    return CONTROLLERS[name](limits=limits, **parameters)


# ---------------------------------------------------------------------------
# Plant with actuator model (benchmarks)
# ---------------------------------------------------------------------------

class ActuatorPlant:
    """Unicycle with a first-order actuator lag, acceleration limits and saturation.

    Commands pass through: saturation to Limits, then each axis follows
    tau x' = (cmd - x) with |x'| <= a_max (linear) / alpha_max (angular).
    Integration uses exact arcs over ``substeps`` per control period.
    """

    def __init__(self, limits=None, tau=0.1, pose=(0.0, 0.0, 0.0)):
        self.limits = limits or Limits()
        self.tau = tau
        self.pose = np.array(pose, dtype=float)
        self.velocity = np.zeros(2)

    def step(self, command, dt, substeps=5):
        v_cmd, w_cmd = self.limits.clamp(*command)
        h = dt / substeps
        for _ in range(substeps):
            dv = (v_cmd - self.velocity[0]) / max(self.tau, h) * h
            dw = (w_cmd - self.velocity[1]) / max(self.tau, h) * h
            dv = min(self.limits.a_max * h, max(-self.limits.a_max * h, dv))
            dw = min(self.limits.alpha_max * h, max(-self.limits.alpha_max * h, dw))
            self.velocity += (dv, dw)
            v, w = self.velocity
            x, y, th = self.pose
            if abs(w) > 1e-9:
                x += v / w * (math.sin(th + w * h) - math.sin(th))
                y -= v / w * (math.cos(th + w * h) - math.cos(th))
            else:
                x += v * math.cos(th) * h
                y += v * math.sin(th) * h
            self.pose = np.array([x, y, th + w * h])
        return self.pose.copy()
