"""Forward bicycle tracking at the fixed axle, with real source steering bounds.

Equations/reference implementations: PythonRobotics Stanley and MPC examples
(Atsushi Sakai et al., MIT); independently implemented for this ROS contract.
https://atsushisakai.github.io/PythonRobotics/modules/6_path_tracking/

Virtual front steering delta gives kappa=tan(delta)/L. The existing drive
adapter converts a positive curvature to negative *rear* steering where
needed. Anti-Ackermann's tire scrub is not represented by this kinematic model.
"""
import math
import time

import numpy as np
from scipy.optimize import LinearConstraint, minimize

from .tracking_control import Reference, wrap


LAWS = ('bicycle_pure_pursuit', 'bicycle_stanley', 'bicycle_mpc')


class BicycleTracker:
    def __init__(self, law, drive, max_speed=.3, lookahead=.45, horizon=10, deadline_s=.035):
        if law not in LAWS or drive.get('type') != 'ackermann':
            raise ValueError('Choose a bicycle law and an authored Ackermann drive profile.')
        self.law = law
        self.length, self.max_delta = float(drive['wheelbase']), abs(float(drive['max_steer']))
        self.delta_rate, self.acceleration = float(drive['max_steer_rate']), float(drive['max_accel'])
        self.speed = min(float(max_speed), float(drive['max_speed']))
        self.axle = drive.get('steer_axle', 'front')
        if self.axle not in ('front', 'rear'):
            raise ValueError('Source steer axle must be front or rear.')
        values = [self.length, self.max_delta, self.delta_rate, self.acceleration, self.speed, lookahead, deadline_s]
        if not all(math.isfinite(v) and v > 0 for v in values) or self.max_delta >= math.pi/2:
            raise ValueError('Finite positive source geometry/limits are required.')
        self.lookahead, self.horizon, self.deadline = float(lookahead), int(horizon), float(deadline_s)
        if not 3 <= self.horizon <= 20:
            raise ValueError('Use a bounded prediction horizon of 3–20 steps.')
        self.delta, self.velocity, self.along = 0., 0., None
        self.solver_state, self.compute_s, self.deadline_misses = 'not called', 0., 0
        self.warm = np.zeros(self.horizon)

    def reference(self, points):
        # Preserve the kinematically feasible path; smoothing can move it
        # through a wall or manufacture a curvature change for a car.
        return Reference(points, v_max=self.speed, a_dec=self.acceleration,
            v_creep=min(.04, self.speed), ds=.03, smooth_m=0.)

    def pursuit(self, pose, reference, along):
        target = reference.state(min(reference.length, along+self.lookahead))
        dx, dy = target[0]-pose[0], target[1]-pose[1]
        lateral = -math.sin(pose[2])*dx+math.cos(pose[2])*dy
        return math.atan(self.length*2*lateral/max(dx*dx+dy*dy, .01))

    def stanley(self, pose, reference, along, measured_v):
        # Use the steered axle as Stanley's control point. Its signed
        # offset also makes the same law work with a fixed *front* axle.
        offset = self.length if self.axle == 'front' else -self.length
        axle = [pose[0]+offset*math.cos(pose[2]), pose[1]+offset*math.sin(pose[2])]
        projected = reference.project(*axle, along, back=self.length+.2, ahead=self.lookahead+self.length)
        x, y, theta, kappa = reference.state(projected)
        cross = -math.sin(theta)*(axle[0]-x)+math.cos(theta)*(axle[1]-y)
        heading = wrap(theta-pose[2])
        # For rear steering the heading term retains virtual-front sign;
        # moving the control point behind the fixed axle changes its feedback
        # authority. A fixed-axle cross-track correction avoids positive
        # feedback from applying the front-only Stanley law unchanged.
        if self.axle == 'rear':
            x, y, theta, kappa = reference.state(along)
            cross = -math.sin(theta)*(pose[0]-x)+math.cos(theta)*(pose[1]-y)
            heading = wrap(theta-pose[2])
        return math.atan(self.length*kappa)+heading-math.atan2(.8*cross, abs(measured_v)+.1)

    def mpc(self, pose, reference, along, measured_v, delta, dt, deadline):
        """Linear time-varying bicycle-error MPC, steering-rate input.

        States [cross-track m, heading rad, virtual steer rad]. Box bounds
        constrain steering rate, linear inequalities constrain every predicted
        steering angle. SLSQP uses an analytic quadratic gradient; deadline
        expiration falls back to bounded geometric tracking for this cycle.
        """
        _, lateral, heading = reference.errors(pose, along)
        state = np.array([lateral, heading, delta])
        v = max(.03, min(self.speed, abs(measured_v)))
        # A ten-step horizon at the 50 ms command period sees only 12.5 cm
        # at lab speed and barely responds to cross-track displacement.
        # Predict in 150 ms blocks, still replan/apply the first rate every
        # actual control period, retaining the physical slew bound.
        prediction_dt = max(dt, .15)
        transitions, offsets = [], []
        state_map, input_map = np.eye(3), np.zeros((3, self.horizon))
        constant = np.zeros(3)
        for k in range(self.horizon):
            curvature = reference.state(min(reference.length, along+v*prediction_dt*k))[3]
            nominal = np.clip(math.atan(self.length*curvature), -self.max_delta, self.max_delta)
            derivative = 1/(self.length*math.cos(nominal)**2)
            a = np.array([[1., v*prediction_dt, 0.], [0., 1., v*prediction_dt*derivative], [0., 0., 1.]])
            bias = np.array([0., v*prediction_dt*(math.tan(nominal)/self.length-derivative*nominal-curvature), 0.])
            state_map, input_map, constant = a@state_map, a@input_map, a@constant+bias
            input_map[2, k] += prediction_dt
            weight = np.sqrt([20., 5., .1])*(2. if k == self.horizon-1 else 1.)
            transitions.append(weight[:, None]*input_map)
            offsets.append(weight*(state_map@state+constant-np.array([0., 0., nominal])))
        matrix = np.vstack([*transitions, .12*np.eye(self.horizon)])
        offset = np.r_[np.concatenate(offsets), np.zeros(self.horizon)]
        def cost(control):
            if time.monotonic() > deadline:
                raise TimeoutError('Bicycle MPC solver deadline exceeded')
            residual = matrix@control+offset
            return .5*float(residual@residual), matrix.T@residual
        cumulative = prediction_dt*np.tril(np.ones((self.horizon, self.horizon)))
        constraint = LinearConstraint(cumulative, -self.max_delta-delta, self.max_delta-delta)
        result = minimize(cost, self.warm, jac=True, method='SLSQP',
            bounds=[(-self.delta_rate, self.delta_rate)]*self.horizon,
            constraints=[constraint], options=dict(maxiter=20, ftol=1e-6))
        if not result.success or not np.all(np.isfinite(result.x)):
            raise ValueError('Bicycle MPC solver did not converge: '+result.message)
        predicted_delta = delta+cumulative@result.x
        if np.any(abs(predicted_delta) > self.max_delta+1e-6):
            raise ValueError('Bicycle MPC solution violates source steering bounds')
        self.warm = np.r_[result.x[1:], result.x[-1]]
        self.solver_state = 'constrained solution'
        return float(delta+dt*result.x[0])

    def command(self, pose, measured, reference, dt):
        started = time.monotonic()
        dt = float(dt)
        if (len(pose) != 3 or len(measured) != 2 or not all(math.isfinite(v) for v in (*pose, *measured))
                or not math.isfinite(dt) or not .001 <= dt <= .2):
            raise ValueError('Fresh finite pose/velocity and bounded control interval required.')
        if self.along is None:
            self.along = reference.project(pose[0], pose[1], 0., back=0., ahead=reference.length)
        self.along = reference.project(pose[0], pose[1], self.along)
        v_actual, yaw_actual = measured
        actual_delta = math.atan(self.length*yaw_actual/v_actual) if abs(v_actual) > .03 else self.delta
        actual_delta = float(np.clip(actual_delta, -self.max_delta, self.max_delta))
        requested = self.pursuit(pose, reference, self.along)
        if self.law == 'bicycle_stanley':
            requested = self.stanley(pose, reference, self.along, v_actual)
            self.solver_state = 'Stanley/fixed-axle geometric correction'
        elif self.law == 'bicycle_mpc':
            try:
                requested = self.mpc(pose, reference, self.along, v_actual, actual_delta, dt, started+self.deadline)
            except (ValueError, TimeoutError) as exc:
                self.solver_state = 'bounded pursuit fallback: '+str(exc)
                if isinstance(exc, TimeoutError):
                    self.deadline_misses += 1
        else:
            self.solver_state = 'geometric pursuit'
        requested = float(np.clip(requested, -self.max_delta, self.max_delta))
        self.delta = float(np.clip(requested, actual_delta-self.delta_rate*dt, actual_delta+self.delta_rate*dt))
        target_v = reference.speed(self.along)
        # Ramp forward speed; a car never receives an in-place yaw command.
        self.velocity = float(np.clip(target_v, max(0., v_actual-self.acceleration*dt),
            min(self.speed, max(0., v_actual)+self.acceleration*dt)))
        self.compute_s = time.monotonic()-started
        return self.velocity, self.velocity*math.tan(self.delta)/self.length


def convex_footprint_blocked(grid, polygon):
    """Conservative cell-box/polygon SAT against lethal/unknown grid cells."""
    polygon = np.asarray(polygon, dtype=float)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3 or not np.all(np.isfinite(polygon)):
        raise ValueError('A finite authored convex footprint is required.')
    origin, res = np.asarray(grid.origin), grid.resolution
    low = np.floor((polygon.min(axis=0)-origin)/res).astype(int)
    high = np.floor((polygon.max(axis=0)-origin)/res).astype(int)
    if low[0] < 0 or low[1] < 0 or high[0] >= grid.width or high[1] >= grid.height:
        return True
    subset = grid.data[low[1]:high[1]+1, low[0]:high[0]+1]
    rows, cols = np.nonzero((subset >= 100)|(subset < 0))
    if not len(rows):
        return False
    centers = origin+res*np.c_[cols+low[0]+.5, rows+low[1]+.5]
    possible = np.ones(len(centers), dtype=bool)
    edges = np.roll(polygon, -1, axis=0)-polygon
    axes = np.vstack([np.eye(2), np.c_[-edges[:, 1], edges[:, 0]]])
    for axis in axes:
        bounds = polygon@axis
        projection = centers@axis
        radius = res*.5*np.sum(abs(axis))
        possible &= (projection+radius >= bounds.min()-1e-9)&(projection-radius <= bounds.max()+1e-9)
    return bool(np.any(possible))


def swept_footprint_blocked(grid, footprint, pose, command, acceleration, horizon=1.):
    """Sweep the entire polygon through stopping distance, including corners.

    Expand by half a cell plus the upper bound on vertex motion between
    samples, so thin lethal cells cannot hide between sampled footprints.
    """
    v, w = command
    curvature = w/v if abs(v) > 1e-9 else 0.
    duration = max(horizon, abs(v)/acceleration+.2)
    extent = float(np.max(np.linalg.norm(footprint, axis=1)))
    vertex_speed = abs(v)+abs(w)*extent
    interval = min(.05, grid.resolution/max(2*vertex_speed, .01))
    x, y, yaw = pose
    radius = vertex_speed*interval+grid.resolution*.5
    for t in np.arange(0., duration+interval, interval):
        if t:
            speed = max(0., abs(v)-acceleration*max(0., t-.2))*math.copysign(1., v)
            step = speed*interval
            if abs(curvature) > 1e-9:
                next_yaw = yaw+curvature*step
                x += (math.sin(next_yaw)-math.sin(yaw))/curvature
                y -= (math.cos(next_yaw)-math.cos(yaw))/curvature
                yaw = next_yaw
            else:
                x += step*math.cos(yaw)
                y += step*math.sin(yaw)
        # Minkowski-expand the convex footprint, conservatively using its
        # axis-aligned bounds in the body frame (authored cars are rectangles).
        lower, upper = np.min(footprint, axis=0)-radius, np.max(footprint, axis=0)+radius
        expanded = np.array([[lower[0], lower[1]], [upper[0], lower[1]], [upper[0], upper[1]], [lower[0], upper[1]]])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        if convex_footprint_blocked(grid, expanded@rotation.T+[x, y]):
            return True
    return False
