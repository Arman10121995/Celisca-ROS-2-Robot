"""Wheel targets for a /cmd_vel twist, shared by every simulator bridge.

A robot's ``drive:`` block in robot_lab_robots/config/robots.yaml picks the
model.  Without a ``type`` (or with ``type: diff``) it is a differential
drive: two wheel joints, ``wheel_radius`` and ``wheel_separation``.

``type: ackermann`` is a four-wheel car-like base: one axle steers, the
other is fixed.  The robot's root frame sits at the centre of the fixed
axle, which is the only point of a car whose velocity has no sideways
component, so /cmd_vel (vx, wz) and the odometry mean the same thing as for
a differential drive.  Keys:

* ``steer_axle`` - ``front`` (an ordinary car) or ``rear`` (rear-wheel
  steering, as on forklifts and some mobile platforms).  With rear steering
  the tail swings out when turning.
* ``steering_geometry`` - how the two steered wheels share the turn:
  ``ackermann`` (the inner wheel steers more, so all wheel axes meet at one
  turning centre and nothing scrubs), ``anti_ackermann`` (the outer wheel
  steers more, used by some race cars; the wheels scrub at low speed) or
  ``parallel`` (both wheels at the same angle).
* ``left_steer_joint`` / ``right_steer_joint`` - steering joints.
* ``steered_left_wheel_joint`` / ``steered_right_wheel_joint`` - the wheels
  on the steered axle.
* ``left_wheel_joint`` / ``right_wheel_joint`` - the wheels on the fixed
  axle (their spin gives the odometry, exactly as for a differential drive).
* ``wheel_radius``, ``wheel_separation`` (track width), ``wheelbase``.
* ``max_steer`` [rad], ``max_steer_rate`` [rad/s], ``max_speed`` [m/s],
  ``max_accel`` [m/s^2] - the limits the targets respect.

All four wheels are driven, each at the speed that rolls it without slip
along its own arc.  A car cannot turn on the spot: a twist with no forward
speed stops the wheels and keeps the steering where it is.

``type: four_wheel_steer`` gives each wheel its own steering axis. Opposite
phase/Ackermann follows a turning circle; crab and in-phase translate on
parallel wheels; pivot aims all wheels along their local tangents. The shipped
joints reach +/-90 degrees, so a lateral Twist can produce true crab travel.

``type: mecanum`` uses physical passive 45-degree rollers. Both mecanum and
4WS crab accept Twist.linear.y in every simulator bridge and the GUI.

"""

import math


def _float(config, key, default):
    try:
        return float(config.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def _optional_float(config, key):
    """A limit that may be absent, meaning "do not clamp"; None if so."""
    value = config.get(key)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


class DriveTargets:
    """Joint targets for one control step: wheel spin rates and steer angles."""

    def __init__(self, velocity=None, position=None, twist=(0.0, 0.0)):
        self.velocity = dict(velocity or {})   # joint -> rad/s
        self.position = dict(position or {})   # joint -> rad
        self.twist = twist  # (vx, wz) the targets realize

    def __repr__(self):
        return "DriveTargets(velocity=%r, position=%r)" % (self.velocity, self.position)


class DiffDrive:
    kind = "diff"

    def __init__(self, left_wheel_joint, right_wheel_joint, wheel_radius,
                 wheel_separation, max_speed=None, max_accel=None,
                 max_angular_speed=None, max_angular_accel=None):
        self.left = left_wheel_joint
        self.right = right_wheel_joint
        self.radius = float(wheel_radius)
        self.track = float(wheel_separation)
        if self.radius <= 0 or self.track <= 0:
            raise ValueError("wheel_radius and wheel_separation must be positive")
        self.max_speed = self._limit(max_speed)
        self.max_accel = self._limit(max_accel)
        self.max_angular_speed = self._limit(max_angular_speed)
        self.max_angular_accel = self._limit(max_angular_accel)
        self._vx = self._wz = 0.0

    @staticmethod
    def _limit(value):
        if value is None:
            return None
        value = float(value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError("drive limits must be finite and positive")
        return value

    def reset(self):
        self._vx = self._wz = 0.0

    @property
    def wheel_joints(self):
        return [j for j in (self.left, self.right) if j]

    steer_joints = ()

    def targets(self, vx, wz, dt=None):
        vx, wz = float(vx), float(wz)
        if not math.isfinite(vx):
            vx = 0.0
        if not math.isfinite(wz):
            wz = 0.0
        if self.max_speed is not None:
            vx = max(-self.max_speed, min(self.max_speed, vx))
        if self.max_angular_speed is not None:
            wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))
        if dt is not None and dt > 0:
            if self.max_accel is not None:
                step = self.max_accel * dt
                vx = self._vx + max(-step, min(step, vx - self._vx))
            if self.max_angular_accel is not None:
                step = self.max_angular_accel * dt
                wz = self._wz + max(-step, min(step, wz - self._wz))
        self._vx, self._wz = vx, wz
        left = (vx - wz * self.track / 2.0) / self.radius
        right = (vx + wz * self.track / 2.0) / self.radius
        return DriveTargets({self.left: left, self.right: right}, {}, (vx, wz))

    def body_twist(self, left_rate, right_rate):
        """(vx, wz) from the measured spin of the two wheels."""
        return (self.radius * (left_rate + right_rate) / 2.0,
                self.radius * (right_rate - left_rate) / self.track)


class AckermannDrive:
    kind = "ackermann"
    GEOMETRIES = ("ackermann", "anti_ackermann", "parallel")

    def __init__(self, config):
        self.steer_axle = str(config.get("steer_axle", "front")).lower()
        if self.steer_axle not in ("front", "rear"):
            raise ValueError("steer_axle must be 'front' or 'rear', not %r" % self.steer_axle)
        self.geometry = str(config.get("steering_geometry", "ackermann")).lower()
        if self.geometry not in self.GEOMETRIES:
            raise ValueError("steering_geometry must be one of %s, not %r"
                             % (", ".join(self.GEOMETRIES), self.geometry))
        self.left_steer = config.get("left_steer_joint", "")
        self.right_steer = config.get("right_steer_joint", "")
        self.steered_left = config.get("steered_left_wheel_joint", "")
        self.steered_right = config.get("steered_right_wheel_joint", "")
        self.left = config.get("left_wheel_joint", "")
        self.right = config.get("right_wheel_joint", "")
        self.radius = _float(config, "wheel_radius", 0.05)
        self.track = _float(config, "wheel_separation", 0.28)
        self.wheelbase = _float(config, "wheelbase", 0.32)
        self.max_steer = abs(_float(config, "max_steer", 0.5))
        self.max_steer_rate = abs(_float(config, "max_steer_rate", 2.0))
        self.max_speed = abs(_float(config, "max_speed", 1.0))
        self.max_accel = abs(_float(config, "max_accel", 1.5))
        if self.radius <= 0 or self.track <= 0 or self.wheelbase <= 0:
            raise ValueError("wheel_radius, wheel_separation and wheelbase must be positive")
        # Signed x of the steered axle in the fixed-axle frame.
        self.steer_x = self.wheelbase if self.steer_axle == "front" else -self.wheelbase
        self._steer = 0.0   # virtual centre-wheel steer angle, rate limited
        self._speed = 0.0   # forward speed, acceleration limited

    @property
    def wheel_joints(self):
        return [j for j in (self.left, self.right, self.steered_left, self.steered_right) if j]

    @property
    def steer_joints(self):
        return tuple(j for j in (self.left_steer, self.right_steer) if j)

    @property
    def max_curvature(self):
        return math.tan(self.max_steer) / self.wheelbase

    @property
    def min_turning_radius(self):
        """Turning radius of the fixed-axle centre at full lock [m]."""
        return 1.0 / self.max_curvature

    def reset(self):
        self._steer = 0.0
        self._speed = 0.0

    def curvature(self):
        """Curvature the current (rate-limited) steering produces [1/m]."""
        return math.tan(self._steer) / self.steer_x

    def targets(self, vx, wz, dt=None):
        """Joint targets for (vx, wz); with *dt* the steer rate and
        acceleration limits apply (successive calls slew toward the command)."""
        vx = max(-self.max_speed, min(self.max_speed, float(vx)))
        wz = float(wz)
        if abs(vx) > 1e-3:
            curvature = wz / vx
            limit = self.max_curvature
            curvature = max(-limit, min(limit, curvature))
            steer_goal = math.atan(self.steer_x * curvature)
        else:
            steer_goal = self._steer  # cannot turn on the spot: hold the wheels
            vx = 0.0
        if dt is None:
            self._steer = steer_goal
            self._speed = vx
        else:
            step = self.max_steer_rate * dt
            self._steer += max(-step, min(step, steer_goal - self._steer))
            dv = self.max_accel * dt
            self._speed += max(-dv, min(dv, vx - self._speed))
        self._steer = max(-self.max_steer, min(self.max_steer, self._steer))
        return self._wheel_targets(self._speed, self.curvature())

    def _wheel_targets(self, speed, curvature):
        half = self.track / 2.0
        velocity, position = {}, {}
        # Fixed axle: a differential drive along the arc.
        velocity[self.left] = speed * (1.0 - curvature * half) / self.radius
        velocity[self.right] = speed * (1.0 + curvature * half) / self.radius
        # Steered axle: each wheel tangent to its own circle about the
        # turning centre (0, 1/curvature).
        angles = {}
        for joint, wheel, y in ((self.left_steer, self.steered_left, half),
                                (self.right_steer, self.steered_right, -half)):
            lateral = 1.0 - curvature * y
            angle = math.atan2(curvature * self.steer_x, lateral)
            if abs(angle) > math.pi / 2:
                angle -= math.copysign(math.pi, angle)
            angles[joint] = angle
            velocity[wheel] = speed * math.hypot(lateral, curvature * self.steer_x) \
                * (1.0 if lateral >= 0 else -1.0) / self.radius
        left_angle, right_angle = angles[self.left_steer], angles[self.right_steer]
        if self.geometry == "anti_ackermann":
            left_angle, right_angle = right_angle, left_angle
        elif self.geometry == "parallel":
            left_angle = right_angle = self._steer
        position[self.left_steer] = left_angle
        position[self.right_steer] = right_angle
        velocity.pop("", None)
        position.pop("", None)
        return DriveTargets(velocity, position, (speed, speed * curvature))

    def body_twist(self, left_rate, right_rate):
        """(vx, wz) from the fixed-axle wheel spin rates."""
        return (self.radius * (left_rate + right_rate) / 2.0,
                self.radius * (right_rate - left_rate) / self.track)


class FourWheelSteerDrive:
    """Four independently steered and driven wheels, at the chassis centre.

    Ackermann uses opposite-phase axles and the individual inner/outer wheel
    angles. Pivot also uses contact-point tangents, including for pure yaw;
    straight wheels counter-rotating would skid rather than pivot. Crab uses
    parallel wheels for (vx, vy) translation. In-phase provides the same
    translation, with a forward/yaw request interpreted as a common steering
    angle atan(wz * half_wheelbase / vx) when vy is absent. Parallel wheels
    cannot translate and rotate simultaneously, so these two modes realize
    zero yaw while translating. A pure yaw request temporarily uses the pivot
    pattern in every mode. Steering is held on stop and wheels wait for a
    large steering change before driving, avoiding scrub at a mode transition.
    """
    kind = "four_wheel_steer"
    MODES = ("ackermann", "in_phase", "crab", "pivot")

    def __init__(self, config):
        self.steering_mode = str(config.get("steering_mode", "ackermann")).lower()
        if self.steering_mode not in self.MODES:
            raise ValueError("steering_mode must be one of %s, not %r"
                             % (", ".join(self.MODES), self.steering_mode))
        self.in_phase_4ws = bool(config.get("in_phase_4ws", False))
        if self.in_phase_4ws and self.steering_mode == "ackermann":
            self.steering_mode = "in_phase"
        self.fl_steer = config.get("front_left_steer_joint", "")
        self.fr_steer = config.get("front_right_steer_joint", "")
        self.rl_steer = config.get("rear_left_steer_joint", "")
        self.rr_steer = config.get("rear_right_steer_joint", "")
        self.fl = config.get("front_left_wheel_joint", "")
        self.fr = config.get("front_right_wheel_joint", "")
        self.rl = config.get("rear_left_wheel_joint", "")
        self.rr = config.get("rear_right_wheel_joint", "")
        self.radius = _float(config, "wheel_radius", 0.05)
        self.track = _float(config, "wheel_separation", 0.32)
        self.wheelbase = _float(config, "wheelbase", 0.32)
        self.max_steer = abs(_float(config, "max_steer", math.pi / 2))
        self.max_steer_rate = abs(_float(config, "max_steer_rate", 2.5))
        self.max_speed = abs(_float(config, "max_speed", 1.0))
        self.max_accel = abs(_float(config, "max_accel", 1.5))
        self.max_angular_speed = abs(_float(config, "max_angular_speed", 2.0))
        self.max_angular_accel = abs(_float(config, "max_angular_accel", 4.0))
        limits = (self.radius, self.track, self.wheelbase, self.max_steer,
                  self.max_steer_rate, self.max_speed, self.max_accel,
                  self.max_angular_speed, self.max_angular_accel)
        if not all(math.isfinite(v) and v > 0 for v in limits):
            raise ValueError("4WS dimensions and limits must be finite and positive")
        if self.max_steer > math.pi / 2 + 1e-9:
            raise ValueError("max_steer must be at most pi/2")
        self._angles = {joint: 0.0 for joint in self.steer_joints}
        self._speed = self._vy = self._wz = 0.0

    @property
    def steer_joints(self):
        return tuple(j for j in (self.fl_steer, self.fr_steer,
                                 self.rl_steer, self.rr_steer) if j)

    @property
    def wheel_joints(self):
        return [j for j in (self.fl, self.fr, self.rl, self.rr) if j]

    @property
    def max_curvature(self):
        # Inner front wheel reaches its limit first, not a virtual centre wheel.
        t = math.tan(self.max_steer)
        return t / (self.wheelbase / 2 + self.track / 2 * t)

    @property
    def min_turning_radius(self):
        return 1.0 / self.max_curvature

    def reset(self):
        # Hold the physical steering on a watchdog stop; only wheel motion resets.
        self._speed = self._vy = self._wz = 0.0

    def curvature(self):
        angle = self._angles.get(self.fl_steer, 0.0)
        t = math.tan(angle)
        denominator = self.wheelbase / 2 + self.track / 2 * t
        return t / denominator if abs(denominator) > 1e-12 else math.copysign(math.inf, t)

    def _contacts(self):
        x, y = self.wheelbase / 2, self.track / 2
        return ((self.fl, self.fl_steer, x, y),
                (self.fr, self.fr_steer, x, -y),
                (self.rl, self.rl_steer, -x, y),
                (self.rr, self.rr_steer, -x, -y))

    def targets(self, vx, wz, dt=None, vy=0.0):
        values = [float(v) for v in (vx, vy, wz)]
        vx, vy, wz = [v if math.isfinite(v) else 0.0 for v in values]
        if self.steering_mode not in ("crab", "in_phase"):
            vy = 0.0
        translating = math.hypot(vx, vy) > 1e-6
        if translating and self.steering_mode in ("crab", "in_phase"):
            if self.steering_mode == "in_phase" and abs(vy) < 1e-6:
                vy = wz * self.wheelbase / 2
            wz = 0.0
        norm = math.hypot(vx, vy)
        if norm > self.max_speed:
            vx, vy = vx * self.max_speed / norm, vy * self.max_speed / norm
        wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))
        if self.steering_mode == "ackermann" and abs(vx) > 1e-6:
            limit = abs(vx) * self.max_curvature
            wz = max(-limit, min(limit, wz))
        if dt is not None and dt > 0:
            dx, dy = vx - self._speed, vy - self._vy
            delta = math.hypot(dx, dy)
            scale = min(1.0, self.max_accel * dt / delta) if delta else 1.0
            vx, vy = self._speed + dx * scale, self._vy + dy * scale
            dw = self.max_angular_accel * dt
            wz = self._wz + max(-dw, min(dw, wz - self._wz))
        self._speed, self._vy, self._wz = vx, vy, wz
        goals = {}
        for wheel, joint, x, y in self._contacts():
            u, v = vx - wz * y, vy + wz * x
            angle = self._angles.get(joint, 0.0)
            if math.hypot(u, v) > 1e-8:
                angle = math.atan2(v, u)
                if angle > math.pi / 2:
                    angle -= math.pi
                elif angle < -math.pi / 2:
                    angle += math.pi
            goals[joint] = angle
            limited = max(-self.max_steer, min(self.max_steer, angle))
            if dt is not None and dt > 0:
                step = self.max_steer_rate * dt
                current = self._angles.get(joint, 0.0)
                limited = current + max(-step, min(step, limited - current))
            self._angles[joint] = limited
        # Zero wheel speeds until the requested rolling directions are reachable.
        aligned = all(abs(goals[j] - self._angles[j]) < 0.05 for j in goals)
        velocity = {}
        for wheel, joint, x, y in self._contacts():
            if wheel:
                angle = self._angles[joint]
                velocity[wheel] = (((vx - wz * y) * math.cos(angle)
                                    + (vy + wz * x) * math.sin(angle)) / self.radius
                                   if aligned else 0.0)
        position = {j: self._angles[j] for j in self.steer_joints}
        result = DriveTargets(velocity, position, (vx, wz) if aligned else (0.0, 0.0))
        result.lateral = vy if aligned else 0.0
        return result

    def body_twist(self, wheel_rates):
        """Legacy two-axis API; use measured_twist with measured steering."""
        rates = list(wheel_rates or [])
        if len(rates) == 2:
            left, right = rates
            return (self.radius * (left + right) / 2,
                    self.radius * (right - left) / self.track)
        vx, _, wz = self.measured_twist(rates, self._angles)
        return vx, wz

    def measured_twist(self, wheel_rates, steer_angles):
        """Least-squares rolling and no-side-slip constraints at each wheel.

        Rolling-only equations lose the transverse velocity with parallel
        steering. Adding the lateral contact constraint makes straight and
        crab odometry observable. This is still wheel odometry, not body truth.
        """
        rates = list(wheel_rates or [])
        if len(rates) != 4 or not all(math.isfinite(v) for v in rates):
            return (0.0, 0.0, 0.0)
        normal = [[0.0] * 4 for _ in range(3)]
        for rate, (_, joint, x, y) in zip(rates, self._contacts()):
            angle = float(steer_angles.get(joint, 0.0))
            if not math.isfinite(angle):
                return (0.0, 0.0, 0.0)
            c, s = math.cos(angle), math.sin(angle)
            equations = (((c, s, -y * c + x * s), self.radius * rate),
                         ((-s, c, y * s + x * c), 0.0))
            for coefficients, value in equations:
                for i in range(3):
                    normal[i][3] += coefficients[i] * value
                    for j in range(3):
                        normal[i][j] += coefficients[i] * coefficients[j]
        for pivot in range(3):
            best = max(range(pivot, 3), key=lambda i: abs(normal[i][pivot]))
            normal[pivot], normal[best] = normal[best], normal[pivot]
            factor = normal[pivot][pivot]
            if abs(factor) < 1e-12:
                return (0.0, 0.0, 0.0)
            for j in range(pivot, 4):
                normal[pivot][j] /= factor
            for row in range(3):
                if row == pivot:
                    continue
                scale = normal[row][pivot]
                for j in range(pivot, 4):
                    normal[row][j] -= scale * normal[pivot][j]
        return tuple(normal[i][3] for i in range(3))


class MecanumDrive:
    """45-degree X-layout mecanum inverse and measured-wheel kinematics.

    The shipped plant uses MIT FUJI passive rollers; command linear.y reaches
    all bridges and GUI strafe controls. Physics truth, rather than this ideal
    solver, establishes lateral travel and wheel slip on each backend.
    """
    kind = "mecanum"

    def __init__(self, config):
        self.fl = config.get("front_left_wheel_joint", "")
        self.fr = config.get("front_right_wheel_joint", "")
        self.rl = config.get("rear_left_wheel_joint", "")
        self.rr = config.get("rear_right_wheel_joint", "")
        self.radius = _float(config, "wheel_radius", 0.05)
        self.track = _float(config, "wheel_separation", 0.32)     # left-right
        self.wheelbase = _float(config, "wheelbase", 0.32)       # front-rear
        self.roller_angle = _float(config, "roller_angle", math.pi / 4.0)
        if abs(self.roller_angle - math.pi / 4.0) > 1e-6:
            raise ValueError("mecanum drive currently models 45-degree rollers only")
        self.max_speed = abs(_float(config, "max_speed", 1.0))
        self.max_accel = abs(_float(config, "max_accel", 1.5))
        # Unlike the other limits these are genuinely optional: a mecanum base
        # has no natural yaw bound, so an absent value means "do not clamp".
        self.max_angular_speed = _optional_float(config, "max_angular_speed")
        self.max_angular_accel = _optional_float(config, "max_angular_accel")
        if self.radius <= 0 or self.track <= 0 or self.wheelbase <= 0:
            raise ValueError("wheel_radius, wheel_separation and wheelbase "
                             "must be positive")
        self._vx = self._vy = self._wz = 0.0

    steer_joints = ()

    @property
    def wheel_joints(self):
        return [j for j in (self.fl, self.fr, self.rl, self.rr) if j]

    def reset(self):
        self._vx = self._vy = self._wz = 0.0

    def targets(self, vx, wz, dt=None, vy=0.0):
        """Joint targets for a body twist (vx, vy, wz).

        A mecanum wheel's contact patch pushes along one horizontal direction
        only - its rolling axis - so the four spins have to be *solved*
        together rather than each read off the twist independently.  Doing it
        per wheel is the classic mistake and yields the same pattern for
        forward and sideways, which can only drive in a straight line.

        This is the closed-form inverse for the "X" layout at ``roller_angle``
        (45 degrees is the usual mounting).  The forward and lateral terms
        enter with opposite signs on the two diagonals, and the yaw term
        scales with each wheel's lever arm about the centre - which is what
        makes (vx, vy, wz) three independent commands.
        """
        vx, vy, wz = float(vx), float(vy), float(wz)
        if not math.isfinite(vx):
            vx = 0.0
        if not math.isfinite(vy):
            vy = 0.0
        if not math.isfinite(wz):
            wz = 0.0
        vx = max(-self.max_speed, min(self.max_speed, vx))
        vy = max(-self.max_speed, min(self.max_speed, vy))
        if self.max_angular_speed is not None:
            wz = max(-self.max_angular_speed,
                     min(self.max_angular_speed, wz))
        if dt is not None and dt > 0:
            if self.max_accel:
                step = self.max_accel * dt
                vx = self._approach(self._vx, vx, step)
                vy = self._approach(self._vy, vy, step)
            if self.max_angular_accel:
                step = self.max_angular_accel * dt
                wz = self._approach(self._wz, wz, step)
        self._vx, self._vy, self._wz = vx, vy, wz

        half = self.track / 2.0
        half_base = self.wheelbase / 2.0
        velocity = {}
        for wheel, x, y, sgn_v, sgn_w in (
                (self.fl, half_base, half, -1.0, +1.0),
                (self.fr, half_base, -half, +1.0, -1.0),
                (self.rl, -half_base, half, +1.0, +1.0),
                (self.rr, -half_base, -half, -1.0, -1.0)):
            if not wheel:
                continue
            # Yaw lever arm: the signed distance from the centre to this
            # wheel measured perpendicular to the forward axis.
            lever = sgn_w * (abs(x) + abs(y))
            velocity[wheel] = (sgn_v * vy + vx
                               - wz * lever) / self.radius
        return DriveTargets(velocity, {}, (vx, wz))

    @staticmethod
    def _approach(current, target, step):
        return current + max(-step, min(step, target - current))

    def body_twist(self, wheel_rates, vy=None):
        """Ideal (vx, vy, wz) from four wheel rates; optionally override vy."""
        rates = list(wheel_rates or [])
        if len(rates) != 4:
            return (0.0, 0.0, 0.0)
        fl, fr, rl, rr = (rate * self.radius for rate in rates)
        vx = (fl + fr + rl + rr) / 4.0
        lateral = (-fl + fr + rl - rr) / 4.0
        lever = (self.track + self.wheelbase) / 2.0
        wz = (-fl + fr - rl + rr) / (4.0 * lever)
        return (vx, lateral if vy is None else vy, wz)



def drive_from_config(config, left_wheel_joint="", right_wheel_joint="",
                      wheel_radius=0.033, wheel_separation=0.17):
    """The drive model for a robots.yaml ``drive:`` block (may be empty).

    The explicit arguments are the bridges' existing differential-drive
    parameters; the block's own values win over them.
    """
    config = dict(config or {})
    kind = str(config.get("type", "diff")).lower()
    if kind == 'skid_steer':
        from .skid_steer import SkidSteerDrive
        return SkidSteerDrive(config)
    if kind in ("ackermann", "car", "car_like"):
        for key, value in (("left_wheel_joint", left_wheel_joint),
                           ("right_wheel_joint", right_wheel_joint),
                           ("wheel_radius", wheel_radius),
                           ("wheel_separation", wheel_separation)):
            config.setdefault(key, value)
        return AckermannDrive(config)
    if kind in ("four_wheel_steer", "4ws", "four_wheel_steering", "swerve"):
        return FourWheelSteerDrive(config)
    if kind in ("mecanum", "roller", "omni"):
        return MecanumDrive(config)
    if kind not in ("diff", "differential", "diff_drive"):
        raise ValueError("unknown drive type %r" % kind)
    return DiffDrive(config.get("left_wheel_joint", left_wheel_joint),
                     config.get("right_wheel_joint", right_wheel_joint),
                     _float(config, "wheel_radius", wheel_radius),
                     _float(config, "wheel_separation", wheel_separation),
                     max_speed=config.get("max_speed"),
                     max_accel=config.get("max_accel"),
                     max_angular_speed=config.get("max_angular_speed"),
                     max_angular_accel=config.get("max_angular_accel"))


def parse_drive_config(text):
    """The ``drive_config`` bridge parameter (JSON) as a dict; {} when empty."""
    import json
    text = str(text or "").strip()
    if not text:
        return {}
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError("drive_config must be a JSON object")
    return value
