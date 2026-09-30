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

``type: four_wheel_steer`` is a four-wheel-steered base: every wheel steers,
so a zero-turn and a crab translate are both reachable.  Its
``steering_mode`` selects the pattern - ``ackermann`` (rear axle opposite
phase, no scrub), ``in_phase`` (both axles the same way, so the base rotates
*and* slides), ``crab`` (all four wheels parallel, sideways translation with
no rotation) and ``pivot`` (pure zero-turn about the centre).  The root frame
sits at the geometric centre, which for the curvature-following patterns is
the point with no sideways velocity, exactly as for a car.

``type: mecanum`` is a four-wheel base on 45-degree rollers, so the four spins
span an independent (vx, vy, wz): it strafes and turns on the spot.  A Twist
carries no lateral component, so the bridges command ``vy = 0`` and what
/cmd_vel buys is the zero-turn; ``targets`` still accepts a real ``vy`` for a
holonomic caller.
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
    """Four-wheel steering: every wheel steers, so the base can crab.

    A car must turn while moving, which is why ``AckermannDrive`` cannot serve
    a zero-turn or a sideways command at all.  A four-wheel-steered base aims
    each wheel independently, so a (vx, wz) twist is reachable from rest.

    Steering patterns (``steering_mode``):

    ``ackermann``
        The rear axle steers in *opposite* phase to the front
        (``in_phase_4ws: false``): in a turn the wheels trace two circles
        about one centre, so nothing scrubs.  This is the default.
    ``in_phase``
        Both axles steer in the *same* phase.  The rear pair then travels
        round its arc the other way, so the base rotates *and* slides - a
        deliberate benchmark contrast, not a better turn.
    ``crab``
        All four wheels are held parallel at one common angle, so the base
        translates along its own heading with no rotation.  The angle follows
        the commanded curvature, so a curvature request still turns once the
        pattern is not crab.
    ``pivot``
        Wheels aimed to spin the base about its own centre with no net
        translation: the left pair drives forward, the right pair backward.
        Reached from a pure ``wz`` command.

    The root frame sits at the geometric centre, which for the two
    curvature-following patterns is the point with no sideways velocity - the
    same property ``AckermannDrive`` relies on, so /cmd_vel and odometry keep
    the same meaning across the wheeled robots.
    """
    kind = "four_wheel_steer"
    MODES = ("ackermann", "in_phase", "crab", "pivot")

    def __init__(self, config):
        self.steering_mode = str(config.get("steering_mode", "ackermann")).lower()
        if self.steering_mode not in self.MODES:
            raise ValueError("steering_mode must be one of %s, not %r"
                             % (", ".join(self.MODES), self.steering_mode))
        # Whether the two axles steer the same way.  Only the two
        # curvature-following patterns are affected.
        self.in_phase_4ws = bool(config.get("in_phase_4ws", False))
        if self.steering_mode == "in_phase":
            self.in_phase_4ws = True

        self.fl_steer = config.get("front_left_steer_joint", "")
        self.fr_steer = config.get("front_right_steer_joint", "")
        self.rl_steer = config.get("rear_left_steer_joint", "")
        self.rr_steer = config.get("rear_right_steer_joint", "")
        self.fl = config.get("front_left_wheel_joint", "")
        self.fr = config.get("front_right_wheel_joint", "")
        self.rl = config.get("rear_left_wheel_joint", "")
        self.rr = config.get("rear_right_wheel_joint", "")

        self.radius = _float(config, "wheel_radius", 0.05)
        self.track = _float(config, "wheel_separation", 0.32)     # left-right
        self.wheelbase = _float(config, "wheelbase", 0.32)       # front-rear
        self.max_steer = abs(_float(config, "max_steer", 0.785))
        self.max_steer_rate = abs(_float(config, "max_steer_rate", 2.5))
        self.max_speed = abs(_float(config, "max_speed", 1.0))
        self.max_accel = abs(_float(config, "max_accel", 1.5))
        if self.radius <= 0 or self.track <= 0 or self.wheelbase <= 0:
            raise ValueError("wheel_radius, wheel_separation and wheelbase "
                             "must be positive")
        self._angles = {joint: 0.0 for joint in self.steer_joints}
        self._speed = 0.0

    @property
    def steer_joints(self):
        return tuple(j for j in (self.fl_steer, self.fr_steer,
                                 self.rl_steer, self.rr_steer) if j)

    @property
    def wheel_joints(self):
        return [j for j in (self.fl, self.fr, self.rl, self.rr) if j]

    @property
    def max_curvature(self):
        return math.tan(self.max_steer) / (self.wheelbase / 2.0)

    @property
    def min_turning_radius(self):
        return 1.0 / self.max_curvature

    def reset(self):
        for joint in self._angles:
            self._angles[joint] = 0.0
        self._speed = 0.0

    def curvature(self):
        """Turn rate the current (rate-limited) front steering produces."""
        return math.tan(self._angles.get(self.fl_steer, 0.0)) / (self.wheelbase / 2.0)

    def _steering_goals(self, speed, wz):
        """The angle each wheel should hold to realise this command."""
        half_base = self.wheelbase / 2.0
        if abs(speed) > 1e-3:
            curvature = max(-self.max_curvature,
                            min(self.max_curvature, wz / speed))
        else:
            curvature = self.curvature()
        if self.steering_mode == "pivot":
            return {j: 0.0 for j in self.steer_joints}
        if self.steering_mode == "crab":
            angle = math.atan(curvature * half_base) if abs(speed) > 1e-3 else 0.0
            return {j: angle for j in self.steer_joints}
        front = math.atan(curvature * half_base)
        rear_sign = 1.0 if self.in_phase_4ws else -1.0
        return {self.fl_steer: front, self.fr_steer: front,
                self.rl_steer: rear_sign * front, self.rr_steer: rear_sign * front}

    def targets(self, vx, wz, dt=None, vy=0.0):
        """Joint targets for (vx, wz).

        *vy* is accepted and ignored: /cmd_vel is a Twist and carries no
        lateral component.  Crab steering is a property of the steering
        pattern, not of the commanded twist - a crab base moves sideways
        because all four wheels point the same way, which the pattern
        selects.  It is accepted so a bridge able to supply a lateral command
        (a holonomic planner, a teleop slider) can pass one through.
        """
        vx = max(-self.max_speed, min(self.max_speed, float(vx)))
        wz = float(wz)
        goals = self._steering_goals(vx, wz)
        if dt is None:
            speed = vx
        else:
            dv = self.max_accel * dt
            self._speed += max(-dv, min(dv, vx - self._speed))
            speed = self._speed
            step = self.max_steer_rate * dt
            for joint, angle in goals.items():
                current = self._angles.get(joint, 0.0)
                self._angles[joint] = current + max(-step, min(step, angle - current))
        if dt is None:
            for joint, angle in goals.items():
                self._angles[joint] = angle
        for joint in self._angles:
            self._angles[joint] = max(-self.max_steer,
                                      min(self.max_steer, self._angles[joint]))
        return self._wheel_targets(speed, wz)

    def _wheel_targets(self, speed, wz):
        half = self.track / 2.0
        half_base = self.wheelbase / 2.0
        velocity, position = {}, {}
        for joint in self.steer_joints:
            position[joint] = self._angles.get(joint, 0.0)

        # Pure yaw with no forward speed is a zero-turn.  Every pattern can do
        # it, because the wheels are steered rather than fixed: aim each one
        # along the tangent of its own circle about the centre and the base
        # spins without translating.  Only the pivot pattern needs a special
        # case (straight wheels, the two diagonals opposed).
        if self.steering_mode == "pivot":
            spin = (speed if abs(speed) > 1e-3 else self.max_speed) * 0.5
            for wheel, y in ((self.fl, half), (self.rl, half),
                             (self.fr, -half), (self.rr, -half)):
                if wheel:
                    velocity[wheel] = -spin * math.copysign(1.0, y) / self.radius
            return DriveTargets(velocity, position, (0.0, wz))

        if abs(speed) <= 1e-3 and abs(wz) > 1e-6:
            # Wheel at (x, y) must point along atan2(-x, y) for its rolling
            # direction to be tangent to its circle about the base centre.
            for wheel, joint, x, y in ((self.fl, self.fl_steer, half_base, half),
                                       (self.fr, self.fr_steer, half_base, -half),
                                       (self.rl, self.rl_steer, -half_base, half),
                                       (self.rr, self.rr_steer, -half_base, -half)):
                if not wheel:
                    continue
                position[joint] = math.atan2(-x, y)
                velocity[wheel] = -wz * y / self.radius
            return DriveTargets(velocity, position, (0.0, wz))

        # Rolling: each wheel's spin is the body twist of its own contact
        # point projected onto the direction that wheel is aimed along.
        for wheel, joint, x, y in ((self.fl, self.fl_steer, half_base, half),
                                   (self.fr, self.fr_steer, half_base, -half),
                                   (self.rl, self.rl_steer, -half_base, half),
                                   (self.rr, self.rr_steer, -half_base, -half)):
            if not wheel:
                continue
            angle = self._angles.get(joint, 0.0)
            vx_w = speed - wz * y
            vy_w = wz * x
            velocity[wheel] = (vx_w * math.cos(angle) + vy_w * math.sin(angle)) / self.radius
        velocity.pop("", None)
        position.pop("", None)
        return DriveTargets(velocity, position, (speed, speed * self.curvature()))

    def body_twist(self, wheel_rates):
        """(vx, wz) from the measured spin rates of the four wheels."""
        rates = list(wheel_rates or [])
        if len(rates) == 2:
            left, right = rates
            return (self.radius * (left + right) / 2.0,
                    self.radius * (right - left) / self.track)
        if len(rates) != 4:
            return (0.0, 0.0)
        fl, fr, rl, rr = rates
        vx = self.radius * (fl + fr + rl + rr) / 4.0
        wz = self.radius * (fr + rr - fl - rl) / (2.0 * self.track)
        return (vx, wz)


class MecanumDrive:
    """Mecanum (roller) wheels: full lateral motion from four fixed wheels.

    Each wheel's rollers sit at 45 degrees to the chassis, alternating between
    a right-handed and a left-handed pattern diagonally.  The rollers let a
    wheel slide sideways as well as roll, so four spins combine into an
    independent (vx, vy, wz): the base can strafe and turn on the spot, which
    neither a differential drive nor a car can do.

    A Twist has no lateral component, so ``targets`` takes ``vy`` explicitly.
    The bridges pass 0.0; a holonomic caller may pass a real one.  What
    /cmd_vel does buy is the zero-turn: a pure ``wz`` with no forward speed is
    realised by driving the two diagonals in opposite directions.

    The roller order is the standard "X" layout: front-left and rear-right
    share one diagonal sense, front-right and rear-left the other.
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

        # A roller angle other than 45 degrees changes how much of the
        # commanded speed each wheel can deliver, so the gains scale by it.
        gain = 1.0 / max(abs(math.sin(self.roller_angle)),
                         abs(math.cos(self.roller_angle)))
        half = self.track / 2.0
        half_base = self.wheelbase / 2.0
        forward, lateral = vx * gain, vy * gain
        velocity = {}
        for wheel, x, y, sgn_v, sgn_w in (
                (self.fl, half_base, half, -1.0, -1.0),
                (self.fr, half_base, -half, +1.0, -1.0),
                (self.rl, -half_base, half, +1.0, +1.0),
                (self.rr, -half_base, -half, -1.0, +1.0)):
            if not wheel:
                continue
            # Yaw lever arm: the signed distance from the centre to this
            # wheel measured perpendicular to the forward axis.
            lever = sgn_w * (abs(x) + abs(y))
            velocity[wheel] = (sgn_v * lateral + forward
                               - wz * lever) / self.radius
        return DriveTargets(velocity, {}, (vx, wz))

    @staticmethod
    def _approach(current, target, step):
        return current + max(-step, min(step, target - current))

    def body_twist(self, wheel_rates, vy=0.0):
        """(vx, vy, wz) from the measured spin rates of the four wheels.

        *vy* is not recoverable from spin rates alone - the forward/lateral
        split is not observable that way - so a caller that needs it must
        supply it or take it from the base's own odometry.
        """
        rates = list(wheel_rates or [])
        if len(rates) != 4:
            return (0.0, 0.0, 0.0)
        fl, fr, rl, rr = (rate * self.radius for rate in rates)
        vx = (fl + fr + rl + rr) / 4.0
        wz = (fr + rr - fl - rl) / (2.0 * self.track)
        return (vx, vy, wz)



def drive_from_config(config, left_wheel_joint="", right_wheel_joint="",
                      wheel_radius=0.033, wheel_separation=0.17):
    """The drive model for a robots.yaml ``drive:`` block (may be empty).

    The explicit arguments are the bridges' existing differential-drive
    parameters; the block's own values win over them.
    """
    config = dict(config or {})
    kind = str(config.get("type", "diff")).lower()
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
