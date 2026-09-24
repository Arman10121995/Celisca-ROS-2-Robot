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
"""

import math


def _float(config, key, default):
    try:
        return float(config.get(key, default))
    except (TypeError, ValueError):
        return float(default)


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
