"""Input aggregation and bounded teleoperation ramps for the GUI drive pad."""

from dataclasses import dataclass
import glob
import math
import os
import struct


def _approach(current, target, increment):
    if current < target:
        return min(target, current + increment)
    return max(target, current - increment)


#: Linux joystick axes arrive in a signed 16-bit field (-32768..32767), but
#: the value an axis rests at is not guaranteed to be 0.  A driver that
#: publishes its axes in the unsigned 0..65535 convention has those readings
#: clamped into that signed field, which pins an untouched stick at 32767 -
#: the largest representable value.  Any reading this far into the range
#: therefore identifies such an axis rather than a signed one being pushed
#: hard over.  See LinuxJoystick._normalise.
_UNSIGNED_THRESHOLD = 16384.0
_UNSIGNED_CENTRE = 32767.0
_SIGNED_HALF_RANGE = 32767.0


def limits_from_drive(profile):
    """Return (linear speed, angular speed, linear/angle delta per 100 ms).

    Car yaw limits follow the steering joint's geometric limit.  URDF joint
    velocity limits alone do not bound chassis speed, so the robot drive
    profile remains the source for body speed and acceleration.
    """
    speed = max(0.0, float(profile.get("max_speed", 0.5)))
    acceleration = max(0.0, float(profile.get("max_accel", 1.0)))
    if profile.get("type") in ("ackermann", "car", "car_like", "four_wheel_steer",
                               "4ws", "four_wheel_steering", "swerve"):
        wheelbase = max(1e-6, float(profile.get("wheelbase", 0.32)))
        curvature = math.tan(float(profile.get("max_steer", 0.58))) / wheelbase
        angular_speed = speed * curvature
        angular_acceleration = acceleration * curvature
    elif profile.get("type") in ("mecanum", "roller", "omni"):
        # Mecanum pivots on the spot, so its yaw limit is its own rather than
        # something derived from a turning radius.
        angular_speed = max(0.0, float(profile.get("max_angular_speed", 1.0)))
        angular_acceleration = max(0.0, float(profile.get("max_angular_accel", 2.0)))
    else:
        angular_speed = max(0.0, float(profile.get("max_angular_speed", 1.0)))
        angular_acceleration = max(0.0, float(profile.get("max_angular_accel", 2.0)))
    return speed, angular_speed, acceleration * 0.1, angular_acceleration * 0.1


@dataclass
class RampDrive:
    """Increment command speed on each 100 ms tick, then ease to zero."""

    max_linear: float = 1.0
    max_angular: float = 2.0
    min_linear: float = -1.0
    min_angular: float = -2.0
    linear: float = 0.0
    angular: float = 0.0

    def step(self, linear_input, angular_input, linear_increment, angular_increment,
             linear_decrement=None, angular_decrement=None):
        linear_input = linear_input if math.isfinite(linear_input) else 0.0
        angular_input = angular_input if math.isfinite(angular_input) else 0.0
        linear_increment = linear_increment if math.isfinite(linear_increment) else 0.0
        angular_increment = angular_increment if math.isfinite(angular_increment) else 0.0
        linear_input = max(-1.0, min(1.0, linear_input))
        angular_input = max(-1.0, min(1.0, angular_input))
        linear_target = linear_input * (self.max_linear if linear_input >= 0 else -self.min_linear)
        angular_target = angular_input * (self.max_angular if angular_input >= 0 else -self.min_angular)
        linear_decrement = linear_increment if linear_decrement is None else linear_decrement
        angular_decrement = angular_increment if angular_decrement is None else angular_decrement
        linear_step = (linear_decrement if self.linear * linear_target < 0
                       or abs(linear_target) < abs(self.linear) else linear_increment)
        angular_step = (angular_decrement if self.angular * angular_target < 0
                        or abs(angular_target) < abs(self.angular) else angular_increment)
        self.linear = _approach(self.linear, linear_target,
                                max(0.0, linear_step) if math.isfinite(linear_step) else 0.0)
        self.angular = _approach(self.angular, angular_target,
                                 max(0.0, angular_step) if math.isfinite(angular_step) else 0.0)
        return self.linear, self.angular

    def stop(self):
        self.linear = self.angular = 0.0
        return 0.0, 0.0


class LinuxJoystick:
    """Nonblocking Linux joystick axes; requires no optional Python package."""

    _EVENT = struct.Struct("IhBB")

    def __init__(self):
        self.fd = None
        self.path = ""
        self.axes = {0: 0.0, 1: 0.0}

    def poll(self):
        if self.fd is None:
            for path in sorted(glob.glob("/dev/input/js*")):
                try:
                    self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                    self.path = path
                    break
                except OSError:
                    continue
        if self.fd is None:
            return 0.0, 0.0
        try:
            while True:
                data = os.read(self.fd, self._EVENT.size)
                if len(data) != self._EVENT.size:
                    self.close()
                    return 0.0, 0.0
                _time, value, kind, number = self._EVENT.unpack(data)
                if kind & 0x02 and number in self.axes:
                    self._normalise(number, value)
        except BlockingIOError:
            pass
        except OSError:
            self.close()
            return 0.0, 0.0
        # Standard left stick: up is negative Y; right is positive X.
        linear = -self.axes[1]
        angular = -self.axes[0]
        return tuple(0.0 if abs(value) < 0.15 else max(-1.0, min(1.0, value))
                     for value in (linear, angular))

    def _normalise(self, number, value):
        """Map a raw axis report to -1..1, with "at rest" as exactly 0.

        A stick left alone is *not* guaranteed to report 0.  Drivers that
        publish their axes in the unsigned 0..65535 convention have those
        readings clamped into the signed 16-bit js_event field, so an
        untouched stick sits at 32767.  Dividing that by 32767 gives a
        full-scale command, so merely *enabling* the joystick teleops the
        robot at full speed into a circle - the stick was never touched.

        The two conventions are told apart by where the reading sits.  A
        signed axis at rest is near 0 and reaches either extreme as it is
        pushed; a clamped one cannot exceed 32767 and rests at it.  Deciding
        per report rather than calibrating over time also covers drivers that
        emit each axis exactly once and then stay silent.
        """
        raw = float(value)
        if raw > _UNSIGNED_THRESHOLD:
            # Clamped axis: 32767 is where an untouched stick sits.
            scaled = (raw - _UNSIGNED_CENTRE) / _SIGNED_HALF_RANGE
        else:
            # Signed axis: 0 is already the rest position.
            scaled = raw / _SIGNED_HALF_RANGE
        self.axes[number] = max(-1.0, min(1.0, scaled))
        return self.axes[number]

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        self.path = ""
        self.axes = {0: 0.0, 1: 0.0}
