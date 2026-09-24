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


@dataclass
class RampDrive:
    """Increment command speed on each 100 ms tick, then ease to zero."""

    max_linear: float = 1.0
    max_angular: float = 2.0
    linear: float = 0.0
    angular: float = 0.0

    def step(self, linear_input, angular_input, linear_increment, angular_increment):
        linear_input = linear_input if math.isfinite(linear_input) else 0.0
        angular_input = angular_input if math.isfinite(angular_input) else 0.0
        linear_increment = linear_increment if math.isfinite(linear_increment) else 0.0
        angular_increment = angular_increment if math.isfinite(angular_increment) else 0.0
        linear_target = max(-1.0, min(1.0, linear_input)) * self.max_linear
        angular_target = max(-1.0, min(1.0, angular_input)) * self.max_angular
        self.linear = _approach(self.linear, linear_target,
                                max(0.0, min(0.1, linear_increment)))
        self.angular = _approach(self.angular, angular_target,
                                 max(0.0, min(0.2, angular_increment)))
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
                    self.axes[number] = value / 32767.0
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

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None
        self.path = ""
        self.axes = {0: 0.0, 1: 0.0}
