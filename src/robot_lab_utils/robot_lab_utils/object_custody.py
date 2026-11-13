"""Measured contact/support and relative slip gates; never command an object."""
import math


class ObjectCustody:
    def __init__(self, dwell=1., slip=.01):
        self.dwell, self.slip = dwell, slip
        self.started = self.previous = self.reference = None

    def observe(self, stamp, relative, left_force, right_force, supported, speed, released=False):
        values = [stamp, *relative, left_force, right_force, speed]
        if len(relative) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError('Finite measured object/contact data required')
        if min(left_force, right_force, speed) < 0:
            raise ValueError('Force magnitudes and speed must be nonnegative')
        if self.previous is not None and stamp < self.previous:
            self.started = self.reference = None
        self.previous = stamp
        contact = left_force >= .1 and right_force >= .1
        valid = (supported and left_force < .1 and right_force < .1 and speed <= .02
                 if released else contact and not supported)
        if not valid:
            self.started = self.reference = None
            return False
        if self.reference is None or math.dist(relative, self.reference) > self.slip:
            self.reference = tuple(relative)
            self.started = stamp
        return stamp - self.started >= self.dwell
