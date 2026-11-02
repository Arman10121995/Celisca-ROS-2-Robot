"""Observation-bound simulation battery model; never hardware battery telemetry."""
import math
import hashlib
from pathlib import Path


def qualified_source(member, certificate):
    """Qualification binds both the generated source and actual readback plugin."""
    if (not certificate.get('evidence') or not member.get('source_description_sha256')
            or certificate.get('source_description_sha256') != member['source_description_sha256']):
        return False
    try:
        library = Path(certificate['runtime_library'])
        return (library.is_absolute() and library.is_file()
                and hashlib.sha256(library.read_bytes()).hexdigest() == certificate['runtime_library_sha256'])
    except (KeyError, OSError, TypeError, ValueError):
        return False


class EnergyEstimate:
    def __init__(self, config, joints, source_verified=False):
        self.capacity = float(config['capacity_J'])
        self.joules = self.capacity * float(config['initial_fraction'])
        self.hotel = float(config['hotel_W'])
        self.efficiency = float(config['drive_efficiency'])
        self.charger = float(config['charge_W'])
        values = (self.capacity, self.joules, self.hotel, self.efficiency, self.charger)
        if (not all(map(math.isfinite, values)) or self.capacity <= 0
                or not 0 <= self.joules <= self.capacity or self.hotel < 0
                or not 0 < self.efficiency <= 1 or self.charger < 0):
            raise ValueError('Finite, bounded battery-model parameters required')
        self.joints = tuple(joints)
        if not self.joints or len(set(self.joints)) != len(self.joints):
            raise ValueError('Complete distinct actuator joint names required')
        self.verified = bool(source_verified)
        self.last = None
        self.at = -math.inf
        self.power = None
        self.charge = 0.
        self.coverage_lost = False
        self.integrated = False
        self.reason = 'Waiting for complete actuator feedback'

    def invalidate(self, reason):
        if self.integrated:
            self.coverage_lost = True
        self.last = None
        self.at = -math.inf
        self.power = None
        self.charge = 0.
        self.reason = reason

    def observe(self, names, velocities, efforts, stamp, clock, now, docked=False):
        # A missing joint/effort must not silently become a lower power draw.
        if (len(names) != len(set(names)) or len(names) != len(velocities)
                or len(names) != len(efforts) or not math.isfinite(stamp)
                or clock is None or not -.5 <= stamp - clock <= .2):
            self.invalidate('Incomplete, duplicate or stale actuator feedback')
            return
        rows = dict(zip(names, zip(velocities, efforts)))
        if not set(self.joints) <= rows.keys():
            self.invalidate('Required actuator feedback missing')
            return
        values = [rows[name] for name in self.joints]
        if not all(math.isfinite(value) for pair in values for value in pair):
            self.invalidate('Nonfinite actuator feedback')
            return
        if self.last is not None and stamp == self.last:
            return  # Duplicate messages do not refresh the observation lease.
        dt = None if self.last is None else stamp - self.last
        if dt is not None and not 0 < dt <= .5:
            self.invalidate('Actuator time discontinuity; energy coverage lost')
            dt = None
        power = self.hotel + sum(max(0., velocity * effort)
                                 for velocity, effort in values) / self.efficiency
        if not math.isfinite(power):
            self.invalidate('Nonfinite computed actuator power')
            return
        charge = self.charger if docked else 0.
        if dt is not None and self.verified and not self.coverage_lost:
            # Trapezoidal integration of actual sampled power, in simulated SI time.
            net = ((self.charge - self.power) + (charge - power)) / 2
            self.joules = min(self.capacity, max(0., self.joules + net * dt))
            self.integrated = True
        self.last, self.at, self.power, self.charge = stamp, now, power, charge
        self.reason = ('Energy coverage lost; restart session for a new authored initial state'
                       if self.coverage_lost else 'Actuator source not qualified'
                       if not self.verified else 'Observed simulation actuator power')

    def state(self, now):
        fresh = now - self.at < .6
        if not fresh and self.integrated:
            self.coverage_lost = True
            self.reason = 'Motor feedback lost; energy coverage unknown'
        valid = fresh and self.verified and self.integrated and not self.coverage_lost
        return dict(fraction=self.joules / self.capacity if valid else None,
                    power_W=self.power if fresh else None,
                    charging_W=self.charge if fresh else None,
                    source_verified=self.verified, coverage_complete=valid,
                    reason=self.reason, source='Authored battery model + sampled wheel effort/rate; no regeneration or hardware battery telemetry')
