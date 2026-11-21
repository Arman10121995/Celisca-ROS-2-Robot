"""Backend-neutral track drive (R10.7: the tracked vehicle beyond Gazebo).

Gazebo drives the tracks with moving contact surfaces (TrackController).
MuJoCo and PyBullet have none, so the backend gives the track geometry
normal-only contacts and this module supplies the tangential contact force
instead: at each track contact the robot's point velocity is driven towards
the belt velocity, bounded by Coulomb friction separately
along (``mu_along``) and across (``mu_lateral``) the belt, as Gazebo's
anisotropic track friction (mu 0.7 across the belt). Along the belt the
target is that track's speed. Across it the target is the commanded
rotation about the hull origin, as Gazebo's TrackController sets each
contact's surface motion about the vehicle's centre of rotation. Holding the
belt still across instead made the rear-loaded hull pivot about its centre
of mass, 0.13 m off its origin in a 1.6 rad turn, while the odometry
reported a turn in place (2026-11-21). For the same reason a contact off the
track line (the flippers, 0.272 m out against the belts' 0.2 m) gets the
turn's speed at its own offset; one belt speed for both made them fight in
a turn and still left 0.07 m of drift. A stopped track still grips.

The command path matches the Gazebo plugins this replaces: a CommandWatchdog
(stale input becomes zero), velocity caps and acceleration limits
(TrackedVehicle), and odometry integrated from the commanded track speeds,
as TrackedVehicle's own dead reckoning.

Gazebo's belts barely slip lengthwise (mu 150 along the belt in an implicit
solver). An explicit velocity servo needs a slip to push, and a skid turn
needs a large push: at gain 50/s the MuJoCo vehicle turned at 0.37 of its
track command (2026-11-21). So each track also integrates its load-weighted
belt slip into a traction share of the normal force (``traction_gain`` per
m/s per s, held within +-``mu_along``), which removes the steady slip
without the step-size limit of a stiffer servo.
"""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np


DEFAULTS = dict(separation=0.4, steering_efficiency=1.0, max_linear=0.3, max_angular=0.5,
                linear_acceleration=0.4, angular_acceleration=0.4, timeout_s=0.4,
                mu_along=1.0, mu_lateral=0.7, gain=50.0, traction_gain=40.0)


def parse_config(config):
    """Validate a track block: {left: [links], right: [links], ...numbers}."""
    if isinstance(config, str):
        config = json.loads(config) if config.strip() else {}
    if not isinstance(config, dict) or not config:
        raise ValueError('A track configuration object is required')
    unknown = set(config) - set(DEFAULTS) - {'left', 'right', 'flippers'}
    if unknown:
        raise ValueError('Unknown track settings: ' + ', '.join(sorted(unknown)))
    result = dict(DEFAULTS)
    for side in ('left', 'right'):
        links = config.get(side)
        if not isinstance(links, list) or not links or not all(isinstance(link, str) and link for link in links):
            raise ValueError('Each side needs its track links: ' + side)
        result[side] = list(links)
    if set(result['left']) & set(result['right']):
        raise ValueError('A link cannot belong to both tracks')
    for key in DEFAULTS:
        value = config.get(key, DEFAULTS[key])
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Track setting must be finite and positive: ' + key)
        result[key] = float(value)
    result['flippers'] = parse_flippers(config.get('flippers', {}))
    return result


FLIPPER_KEYS = ('p', 'i', 'd', 'i_min', 'i_max', 'cmd_max')


def parse_flippers(flippers):
    """{joint: {p, i, d, i_min, i_max, cmd_max}}: Gazebo JointPositionController gains."""
    if not isinstance(flippers, dict):
        raise ValueError('Flipper servos must be an object')
    result = {}
    for joint, gains in flippers.items():
        if not isinstance(joint, str) or not joint or not isinstance(gains, dict) or set(gains) != set(FLIPPER_KEYS):
            raise ValueError('Flipper servo needs ' + ', '.join(FLIPPER_KEYS) + ': ' + str(joint))
        values = {key: gains[key] for key in FLIPPER_KEYS}
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values.values()):
            raise ValueError('Flipper gains must be finite numbers: ' + joint)
        if min(values['p'], values['cmd_max']) <= 0 or min(values['i'], values['d']) < 0 or values['i_min'] > values['i_max']:
            raise ValueError('Invalid flipper gains: ' + joint)
        result[joint] = {key: float(v) for key, v in values.items()}
    return result


def sdf_flippers(sdf_text):
    """The JointPositionController gains of a tracked model.sdf, by joint."""
    model = ET.fromstring(sdf_text).find('model')
    result = {}
    for plugin in model.findall('plugin'):
        if not plugin.get('name', '').endswith('JointPositionController'):
            continue
        def number(tag, default=0.0):
            node = plugin.find(tag)
            return float(node.text) if node is not None and node.text else default
        cap = max(abs(number('cmd_min', -1e9)), abs(number('cmd_max', 1e9)))
        result[plugin.find('joint_name').text.strip()] = dict(p=number('p_gain'), i=number('i_gain'), d=number('d_gain'),
                                                               i_min=number('i_min'), i_max=number('i_max'), cmd_max=cap)
    return result


class FlipperServo:
    """Gazebo's JointPositionController (ignition::math::PID, force mode) for one joint.

    The error is target - position; the integral term (i * accumulated error)
    is held within [i_min, i_max]; the derivative acts on the error's rate
    (-velocity for a held target); the output is held within +-cmd_max and
    the joint's own effort limit.
    """

    def __init__(self, gains, effort):
        self.gains, self.effort = gains, float(effort)
        self.target = 0.0
        self.integral = 0.0

    def reset(self):
        self.target = 0.0
        self.integral = 0.0

    def command(self, value):
        if math.isfinite(value):
            self.target = float(value)

    def torque(self, position, velocity, dt):
        g = self.gains
        error = self.target - position
        if g['i'] > 0:
            self.integral = max(g['i_min'] / g['i'], min(g['i_max'] / g['i'], self.integral + error * dt))
        out = g['p'] * error + g['i'] * self.integral - g['d'] * velocity
        cap = min(g['cmd_max'], self.effort) if self.effort > 0 else g['cmd_max']
        return max(-cap, min(cap, out))


def track_collisions(description, config):
    """{collision name: side} for every collision of the configured track links."""
    root = ET.fromstring(description)
    sides = {link: side for side in ('left', 'right') for link in config[side]}
    result = {}
    found = set()
    for link in root.findall('link'):
        side = sides.get(link.get('name'))
        if side is None:
            continue
        found.add(link.get('name'))
        for collision in link.findall('collision'):
            name = collision.get('name')
            if not name:
                raise ValueError('Track collisions need source names: ' + link.get('name'))
            result[name] = side
    missing = set(sides) - found
    if missing:
        raise ValueError('Track links missing from the description: ' + ', '.join(sorted(missing)))
    if not result:
        raise ValueError('The track links have no collision geometry')
    return result


def joint_efforts(description):
    """{joint: URDF effort limit} for the revolute joints of a description."""
    result = {}
    for joint in ET.fromstring(description).findall('joint'):
        limit = joint.find('limit')
        if joint.get('type') == 'revolute' and limit is not None:
            result[joint.get('name')] = float(limit.get('effort', 0) or 0)
    return result


def track_speeds(linear, angular, separation, steering_efficiency=1.0):
    """Left and right belt speeds for a body twist (TrackedVehicle's law)."""
    turn = angular * separation / (2.0 * steering_efficiency)
    return linear - turn, linear + turn


def contact_forces(speeds, axle, normals, normal_forces, velocities, masses, mu_along, mu_lateral, gain, bias=0.0,
                   targets=None):
    """World forces on the robot at its track contacts (arrays, one row each).

    ``normals`` point from the ground into the robot and ``axle`` is the
    hull's left axis. The belt runs in the hull's vertical plane, so where
    it touches it moves along ``axle x normal``: forward on the ground and
    upward against a riser met by the front sprocket. Across it the belt
    follows ``targets`` (world point velocities, default still), of which
    only the across-belt part is used. Each component is a velocity servo
    bounded by its friction; ``bias`` adds a traction share of the normal force along the
    belt (TrackDrive's slip integral). A contact on the belt's side (normal
    along the axle) has no belt direction and gets no force.
    """
    n = np.asarray(normals, dtype=float).reshape(-1, 3)
    load = np.maximum(np.asarray(normal_forces, dtype=float).reshape(-1), 0.0)
    length = np.linalg.norm(n, axis=1)
    usable = (length > 1e-9) & (load > 0.0)
    n = n / np.where(length > 1e-9, length, 1.0)[:, None]
    along = np.cross(np.asarray(axle, dtype=float), n)
    length = np.linalg.norm(along, axis=1)
    usable &= length > 1e-6
    along = along / np.where(length > 1e-6, length, 1.0)[:, None]
    across = np.cross(n, along)
    velocity = np.asarray(velocities, dtype=float).reshape(-1, 3)
    mass = np.asarray(masses, dtype=float).reshape(-1)
    slip = np.asarray(speeds, dtype=float).reshape(-1) - np.einsum('ij,ij->i', velocity, along)
    push = np.clip(gain * mass * slip + np.asarray(bias, dtype=float) * load, -mu_along * load, mu_along * load)
    target = np.zeros_like(velocity) if targets is None else np.asarray(targets, dtype=float).reshape(-1, 3)
    side = np.clip(gain * mass * np.einsum('ij,ij->i', target - velocity, across), -mu_lateral * load, mu_lateral * load)
    forces = push[:, None] * along + side[:, None] * across
    forces[~usable] = 0.0
    return forces, slip, usable


def contact_force(speed, axle, normal, point_velocity, normal_force, mass, mu_along, mu_lateral, gain):
    """One contact of ``contact_forces``."""
    return contact_forces([speed], axle, [normal], [normal_force], [point_velocity], [mass],
                          mu_along, mu_lateral, gain)[0][0]


class TrackDrive:
    """Command intake, limits, contact forces and dead-reckoned odometry."""

    def __init__(self, config, mass):
        self.config = parse_config(config)
        if not math.isfinite(mass) or mass <= 0:
            raise ValueError('Track drive needs the robot mass')
        self.mass = float(mass)
        self.requested = (0.0, 0.0)
        self.requested_at = -math.inf
        self.reset()

    def reset(self):
        self.bias = dict(left=0.0, right=0.0)
        self.velocity = [0.0, 0.0]
        self.pose = [0.0, 0.0, 0.0]
        self.requested = (0.0, 0.0)
        self.requested_at = -math.inf

    def command(self, linear, angular, now):
        """A body twist from /tracked/cmd_vel; non-finite input stops."""
        if not (math.isfinite(linear) and math.isfinite(angular)):
            linear = angular = 0.0
        c = self.config
        self.requested = (max(-c['max_linear'], min(c['max_linear'], linear)),
                          max(-c['max_angular'], min(c['max_angular'], angular)))
        self.requested_at = now

    def advance(self, now, dt):
        """Slew the commanded twist and integrate odometry over one step."""
        c = self.config
        target = self.requested if now - self.requested_at <= c['timeout_s'] else (0.0, 0.0)
        for i, limit in enumerate((c['linear_acceleration'], c['angular_acceleration'])):
            step = limit * dt
            self.velocity[i] += max(-step, min(step, target[i] - self.velocity[i]))
        linear, angular = self.velocity
        heading = self.pose[2] + 0.5 * angular * dt
        self.pose[0] += linear * math.cos(heading) * dt
        self.pose[1] += linear * math.sin(heading) * dt
        self.pose[2] = math.atan2(math.sin(self.pose[2] + angular * dt), math.cos(self.pose[2] + angular * dt))
        return self.speeds()

    def speeds(self):
        left, right = track_speeds(self.velocity[0], self.velocity[1], self.config['separation'],
                                   self.config['steering_efficiency'])
        return dict(left=left, right=right)

    # A body's contacts together servo at most this many times the body's own
    # mass. The load share of the whole vehicle put 25 kg of servo on a
    # 0.75 kg flipper pressed into a riser; the explicit force then diverged
    # in one 2 ms step (MuJoCo, terrain_stairs_flight, 2026-11-21).
    BODY_MASS_FACTOR = 4.0

    def forces(self, sides, normals, normal_forces, velocities, axle, dt, points=None, origin=None, up=None,
               bodies=None, body_masses=None):
        """World force at each track contact; ``sides`` holds 'left'/'right' per row.

        ``bodies`` (an id per row) and ``body_masses`` ({id: kg}) cap each
        body's servo mass at BODY_MASS_FACTOR times its own mass.

        With ``points``, the hull ``origin`` and its ``up`` axis, the
        across-belt target is the commanded yaw rate about the origin.

        The robot's mass is shared among the contacts by normal force, so the
        servo gain is per vehicle. Each track's traction share integrates its
        load-weighted slip and is dropped while that track carries no load.
        """
        load = np.maximum(np.asarray(normal_forces, dtype=float).reshape(-1), 0.0)
        total = float(load.sum())
        if total <= 0.0:
            self.bias = dict(left=0.0, right=0.0)
            return np.zeros((len(load), 3))
        left = np.asarray([side == 'left' for side in sides], dtype=bool)
        speeds = self.speeds()
        c = self.config
        bias = np.where(left, self.bias['left'], self.bias['right'])
        masses = self.mass * load / total
        if bodies is not None:
            bodies = np.asarray(bodies)
            for body in np.unique(bodies):
                at = bodies == body
                cap = self.BODY_MASS_FACTOR * float(body_masses[body.item()])
                if masses[at].sum() > cap:
                    masses[at] *= cap / masses[at].sum()
        belt = np.where(left, speeds['left'], speeds['right'])
        targets = None
        if points is not None:
            arm = np.asarray(points, dtype=float).reshape(-1, 3) - np.asarray(origin, dtype=float)
            targets = self.velocity[1] * np.cross(np.asarray(up, dtype=float), arm)
            offset = arm @ np.asarray(axle, dtype=float)
            belt = belt + self.velocity[1] * (np.where(left, 0.5, -0.5) * c['separation'] - offset)
        forces, slip, usable = contact_forces(belt, axle, normals, load,
                                              velocities, masses, c['mu_along'], c['mu_lateral'],
                                              c['gain'], bias, targets)
        for name, rows in (('left', left & usable), ('right', ~left & usable)):
            weight = float(load[rows].sum())
            if weight <= 0.0:
                self.bias[name] = 0.0
                continue
            mean = float(np.dot(load[rows], slip[rows])) / weight
            self.bias[name] = max(-c['mu_along'], min(c['mu_along'], self.bias[name] + c['traction_gain'] * mean * dt))
        return forces
