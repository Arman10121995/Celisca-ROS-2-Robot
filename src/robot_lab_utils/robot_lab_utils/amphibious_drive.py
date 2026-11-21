"""Backend-neutral amphibious rover plant (R10.7: lab_amphibious beyond Gazebo).

A port of what Gazebo applies to the rover:

- ``robot_lab::AmphibiousDrive`` (robot_lab_maps/systems/amphibious_drive.cc):
  - Wheel speed servos: 12 Nm per rad/s, held within +-8 Nm, targets from the
    commanded twist (0.22 m radius, 0.49 m half track, +-3 rad/s).
  - Passive suspension springs: 10000 N/m and 200 N s/m, held within +-600 N.
  - Immersion-gated propeller thrust: +-35 N.
  - Hull drag scaled by the wetted fraction.
  - Commands older than 0.4 s give zero wheel effort and thrust.
- Fortress graded ``Buoyancy`` on the rover's collision boxes and cylinders:
  fluid density below the surface, nothing above it. Each shape's submerged
  share is its fraction below the surface along its world vertical extent,
  as the ported marine plants count boxes.

The backend supplies link poses, joint states and the hull velocity and
applies the returned joint efforts and world forces. Leases run on
simulation time, so a stalled controller still loses its effort.
"""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np

from robot_lab_utils.marine_model import rotation

WHEELS = ('front_left_wheel_joint', 'rear_left_wheel_joint', 'front_right_wheel_joint', 'rear_right_wheel_joint')
SUSPENSION = tuple(name.replace('wheel_joint', 'suspension_joint') for name in WHEELS)
PROPELLERS = ('port_propeller_joint', 'starboard_propeller_joint')
GRAVITY = 9.81


def collision_shapes(description):
    """[(link, origin xyz, rotation, half extents, volume)] for box and cylinder collisions.

    A shape on a fixed-joint link (the OAK-D housing) is expressed in its
    nearest moving ancestor's frame, the body MuJoCo fuses it into and
    Gazebo merges it with.
    """
    root = ET.fromstring(description)
    fixed = {}
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            origin = joint.find('origin')
            fixed[joint.find('child').get('link')] = (
                joint.find('parent').get('link'),
                np.array([float(v) for v in origin.get('xyz', '0 0 0').split()]) if origin is not None else np.zeros(3),
                rotation([float(v) for v in origin.get('rpy', '0 0 0').split()]) if origin is not None else np.eye(3))
    shapes = []
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            origin = collision.find('origin')
            xyz = np.array([float(v) for v in origin.get('xyz', '0 0 0').split()]) if origin is not None else np.zeros(3)
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0., 0., 0.]
            R, name = rotation(rpy), link.get('name')
            while name in fixed:
                name, offset, Rj = fixed[name]
                xyz, R = offset + Rj @ xyz, Rj @ R
            box, cylinder = collision.find('geometry/box'), collision.find('geometry/cylinder')
            if box is not None:
                size = np.array([float(v) for v in box.get('size').split()])
                shapes.append((name, xyz, R, size / 2, float(np.prod(size))))
            elif cylinder is not None:
                r, length = float(cylinder.get('radius')), float(cylinder.get('length'))
                shapes.append((name, xyz, R, np.array([r, r, length / 2]), math.pi * r * r * length))
    return shapes


def propeller_mounts(description):
    """{joint: (position in base_link, unit thrust axis in base_link)}."""
    root = ET.fromstring(description)
    mounts = {}
    for name in PROPELLERS:
        joint = root.find("joint[@name='%s']" % name)
        if joint is None:
            raise ValueError('Amphibious propeller joint missing: ' + name)
        pose = joint.find('origin')
        axis = rotation([float(v) for v in pose.get('rpy', '0 0 0').split()]) @ np.array(
            [float(v) for v in joint.find('axis').get('xyz').split()])
        mounts[name] = (np.array([float(v) for v in pose.get('xyz').split()]), axis / np.linalg.norm(axis))
    return mounts


class AmphibiousDrive:
    radius, half_track, torque_limit, wheel_gain, wheel_rate = .22, .49, 8., 12., 3.
    spring, damper, spring_limit, thrust_limit, lease = 10000., 200., 600., 35., .4
    linear, quadratic, rotational = np.array([65., 160., 160.]), np.array([70., 160., 180.]), np.array([12., 18., 16.])

    def __init__(self, description, config):
        config = json.loads(config) if isinstance(config, str) else dict(config)
        unknown = set(config) - {'surface_z', 'density'}
        if unknown:
            raise ValueError('Unknown amphibious settings: ' + ', '.join(sorted(unknown)))
        self.surface = float(config.get('surface_z', 0.))
        self.density = float(config.get('density', 1025.))
        if not (math.isfinite(self.surface) and math.isfinite(self.density) and self.density > 0):
            raise ValueError('Finite water surface and positive density required')
        self.shapes = collision_shapes(description)
        self.mounts = propeller_mounts(description)
        names = {j.get('name') for j in ET.fromstring(description).findall('joint')}
        missing = (set(WHEELS) | set(SUSPENSION)) - names
        if missing:
            raise ValueError('Amphibious source joints missing: ' + ', '.join(sorted(missing)))
        self.reset()

    def reset(self):
        self.drive, self.drive_at = (0., 0.), -math.inf
        self.requested, self.thrust_at = {name: 0. for name in PROPELLERS}, {name: -math.inf for name in PROPELLERS}
        self.state = dict(wet_fraction=0., wheel_input_fresh=False, applied_thrust_N=[0., 0.],
                          wheel_rates_rad_s=[0.] * 4, wheel_efforts_Nm=[0.] * 4)

    def command_wheels(self, linear, angular, lateral, vertical, now):
        """The /amphibious/wheels twist: surge and yaw only; anything else stops."""
        self.drive, self.drive_at = (0., 0.), -math.inf
        if not all(map(math.isfinite, (linear, angular, lateral, vertical))) or abs(lateral) > .001 or abs(vertical) > .001:
            return
        self.drive = (max(-.4, min(.4, linear)), max(-.3, min(.3, angular)))
        self.drive_at = now

    def command_thrust(self, joint, force, now):
        self.requested[joint] = max(-self.thrust_limit, min(self.thrust_limit, force)) if math.isfinite(force) else 0.
        self.thrust_at[joint] = now

    def step(self, now, hull_rotation, hull_origin, hull_com, linear, angular, joints, link_poses):
        """One physics step.

        hull_rotation/origin: base_link pose; hull_com: its centre of mass;
        linear/angular: base_link world velocity; joints: {name: (position,
        velocity)}; link_poses: {link: (origin, rotation)}. Returns
        ({joint: effort}, [(link, world force, world point)], (hull drag
        force, hull drag torque, the centre of mass they act at)).
        """
        R = np.asarray(hull_rotation)
        fresh = now - self.drive_at <= self.lease
        efforts, forces = {}, []
        rates = []
        for i, name in enumerate(WHEELS):
            measured = joints[name][1]
            target = max(-self.wheel_rate, min(self.wheel_rate,
                         (self.drive[0] + (-1. if i < 2 else 1.) * self.half_track * self.drive[1]) / self.radius))
            efforts[name] = max(-self.torque_limit, min(self.torque_limit, self.wheel_gain * (target - measured))) if fresh else 0.
            rates.append(measured)
        for name in SUSPENSION:
            travel, rate = joints[name]
            efforts[name] = max(-self.spring_limit, min(self.spring_limit, -self.spring * travel - self.damper * rate))
        wet = max(0., min(1., (self.surface - (hull_origin[2] - .17)) / .10))
        applied = []
        for name in PROPELLERS:
            position, axis = self.mounts[name]
            point = np.asarray(hull_origin) + R @ position
            thrust = self.requested[name] if now - self.thrust_at[name] <= self.lease else 0.
            thrust *= max(0., min(1., (self.surface - point[2] + .065) / .13))
            applied.append(thrust)
            if thrust:
                forces.append(('base_link', R @ axis * thrust, point))
        local, rates_body = R.T @ np.asarray(linear), R.T @ np.asarray(angular)
        drag = -wet * (self.linear * local + self.quadratic * np.abs(local) * local)
        torque = -wet * self.rotational * rates_body
        hull_force, hull_torque = R @ drag, R @ torque
        for link, xyz, Rc, half, volume in self.shapes:
            origin, Rl = link_poses[link]
            centre = np.asarray(origin) + np.asarray(Rl) @ xyz
            height = float(np.abs((np.asarray(Rl) @ Rc)[2, :]) @ half)
            fraction = max(0., min(1., (self.surface - (centre[2] - height)) / max(2 * height, 1e-9)))
            if fraction <= 0:
                continue
            point = centre.copy()
            point[2] = centre[2] - height + fraction * height
            forces.append((link, np.array([0., 0., self.density * GRAVITY * volume * fraction]), point))
        self.state = dict(wet_fraction=wet, wheel_input_fresh=bool(fresh), applied_thrust_N=applied,
                          wheel_rates_rad_s=rates, wheel_efforts_Nm=[efforts[name] for name in WHEELS])
        return efforts, forces, (hull_force, hull_torque, np.asarray(hull_com))

    def plant_message(self, sim_time):
        """The /amphibious/plant JSON, as AmphibiousDrive publishes it."""
        return json.dumps(dict(sim_time=sim_time, **self.state))
