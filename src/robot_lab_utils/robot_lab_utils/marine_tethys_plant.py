#!/usr/bin/env python3
"""MuJoCo plant for the pinned Tethys AUV (propeller and fins; ROADMAP R10.4).

The unchanged ``auv_controller`` commands ``/marine/auv/propeller_force``
(N) and ``/marine/auv/{horizontal,vertical}_fin`` (rad), and observes
``/marine/odometry``, ``/joint_states`` and ``/clock``. This plant consumes
and publishes the same topics as the Gazebo session's bridge. Its forces are
ports of the systems in the source ``model.sdf`` and the world's buoyancy:

- Buoyancy: rho g x each collision box's volume at the box centre (uniform
  fluid below the surface). The source trim makes the vehicle neutral.
- Hydrodynamics (Fortress Hydrodynamics on base_link): quadratic damping and
  added mass on the velocity relative to the current. The added-mass
  Coriolis wrench is off by default: Fortress 6.18 applies one, but with the
  same controller and probe, Fossen's -C_A(nu) nu made pitch diverge in a
  turn, its opposite turned the wrong way, and none came closest to the
  Gazebo run (``--coriolis``; measured 2026-11-19).
- Thruster (velocity control): the commanded thrust along the propeller
  axis at the propeller link, no reaction torque; the joint turns at
  sign(T) sqrt(|T| / (rho Ct D^4)).
- Fins (JointPositionController, p_gain 0.1, +-2 N m, no damping, as the
  source): each fin angle is a second-order joint state with its source
  inertia about the joint axis, within +-0.2618 rad.
- LiftDrag on each fin (Fortress LiftDrag): velocity at the fin's centre of
  pressure (its link origin), sweep and angle of attack, the stall branches
  of cl and cd, cl and cd scaled by 1 - sin^2(sweep) as the source does,
  zero moment, the wrench at the fin link.

The hull, ballast, fins and propeller are one rigid body with their summed
source masses and inertias (parallel-axis); fin motion only changes the
LiftDrag geometry. The world is a seabed plane.
"""
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from robot_lab_utils.marine_model import rotation

FINS = ('horizontal_fins', 'vertical_fins')
# The added-mass Coriolis wrench: Fossen's -C_A(nu) nu, its opposite, or none.
CORIOLIS_SIGNS = dict(fossen=-1., flipped=1., off=0.)
AXES = 'xyzkmn'
VELOCITIES = 'UVWPQR'


def _floats(text, default='0 0 0 0 0 0'):
    return np.array([float(v) for v in (text or default).split()])


def _pose(element):
    values = _floats(element.findtext('pose') if element is not None else None)
    return values[:3], rotation(values[3:6])


def source_physics(sdf_text):
    """Mass, inertia, buoyancy boxes, joints and plugin parameters of the Tethys model.sdf."""
    model = ET.fromstring(sdf_text).find('model')
    links = {link.get('name'): link for link in model.findall('link')}
    joints = {joint.get('name'): joint for joint in model.findall('joint')}
    plugins = {}
    for plugin in model.findall('plugin'):
        kind = plugin.get('name', '').rsplit('::', 1)[-1]
        if kind == 'LiftDrag':
            plugins.setdefault('lift_drag', {})[plugin.findtext('link_name')] = plugin
        elif kind == 'JointPositionController':
            plugins.setdefault('fin_control', {})[plugin.findtext('joint_name')] = plugin
        else:
            plugins[kind] = plugin
    bodies, boxes = [], []
    for name, link in links.items():
        origin, frame = _pose(link)
        inertial = link.find('inertial')
        if inertial is not None:
            offset, inertial_frame = _pose(inertial)
            tensor = inertial.find('inertia')
            diagonal = np.diag([float(tensor.findtext(k, '0')) for k in ('ixx', 'iyy', 'izz')])
            orientation = frame @ inertial_frame
            bodies.append((float(inertial.findtext('mass')), origin + frame @ offset,
                           orientation @ diagonal @ orientation.T))
        for collision in link.findall('collision'):
            size = collision.find('geometry/box/size')
            if size is not None:
                offset, _ = _pose(collision)
                boxes.append((origin + frame @ offset, _floats(size.text)))
    mass = sum(m for m, _, _ in bodies)
    centre = sum(m * c for m, c, _ in bodies) / mass
    inertia = sum(I + m * (np.dot(c - centre, c - centre) * np.eye(3) - np.outer(c - centre, c - centre))
                  for m, c, I in bodies)
    hydro = plugins['Hydrodynamics']
    read = lambda tag: float(hydro.findtext(tag) or 0.)  # noqa: E731
    fins = {}
    for name in FINS:
        joint = next(j for j in joints.values() if j.findtext('child') == name)
        link = links[name]
        origin, frame = _pose(link)
        offset, inertial_frame = _pose(link.find('inertial'))
        tensor = link.find('inertial/inertia')
        diagonal = np.diag([float(tensor.findtext(k, '0')) for k in ('ixx', 'iyy', 'izz')])
        axis = _floats(joint.findtext('axis/xyz'), '0 0 1')
        orientation = frame @ inertial_frame
        inertia_about = float(axis @ (orientation @ diagonal @ orientation.T) @ axis)
        lift_drag = plugins['lift_drag'][name]
        control = plugins['fin_control'][joint.get('name')]
        limit = joint.find('axis/limit')
        fins[name] = dict(joint=joint.get('name'), origin=origin, axis=axis / np.linalg.norm(axis),
                          inertia=max(inertia_about, 1e-6), p_gain=float(control.findtext('p_gain')),
                          cmd=(float(control.findtext('cmd_min')), float(control.findtext('cmd_max'))),
                          limits=(float(limit.findtext('lower')), float(limit.findtext('upper'))),
                          **{key: float(lift_drag.findtext(key)) for key in
                             ('cla', 'cla_stall', 'cda', 'cda_stall', 'alpha_stall', 'a0', 'area', 'air_density')},
                          forward=_floats(lift_drag.findtext('forward'), '1 0 0'),
                          upward=_floats(lift_drag.findtext('upward'), '0 0 1'),
                          cp=_floats(lift_drag.findtext('cp'), '0 0 0'))
    thruster = plugins['Thruster']
    propeller_joint = joints[thruster.findtext('joint_name')]
    propeller_link = links[propeller_joint.findtext('child')]
    return dict(
        mass=mass, centre=centre, inertia=inertia, boxes=boxes, fins=fins,
        added=np.array([read(a + 'Dot' + v) for a, v in zip(AXES, VELOCITIES)]),
        linear=np.array([read(a + v) for a, v in zip(AXES, VELOCITIES)]),
        quadratic=np.array([read(a + v + 'abs' + v) for a, v in zip(AXES, VELOCITIES)]),
        thruster=dict(joint=propeller_joint.get('name'), origin=_pose(propeller_link)[0],
                      axis=_floats(propeller_joint.findtext('axis/xyz'), '1 0 0'),
                      coefficient=float(thruster.findtext('thrust_coefficient')),
                      density=float(thruster.findtext('fluid_density')),
                      diameter=float(thruster.findtext('propeller_diameter')),
                      limits=(float(thruster.findtext('min_thrust_cmd')), float(thruster.findtext('max_thrust_cmd')))))


def coriolis_added_mass(added, state):
    """Fossen's added-mass Coriolis matrix C_A(nu) for diagonal added mass (nu: u v w p q r)."""
    a = added[:3] * state[:3]
    b = added[3:] * state[3:]
    def skew(v):
        return np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
    C = np.zeros((6, 6))
    C[:3, 3:] = -skew(a)
    C[3:, :3] = -skew(a)
    C[3:, 3:] = -skew(b)
    return C


def lift_drag(fin, velocity, upward, forward):
    """Fortress LiftDrag force (world) for a fin moving at ``velocity`` (world, at its cp)."""
    speed = np.linalg.norm(velocity)
    if speed <= .01:
        return np.zeros(3)
    spanwise = np.cross(forward, upward)
    spanwise /= np.linalg.norm(spanwise)
    sin_sweep = float(np.clip(spanwise @ (velocity / speed), -1., 1.))
    cos2_sweep = 1. - sin_sweep * sin_sweep           # named cosSweepAngle in the source
    in_plane = velocity - (velocity @ spanwise) * spanwise
    plane_speed = np.linalg.norm(in_plane)
    if plane_speed < 1e-9:
        return np.zeros(3)
    drag_direction = -in_plane / plane_speed
    lift_direction = np.cross(spanwise, in_plane)
    lift_direction /= np.linalg.norm(lift_direction)
    cos_alpha = float(np.clip(lift_direction @ upward, -1., 1.))
    alpha = fin['a0'] + (math.acos(cos_alpha) if lift_direction @ forward >= 0. else -math.acos(cos_alpha))
    while abs(alpha) > .5 * math.pi:
        alpha += -math.pi if alpha > 0 else math.pi
    q = .5 * fin['air_density'] * plane_speed * plane_speed
    stall = fin['alpha_stall']
    if alpha > stall:
        cl = max(0., (fin['cla'] * stall + fin['cla_stall'] * (alpha - stall)) * cos2_sweep)
        cd = (fin['cda'] * stall + fin['cda_stall'] * (alpha - stall)) * cos2_sweep
    elif alpha < -stall:
        cl = min(0., (-fin['cla'] * stall + fin['cla_stall'] * (alpha + stall)) * cos2_sweep)
        cd = (-fin['cda'] * stall + fin['cda_stall'] * (alpha + stall)) * cos2_sweep
    else:
        cl = fin['cla'] * alpha * cos2_sweep
        cd = fin['cda'] * alpha * cos2_sweep
    return cl * q * fin['area'] * lift_direction + abs(cd) * q * fin['area'] * drag_direction


def _axis_rotation(axis, angle):
    k = np.asarray(axis, dtype=float)
    K = np.array([[0., -k[2], k[1]], [k[2], 0., -k[0]], [-k[1], k[0], 0.]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


class TethysForces:
    """The ported Tethys wrench and fin/propeller joint states, independent of the physics engine."""
    timestep = .002

    def _setup(self, sdf_text, surface_z, density, coriolis):
        if coriolis not in CORIOLIS_SIGNS:
            raise ValueError('coriolis must be one of %s' % sorted(CORIOLIS_SIGNS))
        self.coriolis_sign = CORIOLIS_SIGNS[coriolis]
        self.source = source_physics(sdf_text)
        self.surface, self.density = float(surface_z), float(density)
        self.current = np.zeros(3)
        self.thrust = 0.
        self.propeller_angle = 0.
        self.fin_target = {name: 0. for name in FINS}
        self.fin_angle = {name: 0. for name in FINS}
        self.fin_rate = {name: 0. for name in FINS}
        self.previous = None
        self.wrench = np.zeros(6)

    # --- commands ---------------------------------------------------------------
    def command_thrust(self, value):
        low, high = self.source['thruster']['limits']
        self.thrust = max(low, min(high, float(value))) if math.isfinite(value) else 0.

    def command_fin(self, name, value):
        if math.isfinite(value):
            self.fin_target[name] = float(value)

    def joints(self):
        rows = {self.source['fins'][name]['joint']: self.fin_angle[name] for name in FINS}
        rows[self.source['thruster']['joint']] = self.propeller_angle
        return rows

    def _propeller_rate(self):
        t = self.source['thruster']
        return math.copysign(math.sqrt(abs(self.thrust) / (t['density'] * t['coefficient'] * t['diameter'] ** 4)),
                             self.thrust)

    def _fins_step(self, dt):
        for name in FINS:
            fin = self.source['fins'][name]
            torque = max(fin['cmd'][0], min(fin['cmd'][1], fin['p_gain'] * (self.fin_target[name] - self.fin_angle[name])))
            self.fin_rate[name] += torque / fin['inertia'] * dt
            angle = self.fin_angle[name] + self.fin_rate[name] * dt
            low, high = fin['limits']
            if not low <= angle <= high:
                angle = min(max(angle, low), high)
                self.fin_rate[name] = 0.
            self.fin_angle[name] = angle

    def advance_joints(self):
        """Fin servos and propeller angle over one physics step."""
        self._fins_step(self.timestep)
        self.propeller_angle += self._propeller_rate() * self.timestep

    def tethys_wrench(self, R, origin, com, linear_body, angular_body):
        """World force and torque about the centre of mass for one physics step.

        R: body rotation; origin: base_link origin (world); com: centre of
        mass (world); linear_body, angular_body: base_link velocity in the
        body frame.
        """
        s = self.source
        force, torque = np.zeros(3), np.zeros(3)
        for centre, size in s['boxes']:
            position = origin + R @ centre
            height = abs(R[2, :]) @ size
            bottom = position[2] - height / 2
            fraction = min(1., max(0., (self.surface - bottom) / max(height, 1e-9)))
            if fraction <= 0:
                continue
            lift = np.array([0., 0., self.density * 9.81 * np.prod(size) * fraction])
            point = position.copy()
            point[2] = bottom + fraction * height / 2
            force += lift
            torque += np.cross(point - com, lift)
        state = np.r_[linear_body - R.T @ self.current, angular_body]
        rate = np.zeros(6) if self.previous is None else (state - self.previous) / self.timestep
        self.previous = state
        body = ((s['linear'] + s['quadratic'] * np.abs(state)) * state - s['added'] * rate
                + self.coriolis_sign * coriolis_added_mass(s['added'], state) @ state)
        force += R @ body[:3]
        torque += R @ body[3:]
        t = s['thruster']
        thrust = R @ (t['axis'] * self.thrust)
        force += thrust
        torque += np.cross(origin + R @ t['origin'] - com, thrust)
        omega_world = R @ angular_body
        v_origin = R @ linear_body
        for name in FINS:
            fin = s['fins'][name]
            fin_rotation = R @ _axis_rotation(fin['axis'], self.fin_angle[name])
            cp = origin + R @ fin['origin'] + fin_rotation @ fin['cp']
            v = v_origin + np.cross(omega_world, cp - origin) - self.current
            f = lift_drag(fin, v, fin_rotation @ fin['upward'], fin_rotation @ fin['forward'])
            force += f
            torque += np.cross(cp - com, f)
        self.wrench = np.r_[force, torque]
        return force, torque


class TethysPlant(TethysForces):
    """MuJoCo rigid body integrating the Tethys wrench."""
    engine = 'mujoco'
    scope = ('MuJoCo rigid body with ported Fortress buoyancy, hydrodynamics, thruster and fin LiftDrag; '
             'added-mass Coriolis off by default; seabed plane only')

    def __init__(self, sdf_text, spawn, surface_z, bottom_z, density=1025., coriolis='off'):
        import mujoco
        self.mujoco = mujoco
        self._setup(sdf_text, surface_z, density, coriolis)
        s = self.source
        eigen, frame = np.linalg.eigh(s['inertia'])
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, frame.ravel())
        hull_centre, hull_size = s['boxes'][0]
        xml = f"""<mujoco model="tethys_plant">
  <compiler angle="radian"/>
  <option timestep="{self.timestep}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <geom name="seabed" type="plane" size="200 200 .1" pos="0 0 {bottom_z}" friction="0.6 0.005 0.0001"/>
    <body name="auv" pos="{spawn[0]} {spawn[1]} {spawn[2]}" euler="0 0 {spawn[3]}">
      <freejoint/>
      <inertial pos="{s['centre'][0]} {s['centre'][1]} {s['centre'][2]}" quat="{quat[0]} {quat[1]} {quat[2]} {quat[3]}"
                mass="{s['mass']}" diaginertia="{eigen[0]} {eigen[1]} {eigen[2]}"/>
      <geom name="hull" type="box" pos="{hull_centre[0]} {hull_centre[1]} {hull_centre[2]}"
            size="{hull_size[0] / 2} {hull_size[1] / 2} {hull_size[2] / 2}" mass="0"/>
    </body>
  </worldbody>
</mujoco>"""
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.body = self.model.body('auv').id

    def pose(self):
        q = self.data.qpos
        return q[:3].copy(), np.array([q[4], q[5], q[6], q[3]])

    def step(self, dt):
        mujoco = self.mujoco
        for _ in range(max(1, int(round(dt / self.timestep)))):
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.body, velocity, 1)
            force, torque = self.tethys_wrench(self.data.xmat[self.body].reshape(3, 3).copy(),
                                               self.data.xpos[self.body].copy(), self.data.xipos[self.body].copy(),
                                               velocity[3:], velocity[:3])
            self.data.xfrc_applied[self.body, :3] = force
            self.data.xfrc_applied[self.body, 3:] = torque
            mujoco.mj_step(self.model, self.data)
            self.advance_joints()
        return self.data.time


class TethysPyBulletPlant(TethysForces):
    """Bullet rigid body integrating the same Tethys wrench."""
    engine = 'pybullet'
    scope = ('PyBullet rigid body with ported Fortress buoyancy, hydrodynamics, thruster and fin LiftDrag; '
             'added-mass Coriolis off by default; seabed plane only')

    def __init__(self, sdf_text, spawn, surface_z, bottom_z, density=1025., coriolis='off'):
        import pybullet as p
        self.p = p
        self._setup(sdf_text, surface_z, density, coriolis)
        s = self.source
        self.client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setTimeStep(self.timestep, physicsClientId=self.client)
        p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE, physicsClientId=self.client),
                          basePosition=[0, 0, bottom_z], physicsClientId=self.client)
        eigen, frame = np.linalg.eigh(s['inertia'])
        if np.linalg.det(frame) < 0:
            frame[:, 0] = -frame[:, 0]
        self.principal = frame                            # inertial frame in the body frame
        hull_centre, hull_size = s['boxes'][0]
        # Bullet's base frame is the inertial frame (centre of mass, principal axes).
        hull = p.createCollisionShape(p.GEOM_BOX, halfExtents=(np.asarray(hull_size) / 2).tolist(),
                                      collisionFramePosition=(frame.T @ (hull_centre - s['centre'])).tolist(),
                                      collisionFrameOrientation=_xyzw_from_matrix(frame.T), physicsClientId=self.client)
        yaw = float(spawn[3])
        spawn_rotation = _axis_rotation([0., 0., 1.], yaw)
        start = np.asarray(spawn[:3], dtype=float) + spawn_rotation @ s['centre']
        self.body = p.createMultiBody(s['mass'], hull, basePosition=start.tolist(),
                                      baseOrientation=_xyzw_from_matrix(spawn_rotation @ frame), physicsClientId=self.client)
        p.changeDynamics(self.body, -1, localInertiaDiagonal=eigen.tolist(), linearDamping=0., angularDamping=0.,
                         physicsClientId=self.client)
        self.time = 0.

    def _state(self):
        p = self.p
        com, q = p.getBasePositionAndOrientation(self.body, physicsClientId=self.client)
        R = np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3) @ self.principal.T     # body (base_link) rotation
        com = np.asarray(com)
        return com, R, com - R @ self.source['centre']

    def pose(self):
        _, R, origin = self._state()
        return origin, np.asarray(_xyzw_from_matrix(R))

    def step(self, dt):
        p = self.p
        for _ in range(max(1, int(round(dt / self.timestep)))):
            com, R, origin = self._state()
            linear_com, angular = (np.asarray(v) for v in p.getBaseVelocity(self.body, physicsClientId=self.client))
            linear_origin = linear_com + np.cross(angular, origin - com)
            force, torque = self.tethys_wrench(R, origin, com, R.T @ linear_origin, R.T @ angular)
            p.applyExternalForce(self.body, -1, force.tolist(), com.tolist(), p.WORLD_FRAME, physicsClientId=self.client)
            p.applyExternalTorque(self.body, -1, torque.tolist(), p.WORLD_FRAME, physicsClientId=self.client)
            p.stepSimulation(physicsClientId=self.client)
            self.advance_joints()
            self.time += self.timestep
        return self.time


def _xyzw_from_matrix(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(np.asarray(R)).as_quat().tolist()


ENGINES = dict(mujoco=TethysPlant, pybullet=TethysPyBulletPlant)


def main(args=None):
    import argparse
    import json
    import rclpy
    from geometry_msgs.msg import Vector3
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64, String
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent()
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    for key in ('model', 'spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw', 'surface_z', 'bottom_z', 'density'):
        parser.add_argument('--' + key.replace('_', '-'), required=key == 'model', default='0')
    parser.add_argument('--coriolis', default='off', choices=sorted(CORIOLIS_SIGNS))
    parser.add_argument('--engine', default='mujoco', choices=sorted(ENGINES))
    o = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    plant = ENGINES[o.engine](Path(o.model).read_text(), [float(o.spawn_x), float(o.spawn_y), float(o.spawn_z), float(o.spawn_yaw)],
                        float(o.surface_z), float(o.bottom_z), float(o.density), o.coriolis)
    node = rclpy.create_node('marine_tethys_plant')
    stamps = dict(thrust=-math.inf)
    def thrust(msg):
        plant.command_thrust(msg.data)
        stamps['thrust'] = time.monotonic()
    node.create_subscription(Float64, '/marine/auv/propeller_force', thrust, 10)
    for axis, name in zip(('horizontal', 'vertical'), FINS):
        node.create_subscription(Float64, '/marine/auv/' + axis + '_fin', lambda msg, n=name: plant.command_fin(n, msg.data), 10)
    def current(msg):
        values = np.array([msg.x, msg.y, msg.z])
        if np.isfinite(values).all():
            plant.current = values
    node.create_subscription(Vector3, '/marine/current', current, 10)
    clock = node.create_publisher(Clock, '/clock', 10)
    odometry = node.create_publisher(Odometry, '/marine/odometry', 10)
    joints = node.create_publisher(JointState, '/joint_states', 10)
    state = node.create_publisher(String, '/marine/tethys_%s_plant' % plant.engine, 10)
    period, last_odom, last_state = .01, -math.inf, -math.inf
    next_tick = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.)
            now = time.monotonic()
            if now - stamps['thrust'] > .5:
                plant.thrust = 0.                       # a stale thrust command is zero
            sim = plant.step(period)
            stamp = Clock()
            stamp.clock.sec, stamp.clock.nanosec = int(sim), int((sim % 1) * 1e9)
            clock.publish(stamp)
            if sim - last_odom >= .02:
                last_odom = sim
                position, quaternion = plant.pose()
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = 'map', 'base_link'
                odom.header.stamp = stamp.clock
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, position)
                (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = map(float, quaternion)
                odometry.publish(odom)
                rows = plant.joints()
                message = JointState()
                message.header.stamp = stamp.clock
                message.name = list(rows)
                message.position = [float(v) for v in rows.values()]
                joints.publish(message)
            if now - last_state > .2:
                last_state = now
                state.publish(String(data=json.dumps(dict(sim_time=sim, thrust_N=plant.thrust, fins_rad=plant.fin_angle,
                                                          wrench=plant.wrench.round(4).tolist(), scope=plant.scope))))
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
