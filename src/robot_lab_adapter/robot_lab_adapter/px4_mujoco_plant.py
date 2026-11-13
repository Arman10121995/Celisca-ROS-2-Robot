"""PX4 SITL flying a MuJoCo quadrotor through PX4's simulator interface (ROADMAP R5.11).

PX4 runs its own external-simulator airframe ``10016_none_iris``; this plant
listens on TCP 4560 and exchanges, in lockstep, ``HIL_SENSOR`` (IMU every
physics step, magnetometer, barometer), ``HIL_GPS`` and
``HIL_STATE_QUATERNION`` for ``HIL_ACTUATOR_CONTROLS``. The protocol and the
lockstep handling follow PX4's simulator_mavlink and the Pegasus Simulator's
PX4 backend (BSD-3-Clause, Marcelo Jacinto, github.com/PegasusSimulator):
after the first actuator message, every sensor message is answered with
exactly one actuator message before physics advances.

The vehicle is an Iris-class quadrotor whose geometry matches the airframe's
control allocation (``CA_ROTOR*``: x ±0.1515, y ±0.245 / ±0.1875 m, moment
ratio ±0.05). Mass, inertia and motor model are the PX4 SITL Iris values
(1.5 kg; motor constant 5.84e-6 N/(rad/s)^2; ω = 100 + 1000·u rad/s when
armed; time constants 12.5 / 25 ms). Sensor noise is small and declared
below. The world is the map's MuJoCo scene (static geometry; actors are not
in it). Frames: world ENU = the lab's map frame; body FLU; PX4 receives NED /
FRD. Ground truth is published as ``/px4/odometry_truth``.
"""
import argparse
import math
import time
import xml.etree.ElementTree as ET

import numpy as np

HOME = dict(lat=47.397742, lon=8.545594, alt=488.)          # PX4 SITL default home (Zurich)
# Earth field at HOME (WMM): 0.48 G, declination 2.4°, inclination 63.4° -> NED gauss.
_F, _D, _I = .48, math.radians(2.4), math.radians(63.4)
MAG_NED = np.array([_F * math.cos(_I) * math.cos(_D), _F * math.cos(_I) * math.sin(_D), _F * math.sin(_I)])
MASS, INERTIA = 1.5, (.029125, .029125, .055225)
ROTORS_FRD = ((.1515, .245, .05), (-.1515, -.1875, .05), (.1515, -.245, -.05), (-.1515, .1875, -.05))
MOTOR_CONSTANT, OMEGA_ZERO, OMEGA_SCALE = 5.84e-06, 100., 1000.
TAU_UP, TAU_DOWN = .0125, .025
DT = .004
# The body rests this high on its gear, as PX4's X500 does: the takeoff check uses the X500 envelope.
REST_HEIGHT_M = .24
NOISE = dict(accel=.004, gyro=.0005, mag=.0005, baro_m=.02)   # declared lab values (1 sigma)
FIELDS = 7 | 56 | 448 | 6656                                  # accel | gyro | mag | baro


def vehicle_body(spawn):
    x, y, z = spawn
    return ('<body name="px4_vehicle" pos="%g %g %g"><freejoint name="px4_free"/>'
            '<inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g"/>'
            '<geom name="px4_frame" type="box" size="0.18 0.18 0.04" rgba="0.15 0.15 0.18 1" mass="0"/>'
            '<geom name="px4_gear" type="box" size="0.12 0.12 0.11" pos="0 0 -0.13" rgba="0.3 0.3 0.3 1" mass="0"/>'
            '%s</body>') % (x, y, z + REST_HEIGHT_M, MASS, *INERTIA, ''.join(
                '<geom name="px4_rotor_%d" type="cylinder" size="0.12 0.005" pos="%g %g 0.05" rgba="0.8 0.8 0.85 0.5" '
                'contype="0" conaffinity="0" mass="0"/>' % (i, rx, -ry) for i, (rx, ry, _) in enumerate(ROTORS_FRD)))


def build_model(world_mjcf, spawn):
    import mujoco
    if world_mjcf:
        root = ET.parse(world_mjcf).getroot()
    else:
        root = ET.fromstring('<mujoco><worldbody><geom type="plane" size="50 50 0.1"/></worldbody></mujoco>')
    option = root.find('option')
    if option is None:
        option = ET.SubElement(root, 'option')
    option.set('timestep', str(DT))
    option.set('gravity', '0 0 -9.81')
    root.find('worldbody').append(ET.fromstring(vehicle_body(spawn)))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    return model, mujoco.MjData(model)


def quat_rotate(q, v):
    """Rotate v by unit quaternion q = (w, x, y, z) (body -> world)."""
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return R @ v, R


def flu_to_frd(v):
    return np.array([v[0], -v[1], -v[2]])


def enu_to_ned(v):
    return np.array([v[1], v[0], -v[2]])


def pressure_hpa(altitude):
    return 1013.25 * (1 - 2.25577e-5 * altitude) ** 5.25588


class Plant:
    def __init__(self, world_mjcf, spawn, seed=0):
        import mujoco
        self.mujoco = mujoco
        self.model, self.data = build_model(world_mjcf, spawn)
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'px4_vehicle')
        self.omega = np.zeros(4)
        self.command = np.zeros(4)
        self.armed = False
        self.time_us = 0
        self.rng = np.random.RandomState(seed)
        self.last_velocity = np.zeros(3)
        mujoco.mj_forward(self.model, self.data)

    # state helpers (world ENU, body FLU)
    def pose(self):
        return self.data.qpos[:3].copy(), self.data.qpos[3:7].copy()

    def velocity(self):
        return self.data.qvel[:3].copy()

    def body_rates(self):
        return self.data.qvel[3:6].copy()          # MuJoCo free joint: angular velocity in the body frame

    def apply_rotors(self):
        target = OMEGA_ZERO + OMEGA_SCALE * np.clip(self.command, 0., 1.) if self.armed else np.zeros(4)
        for i in range(4):
            tau = TAU_UP if target[i] > self.omega[i] else TAU_DOWN
            self.omega[i] += (target[i] - self.omega[i]) * min(1., DT / tau)
        thrust = MOTOR_CONSTANT * self.omega ** 2
        _, q = self.pose()
        _, R = quat_rotate(q, np.zeros(3))
        force_body = np.array([0., 0., thrust.sum()])
        torque_frd = np.zeros(3)
        for (rx, ry, km), t in zip(ROTORS_FRD, thrust):
            r_frd = np.array([rx, ry, 0.])
            torque_frd += np.cross(r_frd, np.array([0., 0., -t]))   # thrust is -z in FRD
            torque_frd[2] += km * t                                   # reaction torque (PX4 CA convention)
        torque_flu = flu_to_frd(torque_frd)                          # the same axis flip maps FRD -> FLU
        self.data.xfrc_applied[self.body, :3] = R @ force_body
        self.data.xfrc_applied[self.body, 3:] = R @ torque_flu

    def step(self):
        self.apply_rotors()
        self.mujoco.mj_step(self.model, self.data)
        self.time_us += int(DT * 1e6)

    def sensors(self):
        position, q = self.pose()
        velocity = self.velocity()
        acceleration = (velocity - self.last_velocity) / DT
        self.last_velocity = velocity
        _, R = quat_rotate(q, np.zeros(3))
        specific_force = R.T @ (acceleration + np.array([0., 0., 9.81]))
        accel = flu_to_frd(specific_force) + self.rng.normal(0, NOISE['accel'], 3)
        gyro = flu_to_frd(self.body_rates()) + self.rng.normal(0, NOISE['gyro'], 3)
        mag_enu = np.array([MAG_NED[1], MAG_NED[0], -MAG_NED[2]])
        mag = flu_to_frd(R.T @ mag_enu) + self.rng.normal(0, NOISE['mag'], 3)
        altitude = HOME['alt'] + position[2] + self.rng.normal(0, NOISE['baro_m'])
        return dict(accel=accel, gyro=gyro, mag=mag, pressure=pressure_hpa(altitude), altitude=altitude,
                    temperature=15. - .0065 * altitude)

    def gps(self):
        position, _ = self.pose()
        velocity = self.velocity()
        lat = HOME['lat'] + math.degrees(position[1] / 6378137.)
        lon = HOME['lon'] + math.degrees(position[0] / (6378137. * math.cos(math.radians(HOME['lat']))))
        ned = enu_to_ned(velocity)
        speed = float(np.hypot(ned[0], ned[1]))
        course = (math.degrees(math.atan2(ned[1], ned[0])) + 360.) % 360.
        return dict(lat=int(lat * 1e7), lon=int(lon * 1e7), alt=int((HOME['alt'] + position[2]) * 1000),
                    vel=int(speed * 100), vn=int(ned[0] * 100), ve=int(ned[1] * 100), vd=int(ned[2] * 100),
                    cog=int(course * 100) if speed > .05 else 65535)


def run(args):
    from pymavlink import mavutil
    plant = Plant(args.world_mjcf, (args.spawn_x, args.spawn_y, args.spawn_z))
    publisher = None
    if args.ros:
        import rclpy
        from nav_msgs.msg import Odometry
        rclpy.init()
        node = rclpy.create_node('px4_mujoco_plant')
        publisher = node.create_publisher(Odometry, '/px4/odometry_truth', 10)
    print('PX4 MuJoCo plant listening on tcp 4560', flush=True)
    link = mavutil.mavlink_connection('tcpin:127.0.0.1:4560', source_system=1, source_component=51)
    link.wait_heartbeat(timeout=120)
    print('PX4 connected', flush=True)
    lockstep, steps, last_heartbeat, last_truth = False, 0, 0., 0.
    while True:
        sensors = plant.sensors()
        link.mav.hil_sensor_send(plant.time_us, *sensors['accel'], *sensors['gyro'], *sensors['mag'],
                                 sensors['pressure'], 0., sensors['altitude'], sensors['temperature'], FIELDS, 0)
        if steps % 25 == 0:                          # 10 Hz GPS
            gps = plant.gps()
            link.mav.hil_gps_send(plant.time_us, 3, gps['lat'], gps['lon'], gps['alt'], 100, 100, gps['vel'],
                                  gps['vn'], gps['ve'], gps['vd'], gps['cog'], 10)
        now = time.monotonic()
        if now - last_heartbeat > 1.:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_heartbeat = now
        # Lockstep: once PX4 has answered once, wait for its answer to every sensor message.
        while True:
            message = link.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=lockstep, timeout=2.)
            if message is None:
                if lockstep:
                    print('PX4 lockstep timeout; waiting', flush=True)
                    continue
                break
            lockstep = True
            plant.armed = bool(message.mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            plant.command = np.asarray(message.controls[:4], dtype=float)
            break
        plant.step()
        steps += 1
        if publisher is not None and now - last_truth > .05:
            from nav_msgs.msg import Odometry
            position, q = plant.pose()
            message = Odometry()
            message.header.frame_id, message.child_frame_id = 'map', 'base_link'
            message.header.stamp = node.get_clock().now().to_msg()
            p = message.pose.pose.position
            p.x, p.y, p.z = (float(v) for v in position)
            o = message.pose.pose.orientation
            o.w, o.x, o.y, o.z = (float(v) for v in q)
            v = message.twist.twist.linear
            v.x, v.y, v.z = (float(v_) for v_ in plant.velocity())
            publisher.publish(message)
            last_truth = now
        if args.real_time:
            time.sleep(max(0., DT - (time.monotonic() - now)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world-mjcf', default='')
    parser.add_argument('--spawn-x', type=float, default=0.)
    parser.add_argument('--spawn-y', type=float, default=0.)
    parser.add_argument('--spawn-z', type=float, default=0.)
    parser.add_argument('--ros', action='store_true', help='publish /px4/odometry_truth')
    parser.add_argument('--real-time', action='store_true', help='pace physics to wall time')
    run(parser.parse_args(argv))


if __name__ == '__main__':
    main()
