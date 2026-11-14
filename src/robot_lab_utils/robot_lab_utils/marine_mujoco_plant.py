#!/usr/bin/env python3
"""MuJoCo marine plant for the native ROV force interface (ROADMAP R10.4).

The unchanged ``marine_controller`` publishes one force per thruster on
``/marine/thrusters/<joint>/force`` and observes ``/marine/odometry`` and
``/clock``. This plant consumes the same topics, so the controller runs on
MuJoCo as it does on Gazebo. Water forces are a port of the Fortress
systems the URDF declares:

- Buoyancy: ρ g × collision-box displacement at the centre of volume
  (Fortress Buoyancy, uniform fluid below the surface; the part of each box
  above the surface displaces nothing).
- Hydrodynamics: linear and quadratic damping on the velocity relative to the
  current, and added mass on its rate (the Fossen coefficients of the
  Fortress Hydrodynamics plugin: xU, xUabsU, xDotU, ...). The added-mass
  Coriolis terms are omitted.
- Thrusters: the commanded force along each thruster's source axis at its
  source mount, clipped to its source bounds (MarineModel); no propeller
  dynamics. Commands older than 0.5 s are zero.

World collision is a seabed plane at the water bottom; other world geometry
is not converted. Simulator pose feeds the controller exactly as Gazebo's
odometry publisher does; it is a plant observation, not an estimator.
"""
import argparse
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from robot_lab_utils.marine_model import MarineModel, hydro_coefficients, rotation

def displacement_boxes(description):
    """Collision boxes of all links (position in base frame, size), as MarineModel counts them."""
    root = ET.fromstring(description)
    boxes = []
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            box = collision.find('geometry/box')
            if box is None:
                continue
            origin = collision.find('origin')
            xyz = np.array([float(v) for v in origin.get('xyz', '0 0 0').split()]) if origin is not None else np.zeros(3)
            if link.get('name') != 'base_link':
                joint = next((j for j in root.findall('joint') if j.find('child').get('link') == link.get('name')), None)
                if joint is not None and joint.find('origin') is not None:
                    xyz = xyz + np.array([float(v) for v in joint.find('origin').get('xyz', '0 0 0').split()])
            boxes.append((xyz, np.array([float(v) for v in box.get('size').split()])))
    return boxes


def mjcf(model, boxes, spawn, bottom_z, inertia):
    """Single rigid body with the source mass/inertia; water forces come from the plant."""
    xyz, size = boxes[0]
    hx, hy, hz = size / 2
    return f"""<mujoco model="marine_plant">
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>
  <worldbody>
    <geom name="seabed" type="plane" size="200 200 .1" pos="0 0 {bottom_z}" friction="0.6 0.005 0.0001"/>
    <body name="rov" pos="{spawn[0]} {spawn[1]} {spawn[2]}" euler="0 0 {spawn[3]}">
      <freejoint/>
      <inertial pos="{model.centre[0]} {model.centre[1]} {model.centre[2]}" mass="{model.mass}"
                diaginertia="{inertia[0]} {inertia[1]} {inertia[2]}"/>
      <geom name="hull" type="box" pos="{xyz[0]} {xyz[1]} {xyz[2]}" size="{hx} {hy} {hz}" mass="0" contype="1" conaffinity="1"/>
    </body>
  </worldbody>
</mujoco>"""


class MarinePlant:
    def __init__(self, description, spawn, surface_z, bottom_z, density=1025.):
        import mujoco
        self.mujoco = mujoco
        self.model_source = MarineModel(description, density)
        root = ET.fromstring(description)
        origin = root.find("link[@name='base_link']/inertial/origin")
        self.model_source.centre = (np.array([float(v) for v in origin.get('xyz', '0 0 0').split()])
                                    if origin is not None else np.zeros(3))
        self.hydro = hydro_coefficients(description)
        self.boxes = displacement_boxes(description)
        self.mounts = self._thruster_mounts(root)
        tensor = root.find("link[@name='base_link']/inertial/inertia")
        inertia = (np.array([float(tensor.get(k)) for k in ('ixx', 'iyy', 'izz')]) if tensor is not None
                   else np.full(3, self.model_source.mass * .01))
        self.surface, self.bottom, self.density = float(surface_z), float(bottom_z), float(density)
        self.model = mujoco.MjModel.from_xml_string(mjcf(self.model_source, self.boxes, spawn, self.bottom, inertia))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.body = self.model.body('rov').id
        self.forces = np.zeros(len(self.model_source.joints))
        self.current = self.hydro['default_current'].copy()
        self.previous_relative = None
        self.wrench = np.zeros(6)

    def _thruster_mounts(self, root):
        mounts = []
        for name in self.model_source.joints:
            joint = root.find("joint[@name='%s']" % name)
            pose = joint.find('origin')
            position = np.array([float(v) for v in pose.get('xyz').split()])
            direction = rotation([float(v) for v in pose.get('rpy', '0 0 0').split()]) @ np.array(
                [float(v) for v in joint.find('axis').get('xyz').split()])
            mounts.append((position, direction / np.linalg.norm(direction)))
        return mounts

    def command(self, index, force):
        low, high = self.model_source.bounds[index]
        self.forces[index] = max(low, min(high, float(force))) if math.isfinite(force) else 0.

    def pose(self):
        q = self.data.qpos
        return q[:3].copy(), np.array([q[4], q[5], q[6], q[3]])  # xyz, quaternion xyzw

    def _rotation(self):
        return self.data.xmat[self.body].reshape(3, 3).copy()

    def step(self, dt):
        mujoco = self.mujoco
        steps = max(1, int(round(dt / self.model.opt.timestep)))
        for _ in range(steps):
            R = self._rotation()
            origin = self.data.xpos[self.body].copy()      # base_link frame origin in world
            com = self.data.xipos[self.body].copy()
            force, torque = np.zeros(3), np.zeros(3)
            # Buoyancy of each box, clipped to the submerged part (uniform fluid below the surface).
            for xyz, size in self.boxes:
                centre = origin + R @ xyz
                height = abs(R[2, :]) @ size                 # world vertical extent of the box
                top, bottom = centre[2] + height / 2, centre[2] - height / 2
                fraction = min(1., max(0., (self.surface - bottom) / max(height, 1e-9)))
                if fraction <= 0:
                    continue
                lift = np.array([0., 0., self.density * 9.81 * np.prod(size) * fraction])
                cob = centre.copy()
                cob[2] = bottom + fraction * height / 2
                force += lift
                torque += np.cross(cob - com, lift)
            # Hydrodynamics on the velocity relative to the current, in the body frame.
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.body, velocity, 1)
            angular, linear = velocity[:3], velocity[3:]
            relative = np.r_[linear - R.T @ self.current, angular]
            rate = np.zeros(6) if self.previous_relative is None else (relative - self.previous_relative) / self.model.opt.timestep
            self.previous_relative = relative
            body = (self.hydro['linear'] + self.hydro['quadratic'] * np.abs(relative)) * relative - self.hydro['added'] * rate
            force += R @ body[:3]
            torque += R @ body[3:]
            # Thrusters at their source mounts.
            for (position, direction), thrust in zip(self.mounts, self.forces):
                world_force = R @ (direction * thrust)
                force += world_force
                torque += np.cross(origin + R @ position - com, world_force)
            self.wrench = np.r_[force, torque]
            self.data.xfrc_applied[self.body, :3] = force
            self.data.xfrc_applied[self.body, 3:] = torque
            mujoco.mj_step(self.model, self.data)
        return self.data.time


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Vector3
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Float64, String
    import json
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent()
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    for key in ('description', 'spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw', 'surface_z', 'bottom_z', 'density'):
        parser.add_argument('--' + key.replace('_', '-'), required=key == 'description', default='0')
    options = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    plant = MarinePlant(Path(options.description).read_text(),
                        [float(options.spawn_x), float(options.spawn_y), float(options.spawn_z), float(options.spawn_yaw)],
                        float(options.surface_z), float(options.bottom_z), float(options.density))
    node = Node('marine_mujoco_plant')
    stamps = [-math.inf] * len(plant.forces)
    def subscribe(index, joint):
        def receive(msg):
            plant.command(index, msg.data)
            stamps[index] = time.monotonic()
        node.create_subscription(Float64, '/marine/thrusters/' + joint + '/force', receive, 10)
    for index, joint in enumerate(plant.model_source.joints):
        subscribe(index, joint)
    def current(msg):
        values = np.array([msg.x, msg.y, msg.z])
        if np.isfinite(values).all():
            plant.current = values
    node.create_subscription(Vector3, '/marine/current', current, 10)
    clock = node.create_publisher(Clock, '/clock', 10)
    odometry = node.create_publisher(Odometry, '/marine/odometry', 10)
    state = node.create_publisher(String, '/marine/mujoco_plant', 10)
    period, last_odom, last_state = .01, -math.inf, -math.inf
    next_tick = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.)
            now = time.monotonic()
            for index, at in enumerate(stamps):
                if now - at > .5:
                    plant.forces[index] = 0.            # stale thruster commands are zero
            sim = plant.step(period)
            message = Clock()
            message.clock.sec, message.clock.nanosec = int(sim), int((sim % 1) * 1e9)
            clock.publish(message)
            if sim - last_odom >= .02:
                last_odom = sim
                position, quaternion = plant.pose()
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = 'map', 'base_link'
                odom.header.stamp = message.clock
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, position)
                (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                 odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = map(float, quaternion)
                odometry.publish(odom)
            if now - last_state > .2:
                last_state = now
                state.publish(String(data=json.dumps(dict(sim_time=sim, thrust_N=plant.forces.tolist(),
                    wrench=plant.wrench.round(4).tolist(), current=plant.current.tolist(),
                    scope='MuJoCo rigid body with ported Fortress buoyancy/hydrodynamics/thruster forces; seabed plane only'))))
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
