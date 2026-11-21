"""Tracks for PyBullet (R10.7): the shared track law on frictionless track links.

Bullet has no moving contact surface, so the track links get zero friction
and ``robot_lab_utils.track_drive`` supplies the tangential force at each of
their contacts with the world, applied with ``applyExternalForce`` before
every ``stepSimulation`` (Bullet clears external forces after each step).
"""
import numpy as np

from robot_lab_utils.track_drive import TrackDrive, parse_config


class TrackPlant:
    def __init__(self, sdk, robot, config, base_link='base_link'):
        self.p, self.robot = sdk, robot
        config = parse_config(config)
        links = {sdk.getBodyInfo(robot)[0].decode(): -1}
        for i in range(sdk.getNumJoints(robot)):
            links[sdk.getJointInfo(robot, i)[12].decode()] = i
        if base_link not in links:
            raise ValueError('Track hull link missing: ' + base_link)
        self.base = links[base_link]
        self.links = {}
        for side in ('left', 'right'):
            for name in config[side]:
                if name not in links:
                    raise ValueError('Track link missing from the loaded robot: ' + name)
                self.links[links[name]] = side
        mass = sum(sdk.getDynamicsInfo(robot, index)[0] for index in links.values())
        self.drive = TrackDrive(config, mass)
        for index in self.links:
            sdk.changeDynamics(robot, index, lateralFriction=0., rollingFriction=0., spinningFriction=0.)
        # The spawner leaves revolute joints unpowered and Bullet does not
        # hold a zero-width limit; the fixed-flipper source declares its
        # flippers with lower == upper, so hold those at that angle.
        self.held = []
        for i in range(sdk.getNumJoints(robot)):
            info = sdk.getJointInfo(robot, i)
            if info[2] == sdk.JOINT_REVOLUTE and info[8] == info[9]:
                sdk.setJointMotorControl2(robot, i, sdk.POSITION_CONTROL, targetPosition=float(info[8]),
                                          force=float(info[10]) or 25.)
                self.held.append(info[1].decode())
        self.report = dict(contacts=0, normal_N=0.0)

    def reset(self):
        self.drive.reset()

    def _state(self, index):
        """World CoM position, linear and angular velocity of one link."""
        if index < 0:
            position, _ = self.p.getBasePositionAndOrientation(self.robot)
            linear, angular = self.p.getBaseVelocity(self.robot)
        else:
            state = self.p.getLinkState(self.robot, index, computeLinkVelocity=1)
            position, linear, angular = state[0], state[6], state[7]
        return np.asarray(position), np.asarray(linear), np.asarray(angular)

    def _hull(self):
        """The hull frame origin and rotation (link frame, not its CoM)."""
        if self.base < 0:
            position, quaternion = self.p.getBasePositionAndOrientation(self.robot)
        else:
            state = self.p.getLinkState(self.robot, self.base, computeForwardKinematics=True)
            position, quaternion = state[4], state[5]
        return np.asarray(position), np.asarray(self.p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)

    def step(self, now, dt):
        """Before stepSimulation: apply this step's track forces."""
        self.drive.advance(now, dt)
        sides, points, normals, load, velocities, owners, states = [], [], [], [], [], [], {}
        for contact in self.p.getContactPoints(bodyA=self.robot):
            link = contact[3]
            if contact[2] == self.robot or link not in self.links:
                continue
            if link not in states:
                states[link] = self._state(link)
            centre, linear, angular = states[link]
            point = np.asarray(contact[5])
            sides.append(self.links[link])
            points.append(point)
            # contactNormalOnB points from B (the world) into A (the robot).
            normals.append(contact[7])
            load.append(max(0., float(contact[9])))
            velocities.append(linear + np.cross(angular, point - centre))
            owners.append(link)
        origin, hull = self._hull()
        forces = self.drive.forces(sides, np.asarray(normals).reshape(-1, 3), load,
                                   np.asarray(velocities).reshape(-1, 3), hull[:, 1], dt,
                                   np.asarray(points).reshape(-1, 3), origin, hull[:, 2])
        # Per contact point: Bullet's applyExternalTorque frame handling has
        # varied between releases, a force at the point needs no torque.
        for link, point, force in zip(owners, points, forces):
            self.p.applyExternalForce(self.robot, link, force.tolist(), point.tolist(), self.p.WORLD_FRAME)
        self.report = dict(contacts=len(sides), normal_N=float(sum(load)))
