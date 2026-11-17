"""MOLAR deck belt for PyBullet (R11 payload transfer), as on MuJoCo.

PyBullet has no moving-surface contact, so the belt owns the deck's
friction: the deck link gets zero lateral friction, and every loose body
touching it gets a horizontal force (``robot_lab_utils.belt.belt_force``)
driving its velocity relative to the deck towards the belt velocity,
bounded by Coulomb friction from the measured contact normal forces. The
belt speed is the measured roller rate times the roller radius. PyBullet
clears external forces after every step, so nothing accumulates.
"""
import numpy as np
import pybullet as p

from robot_lab_utils.belt import belt_force


class PyBulletBelt:
    def __init__(self, robot, deck_link, roller_joint, roller_radius, bodies, mu=0.8, gain=20.0):
        self.robot, self.deck, self.roller = robot, int(deck_link), int(roller_joint)
        self.radius, self.mu, self.gain = float(roller_radius), float(mu), float(gain)
        self.bodies = bodies             # {name: body id}, the world's loose objects
        p.changeDynamics(robot, self.deck, lateralFriction=0.0)

    def step(self):
        """Apply this step's belt forces; returns {name: force along the deck axis}."""
        belt = p.getJointState(self.robot, self.roller)[1] * self.radius
        state = p.getLinkState(self.robot, self.deck, computeLinkVelocity=1)
        rotation = np.array(p.getMatrixFromQuaternion(state[5])).reshape(3, 3)
        axis = rotation[:, 0]
        deck_velocity = np.array(state[6])
        report = {}
        for name, body in self.bodies.items():
            normal = sum(c[9] for c in p.getContactPoints(bodyA=self.robot, bodyB=body, linkIndexA=self.deck))
            if normal <= 0.:
                continue
            mass = p.getDynamicsInfo(body, -1)[0]
            velocity = np.array(p.getBaseVelocity(body)[0])
            force = belt_force(belt, axis, velocity - deck_velocity, mass, normal, self.mu, self.gain)
            position = p.getBasePositionAndOrientation(body)[0]
            p.applyExternalForce(body, -1, force.tolist(), position, p.WORLD_FRAME)
            report[name] = float(np.dot(force, axis / max(np.linalg.norm(axis[:2]), 1e-9)))
        return report
