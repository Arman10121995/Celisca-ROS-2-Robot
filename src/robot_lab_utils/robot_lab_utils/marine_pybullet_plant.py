#!/usr/bin/env python3
"""PyBullet marine plant for the native ROV force interface (ROADMAP R10.4).

The same ported Fortress buoyancy, hydrodynamics and thrusters as the MuJoCo
plant (``marine_mujoco_plant.WaterForces``), integrated by Bullet: one rigid
body with the source mass and inertia, a seabed plane at the water bottom,
and the water/thruster wrench applied about the centre of mass each 2 ms
step. Topics are those of the MuJoCo plant, so the unchanged
``marine_controller`` runs on it.
"""
import math

import numpy as np

from robot_lab_utils.marine_mujoco_plant import WaterForces, plant_arguments, run_plant

TIMESTEP = .002     # s, as the MuJoCo plant


class MarinePyBulletPlant(WaterForces):
    engine = 'pybullet'
    scope = 'PyBullet rigid body with ported Fortress buoyancy/hydrodynamics/thruster forces; seabed plane only'

    def __init__(self, description, spawn, surface_z, bottom_z, density=1025.):
        import pybullet
        self.p = pybullet
        self._load(description, surface_z, bottom_z, density)
        self.client = pybullet.connect(pybullet.DIRECT)
        p, client = pybullet, self.client
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        p.setTimeStep(TIMESTEP, physicsClientId=client)
        seabed = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=client)
        p.createMultiBody(0, seabed, basePosition=[0, 0, self.bottom], physicsClientId=client)
        xyz, size = self.boxes[0]
        centre = np.asarray(self.model_source.centre, dtype=float)
        # Bullet's base frame is the inertial frame: the hull box sits at its
        # base_link offset minus the centre of mass.
        hull = p.createCollisionShape(p.GEOM_BOX, halfExtents=(size / 2).tolist(),
                                      collisionFramePosition=(xyz - centre).tolist(), physicsClientId=client)
        yaw = float(spawn[3])
        self.spawn_rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.], [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
        start = np.asarray(spawn[:3], dtype=float) + self.spawn_rotation @ centre
        self.body = p.createMultiBody(self.model_source.mass, hull, basePosition=start.tolist(),
                                      baseOrientation=p.getQuaternionFromEuler([0, 0, yaw]), physicsClientId=client)
        p.changeDynamics(self.body, -1, localInertiaDiagonal=self.inertia.tolist(), linearDamping=0., angularDamping=0.,
                         lateralFriction=.6, physicsClientId=client)
        self.time = 0.

    def _state(self):
        p = self.p
        com, quaternion = p.getBasePositionAndOrientation(self.body, physicsClientId=self.client)
        R = np.array(p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
        com = np.asarray(com)
        return com, np.asarray(quaternion), R, com - R @ np.asarray(self.model_source.centre, dtype=float)

    def pose(self):
        _, quaternion, _, origin = self._state()
        return origin, quaternion                                   # base_link xyz, quaternion xyzw

    def step(self, dt):
        p = self.p
        for _ in range(max(1, int(round(dt / TIMESTEP)))):
            com, _, R, origin = self._state()
            linear_com, angular = (np.asarray(v) for v in p.getBaseVelocity(self.body, physicsClientId=self.client))
            # base_link origin velocity, in the body frame (as mj_objectVelocity).
            linear_origin = linear_com + np.cross(angular, origin - com)
            force, torque = self.water_wrench(R, origin, com, R.T @ linear_origin, R.T @ angular, TIMESTEP)
            p.applyExternalForce(self.body, -1, force.tolist(), com.tolist(), p.WORLD_FRAME, physicsClientId=self.client)
            p.applyExternalTorque(self.body, -1, torque.tolist(), p.WORLD_FRAME, physicsClientId=self.client)
            p.stepSimulation(physicsClientId=self.client)
            self.time += TIMESTEP
        return self.time


def main(args=None):
    import rclpy
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent()
    rclpy.init(args=args)
    run_plant(MarinePyBulletPlant(*plant_arguments()), 'marine_pybullet_plant')


if __name__ == '__main__':
    main()
