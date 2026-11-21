"""The amphibious rover's drive, suspension, propellers and water for PyBullet (R10.7).

``robot_lab_utils.amphibious_drive`` computes the efforts and forces; joint
efforts go through TORQUE_CONTROL (with the default velocity motors off)
and world forces through ``applyExternalForce`` before every
``stepSimulation`` (Bullet clears external forces after each step).
"""
import numpy as np

from robot_lab_utils.amphibious_drive import AmphibiousDrive, PROPELLERS, SUSPENSION, WHEELS


class AmphibiousPlant:
    def __init__(self, sdk, robot, description, config):
        self.p, self.robot = sdk, robot
        self.drive = AmphibiousDrive(description, config)
        self.links = {sdk.getBodyInfo(robot)[0].decode(): -1}
        joints = {}
        for i in range(sdk.getNumJoints(robot)):
            info = sdk.getJointInfo(robot, i)
            joints[info[1].decode()] = i
            self.links[info[12].decode()] = i
        self.joints = {}
        for name in WHEELS + SUSPENSION:
            if name not in joints:
                raise ValueError('Amphibious joint missing from the loaded robot: ' + name)
            self.joints[name] = joints[name]
            sdk.setJointMotorControl2(robot, joints[name], sdk.VELOCITY_CONTROL, targetVelocity=0, force=0)
        # The propellers spin freely, as Gazebo's Thruster does not drive the joint here.
        for name in PROPELLERS:
            if name in joints:
                sdk.setJointMotorControl2(robot, joints[name], sdk.VELOCITY_CONTROL, targetVelocity=0, force=0)
        for link in {shape[0] for shape in self.drive.shapes} | {'base_link'}:
            if link not in self.links:
                raise ValueError('Amphibious collision link missing from the loaded robot: ' + link)

    def reset(self):
        self.drive.reset()

    def _frame(self, index):
        """Link frame origin, rotation, CoM, linear and angular velocity (world)."""
        if index < 0:
            com, quaternion = self.p.getBasePositionAndOrientation(self.robot)
            linear, angular = self.p.getBaseVelocity(self.robot)
            inertial = self.p.getDynamicsInfo(self.robot, -1)
            inverse = self.p.invertTransform(inertial[3], inertial[4])
            origin, quaternion = self.p.multiplyTransforms(com, quaternion, *inverse)
        else:
            state = self.p.getLinkState(self.robot, index, computeLinkVelocity=1, computeForwardKinematics=True)
            com, origin, quaternion, linear, angular = state[0], state[4], state[5], state[6], state[7]
        R = np.asarray(self.p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
        return np.asarray(origin), R, np.asarray(com), np.asarray(linear), np.asarray(angular)

    def step(self, now):
        joints = {name: tuple(self.p.getJointState(self.robot, index)[:2]) for name, index in self.joints.items()}
        frames = {link: self._frame(self.links[link]) for link in {shape[0] for shape in self.drive.shapes} | {'base_link'}}
        origin, R, com, linear, angular = frames['base_link']
        # Velocity of the hull frame origin from its CoM velocity.
        origin_linear = linear + np.cross(angular, origin - com)
        poses = {link: (frame[0], frame[1]) for link, frame in frames.items()}
        efforts, forces, (drag, torque, at) = self.drive.step(now, R, origin, com, origin_linear, angular, joints, poses)
        for name, effort in efforts.items():
            self.p.setJointMotorControl2(self.robot, self.joints[name], self.p.TORQUE_CONTROL, force=float(effort))
        base = self.links['base_link']
        for link, force, point in forces:
            self.p.applyExternalForce(self.robot, self.links[link], force.tolist(), point.tolist(), self.p.WORLD_FRAME)
        # The hull drag torque as a force couple about the CoM (no applyExternalTorque frame ambiguity).
        self.p.applyExternalForce(self.robot, base, drag.tolist(), at.tolist(), self.p.WORLD_FRAME)
        size = float(np.linalg.norm(torque))
        if size > 1e-9:
            axis = torque / size
            arm = np.cross(axis, [1., 0., 0.])
            if np.linalg.norm(arm) < 1e-6:
                arm = np.cross(axis, [0., 1., 0.])
            arm = arm / np.linalg.norm(arm) * .5
            couple = np.cross(axis, arm)
            couple = couple / np.linalg.norm(couple) * size / .5  # two forces F at +-arm/2 give torque F*|arm|
            self.p.applyExternalForce(self.robot, base, couple.tolist(), (at + arm / 2).tolist(), self.p.WORLD_FRAME)
            self.p.applyExternalForce(self.robot, base, (-couple).tolist(), (at - arm / 2).tolist(), self.p.WORLD_FRAME)
