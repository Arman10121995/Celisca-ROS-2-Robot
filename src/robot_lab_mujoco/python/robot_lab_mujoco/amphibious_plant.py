"""The amphibious rover's drive, suspension, propellers and water for MuJoCo (R10.7).

``robot_lab_utils.amphibious_drive`` computes the efforts and forces; this
applies them through ``qfrc_applied`` (joint efforts at their DOFs, world
forces with ``mj_applyFT``) and replaces, never accumulates, them every step.
"""
import numpy as np

from robot_lab_utils.amphibious_drive import AmphibiousDrive


class AmphibiousPlant:
    def __init__(self, model, data, description, config, base_body):
        import mujoco
        self.mj, self.model, self.data = mujoco, model, data
        self.drive = AmphibiousDrive(description, config)
        self.base = int(base_body)
        self.joints = {}
        for name in ('front_left_wheel_joint', 'rear_left_wheel_joint', 'front_right_wheel_joint', 'rear_right_wheel_joint',
                     'front_left_suspension_joint', 'rear_left_suspension_joint', 'front_right_suspension_joint',
                     'rear_right_suspension_joint'):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint < 0:
                raise ValueError('Amphibious joint missing from the compiled model: ' + name)
            self.joints[name] = (int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint]))
        self.bodies = {}
        for link in {shape[0] for shape in self.drive.shapes}:
            body = self.base if link == 'base_link' else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link)
            if body < 0:
                raise ValueError('Amphibious collision link missing from the compiled model: ' + link)
            self.bodies[link] = int(body)
        self.bodies['base_link'] = self.base
        self.applied = np.zeros(model.nv)
        self.velocity = np.zeros(6)

    def reset(self):
        self.drive.reset()
        self.applied[:] = 0.0

    def step(self, now):
        data, model = self.data, self.model
        data.qfrc_applied[:] -= self.applied
        self.applied[:] = 0.0
        joints = {name: (float(data.qpos[q]), float(data.qvel[d])) for name, (q, d) in self.joints.items()}
        poses = {link: (data.xpos[body].copy(), data.xmat[body].reshape(3, 3).copy()) for link, body in self.bodies.items()}
        # mj_objectVelocity reports the linear velocity at the centre of mass.
        self.mj.mj_objectVelocity(model, data, self.mj.mjtObj.mjOBJ_BODY, self.base, self.velocity, 0)
        base_origin, base_rotation = poses['base_link']
        com = data.xipos[self.base].copy()
        linear = self.velocity[3:] + np.cross(self.velocity[:3], base_origin - com)
        efforts, forces, (drag, torque, at) = self.drive.step(now, base_rotation, base_origin, com, linear,
                                                              self.velocity[:3].copy(), joints, poses)
        for name, effort in efforts.items():
            self.applied[self.joints[name][1]] += effort
        for link, force, point in forces:
            self.mj.mj_applyFT(model, data, force, np.zeros(3), point, self.bodies[link], self.applied)
        self.mj.mj_applyFT(model, data, drag, torque, at, self.base, self.applied)
        data.qfrc_applied[:] += self.applied
