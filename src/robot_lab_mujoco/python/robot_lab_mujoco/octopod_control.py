"""Source octopod position-servo gait, inside the native physics loop.

Metachronal equations adapted from light-robotics/robot_rl_lab at
c2cc2437393155760eeea5f588c5912f4654067c (MIT, copyright 2026
light-robotics). The complete license is retained beside the installed model.
This is a scripted prior, not a trained policy or terrain certificate.
"""
import math
import time
from pathlib import Path

import numpy as np
import yaml

from robot_lab_mujoco.unitree_policy import file_digest, projected_gravity


def metachronal_delta(phase, side_scale):
    """NumPy port of the source flat gait; side scaling is the lab drive adapter."""
    starts = np.array([0, 3, 7, 10, 14, 17, 21, 24])
    side = np.tile([1., -1.], 4)
    angle = phase + np.repeat(np.arange(4), 2)*math.pi/2 + np.tile([0., math.pi], 4)
    delta = np.zeros(27)
    delta[starts] = side * .45 * np.cos(angle) * side_scale
    delta[starts+1] = side * .22 * np.maximum(np.sin(angle), 0.) * np.abs(side_scale)
    return delta


class NativeOctopodControl:
    def __init__(self, model, data, manifest_path, native_path):
        import mujoco
        self.mj, self.model, self.data = mujoco, model, data
        cfg = yaml.safe_load(Path(manifest_path).read_text())
        if cfg.get('controller') != 'light_robotics_octopod' or cfg.get('schema_version') != 1:
            raise ValueError('Unknown source octopod contract')
        if Path(native_path).resolve() != Path(cfg['native_model']).resolve():
            raise ValueError('Source gait requires its exact native model')
        for resource in cfg['resources']:
            if file_digest(resource['path']) != resource['sha256']:
                raise ValueError('Changed octopod source: '+resource['path'])
        names = cfg['joint_names']
        self.actuators = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in names])
        joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
        if len(names) != 27 or len(set(names)) != 27 or min(joints+list(self.actuators)) < 0:
            raise ValueError('Source octopod needs all 27 original joints/servos')
        for j, a, n in zip(joints, self.actuators, names):
            kp = 140. if n.startswith('spine') else 80.
            if (model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE
                    or model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT
                    or model.actuator_trnid[a, 0] != j
                    or model.actuator_dyntype[a] != mujoco.mjtDyn.mjDYN_NONE
                    or model.actuator_gaintype[a] != mujoco.mjtGain.mjGAIN_FIXED
                    or model.actuator_biastype[a] != mujoco.mjtBias.mjBIAS_AFFINE
                    or not np.allclose(model.actuator_gear[a], [1, 0, 0, 0, 0, 0])
                    or not np.allclose(model.actuator_gainprm[a], [kp]+[0.]*9)
                    or not np.allclose(model.actuator_biasprm[a], [0., -kp]+[0.]*8)
                    or not model.actuator_forcelimited[a]
                    or not np.allclose(model.actuator_forcerange[a], [-2.45, 2.45])):
                raise ValueError('Original position servo/force envelope differs: '+n)
        self.qpos = np.array([model.jnt_qposadr[j] for j in joints], dtype=int)
        self.ranges = model.jnt_range[joints].copy()
        self.nominal = np.asarray(cfg['nominal'], dtype=float)
        if (self.nominal.shape != (27,) or not np.all(np.isfinite(self.nominal))
                or not np.all(model.jnt_limited[joints])
                or np.any(self.nominal < self.ranges[:, 0]) or np.any(self.nominal > self.ranges[:, 1])):
            raise ValueError('Invalid source nominal stance')
        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
        if root < 0 or model.jnt_type[root] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError('Source free root is required')
        self.base_q = int(model.jnt_qposadr[root])
        self.foot_geoms = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f'{s}{i}_foot_geom')
                           for i in range(1, 5) for s in ('L', 'R')]
        if min(self.foot_geoms) < 0:
            raise ValueError('Missing source feet')
        self.dt = float(model.opt.timestep)
        if not math.isclose(self.dt, .002):
            raise ValueError('Source gait requires 2 ms physics')
        self.robot = cfg['robot']
        self.reset()

    def reset(self):
        self.steps, self.phase = 0, 0.
        self.command = np.zeros(3)
        self.scale = np.zeros(8)
        self.last_command = None
        self.target = self.nominal.copy()
        self.fault, self.status = '', 'holding_nominal'
        self.data.ctrl[self.actuators] = self.target

    def receive(self, vx, vy, yaw, now=None):
        values = np.array([vx, vy, yaw])
        if not np.all(np.isfinite(values)):
            self.fault = 'Non-finite command; reset required'
            self.command[:] = 0
            return False
        if self.fault:
            return False
        # Source prior has no lateral gait. Reject rather than silently strafe.
        if abs(vy) > 1e-6:
            self.command[:] = 0
            self.last_command = None
            return False
        self.command = np.clip(values, [-.15, 0., -.35], [.15, 0., .35])
        self.last_command = time.monotonic() if now is None else now
        return True

    def current_command(self, now=None):
        now = time.monotonic() if now is None else now
        if self.fault or self.last_command is None or now-self.last_command > .5:
            return np.zeros(3)
        return self.command.copy()

    def before_step(self):
        q = self.data.qpos
        quat = q[self.base_q+3:self.base_q+7]
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(self.data.qvel)):
            self.fault = 'Non-finite physics state; reset required'
        elif abs(np.linalg.norm(quat)-1.) > .01 or projected_gravity(quat)[2] > -math.cos(.9):
            self.fault = 'Fall/invalid orientation; reset required'
        if self.fault:
            # Unit gear, bias=-kp*q: targeting the measured angle removes
            # commanded restoring torque without altering source gains/limits.
            self.data.ctrl[self.actuators] = np.nan_to_num(q[self.qpos])
            self.status = 'safe_stop'
            return
        cmd = self.current_command()
        desired = np.tile([(cmd[0]-.24*cmd[2])/.15, (cmd[0]+.24*cmd[2])/.15], 4)
        desired = np.clip(desired, -1., 1.)
        self.scale += np.clip(desired-self.scale, -2.*self.dt, 2.*self.dt)
        if np.any(np.abs(self.scale) > 1e-6):
            self.phase = (self.phase+2*math.pi*self.dt) % (2*math.pi)
        candidate = self.nominal + metachronal_delta(self.phase, self.scale)
        # Bounded transition into/out of the source gait and neutral stance.
        self.target += np.clip(candidate-self.target, -3.*self.dt, 3.*self.dt)
        self.target = np.clip(self.target, self.ranges[:, 0], self.ranges[:, 1])
        self.data.ctrl[self.actuators] = self.target
        self.status = 'scripted_gait' if np.any(cmd) else 'neutral_stance'

    def after_step(self):
        self.steps += 1

    def state(self):
        contacts = set()
        for c in self.data.contact[:self.data.ncon]:
            contacts.update(g for g in (c.geom1, c.geom2) if g in self.foot_geoms)
        return dict(robot=self.robot, status=self.status, fault=self.fault, sim_time=float(self.data.time),
                    command=self.current_command().tolist(), physics_steps=self.steps,
                    phase_rad=self.phase, measured_foot_contacts=len(contacts),
                    actuator_force_max_Nm=float(np.max(np.abs(self.data.actuator_force[self.actuators]))),
                    root_pose_truth=self.data.qpos[self.base_q:self.base_q+7].tolist(),
                    velocity_tracking='uncalibrated open-loop gait scaling; requested speed is not measured speed',
                    qualification='experimental scripted source prior; missions/terrain unqualified')
