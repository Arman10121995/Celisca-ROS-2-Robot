"""Native, model-matched Unitree RL Gym deployment with operator commands.

Observation layout and PD gains follow the pinned upstream deploy_mujoco.py.
This implementation is an experimental controller, not a walking certificate.
No ROS transport or wall-clock catch-up sits between inference and physics.
"""
import hashlib
import math
import time
from pathlib import Path

import numpy as np
import yaml


def file_digest(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def projected_gravity(quaternion):
    """Upstream gravity convention; quaternion is MuJoCo wxyz."""
    w, x, y, z = quaternion
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x),
                     1 - 2 * (w * w + z * z)], dtype=np.float32)


class NativeUnitreePolicy:
    """Fresh-state, per-physics-step PD with decimated CPU TorchScript inference."""

    def __init__(self, model, data, manifest_path, native_path):
        import mujoco
        import torch

        self.mj, self.model, self.data = mujoco, model, data
        manifest = yaml.safe_load(Path(manifest_path).read_text())
        if manifest.get('schema_version') != 1:
            raise ValueError('Unknown native locomotion contract schema')
        if Path(native_path).resolve() != Path(manifest['native_model']).resolve():
            raise ValueError('Policy must use its exact upstream native robot model')
        for resource in manifest['resources']:
            if file_digest(resource['path']) != resource['sha256']:
                raise ValueError('Changed policy resource: ' + resource['path'])
        self.config = yaml.safe_load(Path(manifest['upstream_config']).read_text())
        cfg = self.config
        self.count = int(cfg['num_actions'])
        if int(cfg['num_obs']) != 11 + 3 * self.count:
            raise ValueError('Unsupported Unitree observation layout')
        self.names = manifest['joint_names']
        if len(self.names) != self.count or len(set(self.names)) != self.count:
            raise ValueError('Joint names must match the exact policy action order')
        joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.names]
        actuators = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                     for name in manifest['actuator_names']]
        if len(actuators) != self.count or min(joints + actuators) < 0:
            raise ValueError('Native policy joint/actuator is missing')
        self.actuators = np.array(actuators, dtype=int)
        for joint, actuator in zip(joints, actuators):
            if (model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE
                    or model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT
                    or model.actuator_trnid[actuator, 0] != joint
                    or not np.allclose(model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0])
                    or model.actuator_dyntype[actuator] != mujoco.mjtDyn.mjDYN_NONE
                    or model.actuator_gaintype[actuator] != mujoco.mjtGain.mjGAIN_FIXED
                    or model.actuator_gainprm[actuator, 0] != 1
                    or model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_NONE):
                raise ValueError('Policy requires the original unit-gear torque motors')
        free = [j for j in range(model.njnt)
                if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
        if not free:
            raise ValueError('Locomotion requires a native free root joint')
        self.base_q = int(model.jnt_qposadr[free[0]])
        self.base_v = int(model.jnt_dofadr[free[0]])
        self.qpos = np.array([model.jnt_qposadr[j] for j in joints], dtype=int)
        self.dof = np.array([model.jnt_dofadr[j] for j in joints], dtype=int)
        if not np.all(model.jnt_limited[joints]):
            raise ValueError('Native policy joints must have authored position limits')
        self.ranges = model.jnt_range[joints].copy()
        self.kp = self.vector(cfg['kps'], 'kp')
        self.kd = self.vector(cfg['kds'], 'kd')
        self.nominal = self.vector(cfg['default_angles'], 'nominal')
        self.effort_limits = self.vector(manifest['effort_limits'], 'effort limits')
        if (np.any(self.kp < 0) or np.any(self.kd < 0) or np.any(self.effort_limits <= 0)
                or np.any(self.nominal < self.ranges[:, 0])
                or np.any(self.nominal > self.ranges[:, 1])):
            raise ValueError('Invalid gains, effort envelope or nominal joint pose')
        self.command_limits = np.asarray(manifest['command_limits'], dtype=np.float32)
        self.cmd_scale = np.asarray(cfg['cmd_scale'], dtype=np.float32)
        if (self.command_limits.shape != (3,) or self.cmd_scale.shape != (3,)
                or not np.all(np.isfinite(self.command_limits))
                or not np.all(np.isfinite(self.cmd_scale)) or np.any(self.command_limits <= 0)):
            raise ValueError('Invalid velocity command contract')
        self.decimation = int(cfg['control_decimation'])
        self.dt = float(cfg['simulation_dt'])
        if self.decimation < 1 or not math.isfinite(self.dt) or self.dt <= 0:
            raise ValueError('Invalid native policy cadence')
        for key in ('ang_vel_scale', 'dof_pos_scale', 'dof_vel_scale', 'action_scale'):
            if not math.isfinite(float(cfg[key])) or float(cfg[key]) <= 0:
                raise ValueError('Invalid policy scale: ' + key)
        self.model.opt.timestep = self.dt
        self.timeout = float(manifest.get('command_timeout_s', .5))
        self.max_tilt = float(manifest.get('max_tilt_rad', .9))
        if not 0 < self.timeout <= 2 or not 0 < self.max_tilt < math.pi / 2:
            raise ValueError('Invalid watchdog or fall threshold')
        torch.set_num_threads(1)
        self.torch = torch
        self.policy = torch.jit.load(manifest['checkpoint'], map_location='cpu').eval()
        self.robot = manifest['robot']
        self.observation = np.zeros(int(cfg['num_obs']), dtype=np.float32)
        self.reset()

    def vector(self, values, label):
        values = np.asarray(values, dtype=np.float32)
        if values.shape != (self.count,) or not np.all(np.isfinite(values)):
            raise ValueError('Invalid ' + label + ' vector')
        return values

    def reset(self):
        self.steps = 0
        self.command = np.zeros(3, dtype=np.float32)
        self.last_command = None
        self.action = np.zeros(self.count, dtype=np.float32)
        self.target = self.nominal.copy()
        self.fault = ''
        self.status = 'holding_nominal'

    def receive(self, vx, vy, yaw, now=None):
        values = np.asarray([vx, vy, yaw], dtype=np.float32)
        if not np.all(np.isfinite(values)):
            self.stop('Non-finite velocity command')
            return False
        if self.fault:
            return False
        self.command = np.clip(values, -self.command_limits, self.command_limits)
        self.last_command = time.monotonic() if now is None else now
        return True

    def stop(self, reason):
        self.fault = reason
        self.command[:] = 0
        self.data.ctrl[self.actuators] = 0
        self.status = 'safe_stop'

    def current_command(self, now=None):
        now = time.monotonic() if now is None else now
        if self.last_command is None or now - self.last_command > self.timeout:
            return np.zeros(3, dtype=np.float32)
        return self.command.copy()

    def before_step(self):
        q, v = self.data.qpos, self.data.qvel
        quaternion = q[self.base_q + 3:self.base_q + 7]
        if (not np.all(np.isfinite(q)) or not np.all(np.isfinite(v))
                or abs(float(np.linalg.norm(quaternion)) - 1) > .01):
            self.stop('Non-finite or invalid native state')
        elif projected_gravity(quaternion)[2] > -math.cos(self.max_tilt):
            self.stop('Fall threshold exceeded; reset required')
        if self.fault:
            self.data.ctrl[self.actuators] = 0
            return
        torque = (self.target - q[self.qpos]) * self.kp - v[self.dof] * self.kd
        torque = np.clip(torque, -self.effort_limits, self.effort_limits)
        for local, actuator in enumerate(self.actuators):
            if self.model.actuator_ctrllimited[actuator]:
                torque[local] = np.clip(torque[local], *self.model.actuator_ctrlrange[actuator])
        self.data.ctrl[self.actuators] = torque

    def after_step(self):
        self.steps += 1
        if self.fault or self.steps % self.decimation:
            return
        command = self.current_command()
        # Before any operator input, hold the checkpoint's nominal pose.
        # A subsequent Stop/watchdog commands zero velocity to the balance
        # policy rather than disabling the joint support under a walking robot.
        if self.last_command is None:
            return
        cfg, n, obs = self.config, self.count, self.observation
        obs[:3] = self.data.qvel[self.base_v + 3:self.base_v + 6] * cfg['ang_vel_scale']
        obs[3:6] = projected_gravity(self.data.qpos[self.base_q + 3:self.base_q + 7])
        obs[6:9] = command * self.cmd_scale
        obs[9:9+n] = (self.data.qpos[self.qpos] - self.nominal) * cfg['dof_pos_scale']
        obs[9+n:9+2*n] = self.data.qvel[self.dof] * cfg['dof_vel_scale']
        obs[9+2*n:9+3*n] = self.action
        phase = (self.steps * self.dt % .8) / .8 * 2 * math.pi
        obs[-2:] = [math.sin(phase), math.cos(phase)]
        try:
            if not np.all(np.isfinite(obs)):
                raise ValueError('Non-finite observation')
            with self.torch.inference_mode():
                output = self.policy(self.torch.from_numpy(obs).unsqueeze(0))
            self.action = self.vector(output.detach().cpu().numpy().reshape(-1), 'policy action')
            candidate = self.nominal + self.action * cfg['action_scale']
            if not np.all(np.isfinite(candidate)):
                raise ValueError('Non-finite joint target')
            self.target = np.clip(candidate, self.ranges[:, 0], self.ranges[:, 1])
            self.status = 'policy_active' if np.any(command) else 'zero_velocity_balance'
        except Exception as exc:
            self.stop('Policy inference failed: ' + str(exc))

    def state(self):
        return dict(robot=self.robot, status=self.status, fault=self.fault,
                    command=self.current_command().tolist(), physics_steps=self.steps,
                    decision_period_s=self.dt * self.decimation,
                    qualification='experimental; broad physical validation pending')
