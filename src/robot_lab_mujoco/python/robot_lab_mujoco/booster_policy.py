"""Native, model-matched Booster T1 locomotion from the vendor's booster_gym.

The policy is the vendor's exported actor (deploy/models/T1.pt), run on the
vendor's MuJoCo model (resources/T1/T1_locomotion.xml). The observation,
PD law and torque clipping follow booster_gym's MuJoCo sim-to-sim script
(play_mujoco.py), and the gait clock and command smoothing follow the
deploy policy (deploy/utils/policy.py) that ships with that checkpoint:

    obs = [gravity_body, gyro, vx, vy, wyaw, cos(2 pi phase), sin(2 pi phase),
           q - q0, 0.1 dq, previous action]          (47 values)
    q* = q0 + action,  tau = kp (q* - q) - kd dq,  clipped to ctrlrange

At zero command the gait clock stops, and the phase and command terms are
zeroed, as both vendor scripts do. This is an experimental controller, not a
walking certificate.
"""
import math
from pathlib import Path

import numpy as np
import yaml

from robot_lab_mujoco.unitree_policy import NativeUnitreePolicy, file_digest


def gravity_in_body(quaternion_wxyz):
    """booster_gym's quat_rotate_inverse(q, (0, 0, -1)); MuJoCo sensors are (w, x, y, z)."""
    w, vector = float(quaternion_wxyz[0]), np.asarray(quaternion_wxyz[1:4], dtype=np.float64)
    v = np.array([0., 0., -1.])
    return v * (2. * w * w - 1.) - np.cross(vector, v) * (w * 2.) + vector * (np.dot(vector, v) * 2.)


def _gains(table, names, label):
    """booster_gym matches gains by substring of the actuator name."""
    values = []
    for name in names:
        matches = [value for key, value in table.items() if key != 'default' and key in name]
        if not matches and 'default' not in table:
            raise ValueError('No %s for %s' % (label, name))
        values.append(matches[-1] if matches else table['default'])
    return np.asarray(values, dtype=np.float32)


class NativeBoosterPolicy(NativeUnitreePolicy):
    """Fresh-state, per-physics-step PD with decimated TorchScript inference."""

    def __init__(self, model, data, manifest_path, native_path):  # noqa: D107 - see the module docstring
        import mujoco
        import torch

        self.mj, self.model, self.data = mujoco, model, data
        manifest = yaml.safe_load(Path(manifest_path).read_text())
        if manifest.get('schema_version') != 1 or manifest.get('controller') != 'booster':
            raise ValueError('Not a Booster native locomotion contract')
        if Path(native_path).resolve() != Path(manifest['native_model']).resolve():
            raise ValueError('Policy must use its exact upstream native robot model')
        for resource in manifest['resources']:
            if file_digest(resource['path']) != resource['sha256']:
                raise ValueError('Changed policy resource: ' + resource['path'])
        env = yaml.safe_load(Path(manifest['upstream_config']).read_text())
        deploy = yaml.safe_load(Path(manifest['deploy_config']).read_text())
        self.config = dict(env=env, deploy=deploy)
        self.count = int(env['env']['num_actions'])
        if int(env['env']['num_observations']) != 11 + 3 * self.count:
            raise ValueError('Unsupported Booster observation layout')
        self.names = list(manifest['actuator_names'])
        actuators = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.names]
        if len(actuators) != self.count or min(actuators) < 0 or model.nu != self.count:
            raise ValueError('Policy actions must be the model actuators, in order')
        self.actuators = np.array(actuators, dtype=int)
        joints = [int(model.actuator_trnid[a, 0]) for a in actuators]
        for joint, actuator in zip(joints, actuators):
            if (model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE
                    or model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT
                    or not np.allclose(model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0])
                    or model.actuator_gaintype[actuator] != mujoco.mjtGain.mjGAIN_FIXED
                    or model.actuator_gainprm[actuator, 0] != 1
                    or model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_NONE):
                raise ValueError('Policy requires the original unit-gear torque motors')
        free = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
        if not free:
            raise ValueError('Locomotion requires a native free root joint')
        self.base_q, self.base_v = int(model.jnt_qposadr[free[0]]), int(model.jnt_dofadr[free[0]])
        self.qpos = np.array([model.jnt_qposadr[j] for j in joints], dtype=int)
        self.dof = np.array([model.jnt_dofadr[j] for j in joints], dtype=int)
        self.ranges = model.jnt_range[joints].copy()
        angles = env['init_state']['default_joint_angles']
        self.nominal = _gains(angles, self.names, 'default angle')
        self.kp = _gains(env['control']['stiffness'], self.names, 'stiffness')
        self.kd = _gains(env['control']['damping'], self.names, 'damping')
        self.effort_limits = np.abs(model.actuator_ctrlrange[self.actuators]).max(axis=1).astype(np.float32)
        if (np.any(self.kp <= 0) or np.any(self.kd < 0) or np.any(self.nominal < self.ranges[:, 0])
                or np.any(self.nominal > self.ranges[:, 1])):
            raise ValueError('Invalid gains or nominal joint pose')
        self.normalization = env['normalization']
        self.action_scale = float(env['control']['action_scale'])
        self.clip_actions = float(self.normalization['clip_actions'])
        self.gait_hz = float(deploy['policy']['gait_frequency'])
        self.command_limits = np.asarray(manifest['command_limits'], dtype=np.float32)
        self.cmd_scale = np.ones(3, dtype=np.float32)
        self.decimation = int(env['control']['decimation'])
        self.dt = float(env['sim']['dt'])
        if self.decimation < 1 or not 0 < self.dt < .01 or not 0 < self.gait_hz <= 3:
            raise ValueError('Invalid native policy cadence')
        self.model.opt.timestep = self.dt
        self.timeout = float(manifest.get('command_timeout_s', .5))
        self.max_tilt = float(manifest.get('max_tilt_rad', .9))
        if not 0 < self.timeout <= 2 or not 0 < self.max_tilt < math.pi / 2:
            raise ValueError('Invalid watchdog or fall threshold')
        self.orientation = model.sensor('orientation').id
        self.gyro = model.sensor('angular-velocity').id
        torch.set_num_threads(1)
        self.torch = torch
        self.policy = torch.jit.load(manifest['checkpoint'], map_location='cpu').eval()
        self.robot = manifest['robot']
        self.observation = np.zeros(11 + 3 * self.count, dtype=np.float32)
        self.reset()

    def reset(self):
        super().reset()
        self.smoothed = np.zeros(3, dtype=np.float32)
        self.gait_process = 0.

    def _sensor(self, sensor):
        start, size = int(self.model.sensor_adr[sensor]), int(self.model.sensor_dim[sensor])
        return np.asarray(self.data.sensordata[start:start + size], dtype=np.float64)

    def after_step(self):
        self.steps += 1
        if self.fault:
            return
        frequency = self.gait_hz if np.linalg.norm(self.smoothed) >= 1e-5 else 0.
        self.gait_process = math.fmod(self.gait_process + self.dt * frequency, 1.)
        if self.steps % self.decimation or self.last_command is None:
            return
        # Command smoothing as the vendor deploy policy: at most one policy
        # period of change per decision.
        period = self.dt * self.decimation
        self.smoothed += np.clip(self.current_command() - self.smoothed, -period, period)
        walking = float(np.linalg.norm(self.smoothed) >= 1e-5)
        norm, n, obs = self.normalization, self.count, self.observation
        obs[0:3] = gravity_in_body(self._sensor(self.orientation)) * norm['gravity']
        obs[3:6] = self._sensor(self.gyro) * norm['ang_vel']
        obs[6:8] = self.smoothed[:2] * norm['lin_vel'] * walking
        obs[8] = self.smoothed[2] * norm['ang_vel'] * walking
        obs[9] = math.cos(2 * math.pi * self.gait_process) * walking
        obs[10] = math.sin(2 * math.pi * self.gait_process) * walking
        obs[11:11 + n] = (self.data.qpos[self.qpos] - self.nominal) * norm['dof_pos']
        obs[11 + n:11 + 2 * n] = self.data.qvel[self.dof] * norm['dof_vel']
        obs[11 + 2 * n:11 + 3 * n] = self.action
        try:
            if not np.all(np.isfinite(obs)):
                raise ValueError('Non-finite observation')
            with self.torch.inference_mode():
                output = self.policy(self.torch.from_numpy(obs).unsqueeze(0))
            action = self.vector(output.detach().cpu().numpy().reshape(-1), 'policy action')
            self.action = np.clip(action, -self.clip_actions, self.clip_actions)
            candidate = self.nominal + self.action * self.action_scale
            if not np.all(np.isfinite(candidate)):
                raise ValueError('Non-finite joint target')
            self.target = np.clip(candidate, self.ranges[:, 0], self.ranges[:, 1])
            self.status = 'policy_active' if walking else 'zero_velocity_balance'
        except Exception as exc:
            self.stop('Policy inference failed: ' + str(exc))
