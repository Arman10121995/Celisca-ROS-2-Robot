"""Train legged-robot policies in MuJoCo, in the lab's deployable contract.

The environment steps many copies of one robot model in parallel with
``mujoco.rollout`` (C threads), from the same URDF, mesh staging and in-step
PD actuators that the MuJoCo bridge uses (robot_lab_mujoco), so a policy
trained here sees the plant it is later run on.  Each robot trains in the
observation/action/PD contract of its provisioned rl_sar policy
(legged_policy.PolicyContract): same joint order, default pose, action
scale, kp/kd, torque limits, ``dt`` and decimation.  Only the observation
terms differ: no history and no linear velocity (not measured on the
robot), i.e. ``ang_vel, gravity_vec, commands, dof_pos, dof_vel, actions``.
An exported policy is therefore an ordinary policy directory
(base.yaml, config.yaml, source.json, policy.onnx) that
``legged_policy:=<directory>`` runs in MuJoCo, Gazebo and Isaac.

Tasks (TASKS) are command distributions plus reward terms, in the style of
legged_gym: ``walk`` (velocity tracking), ``turn_in_place`` (yaw rate with
the base held in place), ``stand`` (hold the default pose upright) and
``push_recovery`` (stand while being pushed).

This is a training tool, not an evaluation of any robot: a trained policy
needs its own measured runs before any claim.
"""
from __future__ import annotations

import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .legged_policy import POLICY_PROFILES, PolicyContract, default_policy_for, mujoco_pd_config

# Robots with a provisioned template policy that this trainer supports:
# quadrupeds with position-controlled legs only (no wheels, no recurrent
# humanoid contract yet).
TRAINABLE = {profile: name for name, profile in POLICY_PROFILES.items()
             if name in ('a1_legged_gym', 'b2_robot_lab', 'go2_himloco')}

OBSERVATIONS = ['ang_vel', 'gravity_vec', 'commands', 'dof_pos', 'dof_vel', 'actions']


@dataclass
class Task:
    """Command distribution and reward weights (per second; scaled by the policy period)."""
    name: str
    description: str
    lin_vel_x: tuple = (0.0, 0.0)
    lin_vel_y: tuple = (0.0, 0.0)
    ang_vel_z: tuple = (0.0, 0.0)
    min_ang_vel_z: float = 0.0        # |wz| sampled at least this (turning tasks)
    resample_s: float = 10.0
    push_interval_s: float = 0.0
    push_velocity: float = 0.0
    rewards: Dict[str, float] = field(default_factory=dict)
    # legged_gym's only_positive_rewards: the per-step sum of every term but
    # termination is clipped at zero, so ending an episode early never pays.
    only_positive: bool = True


# termination is applied once, when the robot falls (not per second).
_REGULARIZE = dict(lin_vel_z=-2.0, ang_vel_xy=-0.05, orientation=-1.0, torques=-2e-4,
                   dof_acc=-2.5e-7, action_rate=-0.01, termination=-10.0)

TASKS = {
    'walk': Task('walk', 'Track commanded forward, lateral and yaw velocity.',
                 lin_vel_x=(-0.6, 1.0), lin_vel_y=(-0.4, 0.4), ang_vel_z=(-1.0, 1.0),
                 rewards=dict(tracking_lin_vel=1.0, tracking_ang_vel=0.5, feet_air_time=1.0,
                              **_REGULARIZE)),
    'turn_in_place': Task('turn_in_place', 'Turn on the spot at the commanded yaw rate.',
                          ang_vel_z=(-1.0, 1.0), min_ang_vel_z=0.3,
                          rewards=dict(tracking_ang_vel=1.5, hold_position=-1.0, feet_air_time=0.5,
                                       **_REGULARIZE)),
    'stand': Task('stand', 'Stand still and upright in the default pose.',
                  rewards=dict(alive=1.0, stand_still=-1.0, hold_position=-1.0, base_height=-20.0,
                               **_REGULARIZE)),
    'push_recovery': Task('push_recovery', 'Stand, and recover from random pushes.',
                          push_interval_s=4.0, push_velocity=0.6,
                          rewards=dict(alive=1.0, stand_still=-0.5, hold_position=-0.5, base_height=-20.0,
                                       **_REGULARIZE)),
}


def _package_map(urdf):
    from ament_index_python.packages import get_package_share_directory
    names = set(re.findall(r"package://([^/]+)/", urdf)) | set(re.findall(r"\$\(find\s+([^)]+)\)", urdf))
    result = {}
    for name in names:
        try:
            result[name] = get_package_share_directory(name)
        except Exception:  # noqa: BLE001 - unresolved packages stay as written
            pass
    return result


def template_directory(robot):
    """The provisioned rl_sar policy directory a robot trains in the contract of."""
    from ament_index_python.packages import get_package_share_directory
    name = TRAINABLE.get(robot)
    if not name:
        raise ValueError('%s has no trainable template policy (trainable: %s)'
                         % (robot, ', '.join(sorted(TRAINABLE))))
    return os.path.join(get_package_share_directory('robot_lab_adapter'), 'policies', 'rl_sar', name)


def build_model_xml(robot, contract, armature=0.01):
    """MJCF of *robot* on a flat floor with the contract's in-step PD actuators.

    The URDF import, mesh staging and PD actuators are the MuJoCo bridge's
    own (robot_lab_mujoco.mujoco_spawner / joint_pd); foot touch sensors are
    added at each leg's lowest collision sphere.
    """
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from robot_lab_mujoco.joint_pd import JointPDCommand
    from robot_lab_mujoco.mujoco_spawner import _build_mjcf_from_urdf, _xacro_to_urdf
    profiles = yaml.safe_load(open(os.path.join(
        get_package_share_directory('robot_lab_robots'), 'config', 'robots.yaml')))['robots']
    profile = profiles[robot]
    urdf = _xacro_to_urdf(os.path.join(get_package_share_directory(profile['package']), profile['xacro']))
    mjcf = _build_mjcf_from_urdf(urdf, _package_map(urdf), robot_name=robot)
    if 'fallback_robot' in mjcf:
        raise RuntimeError('MuJoCo could not import %s' % robot)
    mjcf = JointPDCommand(mujoco_pd_config(contract)).add_actuators(mjcf, armature)
    root = ET.fromstring(mjcf)
    option = root.find('option')
    if option is None:
        option = ET.SubElement(root, 'option')
    option.set('timestep', '%g' % contract.dt)
    option.set('integrator', 'implicitfast')
    worldbody = root.find('worldbody')
    ET.SubElement(worldbody, 'geom', name='training_floor', type='plane', size='100 100 0.1',
                  friction='1 0.005 0.0001')
    return ET.tostring(root, encoding='unicode')


def _feet(model):
    """Lowest collision sphere of each leaf body below the free body: (body, geom)."""
    import mujoco
    children = {}
    for body in range(1, model.nbody):
        children.setdefault(int(model.body_parentid[body]), []).append(body)
    feet = []
    for body in range(2, model.nbody):
        if children.get(body):
            continue
        spheres = [g for g in range(model.ngeom) if model.geom_bodyid[g] == body
                   and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
                   and (model.geom_contype[g] or model.geom_conaffinity[g])]
        if spheres:
            feet.append((body, min(spheres, key=lambda g: model.geom_pos[g][2])))
    return feet


def add_foot_sensors(xml):
    """Touch sensors on spheres around each foot; returns (xml, foot count)."""
    import mujoco
    model = mujoco.MjModel.from_xml_string(xml)
    feet = _feet(model)
    root = ET.fromstring(xml)
    bodies = {element.get('name'): element for element in root.iter('body')}
    sensor = root.find('sensor')
    if sensor is None:
        sensor = ET.SubElement(root, 'sensor')
    for index, (body, geom) in enumerate(feet):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        site = 'training_foot_%d' % index
        ET.SubElement(bodies[name], 'site', name=site, type='sphere',
                      pos='%g %g %g' % tuple(model.geom_pos[geom]),
                      size='%g' % (1.6 * model.geom_size[geom][0]))
        ET.SubElement(sensor, 'touch', name=site + '_touch', site=site)
    return ET.tostring(root, encoding='unicode'), len(feet)


def _quat_rotate_inverse(quaternion, vectors):
    """Rotate (N, 3) world vectors into the bodies given (N, 4) w, x, y, z quaternions."""
    w = quaternion[:, :1]
    q = quaternion[:, 1:]
    a = vectors * (2.0 * w * w - 1.0)
    b = np.cross(q, vectors) * w * 2.0
    c = q * np.sum(q * vectors, axis=1, keepdims=True) * 2.0
    return a - b + c


class LeggedTrainingEnv:
    """Batched MuJoCo environment: ``reset() -> obs``, ``step(actions) -> obs, reward, done, info``."""

    def __init__(self, robot, task='walk', num_envs=256, seed=0, threads=None, episode_s=20.0,
                 model_xml=None):
        import mujoco
        self.mujoco = mujoco
        self.robot = robot
        self.task = TASKS[task]
        self.template = template_directory(robot)
        self.contract = PolicyContract.load(self.template)
        xml = model_xml or build_model_xml(robot, self.contract)
        xml, self.num_feet = add_foot_sensors(xml)
        self.model_xml = xml
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.num_envs = int(num_envs)
        self.rng = np.random.default_rng(seed)
        c = self.contract
        self.dt = c.dt * c.decimation
        self.max_steps = int(round(episode_s / self.dt))
        self.names = c.training_joint_names()
        joint = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.names}
        if min(joint.values()) < 0:
            raise ValueError('training joints missing from the model')
        self.qpos_index = np.array([self.model.jnt_qposadr[joint[n]] for n in self.names])
        self.qvel_index = np.array([self.model.jnt_dofadr[joint[n]] for n in self.names])
        actuator = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n + '_pd') for n in self.names]
        if min(actuator) < 0:
            raise ValueError('PD actuators missing')
        self.actuator_index = np.array(actuator)
        self.state_spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        self.state_size = mujoco.mj_stateSize(self.model, self.state_spec)
        self.threads = [mujoco.MjData(self.model) for _ in range(threads or max(1, (os.cpu_count() or 2) - 2))]
        self._scratch = mujoco.MjData(self.model)
        # Standing height: base raised until the lowest collision point clears the floor.
        self._scratch.qpos[:] = self.model.qpos0
        self._scratch.qpos[self.qpos_index] = c.default_dof_pos
        mujoco.mj_forward(self.model, self._scratch)
        lowest = min(self._scratch.geom_xpos[g][2] - self.model.geom_rbound[g]
                     for g in range(self.model.ngeom)
                     if self.model.geom_bodyid[g] > 0 and (self.model.geom_contype[g] or self.model.geom_conaffinity[g]))
        feet = _feet(self.model)
        foot_bottom = min((self._scratch.geom_xpos[g][2] - self.model.geom_size[g][0] for _, g in feet), default=lowest)
        self.stand_height = float(self.model.qpos0[2] - foot_bottom)
        self.sensor_touch = np.array([self.model.sensor_adr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'training_foot_%d_touch' % i)] for i in range(self.num_feet)])
        self.obs_dim = 3 + 3 + 3 + 3 * c.num_dofs
        self.act_dim = c.num_dofs
        n = self.num_envs
        self.state = np.zeros((n, self.state_size))
        self.actions = np.zeros((n, c.num_dofs))
        self.last_actions = np.zeros((n, c.num_dofs))
        self.last_dof_vel = np.zeros((n, c.num_dofs))
        self.commands = np.zeros((n, 3))
        self.episode_step = np.zeros(n, dtype=np.int64)
        self.feet_air = np.zeros((n, self.num_feet))
        self.home = np.zeros((n, 2))
        self.episode_return = np.zeros(n)
        self.term_sums = {name: np.zeros(n) for name in self.task.rewards}
        self.reset_all()

    # --- state -------------------------------------------------------------
    def _reset_state(self, count):
        """Full physics states for *count* fresh episodes (default pose, small noise)."""
        mujoco, model, data = self.mujoco, self.model, self._scratch
        c = self.contract
        states = np.zeros((count, self.state_size))
        for i in range(count):
            mujoco.mj_resetData(model, data)
            yaw = self.rng.uniform(-math.pi, math.pi)
            data.qpos[0:3] = (0.0, 0.0, self.stand_height + 0.02)
            data.qpos[3:7] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
            data.qpos[self.qpos_index] = c.default_dof_pos + self.rng.uniform(-0.1, 0.1, c.num_dofs)
            data.qvel[0:6] = self.rng.uniform(-0.2, 0.2, 6)
            data.ctrl[self.actuator_index] = c.default_dof_pos
            mujoco.mj_forward(model, data)
            mujoco.mj_getState(model, data, states[i], self.state_spec)
        return states

    def _split(self, states):
        """qpos, qvel, ctrl columns of full physics states."""
        nq, nv = self.model.nq, self.model.nv
        qpos = states[:, 1:1 + nq]
        qvel = states[:, 1 + nq:1 + nq + nv]
        return qpos, qvel

    def _sample_commands(self, index):
        task, k = self.task, len(index)
        if not k:
            return
        commands = np.stack([self.rng.uniform(*task.lin_vel_x, k), self.rng.uniform(*task.lin_vel_y, k),
                             self.rng.uniform(*task.ang_vel_z, k)], axis=1)
        if task.min_ang_vel_z > 0:
            sign = np.where(self.rng.random(k) < 0.5, -1.0, 1.0)
            commands[:, 2] = sign * np.maximum(np.abs(commands[:, 2]), task.min_ang_vel_z)
        elif task.name == 'walk':
            # A share of standing commands keeps the policy able to stop.
            commands[self.rng.random(k) < 0.1] = 0.0
        self.commands[index] = commands

    def reset_all(self):
        index = np.arange(self.num_envs)
        self._reset(index)
        return self.observations()

    def _reset(self, index):
        if not len(index):
            return
        self.state[index] = self._reset_state(len(index))
        qpos, _ = self._split(self.state[index])
        self.home[index] = qpos[:, 0:2]
        self.actions[index] = 0.0
        self.last_actions[index] = 0.0
        self.last_dof_vel[index] = 0.0
        self.episode_step[index] = 0
        self.feet_air[index] = 0.0
        self.episode_return[index] = 0.0
        for sums in self.term_sums.values():
            sums[index] = 0.0
        self._sample_commands(index)

    # --- observation --------------------------------------------------------
    def observations(self):
        c = self.contract
        qpos, qvel = self._split(self.state)
        quat = qpos[:, 3:7]
        gravity = _quat_rotate_inverse(quat, np.tile([0.0, 0.0, -1.0], (self.num_envs, 1)))
        ang_vel = qvel[:, 3:6]       # free-joint angular velocity is in the body frame
        dof_pos = qpos[:, self.qpos_index] - c.default_dof_pos
        dof_vel = qvel[:, self.qvel_index]
        obs = np.concatenate([ang_vel * c.ang_vel_scale, gravity, self.commands * c.commands_scale,
                              dof_pos * c.dof_pos_scale, dof_vel * c.dof_vel_scale, self.actions], axis=1)
        return np.clip(obs, -c.clip_obs, c.clip_obs).astype(np.float32)

    # --- step ---------------------------------------------------------------
    def step(self, actions):
        from mujoco import rollout
        c, model = self.contract, self.model
        self.last_actions = self.actions
        self.actions = np.clip(np.asarray(actions, dtype=np.float64),
                               c.clip_actions_lower, c.clip_actions_upper)
        targets = c.default_dof_pos + self.actions * c.action_scale
        control = np.zeros((self.num_envs, c.decimation, model.nu))
        control[:, :, self.actuator_index] = targets[:, None, :]
        if self.task.push_interval_s > 0:
            period = max(1, int(round(self.task.push_interval_s / self.dt)))
            push = (self.episode_step % period == period - 1)
            if np.any(push):
                qpos, qvel = self._split(self.state)
                qvel[push, 0:2] += self.rng.uniform(-1, 1, (int(push.sum()), 2)) * self.task.push_velocity
        states, sensors = rollout.rollout(model, self.threads, self.state, control,
                                          persistent_pool=True)
        self.state = states[:, -1, :].copy()
        touch = sensors[:, -1, :][:, self.sensor_touch] if self.num_feet else np.zeros((self.num_envs, 0))
        self.episode_step += 1
        reward, terms, fallen = self._rewards(touch)
        timeout = self.episode_step >= self.max_steps
        done = fallen | timeout
        self.episode_return += reward
        info = dict(terms=terms, fallen=fallen, timeout=timeout,
                    finished_returns=self.episode_return[done].copy(),
                    finished_lengths=self.episode_step[done].copy())
        resample = (self.episode_step % max(1, int(round(self.task.resample_s / self.dt))) == 0) & ~done
        self._sample_commands(np.flatnonzero(resample))
        self._reset(np.flatnonzero(done))
        return self.observations(), reward.astype(np.float32), done, info

    def _rewards(self, touch):
        c, task, dt = self.contract, self.task, self.dt
        qpos, qvel = self._split(self.state)
        quat = qpos[:, 3:7]
        lin_vel = _quat_rotate_inverse(quat, qvel[:, 0:3])
        ang_vel = qvel[:, 3:6]
        gravity = _quat_rotate_inverse(quat, np.tile([0.0, 0.0, -1.0], (self.num_envs, 1)))
        dof_vel = qvel[:, self.qvel_index]
        dof_pos = qpos[:, self.qpos_index]
        torque = self._torques(dof_pos, dof_vel)
        height = qpos[:, 2]
        fallen = (gravity[:, 2] > -0.5) | (height < 0.4 * self.stand_height)
        cmd = self.commands
        contact = touch > 1.0
        first_contact = (self.feet_air > 0) & contact
        self.feet_air += dt
        moving = np.linalg.norm(cmd[:, :2], axis=1) + np.abs(cmd[:, 2]) > 0.1
        air_time = np.sum((self.feet_air - 0.5) * first_contact, axis=1) * moving
        self.feet_air[contact] = 0.0
        values = dict(
            tracking_lin_vel=np.exp(-np.sum((cmd[:, :2] - lin_vel[:, :2]) ** 2, axis=1) / 0.25),
            tracking_ang_vel=np.exp(-(cmd[:, 2] - ang_vel[:, 2]) ** 2 / 0.25),
            lin_vel_z=lin_vel[:, 2] ** 2,
            ang_vel_xy=np.sum(ang_vel[:, :2] ** 2, axis=1),
            orientation=np.sum(gravity[:, :2] ** 2, axis=1),
            torques=np.sum(torque ** 2, axis=1),
            dof_acc=np.sum(((dof_vel - self.last_dof_vel) / dt) ** 2, axis=1),
            action_rate=np.sum((self.actions - self.last_actions) ** 2, axis=1),
            feet_air_time=air_time,
            hold_position=np.sum(lin_vel[:, :2] ** 2, axis=1),
            stand_still=np.sum(np.abs(dof_pos - c.default_dof_pos), axis=1),
            base_height=(height - self.stand_height) ** 2,
            alive=np.ones(self.num_envs),
            termination=fallen.astype(np.float64))
        self.last_dof_vel = dof_vel
        reward = np.zeros(self.num_envs)
        terms = {}
        for name, weight in task.rewards.items():
            if name == 'termination':
                continue
            value = weight * values[name] * dt
            reward += value
            self.term_sums[name] += value
            terms[name] = float(np.mean(value))
        if task.only_positive:
            reward = np.maximum(reward, 0.0)
        if 'termination' in task.rewards:
            value = task.rewards['termination'] * values['termination']
            reward += value
            self.term_sums['termination'] += value
            terms['termination'] = float(np.mean(value))
        terms['lin_vel_error'] = float(np.mean(np.linalg.norm(cmd[:, :2] - lin_vel[:, :2], axis=1)))
        terms['ang_vel_error'] = float(np.mean(np.abs(cmd[:, 2] - ang_vel[:, 2])))
        return reward, terms, fallen

    def _torques(self, dof_pos, dof_vel):
        c = self.contract
        targets = c.default_dof_pos + self.actions * c.action_scale
        return np.clip(c.rl_kp * (targets - dof_pos) - c.rl_kd * dof_vel, -c.torque_limits, c.torque_limits)


def policy_config(contract, robot_key, variant):
    """config.yaml section (rl_sar layout) for a policy trained here."""
    c = contract
    vec = lambda values: [float(v) for v in values]  # noqa: E731
    return {robot_key + '/' + variant: dict(
        model_name='policy.onnx', num_observations=3 + 3 + 3 + 3 * c.num_dofs,
        observations=list(OBSERVATIONS), observations_history=[],
        observations_history_priority='time', default_commands=[0.0, 0.0, 0.0],
        clip_obs=float(c.clip_obs), clip_actions_lower=vec(c.clip_actions_lower),
        clip_actions_upper=vec(c.clip_actions_upper), rl_kp=vec(c.rl_kp), rl_kd=vec(c.rl_kd),
        num_of_dofs=c.num_dofs, action_scale=vec(c.action_scale), wheel_indices=list(c.wheel_indices),
        lin_vel_scale=float(c.lin_vel_scale), ang_vel_scale=float(c.ang_vel_scale),
        dof_pos_scale=float(c.dof_pos_scale), dof_vel_scale=float(c.dof_vel_scale),
        commands_scale=vec(c.commands_scale), torque_limits=vec(c.torque_limits),
        default_dof_pos=vec(c.default_dof_pos), joint_mapping=list(c.joint_mapping),
        dt=float(c.dt), decimation=int(c.decimation))}
