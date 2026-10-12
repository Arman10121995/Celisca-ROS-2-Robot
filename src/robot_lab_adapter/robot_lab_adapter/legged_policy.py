"""Model-matched velocity policies for legged robots, in rl_sar's contract.

rl_sar (https://github.com/fan-ziqi/rl_sar, Apache-2.0) ships trained
locomotion policies for several Unitree robots together with the exact
observation, history, scaling, action and PD contract each was trained
with.  This module reproduces that contract (rl_sdk.cpp ComputeObservation,
ObservationBuffer and ComputeOutput at the pinned revision) so the same
checkpoints run on the lab's own robot models:

* observation terms in config order: ``ang_vel`` (body gyro x
  ang_vel_scale), ``gravity_vec`` (gravity in the body frame), ``commands``
  (x commands_scale), ``dof_pos`` (minus default pose, wheels zeroed, x
  dof_pos_scale), ``dof_vel`` (x dof_vel_scale), ``actions`` (previous raw
  actions), ``lin_vel`` (x lin_vel_scale); clipped to +/-clip_obs;
* optional history of whole observation frames, newest first, flattened
  by time ("time") or by term ("term");
* actions clipped, scaled by action_scale; leg joints track
  ``default + scaled`` with kp/kd, wheel joints track the scaled value as a
  velocity with kd; torques limited to torque_limits.

Joint order: config vectors are in *training* order; training index i is
the robot joint ``joint_names[joint_mapping[i]]`` of base.yaml.

The lab applies the PD law inside the simulator at every physics step
(MuJoCo general actuators), so this module returns joint targets and
gains rather than torques.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

# Installed rl_sar policy directories (robot_lab_adapter/setup.py installs
# exactly these; scripts/provision_rl_sar_policies.py produced them).
PROVISIONED_POLICIES = ('a1_legged_gym', 'b2_robot_lab', 'go2_himloco',
                        'go2w_robot_lab')

# Lab robot profile each provisioned policy was trained for.  A policy is
# only ever paired with its own model: the observation/PD contract is
# exact-model, not a family default (each source.json carries the same
# binding as 'profile').
POLICY_PROFILES = {'a1_legged_gym': 'unitree_a1', 'b2_robot_lab': 'unitree_b2',
                   'go2_himloco': 'unitree_go2', 'go2w_robot_lab': 'unitree_go2w'}

_TERMS_WITH_DOFS = ("dof_pos", "dof_vel", "actions")


def _vector(config, key, size=None, default=None):
    value = config.get(key, default)
    if value is None:
        raise ValueError('policy config lacks ' + key)
    array = np.asarray(value, dtype=np.float64).ravel()
    if size is not None and array.size == 1 and size > 1:
        array = np.full(size, float(array[0]))
    if size is not None and array.size != size:
        raise ValueError('%s has %d values, expected %d' % (key, array.size, size))
    if not np.all(np.isfinite(array)):
        raise ValueError(key + ' must be finite')
    return array


@dataclass
class PolicyContract:
    """One robot/configuration from an rl_sar base.yaml + config.yaml pair."""
    robot_joint_names: List[str]          # base.yaml joint_names (robot/SDK order)
    joint_mapping: List[int]              # training index -> robot index
    observations: List[str]
    history: List[int]
    history_priority: str
    dt: float
    decimation: int
    default_dof_pos: np.ndarray           # training order
    action_scale: np.ndarray
    clip_actions_lower: np.ndarray
    clip_actions_upper: np.ndarray
    rl_kp: np.ndarray
    rl_kd: np.ndarray
    torque_limits: np.ndarray
    wheel_indices: List[int]
    commands_scale: np.ndarray
    lin_vel_scale: float = 2.0
    ang_vel_scale: float = 0.25
    dof_pos_scale: float = 1.0
    dof_vel_scale: float = 0.05
    clip_obs: float = 100.0
    num_observations: int = 0
    model_name: str = ''
    source: Dict[str, str] = field(default_factory=dict)

    @property
    def num_dofs(self) -> int:
        return len(self.joint_mapping)

    @property
    def policy_period(self) -> float:
        return self.dt * self.decimation

    def training_joint_names(self) -> List[str]:
        return [self.robot_joint_names[index] for index in self.joint_mapping]

    @classmethod
    def from_rl_sar(cls, base: dict, config: dict, robot: str, variant: str, source=None):
        """Build from parsed base.yaml ({robot: {...}}) and config.yaml ({robot/variant: {...}})."""
        base_section = base[robot]
        section = config[robot + '/' + variant]
        merged = dict(base_section)
        merged.update(section)
        joint_names = list(base_section['joint_names'])
        mapping = [int(v) for v in merged.get('joint_mapping', range(len(joint_names)))]
        dofs = int(merged.get('num_of_dofs', len(mapping)))
        if len(mapping) != dofs or sorted(mapping) != list(range(dofs)) or len(joint_names) < dofs:
            raise ValueError('joint_mapping must be a permutation of the robot joints')
        history = [int(v) for v in merged.get('observations_history') or []]
        priority = str(merged.get('observations_history_priority', 'time'))
        if priority not in ('time', 'term'):
            raise ValueError('observations_history_priority must be time or term')
        observations = list(merged['observations'])
        known = {'lin_vel', 'ang_vel', 'gravity_vec', 'commands', 'dof_pos', 'dof_vel', 'actions'}
        unknown = [term for term in observations if term not in known]
        if unknown:
            raise ValueError('unsupported observation terms: %s' % unknown)
        contract = cls(
            robot_joint_names=joint_names, joint_mapping=mapping, observations=observations,
            history=history, history_priority=priority,
            dt=float(merged['dt']), decimation=int(merged['decimation']),
            default_dof_pos=_vector(merged, 'default_dof_pos', dofs),
            action_scale=_vector(merged, 'action_scale', dofs),
            clip_actions_lower=_vector(merged, 'clip_actions_lower', dofs, [-100.0]),
            clip_actions_upper=_vector(merged, 'clip_actions_upper', dofs, [100.0]),
            rl_kp=_vector(merged, 'rl_kp', dofs), rl_kd=_vector(merged, 'rl_kd', dofs),
            torque_limits=_vector(merged, 'torque_limits', dofs),
            wheel_indices=[int(v) for v in merged.get('wheel_indices') or []],
            commands_scale=_vector(merged, 'commands_scale', 3, [1.0, 1.0, 1.0]),
            lin_vel_scale=float(merged.get('lin_vel_scale', 2.0)),
            ang_vel_scale=float(merged.get('ang_vel_scale', 0.25)),
            dof_pos_scale=float(merged.get('dof_pos_scale', 1.0)),
            dof_vel_scale=float(merged.get('dof_vel_scale', 0.05)),
            clip_obs=float(merged.get('clip_obs', 100.0)),
            num_observations=int(merged.get('num_observations', 0)),
            model_name=str(merged.get('model_name', '')),
            source=dict(source or {}))
        if contract.dt <= 0 or contract.decimation < 1:
            raise ValueError('dt and decimation must be positive')
        if any(not 0 <= index < dofs for index in contract.wheel_indices):
            raise ValueError('wheel_indices outside the joint range')
        return contract

    @classmethod
    def load(cls, directory):
        """Load a provisioned policy directory (base.yaml, config.yaml, source.json)."""
        import json
        import yaml
        directory = Path(directory)
        source = json.loads((directory / 'source.json').read_text())
        return cls.from_rl_sar(yaml.safe_load((directory / 'base.yaml').read_text()),
                               yaml.safe_load((directory / 'config.yaml').read_text()),
                               source['robot'], source['variant'], source)


def quat_rotate_inverse(quaternion_wxyz, vector):
    """Rotate *vector* by the inverse of a (w, x, y, z) quaternion (rl_sar's QuatRotateInverse)."""
    w, x, y, z = quaternion_wxyz
    v = np.asarray(vector, dtype=np.float64)
    q = np.array([x, y, z])
    a = v * (2.0 * w * w - 1.0)
    b = np.cross(q, v) * w * 2.0
    c = q * (np.dot(q, v) * 2.0)
    return a - b + c


@dataclass
class LeggedState:
    """Measured state in robot joint names; quaternion (w, x, y, z), body gyro."""
    joint_positions: Dict[str, float]
    joint_velocities: Dict[str, float]
    quaternion_wxyz: Sequence[float]
    angular_velocity_body: Sequence[float]
    linear_velocity_body: Sequence[float] = (0.0, 0.0, 0.0)


@dataclass
class JointTargets:
    """Per-joint PD command for the simulator (robot joint names)."""
    names: List[str]
    position: np.ndarray      # target position (legs) or 0 (wheels)
    velocity: np.ndarray      # target velocity (wheels) or 0 (legs)
    kp: np.ndarray
    kd: np.ndarray
    torque_limit: np.ndarray


class LeggedPolicy:
    """Observation -> ONNX policy -> PD targets, with rl_sar's exact contract."""

    def __init__(self, contract: PolicyContract, session=None, model_path: Optional[str] = None):
        self.contract = contract
        if session is None:
            import onnxruntime
            session = onnxruntime.InferenceSession(str(model_path), providers=['CPUExecutionProvider'])
        self.session = session
        self.input_name = session.get_inputs()[0].name
        self.names = contract.training_joint_names()
        self.actions = np.zeros(contract.num_dofs)
        self.history_length = (max(contract.history) + 1) if contract.history else 0
        self._history: List[np.ndarray] = []
        self._dims: List[int] = []
        self._wheels = np.zeros(contract.num_dofs, dtype=bool)
        self._wheels[contract.wheel_indices] = True

    def reset(self):
        self.actions[:] = 0.0
        self._history = []

    def _terms(self, state: LeggedState, command):
        c = self.contract
        q = np.array([state.joint_positions[name] for name in self.names])
        dq = np.array([state.joint_velocities[name] for name in self.names])
        terms = []
        for term in c.observations:
            if term == 'lin_vel':
                terms.append(np.asarray(state.linear_velocity_body, float) * c.lin_vel_scale)
            elif term == 'ang_vel':
                terms.append(np.asarray(state.angular_velocity_body, float) * c.ang_vel_scale)
            elif term == 'gravity_vec':
                terms.append(quat_rotate_inverse(state.quaternion_wxyz, (0.0, 0.0, -1.0)))
            elif term == 'commands':
                terms.append(np.asarray(command, float) * c.commands_scale)
            elif term == 'dof_pos':
                relative = q - c.default_dof_pos
                relative[self._wheels] = 0.0
                terms.append(relative * c.dof_pos_scale)
            elif term == 'dof_vel':
                terms.append(dq * c.dof_vel_scale)
            elif term == 'actions':
                terms.append(self.actions.copy())
        return terms

    def observation(self, state: LeggedState, command) -> np.ndarray:
        """The model input for this step (updates the history)."""
        c = self.contract
        terms = self._terms(state, command)
        self._dims = [len(term) for term in terms]
        frame = np.clip(np.concatenate(terms), -c.clip_obs, c.clip_obs)
        if not c.history:
            return frame
        if not self._history:
            # rl_sar starts from a zeroed buffer; we match it.
            self._history = [np.zeros_like(frame) for _ in range(self.history_length)]
        self._history = [frame] + self._history[:-1]
        if c.history_priority == 'time':
            return np.concatenate([self._history[step] for step in c.history])
        parts, offset = [], 0
        for dim in self._dims:
            for step in c.history:
                parts.append(self._history[step][offset:offset + dim])
            offset += dim
        return np.concatenate(parts)

    def step(self, state: LeggedState, command) -> JointTargets:
        c = self.contract
        observation = self.observation(state, command).astype(np.float32)[None, :]
        raw = np.asarray(self.session.run(None, {self.input_name: observation})[0], dtype=np.float64).ravel()
        if raw.size != c.num_dofs or not np.all(np.isfinite(raw)):
            raise ValueError('policy returned %d values (expected %d finite)' % (raw.size, c.num_dofs))
        self.actions = np.clip(raw, c.clip_actions_lower, c.clip_actions_upper)
        return self.targets()

    def targets(self) -> JointTargets:
        """PD targets for the current actions (default pose with zero actions)."""
        c = self.contract
        scaled = self.actions * c.action_scale
        position = np.where(self._wheels, 0.0, scaled + c.default_dof_pos)
        velocity = np.where(self._wheels, scaled, 0.0)
        return JointTargets(list(self.names), position, velocity, c.rl_kp.copy(), c.rl_kd.copy(),
                            c.torque_limits.copy())


def clamp_command(command, limits):
    """Clamp (vx, vy, wz) to symmetric limits; non-finite values become zero."""
    out = []
    for value, limit in zip(command, limits):
        value = float(value)
        out.append(0.0 if not math.isfinite(value) else max(-limit, min(limit, value)))
    return out


def default_policy_for(robot_model):
    """Provisioned policy name for a lab robot profile, or '' if there is none."""
    for name, profile in POLICY_PROFILES.items():
        if profile == robot_model:
            return name
    return ''


LEGGED_PD_CONTROLLER = 'legged_pd_controller'


def ros2_control_pd_parameters(contract, controller=LEGGED_PD_CONTROLLER):
    """ros2_control joint_trajectory_controller parameters with this policy's PD law.

    For Gazebo (ign_ros2_control): the controller commands joint efforts
    from ``p (q* - q) + d (dq* - dq)`` at the controller manager's rate,
    using measured joint velocity (no differentiated position error), which
    is the policy's ``tau = kp (q* - q) + kd (dq* - dq)``.  Wheel joints
    track only their target velocity (p = 0).  Torques are capped by the
    description's effort command limits.  Every message on
    ``/<controller>/joint_trajectory`` is a single point with the targets.
    """
    names = contract.training_joint_names()
    wheels = set(contract.wheel_indices)
    gains = {name: {'p': 0.0 if i in wheels else float(contract.rl_kp[i]),
                    'd': float(contract.rl_kd[i]), 'i': 0.0, 'i_clamp': 0.0,
                    'ff_velocity_scale': 0.0}
             for i, name in enumerate(names)}
    return {controller: {'ros__parameters': {
        'joints': list(names),
        'command_interfaces': ['effort'],
        'state_interfaces': ['position', 'velocity'],
        'gains': gains,
        'allow_partial_joints_goal': False,
        'open_loop_control': False,
        'allow_nonzero_velocity_at_trajectory_end': True,
        'constraints': {'stopped_velocity_tolerance': 0.0, 'goal_time': 0.0},
    }}}


def mujoco_pd_config(contract, topic='/legged_policy/joint_targets',
                     timeout_s=0.5):
    """The MuJoCo bridge ``joint_pd_config`` for this policy (robot_lab_mujoco/joint_pd.py).

    Leg joints track their target position and wheel joints their target
    velocity, both with the policy's own kp/kd and torque limits, in
    training joint order.
    """
    names = contract.training_joint_names()
    wheels = set(contract.wheel_indices)
    return dict(topic=topic, timeout_s=timeout_s, joints=names,
                mode=['velocity' if i in wheels else 'position' for i in range(len(names))],
                kp=[float(v) for v in contract.rl_kp], kd=[float(v) for v in contract.rl_kd],
                torque_limit=[float(v) for v in contract.torque_limits],
                default=[0.0 if i in wheels else float(v)
                         for i, v in enumerate(contract.default_dof_pos)])
