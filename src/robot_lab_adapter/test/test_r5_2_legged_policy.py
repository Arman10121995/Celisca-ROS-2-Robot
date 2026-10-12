"""Model-matched rl_sar legged-policy contracts (R3.6/R5.2 implementation).

Pure-logic contracts for robot_lab_adapter/legged_policy.py and
robot_lab_mujoco/joint_pd.py: provisioned policy loading, observation
term/history math, PD target generation, command clamping and the MuJoCo
in-step PD bridge config.  No ROS graph, no ONNX Runtime and no physics:
a mock session stands in for the policy network, exactly as in the Go2
velocity-policy tests.
"""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pkg = Path(__file__).resolve().parents[1]
if str(pkg) not in sys.path:
    sys.path.insert(0, str(pkg))

from robot_lab_adapter.legged_policy import (
    POLICY_PROFILES,
    PROVISIONED_POLICIES,
    LeggedPolicy,
    LeggedState,
    PolicyContract,
    clamp_command,
    default_policy_for,
    mujoco_pd_config,
)

POLICY_ROOT = pkg / 'policies' / 'rl_sar'

_TERM_WIDTH = {'lin_vel': 3, 'ang_vel': 3, 'gravity_vec': 3, 'commands': 3}


def _load(name):
    return PolicyContract.load(POLICY_ROOT / name)


class Session:
    """ONNX Runtime stand-in recording the observation it was fed."""

    def __init__(self, action=None, dofs=12, obs_width=45):
        self.observation = None
        self.action = (np.zeros((1, dofs), dtype=np.float32)
                       if action is None else action)
        self.obs_width = obs_width

    def get_inputs(self):
        return [SimpleNamespace(name='obs', shape=[1, self.obs_width])]

    def get_outputs(self):
        return [SimpleNamespace(name='actions',
                                shape=[1, self.action.shape[1]])]

    def run(self, _outputs, feeds):
        self.observation = feeds['obs']
        return [self.action]


def _state(contract, offset=0.05):
    names = contract.training_joint_names()
    positions = {name: float(value + offset)
                 for name, value in zip(names, contract.default_dof_pos)}
    velocities = {name: float(0.1 * index)
                  for index, name in enumerate(names)}
    return LeggedState(positions, velocities, (1.0, 0.0, 0.0, 0.0),
                       (0.1, 0.2, 0.3))


def test_every_provisioned_policy_loads_and_matches_its_profile():
    assert set(PROVISIONED_POLICIES) == set(POLICY_PROFILES) == {
        'a1_legged_gym', 'b2_robot_lab', 'go2_himloco', 'go2w_robot_lab'}
    for name in PROVISIONED_POLICIES:
        directory = POLICY_ROOT / name
        source = json.loads((directory / 'source.json').read_text())
        contract = _load(name)
        # The source binding and the static table agree; a policy is only
        # ever paired with the exact model it was trained for.
        assert source['profile'] == POLICY_PROFILES[name]
        assert contract.source['profile'] == POLICY_PROFILES[name]
        assert default_policy_for(POLICY_PROFILES[name]) == name
        dofs = contract.num_dofs
        assert sorted(contract.joint_mapping) == list(range(dofs))
        for vector in (contract.default_dof_pos, contract.action_scale,
                       contract.rl_kp, contract.rl_kd, contract.torque_limits,
                       contract.clip_actions_lower, contract.clip_actions_upper):
            assert vector.shape == (dofs,)
        assert all(0 <= index < dofs for index in contract.wheel_indices)
        # The ONNX input width the export was checked against must equal
        # the observation this module builds (frame x history).
        frame = sum(_TERM_WIDTH.get(term, dofs) for term in contract.observations)
        width = frame * (max(contract.history) + 1 if contract.history else 1)
        assert width == source['input_width'], name


def test_himloco_observation_order_and_history():
    contract = _load('go2_himloco')
    session = Session(np.full((1, 12), 0.5, dtype=np.float32), obs_width=270)
    policy = LeggedPolicy(contract, session=session)
    policy.step(_state(contract), (0.3, -0.2, 0.5))
    obs = session.observation
    assert obs.shape == (1, 270)
    frame = obs[0, :45]
    # observations: commands, ang_vel, gravity_vec, dof_pos, dof_vel, actions
    np.testing.assert_allclose(frame[0:3], [0.3 * 2.0, -0.2 * 2.0, 0.5 * 0.25])
    np.testing.assert_allclose(frame[3:6], np.array([0.1, 0.2, 0.3]) * 0.25)
    np.testing.assert_allclose(frame[6:9], [0.0, 0.0, -1.0])
    np.testing.assert_allclose(frame[9:21], 0.05)
    np.testing.assert_allclose(frame[21:33],
                               np.arange(12) * 0.1 * contract.dof_vel_scale)
    np.testing.assert_allclose(frame[33:45], 0.0)  # actions start zeroed
    # rl_sar starts from a zeroed history buffer: only slot 0 is the new
    # frame, the older five are zeros (time priority).
    np.testing.assert_allclose(obs[0, 45:], 0.0)
    first_frame = obs[0, :45].copy()
    # The PD targets use the contract's training joint order and gains.
    targets = policy.targets()
    assert targets.names == contract.training_joint_names()
    np.testing.assert_allclose(
        targets.position,
        contract.default_dof_pos + 0.5 * contract.action_scale)
    np.testing.assert_allclose(targets.kp, contract.rl_kp)
    # Second step: the observation carries the *previous* raw actions in
    # the 'actions' term (rl_sar semantics) and the previous frame moved
    # into history slot 1; the new actions only appear in the new targets.
    session.action = np.full((1, 12), 0.25, dtype=np.float32)
    policy.step(_state(contract), (0.0, 0.0, 0.0))
    obs = session.observation
    np.testing.assert_allclose(obs[0, 33:45], 0.5)
    np.testing.assert_allclose(policy.targets().position[0],
                               contract.default_dof_pos[0] + 0.25 * contract.action_scale[0])
    np.testing.assert_allclose(obs[0, 45:90], first_frame, rtol=1e-6)
    np.testing.assert_allclose(obs[0, 90:], 0.0)


def test_go2w_wheels_are_velocity_joints_with_zero_position_target():
    contract = _load('go2w_robot_lab')
    assert contract.num_dofs == 16
    assert contract.wheel_indices == [12, 13, 14, 15]
    session = Session(np.full((1, 16), 0.5, dtype=np.float32), obs_width=57)
    policy = LeggedPolicy(contract, session=session)
    targets = policy.step(_state(contract), (0.2, 0.0, 0.0))
    obs = session.observation
    assert obs.shape == (1, 57)
    # dof_pos zeroes the wheel entries even when the wheels moved.
    positions = {name: 9.9 for name in contract.training_joint_names()}
    velocities = {name: 1.0 for name in contract.training_joint_names()}
    state = LeggedState(positions, velocities, (1.0, 0.0, 0.0, 0.0),
                        (0.0, 0.0, 0.0))
    fresh = policy.observation(state, (0.0, 0.0, 0.0))
    # frame: ang_vel(3) gravity(3) commands(3) dof_pos(16) dof_vel(16) actions(16)
    np.testing.assert_allclose(fresh[9:21], 9.9 - contract.default_dof_pos[:12])
    np.testing.assert_allclose(fresh[21:25], 0.0)    # wheels zeroed in dof_pos
    np.testing.assert_allclose(fresh[37:41], 1.0 * contract.dof_vel_scale)
    # Wheels track scaled actions as velocity at zero position; legs track
    # default + scaled action as position with zero velocity command.
    scaled = 0.5 * contract.action_scale
    np.testing.assert_allclose(targets.velocity[12:], scaled[12:])
    np.testing.assert_allclose(targets.velocity[:12], 0.0)
    np.testing.assert_allclose(targets.position[12:], 0.0)
    np.testing.assert_allclose(targets.position[:12],
                               contract.default_dof_pos[:12] + scaled[:12])


def test_term_priority_history_layout():
    base = {'tiny': {'joint_names': ['j0', 'j1'], 'num_of_dofs': 2}}
    config = {'tiny/walk': {
        'observations': ['commands', 'dof_pos'],
        'observations_history': [0, 1],
        'observations_history_priority': 'term',
        'dt': 0.01, 'decimation': 1,
        'default_dof_pos': [0.0, 0.0], 'action_scale': [1.0, 1.0],
        'rl_kp': [10.0, 10.0], 'rl_kd': [1.0, 1.0],
        'torque_limits': [5.0, 5.0], 'commands_scale': [1.0, 1.0, 1.0],
    }}
    contract = PolicyContract.from_rl_sar(base, config, 'tiny', 'walk')
    policy = LeggedPolicy(contract, session=Session(dofs=2, obs_width=10))
    state = LeggedState({'j0': 0.0, 'j1': 0.0}, {'j0': 0.0, 'j1': 0.0},
                        (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    first = policy.observation(state, (1.0, 2.0, 3.0))
    # Term-major: newest-then-older per term, newest first.
    np.testing.assert_allclose(first, [1, 2, 3, 0, 0, 0, 0, 0, 0, 0])
    second = policy.observation(state, (4.0, 5.0, 6.0))
    np.testing.assert_allclose(second, [4, 5, 6, 1, 2, 3, 0, 0, 0, 0])
    policy.reset()
    third = policy.observation(state, (7.0, 8.0, 9.0))
    np.testing.assert_allclose(third, [7, 8, 9, 0, 0, 0, 0, 0, 0, 0])


def test_unsupported_contract_shapes_are_rejected():
    base = {'tiny': {'joint_names': ['j0', 'j1'], 'num_of_dofs': 2}}
    good = {'observations': ['commands'], 'dt': 0.01, 'decimation': 1,
            'default_dof_pos': [0.0, 0.0], 'action_scale': [1.0, 1.0],
            'rl_kp': [10.0, 10.0], 'rl_kd': [1.0, 1.0],
            'torque_limits': [5.0, 5.0]}
    with pytest.raises(ValueError, match='joint_mapping'):
        PolicyContract.from_rl_sar(
            base, {'tiny/walk': dict(good, joint_mapping=[0, 0])},
            'tiny', 'walk')
    with pytest.raises(ValueError, match='unsupported observation terms'):
        PolicyContract.from_rl_sar(
            base, {'tiny/walk': dict(good, observations=['commands', 'magic'])},
            'tiny', 'walk')
    with pytest.raises(ValueError, match='history'):
        PolicyContract.from_rl_sar(
            base, {'tiny/walk': dict(
                good, observations_history_priority='sideways')},
            'tiny', 'walk')
    with pytest.raises(ValueError, match='wheel_indices'):
        PolicyContract.from_rl_sar(
            base, {'tiny/walk': dict(good, wheel_indices=[7])}, 'tiny', 'walk')


def test_clamp_command_limits_and_non_finite():
    assert clamp_command((5.0, -5.0, 0.3), (0.8, 0.4, 1.0)) == [0.8, -0.4, 0.3]
    # Non-finite input (including +-inf) becomes zero, never a limit.
    assert clamp_command((float('nan'), 0.2, float('inf')),
                         (1.0, 1.0, 1.0)) == [0.0, 0.2, 0.0]


def test_mujoco_pd_config_and_bridge_contract():
    from robot_lab_mujoco.joint_pd import JointPDCommand
    for name in PROVISIONED_POLICIES:
        contract = _load(name)
        config = mujoco_pd_config(contract)
        wheels = set(contract.wheel_indices)
        assert config['joints'] == contract.training_joint_names()
        assert config['mode'] == [
            'velocity' if index in wheels else 'position'
            for index in range(contract.num_dofs)]
        bridge = JointPDCommand(json.dumps(config))
        assert bridge.names == tuple(config['joints'])
        assert bridge.mode == tuple(config['mode'])
        # The bridge's default pose holds the policy's own default: legs at
        # their default angles, wheels at zero velocity.
        np.testing.assert_allclose(
            bridge.default,
            [0.0 if index in wheels else value
             for index, value in enumerate(contract.default_dof_pos)])
        # Wrong-sized or non-finite commands are refused, not applied.
        assert bridge.receive([0.0] * (contract.num_dofs - 1), 0.0) is False
        assert bridge.receive(
            [0.0] * (contract.num_dofs - 1) + [float('nan')], 0.0) is False
        assert bridge.receive([0.5] * contract.num_dofs, 1.0) is True
        np.testing.assert_allclose(bridge.command(1.1), 0.5)
        # Command loss falls back to the default pose, never the last value.
        np.testing.assert_allclose(bridge.command(1.1 + bridge.timeout + 0.1),
                                   bridge.default)
        assert bridge.stale(1.1 + bridge.timeout + 0.1) is True


def test_joint_pd_actuators_are_added_once_per_joint():
    import xml.etree.ElementTree as ET
    from robot_lab_mujoco.joint_pd import JointPDCommand
    contract = _load('go2w_robot_lab')
    bridge = JointPDCommand(json.dumps(mujoco_pd_config(contract)))
    joints = ''.join(
        '<joint name="%s" type="hinge"/>' % name for name in bridge.names)
    mjcf = '<mujoco><worldbody><body>%s</body></worldbody></mujoco>' % joints
    staged = bridge.add_actuators(mjcf, armature=0.01)
    actuators = {actuator.get('joint'): actuator
                 for actuator in ET.fromstring(staged).findall('./actuator/general')}
    assert set(actuators) == set(bridge.names)
    position = actuators['FR_hip_joint']
    assert position.get('gaintype') == 'fixed'
    assert float(position.get('gainprm').split()[0]) == contract.rl_kp[0]
    assert float(position.get('forcerange').split()[1]) == contract.torque_limits[0]
    velocity = actuators['FR_foot_joint']
    assert float(velocity.get('gainprm').split()[0]) == contract.rl_kd[12]
    assert velocity.get('biasprm').split()[:2] == ['0', '0']
    # A second application on the same model is refused, not duplicated.
    with pytest.raises(ValueError, match='already actuated'):
        bridge.add_actuators(staged)
    with pytest.raises(ValueError, match='missing from the model'):
        bridge.add_actuators('<mujoco><worldbody><body>'
                             '<joint name="other" type="hinge"/>'
                             '</body></worldbody></mujoco>')


def test_gui_policy_mapping_matches_the_adapter():
    """The GUI mirrors the adapter's policy table.  Parsed from source so
    the fast tier needs neither ament_index_python nor a display."""
    import ast
    launcher = pkg.parent / 'robot_lab_gui' / 'robot_lab_gui' / 'launcher.py'
    tree = ast.parse(launcher.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(target, 'id', '') == 'LEGGED_POLICY_PROFILES'
                for target in node.targets):
            # The GUI table is robot->policy; the adapter's is policy->robot.
            gui_mapping = ast.literal_eval(node.value)
            assert gui_mapping == {profile: name
                                   for name, profile in POLICY_PROFILES.items()}
            break
    else:
        raise AssertionError('launcher.py lacks LEGGED_POLICY_PROFILES')



