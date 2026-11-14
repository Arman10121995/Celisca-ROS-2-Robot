"""BHL training template and export path (R5.3): bhl_training.py."""
import numpy as np
import pytest

torch = pytest.importorskip('torch')

from robot_lab_adapter.bhl_policy import build_observation, load_policy_config  # noqa: E402
from robot_lab_adapter.bhl_training import export_bhl, input_permutation, write_template  # noqa: E402
from robot_lab_adapter.legged_policy import PolicyContract  # noqa: E402
from robot_lab_adapter.ppo import ActorCritic  # noqa: E402


def test_template_is_the_upstream_contract_in_rl_sar_layout(tmp_path):
    upstream = load_policy_config(name='policy_humanoid')
    contract = PolicyContract.load(write_template(tmp_path))
    assert contract.source['profile'] == 'berkeley_humanoid_lite_sim'
    assert contract.training_joint_names() == list(upstream.joints)
    np.testing.assert_allclose(contract.default_dof_pos, upstream.nominal)
    np.testing.assert_allclose(contract.rl_kp, upstream.kp)
    np.testing.assert_allclose(contract.rl_kd, upstream.kd)
    np.testing.assert_allclose(contract.torque_limits, upstream.effort_limits)
    np.testing.assert_allclose(contract.action_scale, upstream.action_scale)
    assert contract.policy_period == pytest.approx(upstream.policy_dt)
    assert contract.dt == pytest.approx(1 / 250.)          # the deployed effort-command rate
    # Unscaled observations, as bhl_policy.build_observation feeds the policy.
    assert (contract.ang_vel_scale, contract.dof_pos_scale, contract.dof_vel_scale) == (1., 1., 1.)
    np.testing.assert_allclose(contract.commands_scale, 1.)


def test_input_permutation_moves_the_command_block_first():
    perm = input_permutation(2)
    # trainer: ang_vel(0-2) gravity(3-5) commands(6-8) | bhl: commands(0-2) gyro(3-5) gravity(6-8)
    assert perm[:9] == [3, 4, 5, 6, 7, 8, 0, 1, 2] and perm[9:] == list(range(9, 15))


def test_exported_bhl_policy_loads_and_sees_the_trainer_observation(tmp_path):
    ort = pytest.importorskip('onnxruntime')
    torch.manual_seed(0)
    model = ActorCritic(75, 22, (32, 32))
    with torch.no_grad():
        model.norm.mean.uniform_(-.5, .5)
        model.norm.var.uniform_(.5, 2.)
    out = export_bhl(model, tmp_path / 'bhl', dict(task='walk', name='t'))
    config = load_policy_config(upstream=out, name='policy_humanoid')
    session = ort.InferenceSession(str(config.checkpoint), providers=['CPUExecutionProvider'])
    rng = np.random.default_rng(2)
    cmd, gyro, grav = rng.normal(size=3), rng.normal(size=3), rng.normal(size=3)
    q, dq, prev = rng.normal(size=22), rng.normal(size=22), rng.normal(size=22)
    bhl_obs = build_observation(cmd, gyro, grav, q, dq, prev, config.nominal)
    trainer_obs = np.concatenate([gyro, grav, cmd, q - config.nominal, dq, prev]).astype(np.float32)
    exported = session.run(None, {session.get_inputs()[0].name: bhl_obs[None, :]})[0]
    with torch.no_grad():
        expected = model.act(torch.as_tensor(trainer_obs[None, :]), deterministic=True)[0].numpy()
    np.testing.assert_allclose(exported, expected, rtol=1e-4, atol=1e-5)
