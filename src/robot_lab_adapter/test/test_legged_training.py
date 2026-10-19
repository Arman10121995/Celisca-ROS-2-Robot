"""Legged policy trainer: PPO, ONNX export and the MuJoCo training environment.

The export must reproduce the trained actor (normalisation folded in) when
onnxruntime runs it through the deployment code (legged_policy.LeggedPolicy),
and an exported directory must load as an ordinary policy bound to its robot.
The environment checks build a two-copy Go2 model and step it; they need the
installed robot description packages and are skipped without them.
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip('torch')

pkg = Path(__file__).resolve().parents[1]
if str(pkg) not in sys.path:
    sys.path.insert(0, str(pkg))

from robot_lab_adapter.legged_policy import LeggedPolicy, LeggedState, PolicyContract  # noqa: E402
from robot_lab_adapter.legged_training import (  # noqa: E402
    OBSERVATIONS, TASKS, TRAINABLE, _quat_rotate_inverse, policy_config)
from robot_lab_adapter.ppo import PPO, ActorCritic, export_onnx  # noqa: E402

POLICY_ROOT = pkg / 'policies' / 'rl_sar'


def _trained_model(obs_dim, act_dim, seed=0):
    torch.manual_seed(seed)
    model = ActorCritic(obs_dim, act_dim, hidden=(32, 16))
    with torch.no_grad():
        model.norm.update(torch.randn(500, obs_dim) * 3.0 + 1.5)
    return model


def test_trainable_robots_are_position_controlled_quadrupeds():
    assert set(TRAINABLE) == {'unitree_a1', 'unitree_b2', 'unitree_go2'}
    for name in TRAINABLE.values():
        contract = PolicyContract.load(POLICY_ROOT / name)
        assert contract.num_dofs == 12 and not list(contract.wheel_indices)
    assert set(TASKS) == {'walk', 'turn_in_place', 'stand', 'push_recovery'}
    for task in TASKS.values():
        assert task.rewards['termination'] < 0 and task.description and task.only_positive


def test_onnx_export_matches_the_actor_with_normalisation_folded_in(tmp_path):
    ort = pytest.importorskip('onnxruntime')
    model = _trained_model(45, 12)
    path = export_onnx(model, str(tmp_path / 'policy.onnx'))
    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    assert [i.name for i in session.get_inputs()] == ['obs']
    obs = np.random.default_rng(1).normal(size=(7, 45)).astype(np.float32) * 2.0
    exported = session.run(None, {'obs': obs})[0]
    with torch.no_grad():
        expected = model.act(torch.as_tensor(obs), deterministic=True)[0].numpy()
    assert exported.shape == (7, 12)
    np.testing.assert_allclose(exported, expected, rtol=1e-4, atol=1e-5)


def test_ppo_update_keeps_the_rollout_normaliser_and_returns_finite_stats():
    torch.manual_seed(0)
    agent = PPO(6, 2, num_envs=8, steps=5, hidden=(16, 16), epochs=2, minibatches=2)
    rng = np.random.default_rng(0)
    before = agent.model.norm.mean.clone()
    obs = rng.normal(size=(8, 6)).astype(np.float32)
    for step in range(5):
        actions = agent.act(obs)
        assert actions.shape == (8, 2)
        # The normaliser is frozen during a rollout.
        assert torch.equal(agent.model.norm.mean, before)
        reward = -np.sum(actions ** 2, axis=1)
        done = np.zeros(8, bool)
        done[step] = step == 4
        agent.record(reward, done, np.zeros(8, bool))
        obs = rng.normal(size=(8, 6)).astype(np.float32)
    stats = agent.update(obs)
    assert set(stats) >= {'value_loss', 'surrogate_loss', 'entropy', 'kl', 'learning_rate', 'action_std'}
    assert all(np.isfinite(v) for v in stats.values())
    assert agent.step == 0
    assert not torch.equal(agent.model.norm.mean, before)


def test_quat_rotate_inverse_matches_a_yawed_body():
    yaw = 0.7
    quat = np.array([[np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]])
    body = _quat_rotate_inverse(quat, np.array([[1.0, 0.0, 0.0]]))[0]
    np.testing.assert_allclose(body, [np.cos(yaw), -np.sin(yaw), 0.0], atol=1e-12)


def _export(tmp_path, robot, obs_dim):
    """A checkpoint shaped like legged_train writes, exported to a policy directory."""
    from robot_lab_adapter.legged_train import export_policy
    model = _trained_model(obs_dim, 12)
    run = dict(robot=robot, task='walk', obs_dim=obs_dim, act_dim=12, hidden=[32, 16], name='unit',
               revision='test')
    checkpoint = tmp_path / 'latest.pt'
    torch.save(dict(model=model.state_dict(), run=run, iteration=3, environment_steps=96), checkpoint)
    return model, export_policy(checkpoint, tmp_path / 'policy')


@pytest.mark.parametrize('robot', sorted(TRAINABLE))
def test_exported_directory_is_an_ordinary_policy_bound_to_its_robot(tmp_path, monkeypatch, robot):
    pytest.importorskip('onnxruntime')
    import robot_lab_adapter.legged_training as training
    monkeypatch.setattr(training, 'template_directory', lambda name: str(POLICY_ROOT / TRAINABLE[name]))
    template = PolicyContract.load(POLICY_ROOT / TRAINABLE[robot])
    obs_dim = 9 + 3 * template.num_dofs
    model, directory = _export(tmp_path, robot, obs_dim)
    contract = PolicyContract.load(directory)
    source = json.loads((directory / 'source.json').read_text())
    assert source['profile'] == robot and source['trained_by'] == 'robot_lab_adapter.legged_train'
    assert source['template_policy'] == TRAINABLE[robot]
    assert contract.observations == OBSERVATIONS and not contract.history
    assert contract.num_observations == obs_dim
    for field in ('default_dof_pos', 'action_scale', 'rl_kp', 'rl_kd', 'torque_limits'):
        np.testing.assert_allclose(getattr(contract, field), getattr(template, field))
    assert contract.training_joint_names() == template.training_joint_names()
    assert (contract.dt, contract.decimation) == (template.dt, template.decimation)
    # The deployment code runs the exported network on the trained actor's input.
    policy = LeggedPolicy(contract, model_path=str(directory / 'policy.onnx'))
    rng = np.random.default_rng(2)
    names = contract.training_joint_names()
    state = LeggedState(dict(zip(names, contract.default_dof_pos + rng.uniform(-.1, .1, 12))),
                        dict(zip(names, rng.uniform(-1, 1, 12))), (0.99, 0.05, -0.05, 0.1), (0.1, -0.2, 0.3))
    command = (0.4, 0.1, -0.2)
    observation = policy.observation(state, command).astype(np.float32)
    policy.reset()
    policy.step(state, command)
    with torch.no_grad():
        expected = model.act(torch.as_tensor(observation[None]), deterministic=True)[0].numpy()[0]
    expected = np.clip(expected, contract.clip_actions_lower, contract.clip_actions_upper)
    np.testing.assert_allclose(policy.actions, expected, rtol=1e-4, atol=1e-5)


def test_policy_config_has_the_trainer_observation_width():
    contract = PolicyContract.load(POLICY_ROOT / 'go2_himloco')
    section = policy_config(contract, 'go2', 'lab_walk_x')
    (key, values), = section.items()
    assert key == 'go2/lab_walk_x'
    assert values['num_observations'] == 9 + 3 * contract.num_dofs == 45
    assert values['observations'] == OBSERVATIONS and values['observations_history'] == []


@pytest.fixture(scope='module')
def go2_env():
    pytest.importorskip('mujoco')
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('robot_lab_robots')
        get_package_share_directory('robot_lab_adapter')
    except Exception:  # noqa: BLE001
        pytest.skip('installed robot_lab packages are needed for the MuJoCo model')
    from robot_lab_adapter.legged_training import LeggedTrainingEnv
    return LeggedTrainingEnv('unitree_go2', 'stand', num_envs=2, seed=0, threads=1, episode_s=2.0)


def test_go2_environment_shapes_feet_and_a_held_default_stance(go2_env):
    env = go2_env
    assert env.num_feet == 4
    assert env.obs_dim == 45 and env.act_dim == 12
    assert 0.2 < env.stand_height < 0.5
    obs = env.reset_all()
    assert obs.shape == (2, 45) and obs.dtype == np.float32
    # Zero actions are the policy contract's default pose under its PD law:
    # the robot must stand, not fall, for a second.
    for _ in range(int(round(1.0 / env.dt))):
        obs, reward, done, info = env.step(np.zeros((2, 12)))
        assert np.all(np.isfinite(obs)) and np.all(np.isfinite(reward))
        assert not np.any(info['fallen'])
        # Clipped at zero until a fall: ending early never pays.
        assert np.all(reward >= 0.0)
    qpos, _ = env._split(env.state)
    assert np.all(qpos[:, 2] > 0.7 * env.stand_height)
    assert set(info['terms']) >= set(TASKS['stand'].rewards)


def test_episodes_time_out_and_reset(go2_env):
    env = go2_env
    env.reset_all()
    finished = 0
    for _ in range(env.max_steps):
        _, _, done, info = env.step(np.zeros((2, 12)))
        finished += int(done.sum())
    assert finished == 2
    assert np.all(env.episode_step == 0)
    assert len(info['finished_returns']) == 2 and np.all(info['timeout'])


def test_rough_terrain_is_smooth_bounded_and_flat_at_the_spawn():
    from robot_lab_adapter.legged_training import TERRAIN_RESOLUTION_M, rough_terrain_heights
    heights = rough_terrain_heights(seed=4, roughness=0.03)
    assert np.abs(heights).max() == pytest.approx(0.03, rel=1e-6)
    axis = (np.arange(heights.shape[0]) - heights.shape[0] // 2) * TERRAIN_RESOLUTION_M
    within_1m = np.hypot(axis[:, None], axis[None, :]) <= 0.99
    assert np.all(heights[within_1m] == 0.0) and np.any(heights[~within_1m] != 0.0)
    # Neighbouring cells 0.1 m apart differ by far less than the full range.
    assert np.abs(np.diff(heights, axis=0)).max() < 0.03 * TERRAIN_RESOLUTION_M / 0.1 * 0.6
    assert not np.array_equal(heights, rough_terrain_heights(seed=5, roughness=0.03))


@pytest.fixture(scope='module')
def go2_rough_env():
    pytest.importorskip('mujoco')
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('robot_lab_robots')
    except Exception:  # noqa: BLE001
        pytest.skip('installed robot_lab packages are needed for the MuJoCo model')
    from robot_lab_adapter.legged_training import LeggedTrainingEnv, Randomization
    return LeggedTrainingEnv('unitree_go2', 'walk', num_envs=4, seed=0, threads=1, episode_s=2.0,
                             randomization=Randomization(variants=3), terrain='rough', obs_noise=1.0)


def test_randomised_variants_differ_and_the_heightfield_matches_the_lookup(go2_rough_env):
    import mujoco
    env = go2_rough_env
    assert len(env.models) == 3
    colliding = env.model.geom_contype != 0
    frictions = {round(float(m.geom_friction[colliding][0, 0]), 6) for m in env.models}
    gains = {round(float(m.actuator_gainprm[env.actuator_index[0], 0]), 6) for m in env.models}
    assert len(frictions) == 3 and len(gains) == 3
    for model in env.models:
        assert 0.5 <= model.geom_friction[colliding][0, 0] <= 1.25
    data = mujoco.MjData(env.model)
    mujoco.mj_forward(env.model, data)
    floor = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM, 'training_floor')
    for x, y in ((3.0, 1.0), (-7.3, 4.2), (12.0, -9.0)):
        geom = np.zeros(1, np.int32)
        distance = mujoco.mj_ray(env.model, data, np.array([x, y, 1.0]), np.array([0, 0, -1.0]),
                                 None, 1, -1, geom)
        assert geom[0] == floor
        assert 1.0 - distance == pytest.approx(env.terrain_height(np.array([[x, y]]))[0], abs=0.004)


def test_randomised_rough_env_stands_and_observations_are_noisy(go2_rough_env):
    env = go2_rough_env
    env.reset_all()
    first, second = env.observations(), env.observations()
    assert not np.array_equal(first[:, :3], second[:, :3]), 'ang_vel carries noise'
    assert np.array_equal(first[:, 6:9], second[:, 6:9]), 'commands carry none'
    for _ in range(int(round(1.0 / env.dt))):
        obs, reward, done, info = env.step(np.zeros((4, 12)))
        assert np.all(np.isfinite(obs)) and np.all(np.isfinite(reward))
        assert not np.any(info['fallen'])
