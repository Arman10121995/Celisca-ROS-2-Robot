"""get_up task (settled collapsed starts) and the stairs curriculum terrain (R5.2/R12)."""
import sys
from pathlib import Path

import numpy as np
import pytest

pkg = Path(__file__).resolve().parents[1]
if str(pkg) not in sys.path:
    sys.path.insert(0, str(pkg))
pytest.importorskip('mujoco')

from robot_lab_adapter.legged_training import (  # noqa: E402
    STAIR_MAX_RISE_M, TASKS, LeggedTrainingEnv, _quat_rotate_inverse, stairs_terrain_heights)


def test_pyramid_stairs_are_flat_at_the_spawn_and_climb_in_treads():
    heights = stairs_terrain_heights(.1)
    centre = heights.shape[0] // 2
    assert heights[centre, centre] == 0.
    row = heights[centre, centre:]
    steps = np.unique(np.round(row, 6))
    assert steps[1] == pytest.approx(.1) and steps.max() == pytest.approx(1.)   # ten 0.1 m rises
    assert np.all(np.diff(row[:int(4. / .1)]) >= -1e-9)                         # climbing outward first


def test_get_up_task_has_no_fall_termination():
    task = TASKS['get_up']
    assert task.reset == 'fallen' and not task.terminate_on_fall and not task.only_positive


@pytest.fixture(scope='module')
def go2_get_up():
    try:
        return LeggedTrainingEnv('unitree_go2', 'get_up', num_envs=8, seed=0, threads=2, episode_s=1.0)
    except Exception as exc:  # robot description packages unavailable
        pytest.skip(str(exc))


def test_get_up_episodes_start_collapsed_and_never_end_on_a_fall(go2_get_up):
    env = go2_get_up
    qpos, _ = env._split(env.state)
    gravity = _quat_rotate_inverse(qpos[:, 3:7], np.tile([0., 0., -1.], (env.num_envs, 1)))
    assert np.mean(gravity[:, 2] < -.9) < .5                     # most starts are not upright
    assert np.all(qpos[:, 2] < env.stand_height)                  # settled on the floor
    for _ in range(int(.8 / env.dt)):
        _, reward, done, info = env.step(np.zeros((env.num_envs, env.act_dim)))
        assert not np.any(info['fallen']) and not np.any(done)
        assert np.all(np.isfinite(reward))
    assert 'upright' in info['terms'] and 'standing_height' in info['terms']


def test_stairs_level_rewrites_the_heightfield():
    try:
        env = LeggedTrainingEnv('unitree_go2', 'walk', num_envs=2, seed=0, threads=1, episode_s=1.0, terrain='stairs')
    except Exception as exc:
        pytest.skip(str(exc))
    low = env.set_terrain_level(0.)
    first = env.model.hfield_data.copy()
    high = env.set_terrain_level(1.)
    assert low == pytest.approx(.02) and high == pytest.approx(STAIR_MAX_RISE_M)
    assert env.model.hfield_data.max() > first.max() * 5
    assert env.terrain_height(np.array([[0., 0.], [1.45, 0.]]))[1] == pytest.approx(STAIR_MAX_RISE_M)   # first tread
