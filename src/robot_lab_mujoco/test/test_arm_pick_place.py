"""Top-down pick-and-place planning on the installed URDF Panda (R5.7, arm_pick_place.py)."""
import math
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from robot_lab_mujoco.arm_pick_place import (  # noqa: E402
    ArmScene, ArmSpec, Box, PlanningError, plan_pick_place, synchronised)

WORLD = Path(__file__).resolve().parents[2] / 'robot_lab_maps/maps/arm_pick_place_lab/worlds/arm_pick_place_lab.world'


@pytest.fixture(scope='module')
def panda():
    try:
        from robot_lab_utils.installed_assets import installed_profiles
        profile = installed_profiles('robots')['asset_franka_panda_panda']
        from robot_lab_adapter.legged_training import _package_map
        from robot_lab_mujoco.mujoco_spawner import _build_mjcf_from_urdf
    except Exception:  # noqa: BLE001
        pytest.skip('the provisioned URDF Panda and installed packages are needed')
    urdf = Path(profile['xacro']).read_text()
    return profile, _build_mjcf_from_urdf(urdf, _package_map(urdf), robot_name='panda')


def scene_for(panda, obj=None):
    from robot_lab_utils.sdf_world import extract_dynamic_bodies, extract_static_shapes
    profile, mjcf = panda
    shapes, _ = extract_static_shapes(str(WORLD), lambda uri, base: '', exclude_dynamic=True)
    obstacles = [Box(s['model'], s['position'], s['orientation'], s['size']) for s in shapes if s['type'] == 'box']
    if obj is None:
        body = extract_dynamic_bodies(str(WORLD))[0]
        obj = Box(body['name'], body['position'], body['orientation'], body['shape']['size'])
    spec = ArmSpec.from_config(profile['pick_place'])
    return ArmScene(mjcf, spec, obstacles, obj), spec, obstacles, obj, profile


def test_plan_from_home_reaches_grasp_and_place_without_collision(panda):
    scene, spec, obstacles, obj, profile = scene_for(panda)
    home = {n: profile['articulation']['home'][n] for n in spec.arm_joints}
    support = next(b for b in obstacles if b.name == 'place_table')
    plan = plan_pick_place(scene, home, obj, support, seeds=[home])
    names = [s['stage'] for s in plan['stages']]
    assert names == ['open', 'above_object', 'down_to_grasp', 'close', 'lift', 'over_support', 'lower', 'release',
                     'retreat']
    reached = {}
    for stage in plan['stages']:
        scene.detach()
        scene.set_joints(dict(stage['waypoints'][-1], **stage['fingers']))
        reached[stage['stage']] = scene.tcp()[0]
    np.testing.assert_allclose(reached['down_to_grasp'], obj.position, atol=0.002)
    np.testing.assert_allclose(reached['lower'], [support.position[0], support.position[1], support.top + 0.02],
                               atol=0.002)
    assert reached['lift'][2] == pytest.approx(obj.position[2] + 0.10, abs=0.002)
    assert plan['grasp_width_m'] == pytest.approx(0.04)


def test_a_colliding_start_or_a_too_wide_object_is_refused(panda):
    scene, spec, obstacles, obj, profile = scene_for(panda)
    support = next(b for b in obstacles if b.name == 'place_table')
    zeros = {n: 0.0 for n in spec.arm_joints}        # the Panda's link5 is inside a finger here
    with pytest.raises(PlanningError, match='start pose in collision'):
        plan_pick_place(scene, zeros, obj, support)
    wide = Box('wide', obj.position, obj.quaternion, (0.09, 0.09, 0.04))
    scene, spec, obstacles, _, profile = scene_for(panda, wide)
    home = {n: profile['articulation']['home'][n] for n in spec.arm_joints}
    with pytest.raises(PlanningError, match='wider than the gripper'):
        plan_pick_place(scene, home, wide, support, seeds=[home])


def test_synchronised_targets_respect_the_rate():
    start = {'a': 0.0, 'b': 0.0}
    out = synchronised([{'a': 1.0, 'b': -0.25}, {'a': 1.0, 'b': 0.0}], start, rate=0.3, period=0.1)
    previous = start
    for target in out:
        assert max(abs(target[k] - previous[k]) for k in target) <= 0.03 + 1e-12
        previous = target
    assert out[-1] == {'a': 1.0, 'b': 0.0}
    assert math.isclose(out[33]['a'], 1.0)          # 1 rad at 0.03 per step: reached at step 34
