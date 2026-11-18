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


# --- mobile manipulation (R5.9) ------------------------------------------------
FETCH_WORLD = WORLD.parents[2] / 'mobile_fetch_lab/worlds/mobile_fetch_lab.world'


def test_standoff_faces_the_nearest_edge_at_reach():
    from robot_lab_mujoco.arm_pick_place import standoff
    table = Box('t', (1.6, 0.0, 0.25), (1, 0, 0, 0), (0.6, 1.0, 0.5))
    x, y, yaw, depth = standoff(table, (1.45, 0.0), 0.62, (0.0, 0.0))
    assert (x, y, yaw) == pytest.approx((0.83, 0.0, 0.0))
    assert depth == pytest.approx(0.15)
    turned = Box('t', (0.0, 0.0, 0.25), (math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)), (0.6, 1.0, 0.5))
    # Turned 90 deg: the object is 0.15 m inside the +y edge, whatever side the robot is on.
    x, y, yaw, depth = standoff(turned, (0.0, 0.15), 0.62, (0.0, -2.0))
    assert (x, y) == pytest.approx((0.0, 0.77)) and yaw == pytest.approx(-math.pi / 2)
    # A centred object is a tie: the side facing the robot wins.
    x, y, yaw, depth = standoff(turned, (0.0, 0.0), 0.62, (0.0, -2.0))
    assert (x, y) == pytest.approx((0.0, -0.62)) and yaw == pytest.approx(math.pi / 2)


def test_place_standoff_uses_the_side_facing_the_robot():
    from robot_lab_mujoco.arm_pick_place import place_standoff
    table = Box('t', (0.0, 2.2, 0.25), (1, 0, 0, 0), (1.0, 0.6, 0.5))
    (x, y, yaw), place = place_standoff(table, (0.83, 0.0), 0.15, 0.62)
    assert place == pytest.approx((0.45, 2.05))
    assert (x, y, yaw) == pytest.approx((0.45, 1.43, math.pi / 2))


def test_segment_clearance_measures_the_closest_box():
    from robot_lab_mujoco.arm_pick_place import segment_clearance
    boxes = [Box('a', (1.6, 0.0, 0.25), (1, 0, 0, 0), (0.6, 1.0, 0.5))]
    distance, name = segment_clearance(boxes, (0.0, -1.0), (0.0, 1.0))
    assert name == 'a' and distance == pytest.approx(1.3)
    assert segment_clearance(boxes, (1.0, 0.0), (2.0, 0.0))[0] == 0.0


def test_mobile_scene_welds_the_base_and_holds_unplanned_joints():
    try:
        from robot_lab_utils.installed_assets import installed_profiles
        profile = installed_profiles('robots')['asset_pal_tiago']
        from robot_lab_adapter.legged_training import _package_map
        from robot_lab_mujoco.mujoco_spawner import _build_mjcf_from_urdf
    except Exception:  # noqa: BLE001
        pytest.skip('the provisioned TIAGo and installed packages are needed')
    from robot_lab_mujoco.arm_pick_place import plan_pick
    if 'pick_place' not in profile:
        pytest.skip('installed overlays without the TIAGo pick_place block')
    urdf = Path(profile['xacro']).read_text()
    mjcf = _build_mjcf_from_urdf(urdf, _package_map(urdf), robot_name='tiago')
    spec = ArmSpec.from_config(profile['pick_place'])
    home = profile['articulation']['home']
    table = Box('pickup_table', (1.6, 0.0, 0.25), (1, 0, 0, 0), (0.6, 1.0, 0.5))
    block = Box('fetch_block', (1.45, 0.0, 0.575), (1, 0, 0, 0), (0.04, 0.04, 0.15))
    held = {n: home[n] for n in ('torso_lift_joint', 'head_1_joint', 'head_2_joint')}
    scene = ArmScene(mjcf, spec, [table], block, base=(0.83, 0.0, 0.006, 0.0), held=held)
    arm = {n: home[n] for n in spec.arm_joints}
    plan = plan_pick(scene, arm, block, seeds=[arm])
    # The PAL finger pads reach below the grasp frame: the grasp rises.
    assert plan['grasp_point'][2] > block.position[2] + 0.03
    assert [s['stage'] for s in plan['stages']] == ['open', 'above_object', 'down_to_grasp', 'close', 'lift']


def test_an_upright_cylinder_is_gripped_across_its_diameter(panda):
    can = Box('pickup_can', [0.5, -0.25, 0.345], [1, 0, 0, 0], [0.05, 0.05, 0.09], 'cylinder')
    scene, spec, obstacles, obj, profile = scene_for(panda, can)
    geom = scene.model.geom('object')
    assert geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER
    np.testing.assert_allclose(geom.size[:2], [0.025, 0.045])
    home = {n: profile['articulation']['home'][n] for n in spec.arm_joints}
    support = next(b for b in obstacles if b.name == 'place_table')
    plan = plan_pick_place(scene, home, can, support, seeds=[home])
    assert plan['grasp_width_m'] == pytest.approx(0.05)
    scene.detach()
    lower = next(s for s in plan['stages'] if s['stage'] == 'lower')
    scene.set_joints(dict(lower['waypoints'][-1], **lower['fingers']))
    # The can is set down upright on the support: its centre half its length above the top.
    assert plan['place_centre'][2] == pytest.approx(support.top + 0.045)
