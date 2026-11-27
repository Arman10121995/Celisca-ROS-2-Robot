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


def test_compound_grasp_uses_the_post_and_places_the_whole_stepped_part(panda):
    from robot_lab_mujoco.arm_pick_place import CompoundPayload
    from robot_lab_utils.sdf_world import extract_dynamic_bodies

    world = WORLD.parents[2] / 'arm_pick_place_compound/worlds/arm_pick_place_compound.world'
    body, = extract_dynamic_bodies(world)
    identity = (1., 0., 0., 0.)
    parts = tuple(Box(shape['collision_name'], shape['position'], shape['orientation'], size, shape['type'])
                  for shape, size in zip(body['shapes'], ((.068, .061, .020), (.04, .04, .068))))
    obj = CompoundPayload(body['name'], body['position'], identity, (.068, .061, .087), 'compound',
                          parts=parts, grasp_index=1)
    scene, spec, obstacles, _, profile = scene_for(panda, obj)
    home = {n: profile['articulation']['home'][n] for n in spec.arm_joints}
    support = next(box for box in obstacles if box.name == 'place_table')
    plan = plan_pick_place(scene, home, obj, support, seeds=[home])
    assert plan['grasp_width_m'] == pytest.approx(.04)
    assert plan['place_centre'][2] == pytest.approx(.322, abs=.0001)
    assert plan['stages'][-1]['stage'] == 'retreat'
    narrow = Box('small support', support.position, identity, (.05, .05, support.size[2]))
    assert narrow.supports(obj.grasp_primitive().at_pose(support.position, identity))
    assert not narrow.supports(obj.at_pose(plan['place_centre'], identity))

    foot_obstacle = Box('stop', (.531, -.25, .31), identity, (.01, .01, .01))
    blocked = ArmScene(panda[1], spec, [foot_obstacle], obj)
    hits = blocked.collisions(allow_fingers=True)
    assert any(part == 'object_part_0' and obstacle == 'obstacle_stop' for part, obstacle, _ in hits)
    assert not any(part == 'object_part_1' and obstacle == 'obstacle_stop' for part, obstacle, _ in hits)


def test_pick_lift_exempts_only_the_payloads_starting_support(panda, monkeypatch):
    from robot_lab_mujoco.arm_pick_place import plan_pick

    scene, spec, obstacles, obj, profile = scene_for(panda)
    home = {name: profile['articulation']['home'][name] for name in spec.arm_joints}
    expected = tuple(box.name for box in obstacles if obj.rests_on(box, height_tolerance=.01))
    assert expected == ('pickup_table',)
    assert any(box.name == 'place_table' for box in obstacles)
    seen = []
    original = scene.straight_line

    def record(*args, **kwargs):
        if scene.attached is not None:
            seen.append(tuple(kwargs.get('allow_supports', ())))
        return original(*args, **kwargs)

    monkeypatch.setattr(scene, 'straight_line', record)
    assert plan_pick(scene, home, obj, seeds=[home])['stages'][-1]['stage'] == 'lift'
    assert seen == [expected]


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


def test_private_arm_scene_retains_cylinder_and_sphere_obstacles():
    xml = """<mujoco><worldbody><body name="tool" pos="0 0 2">
      <joint name="arm" type="hinge"/><geom type="sphere" size=".03"/>
    </body></worldbody></mujoco>"""
    quat = (1., 0., 0., 0.)
    sphere = Box('sphere', (.15, 0., 0.), quat, (.12, .12, .12), 'sphere')
    pillar = Box('pillar', (-.15, 0., 0.), quat, (.12, .12, .4), 'cylinder')
    payload = Box('payload', (0., 0., 0.), quat, (.06, .06, .06))
    scene = ArmScene(xml, ArmSpec(['arm'], [], 'tool'), [sphere, pillar], payload)
    assert not scene.collisions()
    for obstacle in (sphere, pillar):
        scene.move_object(obstacle.position, quat)
        assert any('obstacle_' + obstacle.name in hit[:2] for hit in scene.collisions())


def test_measured_grasp_keeps_only_existing_jaw_contact():
    model = """<mujoco><worldbody><body name="tool">
      <joint name="arm" type="hinge"/>
      <geom name="housing" type="box" pos="0 .1 0" size=".02 .02 .02"/>
      <body pos="-.03 0 0"><joint name="left" type="slide" axis="1 0 0"/>
        <geom name="jaw" type="box" size=".02 .02 .02"/>
        <geom name="left_pad" type="box" pos="0 -.03 0" size=".01 .01 .01"/>
      </body>
      <body pos=".09 0 0"><joint name="right" type="slide" axis="1 0 0"/>
        <geom name="right_pad" type="box" size=".01 .01 .01"/>
      </body>
    </body></worldbody></mujoco>"""
    spec = ArmSpec(['arm'], ['left', 'right'], 'tool', pad_geoms=('left_pad', 'right_pad'))
    payload = Box('payload', (0, 0, 0), (1, 0, 0, 0), (.03, .03, .03))
    scene = ArmScene(model, spec, [], payload)
    assert set(scene.collisions(allow_fingers=True)[0][:2]) == {'object', 'jaw'}
    scene.attach(measured_grasp=True)
    assert scene.collisions(allow_fingers=True) == []
    scene.move_object((-.008, 0, 0), payload.quaternion)
    assert set(scene.collisions(allow_fingers=True)[0][:2]) == {'object', 'jaw'}
    scene.move_object((0, .1, 0), payload.quaternion)
    assert set(scene.collisions(allow_fingers=True)[0][:2]) == {'object', 'housing'}
    scene.detach()
    scene.move_object(payload.position, payload.quaternion)
    assert set(scene.collisions(allow_fingers=True)[0][:2]) == {'object', 'jaw'}


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


def test_floor_allows_frozen_chassis_but_rejects_moving_arm_and_payload_penetration():
    xml = '''<mujoco><worldbody><body name="base">
      <freejoint/><geom type="box" name="chassis" pos="0 0 0" size=".1 .1 .1"/>
      <body name="tool" pos=".3 0 .2">
        <joint name="arm" type="slide" axis="0 0 1" range="-.5 .5"/>
        <geom type="box" name="moving_tool" size=".03 .03 .03"/>
      </body>
    </body></worldbody></mujoco>'''
    spec = ArmSpec.from_config(dict(arm_joints=['arm'], finger_joints=[], tool_body='tool',
                                   tcp_offset=[0., 0., 0.], finger_open=.04, finger_closed=0., max_object_width=.1))
    floor = Box('floor', [0., 0., -.05], [1., 0., 0., 0.], [8., 8., .1], floor=True)
    obj = Box('pickup', [1., 0., .25], [1., 0., 0., 0.], [.04, .04, .04])
    scene = ArmScene(xml, spec, [floor], obj)
    assert not scene.collisions()
    scene.set_joints({'arm': -.22})
    assert any({a, b} == {'moving_tool', 'obstacle_floor'} for a, b, _ in scene.collisions())
    scene.move_object([1., 0., .005], [1., 0., 0., 0.])
    assert any({a, b} == {'object', 'obstacle_floor'} for a, b, _ in scene.collisions())


def test_authored_mesh_obstacle_checks_the_payload_with_native_geometry(tmp_path):
    trimesh = pytest.importorskip('trimesh')
    from robot_lab_mujoco.arm_pick_place import MeshObstacle

    mesh_path = tmp_path/'hull.stl'
    trimesh.creation.box(extents=[.12, .16, .2]).export(mesh_path)
    obstacle = MeshObstacle.from_sdf('fixture', dict(mesh=str(mesh_path), scale=[1., 1., 1.],
                                                       position=[.4, 0., .2], orientation=[1., 0., 0., 0.]))
    spec = ArmSpec.from_config(dict(arm_joints=['arm'], finger_joints=[], tool_body='tool',
                                   tcp_offset=[0., 0., 0.], finger_open=.04, finger_closed=0., max_object_width=.1))
    xml = '''<mujoco><worldbody><body name="tool" pos="0 0 0.2">
        <joint name="arm" type="slide" axis="1 0 0"/><geom type="sphere" size=".03"/>
    </body></worldbody></mujoco>'''
    obj = Box('pickup', [.7, 0., .2], [1., 0., 0., 0.], [.04, .04, .04])
    scene = ArmScene(xml, spec, [obstacle], obj)
    assert scene.model.geom('obstacle_fixture').type[0] == mujoco.mjtGeom.mjGEOM_MESH
    assert not scene.collisions()
    scene.set_joints({'arm': .39})
    assert any('obstacle_fixture' in (a, b) for a, b, _ in scene.collisions())
    scene.set_joints({'arm': 0.})
    scene.move_object([.4, 0, .2], [1., 0., 0., 0.])
    assert any({a, b} == {'object', 'obstacle_fixture'} for a, b, _ in scene.collisions())


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
    lift = plan['stages'][-1]
    scene.detach()
    scene.set_joints(dict(lift['waypoints'][-1], **lift['fingers']))
    points = scene.robot_collision_points()
    assert np.isfinite(points).all()
    assert .5 < np.max(points[:, 0]) < .9
    # Source geometry stays in base coordinates even when the entire robot
    # is translated and rotated relative to the tables and loose object.
    moved = ArmScene(mjcf, spec, [table], block, base=(5., -3., 0.006, math.pi/2), held=held)
    moved.set_joints(dict(lift['waypoints'][-1], **lift['fingers']))
    np.testing.assert_allclose(moved.robot_collision_points(), points, atol=3e-6)


@pytest.mark.parametrize('lying', [False, True], ids=['upright', 'on_side'])
def test_a_cylinder_is_gripped_across_its_diameter_and_supported(panda, lying):
    half_height = .025 if lying else .045
    orientation = [math.sqrt(.5), 0, math.sqrt(.5), 0] if lying else [1, 0, 0, 0]
    can = Box('pickup_can', [0.5, -0.25, 0.3+half_height], orientation, [0.05, 0.05, 0.09], 'cylinder')
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
    placed = Box(can.name, plan['place_centre'], plan['place_orientation'], can.size, can.shape)
    assert plan['place_centre'][2] == pytest.approx(support.top + placed.half_extent((0., 0., 1.)))
    assert plan['place_centre'][2] == pytest.approx(support.top + half_height, abs=.002)
    assert abs(placed.rotation()[:, 2] @ can.rotation()[:, 2]) > .99
    if lying:
        # Compare with the actual hand's frame offset: tool X is along the can
        # axis, so the jaws close across its diameter rather than its ends.
        assert abs(math.cos(plan['tool_yaw'] - spec.tool_yaw_offset)) > .99
        narrow = Box('narrow', support.position, support.quaternion, [.07, .12, .3])
        assert not narrow.supports(placed)


def test_ik_rotation_step_is_declared_per_arm():
    """V6 / R14.3 (2026-11-22): the exact rotation-vector step moved the TIAGo fetch onto a
    wrist branch it never settled into, while the F100 profile was developed on it."""
    base = dict(arm_joints=['arm'], finger_joints=[], tool_body='tool')
    assert ArmSpec.from_config(base).ik_rotation_step == 'cross'
    assert ArmSpec.from_config(dict(base, ik_rotation_step='exact')).ik_rotation_step == 'exact'
    with pytest.raises(ValueError):
        ArmSpec.from_config(dict(base, ik_rotation_step='rotvec'))
