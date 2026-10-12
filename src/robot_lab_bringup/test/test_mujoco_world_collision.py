"""Static concave buildings must leave rooms traversable and walls solid."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
trimesh = pytest.importorskip('trimesh')
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_lab_mujoco/python'))
from robot_lab_mujoco.world_collision import add_static_mesh_collisions


def test_open_room_keeps_floor_and_wall_contacts(tmp_path):
    walls = trimesh.util.concatenate([
        trimesh.creation.box([.1, 4, 1], transform=trimesh.transformations.translation_matrix([x, 0, .5]))
        for x in [-2, 2]])
    mesh = tmp_path / 'walls.stl'; walls.export(mesh)
    root = ET.fromstring(f'''<mujoco><asset><mesh name="walls" file="{mesh}"/></asset>
      <worldbody><geom type="plane" size="5 5 .1"/><geom type="mesh" mesh="walls"/>
      <body pos="0 0 1"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
      </worldbody></mujoco>''')
    assert add_static_mesh_collisions(root) == 1
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    for _ in range(1500): mujoco.mj_step(model, data)
    # A convex hull across both walls would put the robot atop a solid box.
    assert data.qpos[2] == pytest.approx(.1, abs=.002)
    data.qpos[0] = 1.7; data.qpos[2] = .5; data.qvel[0] = 2
    positions = []
    for _ in range(300):
        mujoco.mj_step(model, data); positions.append(data.qpos[0])
    assert 1.8 < max(positions) < 1.87
    assert np.isfinite(data.qpos).all()


def test_open_planar_static_mesh_compiles_and_supports_a_body(tmp_path):
    floor = trimesh.Trimesh(vertices=[[-2, -2, 0], [2, -2, 0],
                                     [2, 2, 0], [-2, 2, 0]],
                           faces=[[0, 1, 2], [0, 2, 3]], process=False)
    path = tmp_path/'open-floor.stl'; floor.export(path)
    root = ET.fromstring(f'''<mujoco><asset><mesh name="floor" file="{path}"/></asset>
      <worldbody><geom type="mesh" mesh="floor"/>
      <body pos="0 0 1"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
      </worldbody></mujoco>''')
    assert add_static_mesh_collisions(root) == 1
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    for _ in range(1500): mujoco.mj_step(model, data)
    assert data.qpos[2] == pytest.approx(.101, abs=.003)
    assert np.isfinite(data.qpos).all()


def _staged_room(tmp_path):
    from robot_lab_mujoco.world_collision import add_ray_proxies
    walls = trimesh.util.concatenate([
        trimesh.creation.box([.1, 4, 1], transform=trimesh.transformations.translation_matrix([x, 0, .5]))
        for x in [-2, 2]] + [trimesh.creation.icosphere(subdivisions=4, radius=.3).apply_translation([0, 1.2, .4])])
    mesh = tmp_path / 'walls.stl'; walls.export(mesh)
    root = ET.fromstring(f'''<mujoco><asset><mesh name="walls" file="{mesh}"/></asset>
      <worldbody><geom type="plane" size="5 5 .1"/><geom type="mesh" mesh="walls"/>
      <body name="robot" pos="0 0 .2"><freejoint/><geom type="box" size=".15 .15 .1" mass="4"/>
        <body name="wheel" pos="0 .17 -.05"><joint type="hinge" axis="0 1 0"/>
          <geom type="cylinder" size=".05 .02" euler="90 0 0" mass=".2"/></body></body>
      </worldbody></mujoco>''')
    add_static_mesh_collisions(root)
    proxies = add_ray_proxies(root, size=60)
    return root, proxies


def test_ray_proxies_return_the_original_mesh_distances(tmp_path):
    root, proxies = _staged_room(tmp_path)
    assert proxies > 3
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    visual = [g for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
              and model.geom_group[g] == 2]
    assert len(visual) == 1
    origin = np.array([0., 0., .45])
    geomid = np.zeros(1, dtype=np.int32)
    only_visual = np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8)
    only_proxies = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)
    for angle in np.linspace(-np.pi, np.pi, 72, endpoint=False):
        direction = np.array([np.cos(angle), np.sin(angle), -.05])
        expected = mujoco.mj_ray(model, data, origin, direction, only_visual, 1, -1, geomid)
        actual = mujoco.mj_ray(model, data, origin, direction, only_proxies, 1, -1, geomid)
        assert actual == pytest.approx(expected, abs=1e-6)


def test_vectorized_scan_matches_per_ray_casting_and_skips_the_robot(tmp_path):
    from robot_lab_mujoco.mujoco_spawner import _ray_skipping_robot, _scan_ranges
    root, _ = _staged_room(tmp_path)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    robot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot')
    origin = np.array([0., 0., .17])   # inside the robot box, at wheel height
    angles = np.linspace(-np.pi, np.pi, 180, endpoint=False)
    ranges = _scan_ranges(model, data, origin, angles, robot, .05, 12.)
    geomid = np.zeros(1, dtype=np.int32)
    for angle, value in zip(angles, ranges):
        direction = np.array([np.cos(angle), np.sin(angle), 0.])
        expected = _ray_skipping_robot(model, data, origin, direction, robot, 12., geomid)
        expected = max(.05, expected) if 0 < expected < 12 else np.inf
        assert value == pytest.approx(expected, abs=1e-9)
    # The walls are 1.95 m away along x; the robot body and wheel are not seen.
    assert ranges[np.argmin(np.abs(angles))] == pytest.approx(1.95, abs=1e-6)


def test_midphase_is_disabled_only_for_large_static_flex_worlds():
    from robot_lab_mujoco.world_collision import MIDPHASE_FLEX_ELEMENTS, configure_world_midphase
    small = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="plane" size="1 1 .1"/></worldbody></mujoco>')
    assert not configure_world_midphase(small)
    assert configure_world_midphase(small, 'off')
    assert small.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_MIDPHASE)
    assert MIDPHASE_FLEX_ELEMENTS > 0
