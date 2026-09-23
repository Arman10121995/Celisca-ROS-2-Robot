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
