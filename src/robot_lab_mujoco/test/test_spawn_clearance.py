"""Spawn clearance from the real lowest collision surface (MuJoCo bridge).

The Bumperbot base mesh's box in its principal-axis frame reached 6.8 cm
below the mesh, so the bridge spawned the robot that high; the drop sank
its wheels through the 1 mm flex floor of small_house (2026-10-19).
"""
import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
pytest.importorskip('rclpy')
from robot_lab_mujoco.mujoco_spawner import _geom_bottom_z  # noqa: E402

# An elongated, tilted wedge: its principal axes are not the body axes.
_WEDGE = ' '.join('%g %g %g' % v for v in [(0, 0, 0), (0.4, 0, 0), (0, 0.1, 0), (0.4, 0.1, 0.25),
                                           (0, 0, 0.05), (0.4, 0, 0.05), (0, 0.1, 0.05)])


def _model():
    return mujoco.MjModel.from_xml_string(f'''
<mujoco><compiler angle="radian"/><asset><mesh name="wedge" vertex="{_WEDGE}"/></asset>
<worldbody><body name="robot" pos="0 0 0.3"><freejoint/>
  <geom name="wedge" type="mesh" mesh="wedge"/>
  <geom name="ball" type="sphere" size="0.02" pos="0.1 0 -0.05"/>
  <geom name="rod" type="capsule" size="0.01 0.05" pos="0.2 0 0" euler="0 0.6 0"/>
</body></worldbody></mujoco>''')


def test_mesh_bottom_is_its_lowest_vertex_not_its_principal_box():
    model = _model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'wedge')
    mesh = model.geom_dataid[geom]
    start = model.mesh_vertadr[mesh]
    vertices = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
    world = vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
    assert _geom_bottom_z(model, data, geom) == pytest.approx(world[:, 2].min(), abs=1e-9)
    assert _geom_bottom_z(model, data, geom) == pytest.approx(0.3, abs=1e-6)
    bounds = model.geom_aabb[geom]
    z_row = data.geom_xmat[geom].reshape(3, 3)[2]
    box = data.geom_xpos[geom, 2] + z_row @ bounds[:3] - abs(z_row) @ bounds[3:]
    assert box < world[:, 2].min() - 0.005, 'the principal-frame box is looser than the mesh'


def test_sphere_and_capsule_bottoms_are_exact():
    model = _model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'ball')
    assert _geom_bottom_z(model, data, ball) == pytest.approx(0.3 - 0.05 - 0.02)
    rod = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'rod')
    assert _geom_bottom_z(model, data, rod) == pytest.approx(0.3 - 0.05 * np.cos(0.6) - 0.01)
