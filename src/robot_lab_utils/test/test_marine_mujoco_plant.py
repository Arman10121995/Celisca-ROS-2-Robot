"""MuJoCo marine plant: ported Fortress buoyancy, damping and thrusters (R10.4)."""
import math
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from robot_lab_utils.marine_model import hydro_coefficients
from robot_lab_utils.marine_mujoco_plant import MarinePlant

XACRO = Path(__file__).resolve().parents[2] / 'robot_lab_robots/lab_rov/urdf/lab_rov.urdf.xacro'


@pytest.fixture(scope='module')
def urdf():
    from robot_lab_utils.robot_description import load_description
    return load_description(str(XACRO), 'model_name:=lab_rov')


def test_coefficients_come_from_the_declared_plugin(urdf):
    hydro = hydro_coefficients(urdf)
    assert hydro['linear'][:3].tolist() == [-8., -12., -15.]
    assert hydro['quadratic'][:3].tolist() == [-20., -30., -35.]
    assert hydro['added'][:3].tolist() == [3., 5., 7.]


def test_surge_reaches_the_damping_terminal_speed(urdf):
    plant = MarinePlant(urdf, [0., 0., -2., 0.], 0., -6.)
    plant.current[:] = 0.
    trim = plant.model_source.gravity_trim
    forces, _ = plant.model_source.allocate([10., 0., trim, 0., 0., 0.])
    for index, value in enumerate(forces):
        plant.command(index, value)
    positions = []
    for _ in range(25):
        plant.step(1.)
        positions.append(plant.pose()[0].copy())
    speed = (positions[-1][0] - positions[-6][0]) / 5.
    assert speed == pytest.approx((-8 + math.sqrt(64 + 800)) / 40, rel=.03)     # 8u + 20u^2 = 10 N


def test_current_drags_a_neutral_body_downstream(urdf):
    plant = MarinePlant(urdf, [0., 0., -2., 0.], 0., -6.)
    plant.current[:] = [0., .1, 0.]
    for _ in range(20):
        plant.step(1.)
    assert plant.pose()[0][1] > 1.                  # carried with the water


def test_buoyancy_slows_a_slightly_heavy_body_released_above_the_surface(urdf):
    plant = MarinePlant(urdf, [0., 0., .5, 0.], 0., -6.)
    plant.current[:] = 0.
    for _ in range(10):
        plant.step(1.)
    z = plant.pose()[0][2]
    assert -3. < z < 0.                             # falls in, then 1 N heavy: sinks at drag speed, not free fall


def test_thrust_commands_are_clipped_to_source_bounds(urdf):
    plant = MarinePlant(urdf, [0., 0., -2., 0.], 0., -6.)
    plant.command(0, 1e6)
    plant.command(1, float('nan'))
    assert plant.forces[0] == plant.model_source.bounds[0][1] and plant.forces[1] == 0.


def test_spawn_yaw_is_in_radians_and_pybullet_integrates_the_same_wrench(urdf):
    import math
    from robot_lab_utils.marine_pybullet_plant import MarinePyBulletPlant
    pytest.importorskip('pybullet')
    def yaw(q):
        x, y, z, w = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    results = []
    for cls in (MarinePlant, MarinePyBulletPlant):
        plant = cls(urdf, [0., 0., -2., .3], 0., -6.)
        plant.current[:] = 0.
        assert abs(yaw(plant.pose()[1]) - .3) < 1e-6
        forces, _ = plant.model_source.allocate([10., 0., plant.model_source.gravity_trim, 0., 0., 2.])
        for index, value in enumerate(forces):
            plant.command(index, value)
        for _ in range(10):
            plant.step(1.)
        position, quaternion = plant.pose()
        results.append((position, yaw(quaternion)))
    (mj_position, mj_yaw), (pb_position, pb_yaw) = results
    assert np.linalg.norm(mj_position - pb_position) < .15 and abs(mj_yaw - pb_yaw) < .05
