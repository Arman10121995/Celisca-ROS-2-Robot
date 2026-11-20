"""MuJoCo Tethys plant: ported Fortress buoyancy, hydrodynamics, thruster and fin LiftDrag (R10.4)."""
import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('mujoco')

from robot_lab_utils.marine_tethys_plant import (TethysPlant, coriolis_added_mass, lift_drag,  # noqa: E402
                                                source_physics)

SDF = (Path(__file__).resolve().parents[2] / 'robot_lab_robots/tethys_auv/model.sdf').read_text()


def test_source_mass_and_volume_make_the_vehicle_neutral():
    s = source_physics(SDF)
    volume = sum(np.prod(size) for _, size in s['boxes'])
    assert s['mass'] == pytest.approx(152.066, abs=1e-3)
    assert 1025. * volume == pytest.approx(s['mass'], rel=1e-4)
    assert s['added'][1] == pytest.approx(129.482857475) and s['quadratic'][0] == pytest.approx(-6.383905)
    plant = TethysPlant(SDF, [0., 0., -2.5, 0.], 0., -6.)
    plant.step(5.)
    assert np.linalg.norm(plant.pose()[0] - [0., 0., -2.5]) < .01


def test_tail_fins_push_the_tail_downstream_and_the_munk_moment_is_destabilising():
    s = source_physics(SDF)
    rudder = s['fins']['vertical_fins']
    force = lift_drag(rudder, np.array([-1., .1, 0.]), rudder['upward'], rudder['forward'])
    assert force[1] == pytest.approx(-5.2, abs=.1)                # lift: 0.5 rho v^2 A cla alpha
    state = np.array([-1., .1, 0., 0., 0., 0.])
    C = coriolis_added_mass(s['added'], state)
    assert abs(state @ C @ state) < 1e-9                          # C_A is skew-symmetric in nu
    assert (-C @ state)[5] == pytest.approx((s['added'][0] - s['added'][1]) * -1. * .1)


def test_thrust_turns_the_propeller_and_drives_the_vehicle_forward():
    plant = TethysPlant(SDF, [0., 0., -2.5, 0.], 0., -6.)
    plant.command_thrust(-10.)                                    # source front is -X
    for _ in range(4):
        plant.step(1.)
    assert plant.pose()[0][0] < -.4                              # 10 N on 157 kg (with added mass): 0.5 m in 4 s
    t = plant.source['thruster']
    assert plant._propeller_rate() == pytest.approx(-math.sqrt(10. / (t['density'] * t['coefficient'] * t['diameter'] ** 4)))
    plant.command_fin('vertical_fins', .2)
    for _ in range(20):
        plant.step(.05)
    assert 0. < plant.fin_angle['vertical_fins'] <= .261799


def test_fortress_mode_flips_only_the_added_mass_sign():
    # Fortress 6.18 applies +Ma nudot (positive SDF Ma) and Fossen's -C_A nu
    # (identified 2026-11-20); the default stays Fossen-correct -M_A nudot.
    from robot_lab_utils.marine_tethys_plant import ADDED_MASS_SIGNS, CORIOLIS_SIGNS
    assert ADDED_MASS_SIGNS['fortress'] == 1. and CORIOLIS_SIGNS['fortress'] == CORIOLIS_SIGNS['fossen']
    assert all(ADDED_MASS_SIGNS[mode] == -1. for mode in ('off', 'fossen', 'flipped'))
