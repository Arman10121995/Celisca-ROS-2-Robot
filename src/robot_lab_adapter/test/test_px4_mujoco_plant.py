"""MuJoCo quadrotor plant for PX4's simulator interface: physics and frame conventions (R5.11)."""
import math

import numpy as np
import pytest

pytest.importorskip('mujoco')
from robot_lab_adapter import px4_mujoco_plant as px


def airborne_plant():
    plant = px.Plant('', (0., 0., 5.))           # 5 m up: no ground contact during the test
    plant.armed = True
    return plant


def hover_command():
    omega = math.sqrt(px.MASS * 9.81 / 4 / px.MOTOR_CONSTANT)
    return (omega - px.OMEGA_ZERO) / px.OMEGA_SCALE


def run(plant, command, seconds):
    plant.command = np.asarray(command, dtype=float)
    for _ in range(int(seconds / px.DT)):
        plant.step()


def test_hover_command_balances_weight():
    plant = airborne_plant()
    u = hover_command()
    assert 0.55 < u < 0.8                                    # PX4's Iris hovers near 0.7 throttle
    run(plant, [u] * 4, .3)                                  # motors spin up (12.5 ms time constant)
    v0 = plant.velocity()[2]                                 # the brief fall while spinning up stays as velocity
    run(plant, [u] * 4, 1.)
    assert abs(plant.velocity()[2] - v0) < .05               # balanced: no vertical acceleration


def test_rotor_differentials_follow_px4_allocation_conventions():
    u = hover_command()
    plant = airborne_plant()
    run(plant, [u] * 4, .3)
    right = [u + .05 if ry > 0 else u - .05 for _, ry, _ in px.ROTORS_FRD]    # FRD +y is right
    run(plant, right, .15)
    roll_frd = px.flu_to_frd(plant.body_rates())[0]
    assert roll_frd < -.1                                    # right side up: negative (left) roll in FRD

    plant = airborne_plant()
    run(plant, [u] * 4, .3)
    front = [u + .05 if rx > 0 else u - .05 for rx, _, _ in px.ROTORS_FRD]
    run(plant, front, .15)
    assert px.flu_to_frd(plant.body_rates())[1] > .1         # nose up: positive pitch in FRD

    plant = airborne_plant()
    run(plant, [u] * 4, .3)
    ccw = [u + .08 if km > 0 else u - .08 for _, _, km in px.ROTORS_FRD]
    run(plant, ccw, .3)
    assert px.flu_to_frd(plant.body_rates())[2] > .05        # CCW rotors faster: body yaws clockwise (FRD +z)


def test_resting_sensors_report_gravity_and_home():
    plant = px.Plant('', (0., 0., 0.))
    for _ in range(200):
        plant.step()
    sensors = plant.sensors()
    sensors = plant.sensors()
    assert sensors['accel'][2] == pytest.approx(-9.81, abs=.2)       # FRD: specific force points up = -z
    assert np.linalg.norm(sensors['gyro']) < .02
    # Body x faces east (ENU world): north field -> FRD -y, east -> +x, down -> +z.
    north, east, down = px.MAG_NED
    assert sensors['mag'] == pytest.approx([east, -north, down], abs=.005)
    gps = plant.gps()
    assert gps['lat'] == pytest.approx(px.HOME['lat'] * 1e7, abs=20)
    assert sensors['pressure'] == pytest.approx(px.pressure_hpa(px.HOME['alt']), abs=.3)
