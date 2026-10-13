"""R7.5 sensor fusion filters (robot_lab_algorithms.fusion_filters)."""
import math

import numpy as np
import pytest

from robot_lab_algorithms.fusion_filters import (
    ATTITUDE_FILTERS, GRAVITY, AttitudeEskf, WheelImuEkf, WheelImuGnssUkf, q_from_euler,
    q_matrix, q_multiply, q_to_euler, wrap)

DT = 0.01


def static_tilt(roll, pitch, seconds=20.0, bias=(0.0, 0.0, 0.0), seed=0):
    """A body held at (roll, pitch): gyro reads only bias + noise."""
    rng = np.random.default_rng(seed)
    q = q_from_euler(roll, pitch, 0.0)
    gravity_body = q_matrix(q).T @ np.array([0.0, 0.0, GRAVITY])
    for _ in range(int(seconds / DT)):
        yield np.array(bias) + rng.normal(0, 0.002, 3), gravity_body + rng.normal(0, 0.03, 3)


@pytest.mark.parametrize('name', sorted(ATTITUDE_FILTERS))
def test_attitude_filters_converge_to_the_accelerometer_tilt(name):
    gains = {'complementary': {'tau': 0.5}, 'mahony': {'kp': 1.0}, 'madgwick': {'beta': 0.1},
             'eskf': {'linear_accel_std': 0.5}}
    flt = ATTITUDE_FILTERS[name](**gains[name])  # starts level
    for gyro, accel in static_tilt(0.3, -0.2):
        flt.update(gyro, accel, DT)
    roll, pitch, _ = q_to_euler(flt.q)
    assert roll == pytest.approx(0.3, abs=0.02)
    assert pitch == pytest.approx(-0.2, abs=0.02)


@pytest.mark.parametrize('name', sorted(ATTITUDE_FILTERS))
def test_attitude_filters_follow_a_pure_rotation(name):
    flt = ATTITUDE_FILTERS[name]()
    q = np.array([1.0, 0.0, 0.0, 0.0])
    omega = np.array([0.0, 0.0, 0.5])
    for _ in range(200):  # level turn: gravity stays vertical in the body
        q = q_multiply(q, np.array([math.cos(0.5 * 0.5 * DT), 0.0, 0.0, math.sin(0.5 * 0.5 * DT)]))
        flt.update(omega, np.array([0.0, 0.0, GRAVITY]), DT)
    assert q_to_euler(flt.q)[2] == pytest.approx(q_to_euler(q)[2], abs=1e-3)


def test_eskf_learns_the_horizontal_gyro_bias():
    flt = AttitudeEskf(linear_accel_std=0.5)
    bias = (0.02, -0.015, 0.0)
    for gyro, accel in static_tilt(0.1, 0.05, seconds=60.0, bias=bias):
        flt.update(gyro, accel, DT)
    assert flt.bias[:2] == pytest.approx(bias[:2], abs=0.003)


def drive(seconds=120.0, dt=0.02, slip=False, seed=0):
    rng = np.random.default_rng(seed)
    x = y = th = 0.0
    for k in range(int(seconds / dt)):
        t = k * dt
        v = 0.5 if (t % 30.0) < 25.0 else 0.0
        w = 0.2 * math.sin(0.3 * t) if v else 0.0
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th = wrap(th + w * dt)
        wheel_w = w * 0.97 + (0.1 if slip and 40 < t < 50 and v else 0.0)
        yield t, (x, y, th), v * 1.02, wheel_w, w + 0.01 + rng.normal(0, 0.003), (
            x + rng.normal(0, 0.5), y + rng.normal(0, 0.5)) if k % 50 == 0 else None


@pytest.mark.parametrize('cls', [WheelImuEkf, WheelImuGnssUkf])
def test_pose_filters_learn_the_gyro_bias_at_standstill(cls):
    flt = cls()
    for t, truth, wv, ww, gz, gnss in drive(slip=True):
        flt.predict(gz, 0.02)
        flt.update_wheels(wv, ww)
    assert flt.state[4] == pytest.approx(0.01, abs=0.002)
    assert math.hypot(flt.state[0] - truth[0], flt.state[1] - truth[1]) < 1.5


@pytest.mark.parametrize('cls', [WheelImuEkf, WheelImuGnssUkf])
def test_gnss_bounds_the_position_error(cls):
    flt = cls()
    errors = []
    for t, truth, wv, ww, gz, gnss in drive(seconds=200.0, slip=True, seed=3):
        flt.predict(gz, 0.02)
        flt.update_wheels(wv, ww)
        if gnss is not None:
            flt.update_gnss(*gnss)
        if t > 20:
            errors.append(math.hypot(flt.state[0] - truth[0], flt.state[1] - truth[1]))
    assert max(errors) < 1.0
    assert np.sqrt(np.mean(np.square(errors))) < 0.4
