"""Wheel odometry from joint states (robot_lab_algorithms.wheel_odometry)."""
import math

import pytest

from robot_lab_algorithms.wheel_odometry import WheelOdometry
from robot_lab_utils.drive_kinematics import drive_from_config


def run(odometry, rates, positions=None, seconds=2.0, dt=0.02):
    for k in range(int(seconds / dt) + 1):
        odometry.update(k * dt, rates, positions or {})
    return odometry


def test_diff_drive_straight_and_turn():
    drive = drive_from_config({}, 'wheel_left_joint', 'wheel_right_joint', 0.033, 0.17)
    straight = run(WheelOdometry(drive), {'wheel_left_joint': 10.0, 'wheel_right_joint': 10.0})
    assert straight.x == pytest.approx(0.33 * 2.0, rel=1e-6) and straight.y == pytest.approx(0.0)
    spin = run(WheelOdometry(drive), {'wheel_left_joint': -5.0, 'wheel_right_joint': 5.0})
    assert spin.twist[2] == pytest.approx(0.033 * 10.0 / 0.17)
    assert math.hypot(spin.x, spin.y) == pytest.approx(0.0, abs=1e-9)


def test_mecanum_strafes():
    config = {'type': 'mecanum', 'front_left_wheel_joint': 'fl', 'front_right_wheel_joint': 'fr',
              'rear_left_wheel_joint': 'rl', 'rear_right_wheel_joint': 'rr', 'wheel_radius': 0.05,
              'wheel_separation': 0.32, 'wheelbase': 0.32}
    odometry = run(WheelOdometry(drive_from_config(config)), {'fl': -4.0, 'fr': 4.0, 'rl': 4.0, 'rr': -4.0})
    vx, vy, wz = odometry.twist
    assert vx == pytest.approx(0.0, abs=1e-9) and wz == pytest.approx(0.0, abs=1e-9) and vy == pytest.approx(0.2)
    assert odometry.y == pytest.approx(0.4, rel=1e-6)
