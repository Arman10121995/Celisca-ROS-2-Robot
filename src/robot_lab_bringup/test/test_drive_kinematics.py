"""Wheel targets of the differential and four-wheel Ackermann drive models."""

import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "robot_lab_utils"))

from robot_lab_utils.drive_kinematics import (  # noqa: E402
    AckermannDrive, DiffDrive, drive_from_config, parse_drive_config)

CAR = {
    "type": "ackermann",
    "left_steer_joint": "fl_steer", "right_steer_joint": "fr_steer",
    "steered_left_wheel_joint": "fl_wheel", "steered_right_wheel_joint": "fr_wheel",
    "left_wheel_joint": "rl_wheel", "right_wheel_joint": "rr_wheel",
    "wheel_radius": 0.05, "wheel_separation": 0.28, "wheelbase": 0.32,
    "max_steer": 0.5, "max_steer_rate": 2.0, "max_speed": 1.0, "max_accel": 1.5,
}


def _car(**overrides):
    config = dict(CAR)
    config.update(overrides)
    return AckermannDrive(config)


def _turning_centres(drive, targets):
    """Where each wheel's axis crosses the fixed axle's line (x = 0)."""
    half = drive.track / 2.0
    centres = []
    for joint, y in (("fl_steer", half), ("fr_steer", -half)):
        angle = targets.position[joint]
        # Axis through the wheel (steer_x, y) perpendicular to its heading.
        centres.append(y + drive.steer_x / math.tan(angle))
    return centres


def test_diff_drive_matches_the_bridges_formula():
    drive = drive_from_config({}, "l", "r", 0.033, 0.17)
    assert isinstance(drive, DiffDrive)
    targets = drive.targets(0.2, 1.0)
    assert targets.velocity["l"] == pytest.approx((0.2 - 0.085) / 0.033)
    assert targets.velocity["r"] == pytest.approx((0.2 + 0.085) / 0.033)
    assert targets.position == {}


def test_straight_line_rolls_every_wheel_at_the_same_rate():
    targets = _car().targets(0.5, 0.0)
    for joint in ("fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"):
        assert targets.velocity[joint] == pytest.approx(10.0)
    assert targets.position == {"fl_steer": 0.0, "fr_steer": 0.0}


@pytest.mark.parametrize("vx, wz", [(0.5, 0.4), (0.5, -0.4), (-0.3, 0.3), (0.4, -0.5)])
def test_ackermann_wheels_share_the_commanded_turning_centre(vx, wz):
    drive = _car()
    targets = drive.targets(vx, wz)
    radius = vx / wz
    for centre in _turning_centres(drive, targets):
        assert centre == pytest.approx(radius, rel=1e-9)
    assert targets.twist == pytest.approx((vx, wz))


def test_inner_front_wheel_steers_more_and_outer_wheels_roll_faster():
    targets = _car().targets(0.5, 0.5)  # left turn, R = 1 m
    assert targets.position["fl_steer"] > targets.position["fr_steer"] > 0
    assert targets.velocity["rr_wheel"] > targets.velocity["rl_wheel"]
    assert targets.velocity["fr_wheel"] > targets.velocity["fl_wheel"]
    # The steered wheels travel farther than the fixed ones on the same side.
    assert targets.velocity["fl_wheel"] > targets.velocity["rl_wheel"]


def test_rear_steering_turns_its_wheels_the_other_way():
    front = _car().targets(0.5, 0.5)
    rear = _car(steer_axle="rear").targets(0.5, 0.5)
    assert rear.position["fl_steer"] < 0 and rear.position["fr_steer"] < 0
    assert rear.twist == pytest.approx(front.twist)
    drive = _car(steer_axle="rear")
    for centre in _turning_centres(drive, rear):
        assert centre == pytest.approx(1.0)


def test_anti_ackermann_gives_the_outer_wheel_the_larger_angle():
    ackermann = _car().targets(0.5, 0.5).position
    anti = _car(steering_geometry="anti_ackermann").targets(0.5, 0.5).position
    assert anti["fl_steer"] == pytest.approx(ackermann["fr_steer"])
    assert anti["fr_steer"] == pytest.approx(ackermann["fl_steer"])
    assert anti["fr_steer"] > anti["fl_steer"]


def test_parallel_steering_sets_both_wheels_alike():
    targets = _car(steering_geometry="parallel").targets(0.5, 0.5)
    assert targets.position["fl_steer"] == pytest.approx(targets.position["fr_steer"])
    assert targets.position["fl_steer"] == pytest.approx(math.atan(0.32 * 1.0))


def test_curvature_is_clamped_to_full_lock():
    drive = _car()
    targets = drive.targets(0.2, 5.0)
    assert targets.twist[1] == pytest.approx(0.2 * drive.max_curvature)
    assert max(abs(a) for a in targets.position.values()) <= math.pi / 2
    assert drive.min_turning_radius == pytest.approx(0.32 / math.tan(0.5))


def test_no_turning_on_the_spot():
    drive = _car()
    drive.targets(0.5, 0.5)
    held = drive.targets(0.0, 1.0)
    assert all(rate == 0.0 for rate in held.velocity.values())
    assert held.position["fl_steer"] > 0  # the wheels stay where they were
    assert held.twist == (0.0, 0.0)


def test_rate_and_acceleration_limits_slew_toward_the_command():
    drive = _car()
    first = drive.targets(1.0, 1.0, dt=0.1)
    assert first.twist[0] == pytest.approx(0.15)  # 1.5 m/s^2 for 0.1 s
    centre = math.atan(0.32 * 1.0)
    for _ in range(50):
        last = drive.targets(1.0, 1.0, dt=0.1)
    assert last.twist == pytest.approx((1.0, 1.0))
    assert abs(math.atan(drive.steer_x * drive.curvature()) - centre) < 1e-9


def test_reverse_travel_keeps_the_geometry():
    drive = _car()
    targets = drive.targets(-0.4, 0.4)
    assert all(rate < 0 for rate in targets.velocity.values())
    assert targets.position["fl_steer"] < 0  # backing up while turning left
    assert targets.twist == pytest.approx((-0.4, 0.4))


def test_fixed_axle_odometry_inverts_the_targets():
    drive = _car()
    targets = drive.targets(0.4, 0.3)
    twist = drive.body_twist(targets.velocity["rl_wheel"], targets.velocity["rr_wheel"])
    assert twist == pytest.approx((0.4, 0.3))


def test_bad_configuration_is_refused():
    with pytest.raises(ValueError):
        _car(steer_axle="middle")
    with pytest.raises(ValueError):
        _car(steering_geometry="reverse")
    with pytest.raises(ValueError):
        drive_from_config({"type": "omni"})
    assert parse_drive_config("") == {}
    assert parse_drive_config('{"type": "ackermann"}') == {"type": "ackermann"}
