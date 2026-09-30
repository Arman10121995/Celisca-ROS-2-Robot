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


def test_diff_drive_limits_speed_and_acceleration_and_reset():
    drive = drive_from_config({
        "max_speed": 0.8, "max_accel": 1.5,
        "max_angular_speed": 2.0, "max_angular_accel": 4.0,
    }, "l", "r", 0.06, 0.3)
    first = drive.targets(100.0, 100.0, dt=0.1)
    assert first.twist == pytest.approx((0.15, 0.4))
    for _ in range(20):
        last = drive.targets(100.0, 100.0, dt=0.1)
    assert last.twist == pytest.approx((0.8, 2.0))
    assert drive.body_twist(last.velocity["l"], last.velocity["r"]) == \
        pytest.approx(last.twist)
    decel = drive.targets(0.0, 0.0, dt=0.1)
    assert decel.twist == pytest.approx((0.65, 1.6))
    drive.reset()  # watchdog/reset must zero accumulated commands
    assert drive.targets(0.0, 0.0, dt=0.1).twist == (0.0, 0.0)


def test_diff_drive_rejects_invalid_limit_configuration():
    with pytest.raises(ValueError):
        drive_from_config({"max_speed": -1}, "l", "r")
    with pytest.raises(ValueError):
        drive_from_config({"max_accel": float("nan")}, "l", "r")


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
        drive_from_config({"type": "omnidirectional"})   # not a real alias
    assert parse_drive_config("") == {}
    assert parse_drive_config('{"type": "ackermann"}') == {"type": "ackermann"}


# --------------------------------------------------------------------------
# Four-wheel steering and mecanum bases
# --------------------------------------------------------------------------

FOUR_WHEEL = {
    "wheel_radius": 0.05, "wheel_separation": 0.32, "wheelbase": 0.32,
    "max_steer": 0.785, "max_steer_rate": 2.5, "max_speed": 2.0,
    "max_accel": 1.5,
    "front_left_steer_joint": "fl_s", "front_right_steer_joint": "fr_s",
    "rear_left_steer_joint": "rl_s", "rear_right_steer_joint": "rr_s",
    "front_left_wheel_joint": "fl_w", "front_right_wheel_joint": "fr_w",
    "rear_left_wheel_joint": "rl_w", "rear_right_wheel_joint": "rr_w",
}


def _4ws(mode=None, **overrides):
    config = dict(FOUR_WHEEL, type="four_wheel_steer")
    if mode:
        config["steering_mode"] = mode
    config.update(overrides)
    return drive_from_config(config)


def _mecanum(**overrides):
    return drive_from_config(dict(FOUR_WHEEL, type="mecanum", **overrides))


@pytest.mark.parametrize("mode", ["ackermann", "in_phase", "crab", "pivot"])
def test_every_four_wheel_steering_mode_turns_on_the_spot(mode):
    """The point of four-wheel steering over a car: wz with no vx must work.

    A car cannot do this at all (test_no_turning_on_the_spot), so each mode
    has to realise a pure yaw some other way.
    """
    targets = _4ws(mode).targets(0.0, 1.0)
    assert targets.twist[0] == 0.0, "a zero-turn must not translate"
    assert any(abs(rate) > 1e-6 for rate in targets.velocity.values())
    if mode == "pivot":
        # Straight wheels, the two diagonals opposed.
        assert set(round(a, 9) for a in targets.position.values()) == {0.0}
    else:
        # Each wheel is aimed tangentially to its own circle about the centre.
        assert len(set(round(a, 6) for a in targets.position.values())) > 1


def test_four_wheel_steering_zero_turns_do_not_translate():
    for mode in ("ackermann", "in_phase", "crab", "pivot"):
        drive = _4ws(mode)
        targets = drive.targets(0.0, 1.0)
        rates = [targets.velocity.get(j, 0.0) for j in drive.wheel_joints]
        vx, wz = drive.body_twist(rates)
        assert vx == pytest.approx(0.0, abs=1e-9), mode
        assert abs(wz) > 1e-6, mode


def test_four_wheel_steering_axle_phases_differ_between_the_two_patterns():
    """ackermann steers the axles oppositely; in_phase steers them alike."""
    ackermann = _4ws("ackermann").targets(1.0, 0.5).position
    in_phase = _4ws("in_phase").targets(1.0, 0.5).position
    assert ackermann["fl_s"] == pytest.approx(-ackermann["rl_s"])
    assert in_phase["fl_s"] == pytest.approx(in_phase["rl_s"])
    assert in_phase["fl_s"] == pytest.approx(in_phase["fr_s"])


def test_four_wheel_steering_crab_holds_all_four_wheels_parallel():
    targets = _4ws("crab").targets(1.0, 0.5)
    angles = set(round(a, 9) for a in targets.position.values())
    assert len(angles) == 1, "crab steering points every wheel the same way"


def test_four_wheel_steering_rejects_an_unknown_pattern():
    with pytest.raises(ValueError):
        _4ws("diagonal")


def test_mecanum_forward_and_strafe_use_different_wheel_patterns():
    """The whole point of rollers: lateral motion independent of forward.

    Solving each wheel independently from the twist - the obvious wrong
    implementation - produces the *same* pattern for both, and the base can
    then only ever drive in a straight line.
    """
    drive = _mecanum()
    forward = drive.targets(1.0, 0.0).velocity
    strafe = drive.targets(0.0, 0.0, vy=1.0).velocity
    assert forward != strafe
    # Forward is symmetric; strafe is antisymmetric across the base.
    assert forward["fl_w"] == pytest.approx(forward["fr_w"])
    assert strafe["fl_w"] == pytest.approx(-strafe["fr_w"])
    # Strafe right is the mirror of strafe left.
    left = drive.targets(0.0, 0.0, vy=-1.0).velocity
    assert all(left[j] == pytest.approx(-strafe[j]) for j in strafe)


def test_mecanum_pivots_without_translating():
    drive = _mecanum()
    targets = drive.targets(0.0, 1.0)
    assert sum(targets.velocity.values()) == pytest.approx(0.0, abs=1e-9)
    assert any(abs(rate) > 1e-6 for rate in targets.velocity.values())


def test_mecanum_strafe_realises_the_commanded_lateral_speed():
    """body_twist cannot recover vy from spin rates, so it is passed back in."""
    drive = _mecanum()
    rates = [drive.targets(0.0, 0.0, vy=0.5).velocity[j]
             for j in drive.wheel_joints]
    vx, vy, wz = drive.body_twist(rates, vy=0.5)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(0.5)
    assert wz == pytest.approx(0.0, abs=1e-9)


def test_mecanum_and_four_wheel_steering_come_from_the_drive_block():
    assert _4ws().kind == "four_wheel_steer"
    assert _mecanum().kind == "mecanum"
    # The documented aliases resolve to the same models.
    assert drive_from_config(dict(FOUR_WHEEL, type="4ws")).kind == "four_wheel_steer"
    assert drive_from_config(dict(FOUR_WHEEL, type="roller")).kind == "mecanum"
    for drive in (_4ws(), _mecanum()):
        assert len(drive.wheel_joints) == 4
    # Only the four-wheel-steered base has steering joints; mecanum rollers
    # are fixed to the hub, so the distinction has to survive the factory.
    assert len(_4ws().steer_joints) == 4
    assert tuple(_mecanum().steer_joints) == ()
