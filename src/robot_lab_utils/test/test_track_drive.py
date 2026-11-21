"""Shared track drive law (R10.7 ported tracks)."""
import math
from pathlib import Path

import numpy as np
import pytest

from robot_lab_utils.track_drive import (TrackDrive, contact_force, parse_config, track_collisions,
                                         track_speeds)

ROBOTS = Path(__file__).resolve().parents[2] / 'robot_lab_robots'
TRACKS = dict(left=['left_track', 'front_left_flipper', 'rear_left_flipper'],
              right=['right_track', 'front_right_flipper', 'rear_right_flipper'])


def test_config_validation():
    assert parse_config(TRACKS)['separation'] == 0.4
    with pytest.raises(ValueError):
        parse_config(dict(TRACKS, gain=-1))
    with pytest.raises(ValueError):
        parse_config(dict(TRACKS, colour=1))
    with pytest.raises(ValueError):
        parse_config(dict(left=['a'], right=['a']))


def test_source_collisions_by_side():
    description = (ROBOTS / 'lab_tracked/robot.urdf').read_text()
    sides = track_collisions(description, parse_config(TRACKS))
    assert sides['left_track_collision_2'] == 'left'
    assert sides['rear_right_flipper_collision_large_wheel_collision'] == 'right'
    assert 'base_link_collision' not in sides
    assert len(sides) == 26


def test_speeds_follow_the_twist():
    left, right = track_speeds(0.2, 0.5, 0.4)
    assert left == pytest.approx(0.1) and right == pytest.approx(0.3)


def test_contact_force_is_a_bounded_servo():
    up = [0, 0, 1]
    # At rest under a 0.2 m/s belt: forward push, nothing sideways.
    force = contact_force(0.2, [0, 1, 0], up, [0, 0, 0], 100.0, 10.0, 1.0, 0.7, 50.0)
    assert force == pytest.approx([100.0, 0, 0])      # gain*m*dv = 100 N, at the bound mu*N
    force = contact_force(0.2, [0, 1, 0], up, [0.19, 0, 0], 100.0, 10.0, 1.0, 0.7, 50.0)
    assert force[0] == pytest.approx(5.0)
    # A sideways skid is resisted at the lateral Coulomb bound.
    force = contact_force(0.0, [0, 1, 0], up, [0, 1.0, 0], 100.0, 10.0, 1.0, 0.7, 50.0)
    assert force == pytest.approx([0, -70.0, 0])
    # No load, no force.
    assert not np.any(contact_force(0.2, [0, 1, 0], up, [0, 0, 0], 0.0, 10.0, 1.0, 0.7, 50.0))


def test_belt_follows_the_contact_plane():
    # A riser met by the front sprocket (normal pointing back at the
    # robot): the belt there runs straight up.
    force = contact_force(0.2, [0, 1, 0], [-1, 0, 0], [0, 0, 0], 50.0, 10.0, 1.0, 0.7, 50.0)
    assert force == pytest.approx([0, 0, 50.0])
    # A wall against the belt's side: no belt direction, no force.
    assert not np.any(contact_force(0.2, [0, 1, 0], [0, -1, 0], [0, 0, 0], 50.0, 10.0, 1.0, 0.7, 50.0))


def test_watchdog_limits_and_odometry():
    drive = TrackDrive(TRACKS, 25.0)
    drive.command(1.0, 0.0, 0.0)            # capped at 0.3 m/s
    assert drive.requested == (0.3, 0.0)
    for i in range(1000):                    # 1 s at 1 ms: 0.4 m/s^2 slew
        drive.advance(0.0, 0.001)
    assert drive.velocity[0] == pytest.approx(0.3)
    # Then the command goes stale after 0.4 s and the vehicle slews to rest.
    for i in range(1000):
        drive.advance(0.401 + i * 0.001, 0.001)
    assert drive.velocity[0] == pytest.approx(0.0)
    assert drive.pose[0] == pytest.approx(0.1125 + 0.3 * 0.25 + 0.1125, abs=2e-3)
    drive.command(float('nan'), 0.2, 3.0)
    assert drive.requested == (0.0, 0.0)


def test_forces_share_the_mass_by_load():
    drive = TrackDrive(TRACKS, 20.0)
    drive.command(0.2, 0.0, 0.0)
    drive.velocity = [0.2, 0.0]
    forces = drive.forces(['left', 'right'], [[0, 0, 1], [0, 0, 1]], [30.0, 10.0],
                          [[0.1, 0, 0], [0.1, 0, 0]], [0, 1, 0], 0.001)
    # 50/s * 15 kg * 0.1 m/s = 75 N and 50/s * 5 kg * 0.1 m/s = 25 N, each
    # bounded by mu_along * N.
    assert forces[0][0] == pytest.approx(30.0)
    assert forces[1][0] == pytest.approx(10.0)


def test_traction_integral_removes_steady_slip():
    drive = TrackDrive(dict(TRACKS, mu_along=10.0), 20.0)
    drive.velocity = [0.2, 0.0]
    args = (['left', 'right'], [[0, 0, 1], [0, 0, 1]], [100.0, 100.0])
    # A constant 0.05 m/s slip under a resisting load: the traction share grows.
    first = drive.forces(*args, [[0.15, 0, 0], [0.15, 0, 0]], [0, 1, 0], 0.01)
    for _ in range(100):
        later = drive.forces(*args, [[0.15, 0, 0], [0.15, 0, 0]], [0, 1, 0], 0.01)
    assert drive.bias['left'] == pytest.approx(40.0 * 0.05 * 1.01, rel=1e-6)
    assert later[0][0] > first[0][0] + 150
    # Unloaded tracks drop their traction share.
    drive.forces(['left'], [[0, 0, 1]], [0.0], [[0, 0, 0]], [0, 1, 0], 0.01)
    assert drive.bias == dict(left=0.0, right=0.0)


def test_turn_targets_rotation_about_the_origin():
    drive = TrackDrive(TRACKS, 20.0)
    drive.velocity = [0.0, 0.5]
    # A rear contact at rest during a left turn: its across-belt target is
    # the rotation about the origin, w x r = 0.5 * z x (-0.3, 0.2, 0) = (-0.1, -0.15, 0).
    forces = drive.forces(['left'], [[0, 0, 1]], [1000.0], [[0, 0, 0]], [0, 1, 0], 0.001,
                          [[-0.3, 0.2, 0]], [0, 0, 0], [0, 0, 1])
    assert forces[0][1] == pytest.approx(-0.15 * 50.0 * 20.0)
    # Along the belt the target stays the track speed, -0.5 * 0.4 / 2.
    assert forces[0][0] == pytest.approx(-0.1 * 50.0 * 20.0)


def test_off_line_contacts_get_their_own_turn_speed():
    drive = TrackDrive(TRACKS, 20.0)
    drive.velocity = [0.0, 0.5]
    # A flipper contact 0.272 m out moves at -0.5 * 0.272 in the turn, not
    # at the belt line's -0.5 * 0.2.
    forces = drive.forces(['left'], [[0, 0, 1]], [1000.0], [[0, 0, 0]], [0, 1, 0], 0.001,
                          [[0.4, 0.272, 0]], [0, 0, 0], [0, 0, 1])
    assert forces[0][0] == pytest.approx(-0.136 * 50.0 * 20.0)
