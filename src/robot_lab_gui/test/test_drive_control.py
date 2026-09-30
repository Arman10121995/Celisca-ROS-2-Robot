"""Teleoperation ramps and Linux joystick event decoding."""

import struct
import sys
from pathlib import Path
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_lab_gui.drive_control import LinuxJoystick, RampDrive, limits_from_drive


def test_hold_release_reverse_and_emergency_stop():
    drive = RampDrive()
    assert drive.step(1, 0, 0.025, 0.08) == (0.025, 0.0)
    for _ in range(100):
        drive.step(1, 1, 0.025, 0.08)
    assert drive.linear == 1.0
    assert drive.angular == 2.0
    assert drive.step(0, 0, 0.025, 0.08) == (0.975, 1.92)
    for _ in range(100):
        drive.step(-1, -1, 0.025, 0.08)
    assert drive.linear == -1.0
    assert drive.angular == -2.0
    assert drive.stop() == (0.0, 0.0)
    # Target speed is bounded even when a large acceleration is entered.
    assert drive.step(1, 1, 99, 99) == (1.0, 2.0)


def test_robot_profile_sets_speed_acceleration_and_car_yaw():
    assert limits_from_drive({"max_speed": 1.0, "max_accel": 2.0,
                              "max_angular_speed": 2.0,
                              "max_angular_accel": 4.0}) == (1.0, 2.0, 0.2, 0.4)
    car = limits_from_drive({"type": "ackermann", "max_speed": 1.0,
                             "max_accel": 1.5, "max_steer": 0.58,
                             "wheelbase": 0.32})
    assert 2.0 < car[1] < 2.1
    assert 0.30 < car[3] < 0.32
    drive = RampDrive(max_linear=1.0, max_angular=2.0)
    assert drive.step(1, 1, 0.2, 0.4) == (0.2, 0.4)
    assert drive.step(0, 0, 0.2, 0.4, 0.05, 0.1) == pytest.approx((0.15, 0.3))


def test_joystick_axis_deadzone_direction_and_disconnect():
    reader = LinuxJoystick()
    event = struct.Struct("IhBB")
    samples = [event.pack(0, 0, 0x82, 0), event.pack(0, 0, 0x82, 1),
               event.pack(0, 10000, 2, 0), event.pack(0, -20000, 2, 1)]
    with patch("glob.glob", return_value=["/dev/input/js0"]), \
            patch("os.open", return_value=9), \
            patch("os.read", side_effect=lambda *_: samples.pop(0) if samples else (_ for _ in ()).throw(BlockingIOError)), \
            patch("os.close") as close:
        linear, angular = reader.poll()
        assert 0.6 < linear < 0.62
        assert -0.31 < angular < -0.30
        assert reader.path == "/dev/input/js0"
        with patch("os.read", return_value=b""):
            assert reader.poll() == (0.0, 0.0)
        assert reader.path == ""
        close.assert_called_once_with(9)
    assert reader.axes == {0: 0.0, 1: 0.0}
    assert reader.centres == {}


def _poll_raw(raw_values):
    """One poll() over (value, axis[, is_initial]) joystick events."""
    reader = LinuxJoystick()
    samples = [LinuxJoystick._EVENT.pack(0, sample[0],
               0x82 if len(sample) > 2 and sample[2] else 2, sample[1])
               for sample in raw_values]
    with patch("glob.glob", return_value=["/dev/input/js0"]), \
            patch("os.open", return_value=9), \
            patch("os.read", side_effect=lambda *_: samples.pop(0) if samples
                  else (_ for _ in ()).throw(BlockingIOError)), \
            patch("os.close"):
        return reader.poll()


def test_untouched_joystick_never_commands_a_velocity():
    """Enabling the joystick must not move the robot until it is pushed.

    A driver that publishes its axes in the unsigned 0..65535 convention has
    those readings clamped into the signed 16-bit js_event field, so an
    untouched stick reports 32767.  The old decoder divided the raw value by
    32767, turning that rest position into a full-scale command: the robot
    drove off in a circle the moment the checkbox was ticked, with nobody
    touching the stick.  Both axis conventions must read as exactly zero.
    """
    for label, rest in [("signed axis", 0), ("endpoint-centred axis", 32767),
                        ("negative endpoint-centred axis", -32768)]:
        assert _poll_raw([(rest, 0), (rest, 1)]) == (0.0, 0.0), label
    # A small resting offset (worn pot, gyro drift) is inside the deadband.
    assert _poll_raw([(900, 0), (900, 1)]) == (0.0, 0.0)


def test_joystick_still_drives_when_pushed_on_either_axis_convention():
    """The rest-position fix must not flatten real stick input."""
    # Signed axis: fully forward, then fully left.
    linear, angular = _poll_raw([(0, 0, True), (0, 1, True), (-32767, 1)])
    assert linear == pytest.approx(1.0, abs=1e-3)
    assert angular == 0.0
    linear, angular = _poll_raw([(0, 0, True), (0, 1, True), (-32767, 0)])
    assert linear == 0.0
    assert angular == pytest.approx(1.0, abs=1e-3)

    # Clamped axis resting at 32767: pushing it down the only way it can
    # travel must still command motion, not read as "more of the same".
    linear, angular = _poll_raw([(32767, 0, True), (32767, 1, True),
                                 (32767 - 16000, 1)])
    assert linear == pytest.approx(16000 / 65535.0, abs=1e-3)
    assert angular == 0.0
    linear, angular = _poll_raw([(32767, 0, True), (32767, 1, True),
                                 (32767 - 16000, 0)])
    assert linear == 0.0
    assert angular == pytest.approx(16000 / 65535.0, abs=1e-3)

    # A signed axis past half travel must stay a positive command.  The
    # previous per-event heuristic flipped it negative at 16384.
    linear, angular = _poll_raw([(0, 0, True), (0, 1, True), (20000, 0)])
    assert linear == 0.0
    assert angular == pytest.approx(-20000 / 32767.0, abs=1e-3)


def test_joystick_missing_initial_event_arms_on_first_sample():
    assert _poll_raw([(20000, 0), (32767, 1)]) == (0.0, 0.0)
    linear, angular = _poll_raw([(20000, 0), (32767, 1), (0, 0), (16000, 1)])
    assert linear > 0.0
    assert angular > 0.0



def test_four_wheel_and_mecanum_bases_get_their_own_yaw_limits():
    """A 4WS base derives yaw from its steering geometry like a car does.

    A mecanum base pivots on the spot, so its yaw limit is its own declared
    value - deriving one from a turning radius would wrongly throttle it.
    """
    four_wheel = limits_from_drive({"type": "four_wheel_steer", "max_speed": 1.0,
                                    "max_accel": 1.5, "max_steer": 0.785,
                                    "wheelbase": 0.32})
    car = limits_from_drive({"type": "ackermann", "max_speed": 1.0,
                             "max_accel": 1.5, "max_steer": 0.785,
                             "wheelbase": 0.32})
    assert four_wheel[1] == pytest.approx(car[1])
    assert four_wheel[1] > 1.0, "a 4WS base turns far tighter than a car"

    mecanum = limits_from_drive({"type": "mecanum", "max_speed": 1.0,
                                 "max_accel": 1.5, "max_angular_speed": 2.5,
                                 "max_angular_accel": 5.0})
    assert mecanum[1] == pytest.approx(2.5)
    assert mecanum[3] == pytest.approx(0.5)

    # A mecanum base with no declared yaw limit falls back rather than failing.
    assert limits_from_drive({"type": "mecanum", "max_speed": 1.0})[1] == 1.0
