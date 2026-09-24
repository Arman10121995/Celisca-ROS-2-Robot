"""Teleoperation ramps and Linux joystick event decoding."""

import struct
import sys
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_lab_gui.drive_control import LinuxJoystick, RampDrive


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
    # Typed values outside Spinbox bounds must not bypass the ramp limits.
    assert drive.step(1, 1, 99, 99) == (0.1, 0.2)


def test_joystick_axis_deadzone_direction_and_disconnect():
    reader = LinuxJoystick()
    event = struct.Struct("IhBB")
    samples = [event.pack(0, 10000, 2, 0), event.pack(0, -20000, 2, 1)]
    with patch("glob.glob", return_value=["/dev/input/js0"]), \
            patch("os.open", return_value=9), \
            patch("os.read", side_effect=lambda *_: samples.pop() if samples else (_ for _ in ()).throw(BlockingIOError)), \
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
