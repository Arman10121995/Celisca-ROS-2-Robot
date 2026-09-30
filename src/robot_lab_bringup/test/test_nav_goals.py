"""Hermetic contracts for the goal restamping rule the nav2 relay applies.

The rule exists because nav2's RViz panel stamps goals with the system clock
(``rclcpp::Clock().now()`` in ``Nav2Panel::onNewGoal``, Humble), which a
simulated run's planner cannot transform against /clock TF: the goal aborts
with "extrapolation into the future" before a path is ever computed.  The
relay discards the client stamp and applies its own.  The helpers are ROS-free
so this file stays in the fast tier.
"""

import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parents[1] / "robot_lab_utils"))

from robot_lab_utils.nav_goals import (  # noqa: E402
    FALLBACK_FRAME, pose_is_finite, restamped)


class _Stamp:
    """Stand-in for builtin_interfaces/msg/Time."""

    def __init__(self, seconds):
        self.sec = int(seconds)
        self.nanosec = int(round((seconds - int(seconds)) * 1e9))


class _Header:
    def __init__(self, frame_id, seconds):
        self.frame_id = frame_id
        self.stamp = _Stamp(seconds)


class _Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Pose:
    def __init__(self):
        self.position = _Point()
        self.orientation = _Quaternion()


def test_the_relay_discards_a_wall_clock_goal_stamp():
    """The wall-clock stamp the RViz panel produces is replaced, not trusted."""
    header = _Header("map", seconds=1790755942.825638)  # the failing log's goal
    now = _Stamp(15.36)                                 # the run's /clock time
    returned = restamped(header, now)
    assert returned is header
    assert header.stamp is now
    assert header.frame_id == "map"


def test_a_goal_without_a_frame_is_read_in_the_map():
    header = _Header("", seconds=0.0)
    restamped(header, _Stamp(3.0))
    assert header.frame_id == FALLBACK_FRAME


def test_an_explicit_goal_frame_is_kept():
    header = _Header("odom", seconds=0.0)
    restamped(header, _Stamp(3.0))
    assert header.frame_id == "odom"


def test_non_finite_and_absurd_poses_are_refused():
    pose = _Pose()
    assert pose_is_finite(pose)
    pose.position.x = float("nan")
    assert not pose_is_finite(pose)
    pose.position.x = 2.0e6
    assert not pose_is_finite(pose)
    pose.position.x = 1.0
    pose.orientation.w = float("inf")
    assert not pose_is_finite(pose)
    pose.orientation.w = 1.0
    assert pose_is_finite(pose)


if __name__ == "__main__":
    pytest.main([__file__, "-q"])
