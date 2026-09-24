"""Modes a robot profile can run, per simulator (robot_lab_utils.mode_capability).

The rules follow what was measured live on 2026-09-23: Bumperbot and Labbot
reach Nav2 goals in all four backends; every legged and humanoid profile
localizes where it stands in PyBullet and MuJoCo (the bridges cast its scan
and hold its joints), not in Isaac, whose hold does not keep them upright;
none of them walks on /cmd_vel, so none can map by moving or navigate.
"""
import os
import sys

import pytest
import yaml

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_SRC, "robot_lab_utils"))

from robot_lab_utils.mode_capability import missing_features  # noqa: E402

PROFILES = yaml.safe_load(open(os.path.join(
    _SRC, "robot_lab_robots", "config", "robots.yaml")))["robots"]
SIMULATORS = ("gazebo", "pybullet", "mujoco", "isaac")


def runnable(robot, simulator):
    profile = PROFILES[robot]
    return {mode for mode in profile.get("supported_modes", [])
            if not missing_features(profile, mode, simulator)}


@pytest.mark.parametrize("robot", ["bumperbot", "labbot"])
@pytest.mark.parametrize("simulator", SIMULATORS)
def test_wheeled_bases_run_every_mode_everywhere(robot, simulator):
    assert runnable(robot, simulator) == {"display", "loc", "slam", "3d_slam", "nav"}


@pytest.mark.parametrize("robot", [r for r in PROFILES
                                   if r.startswith(("unitree_", "berkeley_humanoid_lite"))])
def test_legged_and_humanoid_robots_localize_where_the_bridges_hold_them(robot):
    assert runnable(robot, "pybullet") == {"display", "loc"}
    assert runnable(robot, "mujoco") == {"display", "loc"}
    assert runnable(robot, "isaac") == {"display"}
    assert runnable(robot, "gazebo") == {"display"}


def test_the_quadrotor_does_not_claim_flight():
    for simulator in SIMULATORS:
        assert not {"slam", "3d_slam", "nav"} & runnable("quadrotor_sitl", simulator)


def test_sim_modes_requirements_are_added():
    profile = {"features": ["lidar_2d", "stands", "velocity_base"]}
    assert missing_features(profile, "3d_slam", "gazebo") == ["rgbd_camera"]
    assert missing_features(profile, "nav", "gazebo", {"required_features": ["gps"]}) == ["gps"]
