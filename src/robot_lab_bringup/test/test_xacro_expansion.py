# Copyright 2026 Bumperbot contributors
# Licensed under the Apache License, Version 2.0

"""Integration tier (R1.2): cross-robot xacro URDF/Xacro expansion.

Each robot profile is expanded on the real xacro executable (ROS-2-sourced).
This requires a sourced ROS 2 environment and the ``xacro`` tool, so it runs
in the **integration** tier, never in the fast/unit suite. Fast tests must
not spawn subprocesses or touch a shared ROS graph.
"""

from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


_PACKAGE_DIR = Path(__file__).resolve().parents[1]
_SRC_DIR = _PACKAGE_DIR.parent
_ROBOTS_PATH = _SRC_DIR / "robot_lab_robots" / "config" / "robots.yaml"


def _load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


ROBOTS = _load(_ROBOTS_PATH)["robots"]


@pytest.mark.integration
@pytest.mark.parametrize("robot_name,robot_config", ROBOTS.items())
def test_robot_profile_can_be_expanded(robot_name, robot_config):
    model_path = _SRC_DIR / "robot_lab_robots" / robot_config["xacro"]
    assert model_path.is_file(), f"{robot_name}: missing model {model_path}"
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        pytest.skip("xacro executable not available")
    result = subprocess.run(
        ["bash", "-c",
         f"source /opt/ros/humble/setup.bash && {xacro_bin} {model_path}"],
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, (
        f"{robot_name}: xacro expansion failed:\n{result.stderr}")


@pytest.mark.integration
def test_four_wheel_steering_hubs_occupy_four_distinct_corners():
    """Four coincident wheels can drive straight but cannot steer the base."""
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        pytest.skip("xacro executable not available")
    model = _SRC_DIR / "robot_lab_robots" / ROBOTS["four_wheel_steer_car"]["xacro"]
    result = subprocess.run(
        ["bash", "-c",
         f"source /opt/ros/humble/setup.bash && {xacro_bin} {model}"],
        capture_output=True, text=True, timeout=120, check=True)
    root = ET.fromstring(result.stdout)
    expected = {"front_left": (0.16, 0.16),
                "front_right": (0.16, -0.16),
                "rear_left": (-0.16, 0.16),
                "rear_right": (-0.16, -0.16)}
    for corner, xy in expected.items():
        joint = root.find(f"./joint[@name='{corner}_steer_joint']")
        assert joint is not None
        assert tuple(float(v) for v in joint.find("origin").get("xyz").split()[:2]) == xy
        wheel = root.find(f"./joint[@name='{corner}_wheel_joint']")
        assert float(wheel.find("origin").get("xyz").split()[2]) == -0.012
