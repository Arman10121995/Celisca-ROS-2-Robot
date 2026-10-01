"""The display hold must never pin a driven wheel's passive parts (R5.6).

A mecanum hub is a driven wheel with fifteen passive roller bodies.  When the
spawner's display hold springs/motors those rollers at their spawn pose they
cannot spin, the roller wheel acts as a solid tire, and the lateral motion
the drive model commands is resisted: measured 0.014 m/s of a commanded
0.30 m/s strafe through the MuJoCo bridge against 0.28 m/s with the rollers
free in the same model (2026-10-01).  These tests pin the helper the
bridges filter their hold list with.
"""

import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

from robot_lab_utils.urdf_joints import driven_assembly_joints, link_parents

_PACKAGE_DIR = Path(__file__).resolve().parents[1]
_SRC_DIR = _PACKAGE_DIR.parent
_ROBOTS = yaml.safe_load(
    (_SRC_DIR / "robot_lab_robots" / "config" / "robots.yaml")
    .read_text(encoding="utf-8"))["robots"]

_ROLLER_URDF = """<?xml version="1.0"?>
<robot name="roller_base">
  <link name="base_link"/>
  <link name="front_left_wheel_link"/>
  <link name="front_left_wheel_roller1_link"/>
  <link name="front_left_wheel_roller2_link"/>
  <link name="arm_link"/>
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="front_left_wheel_link"/>
  </joint>
  <joint name="front_left_wheel_roller1_joint" type="continuous">
    <parent link="front_left_wheel_link"/>
    <child link="front_left_wheel_roller1_link"/>
  </joint>
  <joint name="front_left_wheel_roller2_joint" type="continuous">
    <parent link="front_left_wheel_roller1_link"/>
    <child link="front_left_wheel_roller2_link"/>
  </joint>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/><child link="arm_link"/>
  </joint>
</robot>
"""


def test_rollers_below_a_driven_wheel_are_passive_parts():
    passive = driven_assembly_joints(
        _ROLLER_URDF, ["front_left_wheel_joint"])
    assert passive == {"front_left_wheel_roller1_joint",
                       "front_left_wheel_roller2_joint"}


def test_driven_wheel_and_unrelated_joints_are_not_passive():
    passive = driven_assembly_joints(
        _ROLLER_URDF, ["front_left_wheel_joint"])
    assert "front_left_wheel_joint" not in passive
    assert "arm_joint" not in passive


def test_no_wheels_and_unknown_wheels_give_nothing():
    assert driven_assembly_joints(_ROLLER_URDF, []) == set()
    assert driven_assembly_joints(_ROLLER_URDF, ["no_such_joint"]) == set()


def test_link_parents_follow_the_whole_chain():
    parents = link_parents(_ROLLER_URDF)
    assert parents["front_left_wheel_roller2_link"] == \
        "front_left_wheel_roller1_link"
    assert parents["front_left_wheel_link"] == "base_link"


@pytest.mark.integration
def test_the_shipped_mecanum_description_is_covered():
    """The real robot: every roller of all four hubs must be excluded."""
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        pytest.skip("xacro executable not available")
    model = _SRC_DIR / "robot_lab_robots" / _ROBOTS["mecanum_car"]["xacro"]
    result = subprocess.run(
        ["bash", "-c",
         f"source /opt/ros/humble/setup.bash && {xacro_bin} {model}"],
        capture_output=True, text=True, timeout=180, check=True)
    wheels = ["front_left_wheel_joint", "front_right_wheel_joint",
              "rear_left_wheel_joint", "rear_right_wheel_joint"]
    passive = driven_assembly_joints(result.stdout, wheels)
    # 15 rollers per hub, 4 hubs, and none of the driven wheels.
    assert len(passive) == 60, sorted(passive)[:4]
    assert not set(passive) & set(wheels)
