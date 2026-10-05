# Copyright 2026 Bumperbot contributors
# Licensed under the Apache License, Version 2.0

"""Integration tier (R1.2): cross-robot xacro URDF/Xacro expansion.

Each xacro robot profile is expanded on the real executable (ROS-2-sourced).
Native PX4 supplies an upstream SDF through its flight adapter instead.
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
@pytest.mark.parametrize("robot_name,robot_config", [
    (name, config) for name, config in ROBOTS.items() if config.get("xacro")])
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
@pytest.mark.parametrize("robot_name,robot_config", [
    (name, config) for name, config in ROBOTS.items() if not config.get("xacro")])
def test_native_profile_has_an_explicit_model_loader(robot_name, robot_config):
    """A missing xacro must be deliberate, never an accidental missing model."""
    assert robot_name == "px4_x500", f"{robot_name}: no declared native model loader"
    assert robot_config['package'] == 'robot_lab_adapter'
    assert robot_config['drive']['type'] == 'px4'
    assert 'flight' in robot_config['supported_modes']
    assert 'flight_controller' in robot_config['features']
    assert (_SRC_DIR/'robot_lab_adapter/robot_lab_adapter/px4_sitl_runtime.py').is_file()


@pytest.mark.integration
def test_steered_wheel_sweep_clears_the_chassis_collision_box():
    """A full-width chassis box collided with the tires at steer angles.

    MuJoCo resolved the overlap between the sweep of a 0.05 m tire and the
    body edge at the steering limit with a ~113 N contact that saturated the
    10 N*m steering servos (0.23 rad measured of 0.45 commanded, 2026-10-01);
    PyBullet masked the defect because its steering motor is effectively
    kinematic.  The chassis *collision* is now a narrower wheel-well box and
    the visual shell stays full width, so this pins the clearance computed
    from the expanded URDF numbers themselves.
    """
    xacro_bin = shutil.which("xacro")
    if xacro_bin is None:
        pytest.skip("xacro executable not available")
    for robot in ("ackermann_car", "four_wheel_steer_car"):
        model = _SRC_DIR / "robot_lab_robots" / ROBOTS[robot]["xacro"]
        result = subprocess.run(
            ["bash", "-c",
             f"source /opt/ros/humble/setup.bash && {xacro_bin} {model}"],
            capture_output=True, text=True, timeout=120, check=True)
        root = ET.fromstring(result.stdout)
        chassis = root.find("./link[@name='base_link']/collision/geometry/box")
        assert chassis is not None, f"{robot}: base_link has no box collision"
        half_y = float(chassis.get("size").split()[1]) / 2.0
        steered = 0
        for joint in root.findall("./joint"):
            if not (joint.get("name") or "").endswith("_steer_joint"):
                continue
            limit = joint.find("limit")
            bound = max(abs(float(limit.get("lower"))),
                        abs(float(limit.get("upper"))))
            knuckle = joint.find("child").get("link")
            y_center = abs(float(joint.find("origin").get("xyz").split()[1]))
            wheel_joint = [j for j in root.findall("./joint")
                           if j.find("parent") is not None
                           and j.find("parent").get("link") == knuckle
                           and "wheel_link" in j.find("child").get("link")]
            assert wheel_joint, f"{robot}: no wheel under {knuckle}"
            offset = float(wheel_joint[0].find("origin").get("xyz").split()[1])
            y_center += abs(offset)
            wheel = wheel_joint[0].find("child").get("link")
            cylinder = root.find(
                f"./link[@name='{wheel}']/collision/geometry/cylinder")
            radius = float(cylinder.get("radius"))
            width = float(cylinder.get("length"))
            # The wheel plane turns with the knuckle: its inner face pulls in
            # by (width/2)*cos(theta) plus the rim swing radius*sin(theta).
            import math
            sweep_min = (y_center - (width / 2.0) * math.cos(bound)
                         - radius * math.sin(bound))
            steered += 1
            assert half_y + 0.005 <= sweep_min, (
                f"{robot}: chassis collision half-width {half_y:.3f} m does "
                f"not clear {wheel}'s swept inner edge {sweep_min:.3f} m at "
                f"{bound:.3f} rad")
        assert steered >= 2, f"{robot}: expected steered wheels"
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
