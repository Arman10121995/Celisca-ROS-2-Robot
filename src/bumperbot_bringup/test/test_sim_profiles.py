# Copyright 2026 Bumperbot contributors
# Licensed under the Apache License, Version 2.0

from pathlib import Path
import subprocess

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
MODES_PATH = PACKAGE_DIR / "config" / "sim_modes.yaml"
MAPS_PATH = PACKAGE_DIR / "config" / "sim_maps.yaml"
ROBOTS_PATH = SRC_DIR / "robots" / "config" / "robots.yaml"


def _load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


MODES = _load(MODES_PATH)["modes"]
MAPS = _load(MAPS_PATH)["maps"]
ROBOTS = _load(ROBOTS_PATH)["robots"]


def test_expected_profiles_are_present():
    assert set(MODES) == {"display", "loc", "slam", "3d_slam", "nav"}
    assert len(MAPS) == 14
    assert len(ROBOTS) == 15


@pytest.mark.parametrize("map_name,map_config", MAPS.items())
def test_map_profile_references_exist(map_name, map_config):
    world_path = SRC_DIR / "maps" / map_config["gazebo"]["world_path"]
    assert world_path.is_file(), f"{map_name}: missing world {world_path}"

    map_config = map_config.get("map", {})
    if not map_config.get("has_2d_map", False):
        return

    map_yaml = SRC_DIR / "maps" / map_config["path"]
    assert map_yaml.is_file(), f"{map_name}: missing map YAML {map_yaml}"
    metadata = _load(map_yaml)
    image_path = map_yaml.parent / metadata["image"]
    assert image_path.is_file(), f"{map_name}: missing map image {image_path}"


@pytest.mark.parametrize("robot_name,robot_config", ROBOTS.items())
def test_robot_profile_can_be_expanded(robot_name, robot_config):
    model_path = SRC_DIR / "robots" / robot_config["xacro"]
    assert model_path.is_file(), f"{robot_name}: missing model {model_path}"
    subprocess.run(
        ["xacro", str(model_path)],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )


@pytest.mark.parametrize("robot_name,robot_config", ROBOTS.items())
def test_robot_modes_and_features_are_compatible(robot_name, robot_config):
    supported_modes = robot_config.get("supported_modes", [])
    assert supported_modes, f"{robot_name}: supported_modes must not be empty"
    assert set(supported_modes) <= set(MODES)

    features = set(robot_config.get("features", []))
    for mode_name in supported_modes:
        required_features = set(MODES[mode_name].get("required_features", []))
        assert required_features <= features, (
            f"{robot_name}: mode {mode_name} requires "
            f"{sorted(required_features - features)}"
        )


@pytest.mark.parametrize("mode_name,mode_config", MODES.items())
@pytest.mark.parametrize("map_name,map_config", MAPS.items())
def test_bumperbot_mode_map_matrix(mode_name, mode_config, map_name, map_config):
    assert mode_name in ROBOTS["bumperbot"]["supported_modes"]
    has_2d_map = map_config.get("map", {}).get("has_2d_map", False)
    compatible = not mode_config.get("requires_2d_map", False) or has_2d_map
    if map_name in {"simple_box", "empty", "residential_demo"}:
        assert compatible == (mode_name not in {"loc", "nav"})
    else:
        assert compatible


def test_room_vacuum_launch_uses_an_installed_executable():
    launch_text = (PACKAGE_DIR / "launch" / "simulated_room_vacuum.launch.py").read_text(
        encoding="utf-8"
    )
    controller_cmake = (SRC_DIR / "bumperbot_controller" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    assert 'executable="mapping_controller.py"' in launch_text
    assert "${PROJECT_NAME}/mapping_controller.py" in controller_cmake


def test_runtime_topic_contracts_are_consistent():
    ekf = _load(SRC_DIR / "bumperbot_localization" / "config" / "ekf.yaml")
    assert ekf["ekf_filter_node"]["ros__parameters"]["odom0"] == (
        "/bumperbot_controller/odom"
    )
    assert MODES["3d_slam"]["rtabmap"]["odom_topic"] == "/odometry/filtered"

    mux = _load(
        SRC_DIR / "bumperbot_controller" / "config" / "twist_mux_topics.yaml"
    )
    topics = mux["twist_mux"]["ros__parameters"]["topics"]
    assert {entry["topic"] for entry in topics.values()} == {
        "/joy_vel",
        "/key_vel",
        "/cmd_vel",
    }

    nav = _load(SRC_DIR / "bumperbot_navigation" / "config" / "bt_navigator.yaml")
    assert nav["bt_navigator"]["ros__parameters"]["odom_topic"] == (
        "/odometry/filtered"
    )
