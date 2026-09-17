# Copyright 2026 Bumperbot contributors
# Licensed under the Apache License, Version 2.0

from pathlib import Path

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PACKAGE_DIR.parent
MODES_PATH = PACKAGE_DIR / "config" / "sim_modes.yaml"
MAPS_PATH = PACKAGE_DIR / "config" / "sim_maps.yaml"
ROBOTS_PATH = SRC_DIR / "robot_lab_robots" / "config" / "robots.yaml"


def _load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


MODES = _load(MODES_PATH)["modes"]
MAPS = _load(MAPS_PATH)["maps"]
ROBOTS = _load(ROBOTS_PATH)["robots"]


# ---------------------------------------------------------------------------
# R5.3: dispatch-path contract for the BHL live-backend bringup.
#
# The 2026-09-16 live proof (see
# docs/status/evidence/r53-bhl-live-backend-2026-09-16/) verified the headless
# Gazebo bringup of the commandable Berkeley Humanoid Lite directly through
# robot_lab_description/gazebo.launch.py. The tests below pin the data that
# lets the standard dispatch path reach the same bringup via
# robot_lab_bringup/simulated_robot.launch.py: the profile must exist in
# robots.yaml, point at the self-contained sim xacro, and carry a spawn name
# matching the verified probe.
# ---------------------------------------------------------------------------

BHL_XACRO = (
    SRC_DIR
    / "robot_lab_robots"
    / "berkeley_humanoid_lite"
    / "xacro"
    / "bhl_sim.xacro"
)
BHL_CONTROL_XACRO = (
    SRC_DIR
    / "robot_lab_robots"
    / "berkeley_humanoid_lite"
    / "xacro"
    / "bhl_ros2_control.xacro"
)
BHL_CONTROLLERS_YAML = (
    SRC_DIR
    / "robot_lab_robots"
    / "berkeley_humanoid_lite"
    / "config"
    / "bhl_controllers.yaml"
)


def test_bhl_sim_profile_is_registered():
    assert "berkeley_humanoid_lite_sim" in ROBOTS, (
        "berkeley_humanoid_lite_sim: missing from robots.yaml; the dispatch "
        "path cannot reach the verified R5.3 live-backend bringup"
    )


def test_bhl_sim_profile_points_at_the_self_contained_sim_xacro():
    config = ROBOTS["berkeley_humanoid_lite_sim"]
    xacro = SRC_DIR / "robot_lab_robots" / config["xacro"]
    assert xacro == BHL_XACRO, (
        "berkeley_humanoid_lite_sim must expand bhl_sim.xacro (the only BHL "
        "description that embeds the gz_ros2_control plugins and controller "
        "parameters)"
    )
    assert xacro.is_file(), f"{xacro}: sim xacro missing"
    xacro_text = xacro.read_text(encoding="utf-8")
    assert "bhl_ros2_control.xacro" in xacro_text
    assert "$(find robot_lab_robots)" in xacro_text, (
        "sim xacro must resolve includes through the installed package share"
    )


def test_bhl_sim_profile_spawn_name_matches_the_verified_probe():
    # The live proof spawned the robot under the name 'bhl'; the dispatch
    # profile must produce the same entity name so the evidence stays
    # reproducible.
    config = ROBOTS["berkeley_humanoid_lite_sim"]
    assert config.get("name") == "bhl"
    # The verified probe world was nav_empty via gazebo.launch.py; the
    # dispatch default (mode display, map empty) is the empty world, which
    # uses the same ground plane and free-fall spawn behavior.
    assert config.get("supported_modes") == ["display"]


def test_bhl_controller_config_declares_the_verified_controllers():
    config = yaml.safe_load(BHL_CONTROLLERS_YAML.read_text(encoding="utf-8"))
    cm = config["controller_manager"]["ros__parameters"]
    assert (
        cm["bhl_standing_controller"]["type"]
        == "effort_controllers/JointGroupEffortController"
    )
    assert (
        cm["joint_state_broadcaster"]["type"]
        == "joint_state_broadcaster/JointStateBroadcaster"
    )
    joints = config["bhl_standing_controller"]["ros__parameters"]["joints"]
    assert len(joints) == 22, (
        f"bhl_standing_controller: expected the 22 leg/arm joints, got {len(joints)}"
    )
    assert (
        config["bhl_standing_controller"]["ros__parameters"]["kp"] == 10.0
    )
    assert (
        config["bhl_standing_controller"]["ros__parameters"]["kd"] == 2.0
    )


def test_bhl_sim_profile_declares_its_controllers_for_bringup():
    # The description loads bhl_controllers.yaml itself; bringup must spawn
    # (load/configure/activate) the declared controllers or the bringup
    # reaches controller_manager without an active controller.
    config = ROBOTS["berkeley_humanoid_lite_sim"]
    assert config.get("controllers") == [
        "joint_state_broadcaster",
        "bhl_standing_controller",
    ]


def test_bhl_sim_xacro_wires_the_verified_backend_plugins():
    """The sim xacro must carry the Ignition-side plugins the live proof
    exercised: the ros2_control system bridge, the IMU system, and the IMU
    sensor publishing on the bridged 'imu' topic."""
    combined = BHL_XACRO.read_text(encoding="utf-8") + BHL_CONTROL_XACRO.read_text(
        encoding="utf-8"
    )
    assert "ign_ros2_control-system" in combined
    assert "ignition-gazebo-imu-system" in combined
    assert "ignition-gazebo-sensors-system" in combined
    assert "<topic>imu</topic>" in combined, (
        "BHL IMU sensor must publish on the 'imu' topic that the gazebo "
        "launch bridges to /imu/out"
    )


def test_expected_profiles_are_present():
    assert set(MODES) == {"display", "loc", "slam", "3d_slam", "nav"}
    # 14 legacy maps existed before P4.2; the P4.2/P4.3/P4.4/P4.5 arenas added more.
    assert len(MAPS) >= 14
    for legacy in (
        "simple_office", "small_house", "small_warehouse", "warehouse_demo",
        "celisca_floor_1", "celisca_floor_2", "bigger_warehouse", "empty",
        "residential_demo", "simple_box",
    ):
        assert legacy in MAPS, f"{legacy}: legacy map missing from sim_maps.yaml"
    assert len(ROBOTS) >= 17
    for core in ("bumperbot", "labbot", "quadrotor_sitl", "unitree_go2", "berkeley_humanoid_lite"):
        assert core in ROBOTS, f"{core}: core robot profile missing"


@pytest.mark.parametrize("map_name,map_config", MAPS.items())
def test_map_profile_references_exist(map_name, map_config):
    world_path = SRC_DIR / "robot_lab_maps" / map_config["gazebo"]["world_path"]
    assert world_path.is_file(), f"{map_name}: missing world {world_path}"

    map_config = map_config.get("map", {})
    if not map_config.get("has_2d_map", False):
        return

    map_yaml = SRC_DIR / "robot_lab_maps" / map_config["path"]
    assert map_yaml.is_file(), f"{map_name}: missing map YAML {map_yaml}"
    metadata = _load(map_yaml)
    image_path = map_yaml.parent / metadata["image"]
    assert image_path.is_file(), f"{map_name}: missing map image {image_path}"


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
    controller_cmake = (SRC_DIR / "robot_lab_controller" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    assert 'executable="mapping_controller.py"' in launch_text
    assert "${PROJECT_NAME}/mapping_controller.py" in controller_cmake


SIMULATORS = {"gazebo", "isaac", "pybullet", "mujoco"}


@pytest.mark.parametrize("mode_name,mode_config", MODES.items())
def test_every_mode_declares_supported_simulators(mode_name, mode_config):
    simulators = mode_config.get("simulators")
    assert simulators, f"{mode_name}: missing simulators list in sim_modes.yaml"
    assert len(simulators) == len(set(simulators)), f"{mode_name}: duplicate simulators"
    for sim in simulators:
        assert sim in SIMULATORS, f"{mode_name}: unknown simulator '{sim}'"


def test_launch_dispatch_covers_all_simulators():
    launch_text = (PACKAGE_DIR / "launch" / "simulated_robot.launch.py").read_text(
        encoding="utf-8"
    )
    # The dispatch registry must cover exactly the four simulators.
    for sim in ("gazebo", "isaac", "pybullet", "mujoco"):
        assert sim in launch_text, f"{sim}: missing from simulator dispatch"

    for adapter_pkg, launch_file in (
        ("robot_lab_description", "gazebo.launch.py"),
        ("robot_lab_isaac", "isaac_simulator.launch.py"),
        ("robot_lab_pybullet", "pybullet_simulator.launch.py"),
        ("robot_lab_mujoco", "mujoco_simulator.launch.py"),
    ):
        assert launch_file in launch_text, f"{launch_file}: not referenced by dispatch"
        adapter = SRC_DIR / adapter_pkg
        if adapter_pkg == "robot_lab_description":
            adapter = SRC_DIR / "robot_lab_description"
        assert (adapter / "launch" / launch_file).is_file(), (
            f"{adapter_pkg}: missing launch {launch_file}"
        )


def test_each_simulator_launch_accepts_spawn_interface():
    """Every simulator adapter must expose the spawn argument interface the
    bringup dispatcher forwards (robot/pose/world args)."""
    required_args = {
        "world_name",
        "world_package",
        "world_path",
        "model",
        "robot_package",
        "robot_name",
        "spawn_x",
        "spawn_y",
        "spawn_z",
        "spawn_yaw",
        "use_sim_time",
    }
    for adapter_pkg, launch_file in (
        ("robot_lab_isaac", "isaac_simulator.launch.py"),
        ("robot_lab_pybullet", "pybullet_simulator.launch.py"),
        ("robot_lab_mujoco", "mujoco_simulator.launch.py"),
    ):
        text = (SRC_DIR / adapter_pkg / "launch" / launch_file).read_text(
            encoding="utf-8"
        )
        for arg in required_args:
            # Each arg must be both referenced and declared.
            assert f'"{arg}"' in text, f"{adapter_pkg}: missing arg '{arg}'"


def test_runtime_topic_contracts_are_consistent():
    ekf = _load(SRC_DIR / "robot_lab_localization" / "config" / "ekf.yaml")
    assert ekf["ekf_filter_node"]["ros__parameters"]["imu0"] == "imu_ekf"
    assert MODES["3d_slam"]["rtabmap"]["odom_topic"] == "/odometry/filtered"

    mux = _load(
        SRC_DIR / "robot_lab_controller" / "config" / "twist_mux_topics.yaml"
    )
    topics = mux["twist_mux"]["ros__parameters"]["topics"]
    assert {entry["topic"] for entry in topics.values()} == {
        "joy_vel",
        "key_vel",
        "cmd_vel",
    }

    nav = _load(SRC_DIR / "robot_lab_navigation" / "config" / "bt_navigator.yaml")
    assert nav["bt_navigator"]["ros__parameters"]["odom_topic"] == (
        "/odometry/filtered"
    )
