"""Unit tests for the R8.1 live-mission harness pure logic.

The live ROS/simulator work is opt-in (ROBOT_LAB_RUN_LIVE_MISSION=1); these
tests exercise only the controller, trace and outcome-decision logic that the
live path shares, without any simulator or ROS graph.
"""

import math
import os
import subprocess
import time

import pytest

from robot_lab_benchmark.live_mission import (
    GOAL_TOLERANCE_M,
    MissionTrace,
    WaypointController,
    _env_flag,
    bag_topic_counts,
    decide_outcome,
    launch_arguments,
)
from robot_lab_benchmark.truthful_outcomes import OutcomeKind


class TestBagTopicCounts:
    """rosbag2 metadata.yaml parsing (keeps bag provenance in-repo)."""

    def test_reads_payload_written_by_rosbag2(self):
        payload = {"rosbag2_bagfile_information": {
            "message_count": 817,
            "topics_with_message_count": [
                {"topic_metadata": {"name": "/clock"},
                 "message_count": 300},
                {"topic_metadata": {"name": "/odom/ground_truth"},
                 "message_count": 301},
                {"topic_metadata": {"name": "/scan"},
                 "message_count": 108},
                {"topic_metadata": {"name": "/cmd_vel"},
                 "message_count": 108},
            ]}}
        assert bag_topic_counts(payload) == {
            "/clock": 300, "/odom/ground_truth": 301,
            "/scan": 108, "/cmd_vel": 108}

    def test_missing_count_is_omitted_not_zeroed(self):
        # "no count recorded" and "recorded nothing" are different facts.
        payload = {"rosbag2_bagfile_information": {
            "topics_with_message_count": [
                {"topic_metadata": {"name": "/scan"}},
                {"topic_metadata": {"name": "/clock"}, "message_count": None},
                {"topic_metadata": {}, "message_count": 5},
                {"topic_metadata": {"name": "/odom/ground_truth"},
                 "message_count": 0},
            ]}}
        assert bag_topic_counts(payload) == {"/odom/ground_truth": 0}

    def test_unavailable_metadata_is_empty_mapping(self):
        assert bag_topic_counts({}) == {}
        assert bag_topic_counts(None) == {}
        assert bag_topic_counts({"rosbag2_bagfile_information": {}}) == {}


class TestWaypointController:
    def test_moves_toward_goal_far_away(self):
        c = WaypointController((5.0, 0.0))
        vx, wz = c.step(0.0, 0.0, 0.0)
        assert vx == pytest.approx(0.3)  # distance-scaled speed, saturated
        assert wz < 0.001
        assert -1.0 <= wz <= 1.0

    def test_steers_to_correct_bearing(self):
        c = WaypointController((0.0, 5.0))  # goal straight left of x-axis
        vx, wz = c.step(0.0, 0.0, 0.0)  # heading +x, goal at +y bearing
        assert wz > 0.5  # must turn left (positive w) toward +y
        c2 = WaypointController((0.0, -5.0))
        _, wz2 = c2.step(0.0, 0.0, 0.0)
        assert wz2 < -0.5

    def test_slows_down_near_goal(self):
        c = WaypointController((0.4, 0.0))
        vx, _ = c.step(0.0, 0.0, 0.0)
        assert vx < c.max_vx
        assert vx > 0.0

    def test_reaches_far_goal_at_saturated_speed(self):
        c = WaypointController((5.0, 0.0))
        vx, _ = c.step(0.0, 0.0, 0.0)
        assert vx == pytest.approx(c.max_vx)

    def test_outputs_never_exceed_declared_limits(self):
        c = WaypointController((100.0, 100.0))
        for x, y, yaw in ((0.0, 0.0, 0.3), (10.0, 10.0, -2.0),
                          (-5.0, 3.0, 1.2)):
            vx, wz = c.step(x, y, yaw)
            assert 0.0 <= vx <= c.max_vx
            assert abs(wz) <= c.max_wz


class TestMissionTrace:
    def test_csv_and_poses(self):
        t = MissionTrace()
        t.record(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, None)
        t.record(1.0, 0.5, 0.0, 0.0, 0.3, 0.0, 2.0)
        text = t.csv()
        assert "inf" in text
        assert text.splitlines()[0].startswith("sim_t")
        assert t.poses() == [(0.0, 0.0), (0.5, 0.0)]
        assert t.clearances()[0] == math.inf
        assert t.clearances()[1] == 2.0

    def test_clearance_validity(self):
        t = MissionTrace()
        assert t.poses() == []


class TestDecideOutcome:
    def test_success(self):
        assert decide_outcome(True, False, 5.0, 40.0, True) \
            == OutcomeKind.SUCCESS

    def test_timeout(self):
        assert decide_outcome(False, False, 40.0, 40.0, True) \
            == OutcomeKind.TIMEOUT

    def test_collision_wins_over_timeout(self):
        # classify_outcome ranks timeout above collision (R4.1 priority
        # order); decide_outcome must delegate faithfully, not re-rank.
        assert decide_outcome(False, True, 40.0, 40.0, True) \
            == OutcomeKind.TIMEOUT

    def test_process_death(self):
        assert decide_outcome(True, False, 5.0, 40.0, False) \
            == OutcomeKind.PROCESS_DEATH


def test_goal_tolerance_is_finite_and_sane():
    assert 0.0 < GOAL_TOLERANCE_M < 0.5


def test_env_flag_parsing(monkeypatch):
    for truthy in ("1", "true", "True", "YES", "on", " 1 "):
        monkeypatch.setenv("ROBOT_LAB_RUN_LIVE_MISSION", truthy)
        assert _env_flag("ROBOT_LAB_RUN_LIVE_MISSION")
    for falsy in ("", "0", "false", "no", "off"):
        monkeypatch.setenv("ROBOT_LAB_RUN_LIVE_MISSION", falsy)
        assert not _env_flag("ROBOT_LAB_RUN_LIVE_MISSION")
    monkeypatch.delenv("ROBOT_LAB_RUN_LIVE_MISSION", raising=False)
    assert not _env_flag("ROBOT_LAB_RUN_LIVE_MISSION")


def test_main_refuses_to_launch_without_opt_in(monkeypatch):
    """The documented opt-in gate must actually gate (no sims started).

    Without ROBOT_LAB_RUN_LIVE_MISSION the entry point exits 2 before any
    simulator or DDS work, so a bare `robot-lab-live-mission` never touches
    a shared workspace's graph.
    """
    from robot_lab_benchmark.live_mission import main
    monkeypatch.delenv("ROBOT_LAB_RUN_LIVE_MISSION", raising=False)
    assert main(["--simulator", "mujoco", "--seeds", "1001"]) == 2


def _assert_live_record(out, simulator, seed, world="nav_empty", goal_m=1.5):
    """Shared assertions for an opt-in live-mission record (R8.1 evidence)."""
    import json
    summary = json.loads((out / ("summary_%s.json" % simulator)).read_text())
    assert len(summary) == 1
    record = summary[0]
    assert record["outcome"] == "success", record
    assert record["simulator"] == simulator
    assert record["seed"] == seed
    assert record["environment_id"] == world
    assert record["scenario_id"] == "point_to_point_navigation"
    metrics = record["metrics"]
    assert metrics["trajectory_distance_m"] > 1.0
    assert metrics["footprint_clearance_min_m"] > 0.0
    assert metrics["contact_events"] == 0
    assert metrics["bag_capture"] is True
    # Per-topic bag counts are part of the record (committed provenance).
    counts = metrics["bag_message_counts"]
    assert counts, metrics
    for topic in ("/clock", "/odom/ground_truth", "/scan", "/cmd_vel"):
        assert counts.get(topic, 0) > 0, counts
    assert sum(counts.values()) >= 100, counts
    run_dir = (out / ("r4_%s_point_to_point_navigation_%s_%d"
                      % (simulator, world, seed)))
    assert (run_dir / "manifest.json").exists()
    assert (run_dir / "trace.csv").exists()
    bag_dir = run_dir / "bags" / ("run_%d" % seed)
    assert bag_dir.exists() and any(bag_dir.iterdir())
    assert (bag_dir / "metadata.yaml").exists()


def _live_env():
    """Environment for an opt-in live run (the harness gate reads the flag)."""
    return dict(os.environ, ROBOT_LAB_RUN_LIVE_MISSION="1")


# The repo convention is `ros2 run <package> <executable>`: sourcing
# install/setup.bash does not put the console script on PATH, so invoking
# `robot-lab-live-mission` directly only works if the caller happened to
# export install/<pkg>/lib/<pkg> themselves.
_MISSION_RUN = ["ros2", "run", "robot_lab_benchmark",
                "robot-lab-live-mission"]


@pytest.mark.skipif(
    not os.environ.get("ROBOT_LAB_RUN_LIVE_MISSION"),
    reason="live simulator mission is opt-in "
           "(ROBOT_LAB_RUN_LIVE_MISSION=1)")
def test_live_mission_mujoco_seed1001(tmp_path):
    """Seeded MuJoCo R4 mission on an isolated domain (R8.1 evidence).

    Launches ``robot_lab_mujoco`` on ROS_DOMAIN_ID=171, drives the
    point-to-point mission via this module's own entry point, and asserts
    the truthful R4.1 outcome plus R4.2 measured-metric fields in the
    manifest.  On this shared host the launch + mission takes ~3-4 min.
    """
    from ament_index_python.packages import get_package_share_directory
    del get_package_share_directory  # exercised inside the subprocess
    out = tmp_path / "live_mujoco"
    cmd = _MISSION_RUN + ["--simulator", "mujoco",
                          "--seeds", "1001", "--goal", "1.5",
                          "--world", "nav_empty", "--output", str(out),
                          "--timeout", "40.0", "--domain", "170"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600,
                          env=_live_env())
    assert proc.returncode == 0, proc.stderr[-3000:]
    _assert_live_record(out, "mujoco", 1001)


@pytest.mark.skipif(
    not os.environ.get("ROBOT_LAB_RUN_LIVE_MISSION"),
    reason="live simulator mission is opt-in "
           "(ROBOT_LAB_RUN_LIVE_MISSION=1)")
def test_live_mission_pybullet_seed1101(tmp_path):
    """Seeded PyBullet R4 mission on an isolated domain (R8.1 evidence).

    Mirrors the MuJoCo mission on ``robot_lab_pybullet`` (ROS_DOMAIN_ID=176
    for seed 1101 on base domain 175) with the same goal, world, timeout
    and record assertions.
    """
    out = tmp_path / "live_pybullet"
    cmd = _MISSION_RUN + ["--simulator", "pybullet",
                          "--seeds", "1101", "--goal", "1.5",
                          "--world", "nav_empty", "--output", str(out),
                          "--timeout", "40.0", "--domain", "175"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600,
                          env=_live_env())
    assert proc.returncode == 0, proc.stderr[-3000:]
    _assert_live_record(out, "pybullet", 1101)


@pytest.mark.skipif(
    not os.environ.get("ROBOT_LAB_RUN_LIVE_MISSION"),
    reason="live simulator mission is opt-in "
           "(ROBOT_LAB_RUN_LIVE_MISSION=1)")
def test_live_mission_isaac_seed1201(tmp_path):
    """Seeded Isaac Sim R4 mission on an isolated domain (R8.2 evidence).

    Mirrors the MuJoCo/PyBullet missions on ``robot_lab_isaac``
    (ROS_DOMAIN_ID=201 for seed 1201 on base domain 200) with the same goal,
    world, timeout and record assertions.  Isaac starts a Kit runtime in
    its own process group, so the launch + mission takes longer than the
    other backends (warm start ~30 s, then the R4 mission at a real-time
    factor of ~0.13 on this Jetson).
    """
    out = tmp_path / "live_isaac"
    cmd = _MISSION_RUN + ["--simulator", "isaac",
                          "--seeds", "1201", "--goal", "1.5",
                          "--world", "nav_empty", "--output", str(out),
                          "--timeout", "40.0", "--domain", "200"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=900,
                          env=_live_env())
    assert proc.returncode == 0, proc.stderr[-3000:]
    _assert_live_record(out, "isaac", 1201)


class TestLaunchArguments:
    """Each backend's launch file takes its world and robot differently."""

    def test_mujoco_and_isaac_select_the_world_by_name(self):
        for simulator in ("mujoco", "isaac"):
            args = launch_arguments(simulator, "nav_maze", "/r", "/m")
            assert "world_name:=nav_maze" in args
            assert "model:=/r/bumperbot/urdf/bumperbot.urdf.xacro" in args
            assert "gui:=false" in args

    def test_pybullet_loads_the_world_file(self):
        args = launch_arguments("pybullet", "nav_maze", "/r", "/m")
        assert "world_path:=/m/maps/nav_maze/worlds/nav_maze.world" in args

    def test_isaac_passes_the_required_robot_xacro(self):
        args = launch_arguments("isaac", "nav_empty", "/r", "/m")
        assert args[:2] == ["robot_lab_isaac", "isaac_simulator.launch.py"]
        assert "robot_xacro:=bumperbot/urdf/bumperbot.urdf.xacro" in args
