"""R5.1 tests: golden-harness qualification for Bumperbot and Labbot.

Covers the R5.1 acceptance bar:

- Robot descriptions parsed from processed URDF XML yield wheel geometry,
  footprint, sensors, and actuated joints matching the golden values that
  were measured from the real robot sources.
- Both bases pass the static golden checks; a tampered value fails.
- Capabilities are claimed only when implemented: an RGB-D claim requires
  a depth camera; Labbot (no RGB-D) does not claim it.
- Task-level qualification is injected; with no executor the report is
  honestly not a pass.
- Truth and odometry trajectories stay separate; odometry drift is
  reported as its own observation, never merged into truth.
"""

from __future__ import annotations

import dataclasses
import json
import math
import sys
from pathlib import Path

import pytest

_benchmark_pkg = Path(__file__).resolve().parents[3] / "robot_lab" / "robot_lab_benchmark"
if str(_benchmark_pkg) not in sys.path:
    sys.path.insert(0, str(_benchmark_pkg))

from robot_lab_benchmark.qualification import (
    GOLDEN,
    QualificationReport,
    RobotDescription,
    TaskStageResult,
    TaskTrialObservation,
    TaskTrialSpec,
    golden_for,
    normalize_sensor_types,
    parse_robot_description,
    qualify_robot,
    run_static_checks,
    write_qualification_report,
)


# ----------------------------------------------------------------------
# Synthetic URDFs mirroring the real measured descriptions
# ----------------------------------------------------------------------


def _link(name, mass="1.0", inertia=True, collision=""):
    ix = "10" if inertia else None
    inertial = ""
    if mass is not None:
        inertia_xml = (
            "<inertia ixx='%s' ixy='0' ixz='0' iyy='%s' iyz='0' izz='%s'/>"
            % (ix, ix, ix) if inertia else ""
        )
        inertial = "<inertial><mass value='%s'/>%s</inertial>" % (mass, inertia_xml)
    return "<link name='%s'>%s%s</link>" % (name, inertial, collision)


def _wheel_link(name, radius, shape="cylinder", length="0.04"):
    if shape == "sphere":
        geom = "<sphere radius='%s'/>" % radius
    else:
        geom = "<cylinder radius='%s' length='%s'/>" % (radius, length)
    return _link(name, collision="<collision><geometry>%s</geometry></collision>" % geom)


def _wheel_joint(joint, parent, child, y):
    return (
        "<joint name='%s' type='continuous'>"
        "<parent link='%s'/><child link='%s'/>"
        "<origin xyz='0 %s 0' rpy='0 0 0'/><axis xyz='0 1 0'/>"
        "</joint>" % (joint, parent, child, y)
    )


def _sensor(name, stype):
    return "<gazebo reference='base_link'><sensor name='%s' type='%s'/></gazebo>" % (
        name, stype,
    )


def _bumperbot_urdf(with_rgbd=True):
    parts = [
        "<robot name='bumperbot'>",
        "<link name='base_footprint'/>",
        "<joint name='base_joint' type='fixed'>"
        "<parent link='base_footprint'/><child link='base_link'/>"
        "<origin xyz='0 0 0.033' rpy='0 0 0'/></joint>",
        # base_link: box collision -> footprint via box extents
        "<link name='base_link'><inertial><mass value='2.0'/>"
        "<inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01'/></inertial>"
        "<collision><geometry><box size='0.18 0.14 0.06'/></geometry></collision></link>",
        _wheel_link("wheel_right_link", "0.033", shape="sphere"),
        _wheel_link("wheel_left_link", "0.033", shape="sphere"),
        _wheel_joint("wheel_right_joint", "base_link", "wheel_right_link", "-0.0701101849418637"),
        _wheel_joint("wheel_left_joint", "base_link", "wheel_left_link", "0.0701101849418642"),
        _sensor("lidar", "ray"),
        _sensor("imu", "imu"),
    ]
    if with_rgbd:
        parts.append(_sensor("oakd", "depth"))
    parts.append(
        "<ros2_control name='diff_drive'><joint name='wheel_left_joint'>"
        "<command_interface name='velocity'/></joint>"
        "<joint name='wheel_right_joint'><command_interface name='velocity'/>"
        "</joint></ros2_control>"
    )
    parts.append("</robot>")
    return "".join(parts)


def _labbot_urdf():
    return "".join([
        "<robot name='labbot'>",
        "<link name='base_footprint'/>",
        "<joint name='base_joint' type='fixed'>"
        "<parent link='base_footprint'/><child link='base_link'/>"
        "<origin xyz='0 0 0.12' rpy='0 0 0'/></joint>",
        "<link name='base_link'><inertial><mass value='4.0'/>"
        "<inertia ixx='0.02' ixy='0' ixz='0' iyy='0.02' iyz='0' izz='0.02'/></inertial>"
        "<collision><geometry><box size='0.30 0.30 0.24'/></geometry></collision></link>",
        _wheel_link("labbot_left_wheel_link", "0.06"),
        _wheel_link("labbot_right_wheel_link", "0.06"),
        _wheel_joint("labbot_left_wheel_joint", "base_link", "labbot_left_wheel_link", "0.15"),
        _wheel_joint("labbot_right_wheel_joint", "base_link", "labbot_right_wheel_link", "-0.15"),
        _sensor("lidar", "ray"),
        _sensor("imu", "imu"),
        "<ros2_control name='diff_drive'><joint name='labbot_left_wheel_joint'>"
        "<command_interface name='velocity'/></joint>"
        "<joint name='labbot_right_wheel_joint'><command_interface name='velocity'/>"
        "</joint></ros2_control>",
        "</robot>",
    ])


def _all_stages_pass(robot_id, spec):
    return [
        TaskStageResult("spawn", True, "spawned at (%g, %g)" % (spec.spawn_x, spec.spawn_y)),
        TaskStageResult("drive", True, "drove %g m" % spec.drive_distance_m),
        TaskStageResult("turn", True, "turned %g deg" % spec.turn_degrees),
        TaskStageResult("stop", True, "stopped within tolerance"),
        TaskStageResult("nav_clear", True, "cleared navigation"),
        TaskStageResult("nav_obstacle", True, "obstacle navigation, no contacts"),
    ]


# ----------------------------------------------------------------------
# Description extraction
# ----------------------------------------------------------------------


class TestDescriptionExtraction:
    def test_bumperbot_parse(self):
        desc = parse_robot_description(_bumperbot_urdf())
        assert desc.robot_name == "bumperbot"
        assert len(desc.wheels) == 2
        assert desc.wheel_radius() == pytest.approx(0.033)
        assert desc.wheel_separation == pytest.approx(0.1402203698837279, rel=1e-9)
        assert desc.base_footprint_offset_z == pytest.approx(0.033)
        assert {w.joint for w in desc.wheels} == {
            "wheel_left_joint", "wheel_right_joint",
        }

    def test_labbot_parse(self):
        desc = parse_robot_description(_labbot_urdf())
        assert desc.robot_name == "labbot"
        assert desc.wheel_radius() == pytest.approx(0.06)
        assert desc.wheel_separation == pytest.approx(0.30)
        assert desc.base_footprint_offset_z == pytest.approx(0.12)
        assert {w.joint for w in desc.wheels} == {
            "labbot_left_wheel_joint", "labbot_right_wheel_joint",
        }

    def test_bumperbot_footprint_from_box(self):
        desc = parse_robot_description(_bumperbot_urdf())
        # box 0.18 x 0.14 -> quarter-diagonal footprint radius
        expected = math.hypot(0.18 / 2, 0.14 / 2)
        assert desc.footprint["radius"] == pytest.approx(expected)

    def test_bumperbot_sensors_include_depth(self):
        desc = parse_robot_description(_bumperbot_urdf(with_rgbd=True))
        assert {"ray", "imu", "depth"} <= set(desc.sensor_types)

    def test_labbot_sensors_exclude_depth(self):
        desc = parse_robot_description(_labbot_urdf())
        assert {"ray", "imu"} <= set(desc.sensor_types)
        assert "depth" not in desc.sensor_types

    def test_actuated_joints_from_ros2_control(self):
        desc = parse_robot_description(_bumperbot_urdf())
        assert set(desc.actuated_joints) == {"wheel_left_joint", "wheel_right_joint"}

    def test_all_links_have_inertia(self):
        for xml in (_bumperbot_urdf(), _labbot_urdf()):
            desc = parse_robot_description(xml)
            physical = [l for l in desc.links if not l.is_virtual]
            assert all(l.has_inertia and l.mass and l.mass > 0 for l in physical)

    def test_base_footprint_is_virtual(self):
        # base_footprint is a frame-only link in both real descriptions;
        # it must be detected as virtual, not flagged as missing inertia.
        for xml in (_bumperbot_urdf(), _labbot_urdf()):
            desc = parse_robot_description(xml)
            fp = next(l for l in desc.links if l.name == "base_footprint")
            assert fp.is_virtual is True


# ----------------------------------------------------------------------
# Capabilities claimed only when implemented
# ----------------------------------------------------------------------


class TestCapabilities:
    def test_bumperbot_claims_rgbd(self):
        desc = parse_robot_description(_bumperbot_urdf(with_rgbd=True))
        assert "rgbd" in desc.capabilities

    def test_bumperbot_without_depth_does_not_claim_rgbd(self):
        desc = parse_robot_description(_bumperbot_urdf(with_rgbd=False))
        assert "rgbd" not in desc.capabilities

    def test_labbot_does_not_claim_rgbd(self):
        desc = parse_robot_description(_labbot_urdf())
        assert "rgbd" not in desc.capabilities

    def test_both_claim_diff_drive_lidar_imu(self):
        for xml in (_bumperbot_urdf(), _labbot_urdf()):
            caps = parse_robot_description(xml).capabilities
            for cap in ("differential_drive", "lidar", "imu"):
                assert cap in caps


# ----------------------------------------------------------------------
# Sensor-type normalization (simulator dialects)
# ----------------------------------------------------------------------


class TestSensorNormalization:
    def test_classic_dialect(self):
        assert normalize_sensor_types(["ray", "imu", "depth"]) == (
            "lidar", "imu", "depth")

    def test_ignition_dialect(self):
        assert normalize_sensor_types(["gpu_lidar", "imu", "rgbd_camera"]) == (
            "lidar", "imu", "depth")

    def test_unknown_types_dropped(self):
        assert normalize_sensor_types(["wibble", "ray"]) == ("lidar",)

    def test_duplicates_removed(self):
        assert normalize_sensor_types(["ray", "gpu_lidar"]) == ("lidar",)

    def test_real_descriptions_normalize(self):
        for xml in (_bumperbot_urdf(), _labbot_urdf()):
            desc = parse_robot_description(xml)
            assert "lidar" in desc.normalized_sensor_types
            assert "imu" in desc.normalized_sensor_types

    def test_ignition_bumperbot_passes_sensor_check(self):
        # gpu_lidar/rgbd_camera dialect must qualify against the golden
        # normalized requirement (lidar/imu/depth)
        desc = parse_robot_description(_bumperbot_urdf())
        gz_desc = dataclasses.replace(
            desc,
            sensor_types=("gpu_lidar", "imu", "rgbd_camera"),
        )
        checks, issues = run_static_checks(gz_desc, golden_for("bumperbot"))
        assert issues == [], issues
        # the rgbd claim follows the depth sensor through the dialect mapping
        assert "rgbd" in gz_desc.capabilities


# ----------------------------------------------------------------------
# Static golden checks
# ----------------------------------------------------------------------


class TestStaticChecks:
    def test_both_robots_pass(self):
        for rid, xml in (("bumperbot", _bumperbot_urdf()),
                         ("labbot", _labbot_urdf())):
            checks, issues = run_static_checks(
                parse_robot_description(xml), golden_for(rid))
            assert issues == [], (rid, issues)
            assert all(c.passed for c in checks)

    def test_tampered_wheel_radius_fails(self):
        xml = _bumperbot_urdf().replace(
            "<sphere radius='0.033'/>", "<sphere radius='0.05'/>", 1)
        checks, issues = run_static_checks(
            parse_robot_description(xml), golden_for("bumperbot"))
        assert issues
        assert any("wheel" in i.lower() for i in issues)

    def test_tampered_wheel_separation_fails(self):
        xml = _labbot_urdf().replace(
            "<origin xyz='0 0.15 0'", "<origin xyz='0 0.2 0'", 1)
        checks, issues = run_static_checks(
            parse_robot_description(xml), golden_for("labbot"))
        assert any("wheel" in i.lower() or "separation" in i.lower()
                   for i in issues)

    def test_missing_depth_fails_bumperbot(self):
        xml = _bumperbot_urdf(with_rgbd=False)
        checks, issues = run_static_checks(
            parse_robot_description(xml), golden_for("bumperbot"))
        assert any("sensor" in i.lower() for i in issues)

    def test_base_offset_mismatch_fails(self):
        xml = _labbot_urdf().replace("0 0 0.12", "0 0 0.2")
        checks, issues = run_static_checks(
            parse_robot_description(xml), golden_for("labbot"))
        assert any("offset" in i.lower() for i in issues)

    def test_mesh_footprint_qualifies_without_fabricated_radius(self):
        # The real bumperbot base collision is a mesh: no circular radius is
        # derived, the check passes and says so explicitly.
        desc = parse_robot_description(_bumperbot_urdf())
        mesh_desc = dataclasses.replace(
            desc, footprint={}, base_collision_kind="mesh")
        assert mesh_desc.footprint.get("radius", 0.0) == 0.0
        checks, issues = run_static_checks(mesh_desc, golden_for("bumperbot"))
        fp = next(c for c in checks if c.name == "footprint")
        assert fp.passed
        assert "mesh" in fp.detail
        assert issues == []

    def test_no_collision_footprint_fails(self):
        xml = _labbot_urdf().replace(
            "<collision><geometry><box size='0.30 0.30 0.24'/></geometry>"
            "</collision>", "")
        checks, issues = run_static_checks(
            parse_robot_description(xml), golden_for("labbot"))
        fp = next(c for c in checks if c.name == "footprint")
        assert not fp.passed

    def test_rgbd_claim_follows_depth_hardware(self):
        # capabilities are derived, so a false claim is impossible by
        # construction: labbot (no depth) never claims rgbd, and adding a
        # depth sensor to the description is what produces the claim
        desc = parse_robot_description(_labbot_urdf())
        assert "rgbd" not in desc.capabilities
        with_depth = dataclasses.replace(
            desc, sensor_types=desc.sensor_types + ("rgbd_camera",))
        assert "rgbd" in with_depth.capabilities


# ----------------------------------------------------------------------
# Task-level qualification (injected executor)
# ----------------------------------------------------------------------


class TestTaskQualification:
    def test_all_stages_pass_qualifies(self):
        for rid, xml in (("bumperbot", _bumperbot_urdf()),
                         ("labbot", _labbot_urdf())):
            report = qualify_robot(
                rid, parse_robot_description(xml), _all_stages_pass)
            assert report.passed, report.issues
            assert not report.issues

    def test_failed_stage_blocks_pass(self):
        def failing(robot_id, spec):
            stages = _all_stages_pass(robot_id, spec)
            stages[1] = TaskStageResult("drive", False, "stalled at 0.4 m")
            return stages

        report = qualify_robot(
            "bumperbot", parse_robot_description(_bumperbot_urdf()), failing)
        assert not report.passed
        assert any("drive" in i for i in report.issues)

    def test_no_executor_is_honestly_not_a_pass(self):
        report = qualify_robot(
            "bumperbot", parse_robot_description(_bumperbot_urdf()), None)
        assert report.passed is False
        assert any("no executor" in i for i in report.issues)

    def test_no_executor_still_records_static_checks(self):
        report = qualify_robot(
            "labbot", parse_robot_description(_labbot_urdf()), None)
        assert report.checks  # static checks ran
        assert all(c.passed for c in report.checks)

    def test_drift_reported_separately(self):
        observation = TaskTrialObservation(
            truth_trajectory=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            odom_trajectory=((0.0, 0.0, 0.0), (0.9, 0.2, 0.05)),
            min_clearance_m=0.3,
            contacts=0,
        )
        report = qualify_robot(
            "bumperbot", parse_robot_description(_bumperbot_urdf()),
            _all_stages_pass, observation=observation)
        drift = next(c for c in report.checks if c.name == "odom_drift_reported")
        assert drift.passed
        assert "0.2236" in drift.detail  # hypot(0.1, 0.2)

    def test_drift_summary_none_without_trajectories(self):
        assert TaskTrialObservation().drift_summary() == {"odom_drift_m": None}

    def test_missing_drift_observation_blocks_pass(self):
        report = qualify_robot(
            "bumperbot", parse_robot_description(_bumperbot_urdf()),
            _all_stages_pass,
            observation=TaskTrialObservation(min_clearance_m=0.3))
        assert not report.passed
        assert any("drift" in i.lower() for i in report.issues)

    def test_spec_defaults_are_predeclared(self):
        spec = TaskTrialSpec()
        assert spec.drive_tolerance_m > 0
        assert spec.turn_tolerance_deg > 0
        assert spec.min_clearance_m > 0
        assert spec.stage_timeout_seconds > 0


# ----------------------------------------------------------------------
# Report output
# ----------------------------------------------------------------------


class TestReportOutput:
    def test_report_json_roundtrip(self, tmp_path):
        report = qualify_robot(
            "bumperbot", parse_robot_description(_bumperbot_urdf()),
            _all_stages_pass)
        path = write_qualification_report(report, tmp_path / "q.json")
        data = json.loads(path.read_text())
        assert data["robot_id"] == "bumperbot"
        assert data["passed"] is True
        assert data["checks"]
        assert data["issues"] == []

    def test_failed_report_json_retains_issues(self, tmp_path):
        report = qualify_robot(
            "labbot", parse_robot_description(_labbot_urdf()), None)
        path = write_qualification_report(report, tmp_path / "q.json")
        data = json.loads(path.read_text())
        assert data["passed"] is False
        assert any("no executor" in i for i in data["issues"])

    def test_golden_covers_both_robots(self):
        assert set(GOLDEN) == {"bumperbot", "labbot"}

    def test_golden_separation_matches_measured(self):
        g = GOLDEN["bumperbot"]
        assert math.isclose(g.wheel_separation, 0.1402203698837279, abs_tol=1e-9)
        assert math.isclose(GOLDEN["labbot"].wheel_separation, 0.30, abs_tol=1e-9)

    def test_labbot_golden_is_not_rgbd_capable(self):
        assert GOLDEN["labbot"].rgbd_capable is False
        assert "depth" not in GOLDEN["labbot"].required_sensor_types
