"""Hermetic contracts for R5.2 reverse-sweep and flat-ground evidence tooling."""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / "docs/status/evidence/r52-go2-policy-2026-09-25/analyze_reverse_sweep.py"
SUITE_ANALYZER = ROOT / "docs/status/evidence/r52-go2-policy-2026-09-25/analyze_flat_ground_suite.py"
_spec = importlib.util.spec_from_file_location("r52_reverse_sweep_analyzer", ANALYZER)
assert _spec and _spec.loader
analyzer = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = analyzer
_spec.loader.exec_module(analyzer)
suite_spec = importlib.util.spec_from_file_location("r52_flat_ground_analyzer", SUITE_ANALYZER)
assert suite_spec and suite_spec.loader
suite_analyzer = importlib.util.module_from_spec(suite_spec)
sys.modules[suite_spec.name] = suite_analyzer
suite_spec.loader.exec_module(suite_analyzer)


def _write_trial(root: Path, command: float, delta_x: float, domain: int,
                 reverse_map: str = "feedforward") -> None:
    result = {
        "sim_duration_s": 7.0,
        "motion": {"drive_delta_x_m": delta_x, "stop_delta_xy_m": 0.04},
        "max_tilt_rad": 0.08,
        "xy_drift_m": 0.6,
        "max_command_nm": 20.0,
        "effort_messages": 100,
        "contact_messages": 80,
        "peak_foot_force_n": {"FL": 10.0, "FR": 2.0, "RL": 10.0, "RR": 2.0},
        "yaw_change_rad": 0.2,
    }
    (root / "trial.json").write_text(json.dumps(result))
    meta = {
        "command_vx_mps": command,
        "reverse_command_map": reverse_map,
        "ros_domain_id": domain,
        "probe_result": "trial.json",
        "probe_log": "trial.probe.log",
        "launch_log": "trial.launch.log",
        "probe_returncode": 0,
        "launch_returncode": 0,
    }
    (root / "trial.meta.json").write_text(json.dumps(meta))


def test_analyzer_reports_observed_velocity_and_screening_interpretation(tmp_path):
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({"drive_start_s": 1.0, "drive_end_s": 4.0}))
    _write_trial(tmp_path, -0.25, -0.3, 181, reverse_map="inverse")
    trials = analyzer.load_trials(manifest)
    assert len(trials) == 1
    assert trials[0]["observed_vx_mps"] == pytest.approx(-0.1)
    assert trials[0]["tracking_ratio"] == pytest.approx(0.4)
    assert trials[0]["contact_messages"] == 80
    assert trials[0]["reverse_command_map"] == "inverse"


def test_analyzer_flags_missing_result_without_crashing(tmp_path):
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({"drive_start_s": 1.0, "drive_end_s": 4.0}))
    meta = {
        "command_vx_mps": -0.25,
        "ros_domain_id": 181,
        "probe_result": "missing.json",
        "probe_log": "missing.probe.log",
        "launch_log": "missing.launch.log",
        "probe_returncode": 1,
        "launch_returncode": 1,
    }
    (tmp_path / "missing.meta.json").write_text(json.dumps(meta))
    trials = analyzer.load_trials(manifest)
    assert trials[0]["error"] == "missing probe result"


def _write_suite_trial(root: Path, case: str, vx: float, wz: float,
                       drop: bool, domain: int, drive_x: float,
                       drive_yaw: float, stop_xy: float, stop_yaw: float) -> None:
    result = {
        "sim_duration_s": 7.0,
        "motion": {
            "drive_delta_x_m": drive_x,
            "drive_delta_yaw_rad": drive_yaw,
            "stop_delta_xy_m": stop_xy,
            "stop_delta_yaw_rad": stop_yaw,
        },
        "max_tilt_rad": 0.08,
        "xy_drift_m": 0.2,
        "max_command_nm": 20.0,
        "effort_messages": 100,
        "contact_messages": 80,
        "peak_foot_force_n": {"FL": 10.0, "FR": 2.0, "RL": 10.0, "RR": 2.0},
    }
    (root / f"{case}.json").write_text(json.dumps(result))
    meta = {
        "case": case,
        "command_vx_mps": vx,
        "command_wz_radps": wz,
        "drop_command_after_drive": drop,
        "reverse_command_map": "inverse",
        "ros_domain_id": domain,
        "probe_result": f"{case}.json",
        "probe_log": f"{case}.probe.log",
        "launch_log": f"{case}.launch.log",
        "probe_returncode": 0,
        "launch_returncode": 0,
    }
    (root / f"{case}.meta.json").write_text(json.dumps(meta))


def test_flat_ground_analyzer_reports_cases_and_command_loss(tmp_path):
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({
        "drive_start_s": 1.0,
        "drive_end_s": 4.0,
        "cases": [
            {"case": "forward_stop"},
            {"case": "forward_command_loss"},
            {"case": "zero_settle"},
        ],
    }))
    _write_suite_trial(tmp_path, "forward_stop", 0.25, 0.0, False, 211, 0.8, 0.0, 0.04, 0.0)
    _write_suite_trial(tmp_path, "forward_command_loss", 0.25, 0.0, True, 214, 0.7, 0.0, 0.05, 0.0)
    _write_suite_trial(tmp_path, "zero_settle", 0.0, 0.0, False, 215, 0.01, 0.0, 0.01, 0.0)
    trials = suite_analyzer._load_trials(manifest)
    assert [trial["case"] for trial in trials] == [
        "forward_stop", "forward_command_loss", "zero_settle"]
    assert trials[0]["screening_pass"] is True
    assert trials[1]["drop_command_after_drive"] is True
    assert trials[1]["screening_pass"] is True
    assert trials[2]["screening_pass"] is True
