"""Hermetic contracts for the ladder-engagement analysis tooling."""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / "docs/status/evidence/r52-go2-policy-2026-09-25/analyze_tuck_entry.py"
EVIDENCE = ROOT / "docs/status/evidence/r52-go2-policy-2026-09-25"
_spec = importlib.util.spec_from_file_location("r52_tuck_entry_analyzer", ANALYZER)
assert _spec and _spec.loader
analyzer = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = analyzer
_spec.loader.exec_module(analyzer)


def _write_trial(root: Path, *, recovery: bool, trace: list[dict],
                 transitions: list[dict]) -> None:
    (root / "manifest.json").write_text(json.dumps(
        {"enable_fall_recovery": recovery, "force_n": 60,
         "perturbation_axis": "y"}))
    (root / "probe.json").write_text(json.dumps({
        "trace": trace, "recovery_transitions": transitions,
        "final_height_m": trace[-1]["xyz"][2]}))


def _sample(sim_s: float, tilt: float, height: float, effort: float,
            state: str, calf: float) -> dict:
    return {"sim_s": sim_s, "xyz": [0.0, 0.0, height], "tilt_rad": tilt,
            "recovery_state": state, "max_effort_nm": effort,
            "rr_calf_rad": calf}


def test_summarize_extracts_rest_engagement_and_inversion(tmp_path):
    _write_trial(tmp_path, recovery=True, trace=[
        _sample(3.85, 0.52, 0.139, 0.0, "waiting", -2.70),
        _sample(4.58, 0.52, 0.139, 0.0, "waiting", -1.49),
        _sample(4.69, 0.53, 0.139, 35.55, "attempting:tuck", -2.12),
        _sample(4.79, 1.50, 0.181, 2.83, "attempting:tuck", -2.69),
        _sample(5.08, 3.14, 0.057, 1.42, "attempting:tuck", -2.70),
        _sample(5.30, 3.14, 0.057, 1.42, "attempting:tuck", -2.70),
    ], transitions=[
        {"sim_s": 3.80, "state": "waiting"},
        {"sim_s": 4.68, "state": "attempting:tuck"},
        {"sim_s": 11.6, "state": "unrecoverable:tuck"},
    ])
    trial = analyzer.summarize_trial(tmp_path)
    assert trial["attempt_start_s"] == 4.68
    assert trial["rest"]["tilt_rad"] == 0.52
    assert trial["rest"]["height_m"] == 0.139
    assert trial["rest"]["rr_calf_rad"] == -1.49
    assert trial["waiting_max_sampled_effort_nm"] == 0.0
    assert trial["engage"]["sampled_effort_nm"] == 35.55
    assert trial["tuck_calf_error_rad"] == pytest.approx(-1.21)
    assert trial["tilt_at_engage_plus_0_4_s"] == 3.14
    assert trial["inverted"] is True
    assert trial["terminal_recovery_state"] == "unrecoverable:tuck"


def test_summarize_disabled_trial_never_attempts(tmp_path):
    _write_trial(tmp_path, recovery=False, trace=[
        _sample(5.0, 0.52, 0.139, 0.0, "disabled", -2.61),
        _sample(19.9, 0.52, 0.139, 0.0, "disabled", -2.61),
    ], transitions=[])
    trial = analyzer.summarize_trial(tmp_path)
    assert trial["attempt_start_s"] is None
    assert trial["engage"] is None
    assert trial["rest"]["sim_s"] == 19.9
    assert trial["inverted"] is False


def test_tuck_calf_target_matches_the_source_constant():
    from robot_lab_adapter.go2_locomotion import TUCK_POSE
    assert analyzer.TUCK_CALF_TARGET_RAD == TUCK_POSE["calf"]


def test_delayed_runs_hold_the_null_rest_then_invert():
    for name in ("fall_ladder_delayed_20260928Td1.0",
                 "fall_ladder_delayed_20260928Td1.0b",
                 "fall_ladder_delayed_20260928Td2.0"):
        trial = analyzer.summarize_trial(EVIDENCE / name)
        assert trial["enable_fall_recovery"] is True
        assert 0.51 <= trial["rest"]["tilt_rad"] <= 0.53
        assert 0.135 <= trial["rest"]["height_m"] <= 0.143
        assert trial["waiting_max_sampled_effort_nm"] == 0.0
        assert trial["final_tilt_rad"] >= 3.0
        assert trial["terminal_recovery_state"] == "unrecoverable:tuck"


def test_null_perturb_runs_settle_and_stay():
    for name in ("fall_ladder_null_perturb_20260928Tf50",
                 "fall_ladder_null_perturb_20260928Tf60",
                 "fall_ladder_null_perturb_20260928Tf60b",
                 "fall_ladder_null_perturb_20260928Tf60c"):
        trial = analyzer.summarize_trial(EVIDENCE / name)
        assert trial["enable_fall_recovery"] is False
        assert trial["attempt_start_s"] is None
        assert 0.51 <= trial["final_tilt_rad"] <= 0.53
        assert trial["final_height_m"] == pytest.approx(0.139, abs=0.005)


def test_sampled_spike_size_does_not_classify_the_outcome():
    """A clamped engagement sample appears in successes and failures alike."""
    inverted_small = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_20260928T50N")
    assert inverted_small["inverted"] is True
    assert inverted_small["engage"]["sampled_effort_nm"] <= 10.0
    succeeded_clamped = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_placed_20260928Tpitch1.4")
    assert succeeded_clamped["terminal_recovery_state"] == "succeeded:stand"
    assert succeeded_clamped["engage"]["sampled_effort_nm"] >= 35.0


def test_same_tuck_command_rights_a_pitched_rest():
    trial = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_placed_20260928Tpitch1.4")
    assert trial["rest"]["height_m"] >= 0.26
    assert trial["tilt_at_engage_plus_0_4_s"] < trial["rest"]["tilt_rad"] - 0.1
    assert trial["final_tilt_rad"] < 0.1
