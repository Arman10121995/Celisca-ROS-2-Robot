"""Hermetic contracts for the tuck-entry contact-attribution tooling."""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / "docs/status/evidence/r52-go2-policy-2026-09-25/analyze_tuck_forces.py"
EVIDENCE = ROOT / "docs/status/evidence/r52-go2-policy-2026-09-25"
_spec = importlib.util.spec_from_file_location("r52_tuck_forces_analyzer", ANALYZER)
assert _spec and _spec.loader
analyzer = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = analyzer
_spec.loader.exec_module(analyzer)

ENGAGE_S = 4.0
_CLAMP = {"FL_hip_joint": 23.7, "FL_thigh_joint": -23.7, "FL_calf_joint": -35.55}


def _sample(sim_s: float, roll: float, *, fl: float = 0.0, fr: float = 0.0,
            rl: float = 0.0, rr: float = 0.0, effort: dict | None = None,
            height: float = 0.139) -> dict:
    return {
        "sim_s": sim_s,
        "roll_rad": roll,
        "pitch_rad": 0.0,
        "xyz": [0.0, 0.0, height],
        "foot_force_n": {"FL": fl, "FR": fr, "RL": rl, "RR": rr},
        "max_effort_nm": max((abs(value) for value in (effort or {}).values()),
                             default=0.0),
        "effort": {name: 0.0 for name in analyzer.JOINT_NAMES} | (effort or {}),
        "v": {name: 0.0 for name in analyzer.JOINT_NAMES},
    }


def _write_trial(root: Path, *, recovery: bool, trace: list[dict],
                 transitions: list[dict], trace_joints: bool = True) -> None:
    (root / "manifest.json").write_text(json.dumps({
        "enable_fall_recovery": recovery, "ros_domain_id": 999,
        "trace_joints": trace_joints, "trace_interval_s": 0.02}))
    (root / "probe.json").write_text(json.dumps({
        "trace": trace, "recovery_transitions": transitions}))


def test_summarize_reports_unilateral_strike_and_free_roll(tmp_path):
    """One down-side foot strikes, then the trunk rolls with every foot free."""
    _write_trial(tmp_path, recovery=True, trace=[
        _sample(3.70, -0.520, fl=2.4, fr=4.1, rl=2.2),
        _sample(3.98, -0.520, fl=2.4, fr=4.1, rl=2.2),
        _sample(4.00, -0.520, effort=_CLAMP),
        _sample(4.02, -0.600, effort=_CLAMP),
        _sample(4.04, -0.700, fl=150.0, effort=_CLAMP),
        _sample(4.06, -0.800),
        _sample(4.30, -2.000),
        _sample(8.00, -2.000),
    ], transitions=[
        {"sim_s": 3.80, "state": "waiting"},
        {"sim_s": 4.00, "state": "attempting:tuck"},
        {"sim_s": 11.0, "state": "unrecoverable:tuck"},
    ])
    trial = analyzer.summarize_trial(tmp_path)
    assert trial["attempt_start_s"] == 4.00
    assert trial["rest"]["roll_rad"] == pytest.approx(-0.52)
    assert trial["rest"]["foot_force_median_n"] == {
        "FL": 2.4, "FR": 4.1, "RL": 2.2, "RR": 0.0}
    assert trial["rest"]["max_effort_nm"] == 0.0
    assert trial["strike"]["foot"] == "FL"
    assert trial["strike"]["peak_force_n"] == pytest.approx(150.0)
    assert trial["strike"]["second_foot_force_n"] == 0.0
    assert trial["strike"]["offset_after_engage_s"] == pytest.approx(0.04)
    assert trial["strike"]["roll_rate_before_rad_s"] == pytest.approx(-2.0)
    assert trial["strike"]["roll_rate_at_strike_rad_s"] == pytest.approx(-5.0)
    assert trial["striking_leg_effort_at_engage_nm"] == {
        "hip": 23.7, "thigh": -23.7, "calf": -35.55}
    free = trial["post_strike_contact_free"]
    assert free["span_s"] == pytest.approx(3.96)
    assert free["roll_delta_rad"] == pytest.approx(-1.3)
    assert trial["roll_extreme"]["max_abs_roll_rad"] == pytest.approx(2.0)
    assert trial["roll_extreme"]["offset_after_engage_s"] == pytest.approx(0.3)
    assert trial["terminal_recovery_state"] == "unrecoverable:tuck"


def test_summarize_reports_settled_rest_without_engagement(tmp_path):
    """A disabled trial settles and reports medians instead of a strike."""
    _write_trial(tmp_path, recovery=False, trace=[
        _sample(4.00, -0.519, fl=9.2, fr=1.3, rl=9.1, rr=1.4),
        _sample(6.50, -0.519, fl=9.2, fr=1.3, rl=9.1, rr=1.4),
        _sample(8.00, -0.519, fl=9.2, fr=1.3, rl=9.1, rr=1.4),
    ], transitions=[{"sim_s": 0.008, "state": "disabled"}])
    trial = analyzer.summarize_trial(tmp_path)
    assert trial["attempt_start_s"] is None
    assert "strike" not in trial
    assert trial["settled"]["roll_rad_min"] == pytest.approx(-0.519)
    assert trial["settled"]["roll_rad_max"] == pytest.approx(-0.519)
    assert trial["settled"]["foot_force_median_n"] == {
        "FL": 9.2, "FR": 1.3, "RL": 9.1, "RR": 1.4}
    assert trial["settled"]["max_effort_nm"] == 0.0
    assert trial["final_height_m"] == pytest.approx(0.139)


def test_continuous_contact_leaves_no_contact_free_span(tmp_path):
    """While a foot stays loaded there is no unloaded stretch to roll through."""
    _write_trial(tmp_path, recovery=True, trace=[
        _sample(3.70, 0.000),
        _sample(3.98, 0.000),
        _sample(4.00, 0.000),
        _sample(4.06, 0.000, fl=50.0, fr=50.0),
        _sample(4.12, 0.000, fl=60.0, fr=60.0),
        _sample(4.20, 0.000, fl=60.0, fr=60.0),
        _sample(4.30, 0.000, fl=55.0, fr=55.0),
        _sample(8.00, 0.000, fl=40.0, fr=40.0, height=0.33),
    ], transitions=[
        {"sim_s": 0.10, "state": "idle"},
        {"sim_s": 4.00, "state": "attempting:tuck"},
        {"sim_s": 8.00, "state": "succeeded:stand"},
    ])
    trial = analyzer.summarize_trial(tmp_path)
    assert trial["strike"]["foot"] in ("FL", "FR")
    assert trial["strike"]["second_foot_force_n"] >= 40.0
    assert trial["tuck_window_peak_foot_force_n"]["RL"] == 0.0
    assert trial["post_strike_contact_free"] == {
        "last_contact_offset_s": 0.3, "span_s": 0.0, "roll_delta_rad": 0.0}
    assert trial["terminal_recovery_state"] == "succeeded:stand"


def test_delayed_traced_runs_tip_on_a_single_down_side_foot_strike():
    """Both traced tipping runs: one loaded foot strikes, second foot idle."""
    for name, foot in (("fall_ladder_delayed_trace_20260928Td1.0", "FL"),
                       ("fall_ladder_delayed_trace_20260928Td1.0b", "RL")):
        trial = analyzer.summarize_trial(EVIDENCE / name)
        assert trial["enable_fall_recovery"] is True
        assert trial["trace_interval_s"] == 0.02
        assert trial["attempt_start_s"] is not None
        assert 0.51 <= -trial["rest"]["roll_rad"] <= 0.53
        assert trial["rest"]["max_effort_nm"] == 0.0
        strike = trial["strike"]
        assert strike["foot"] == foot
        assert strike["peak_force_n"] >= analyzer.STRIKE_CLASSIFY_N
        assert strike["second_foot_force_n"] <= 1.0
        assert strike["offset_after_engage_s"] <= 0.1
        assert (strike["roll_rate_at_strike_rad_s"]
                < strike["roll_rate_before_rad_s"])
        efforts = trial["striking_leg_effort_at_engage_nm"]
        assert all(abs(efforts[kind]) >= 20.0
                   for kind in ("hip", "thigh", "calf"))
        free = trial["post_strike_contact_free"]
        assert free["span_s"] >= 0.3
        assert free["roll_delta_rad"] <= -0.7
        assert trial["roll_extreme"]["max_abs_roll_rad"] >= 1.7


def test_delayed_traced_outcomes_differ_but_share_the_strike_pattern():
    rolled_back = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_delayed_trace_20260928Td1.0")
    inverted = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_delayed_trace_20260928Td1.0b")
    assert rolled_back["terminal_recovery_state"] == "failed:roll"
    assert abs(rolled_back["final_roll_rad"]) <= 0.6
    assert rolled_back["roll_extreme"]["max_abs_roll_rad"] == pytest.approx(
        1.72, abs=0.05)
    assert inverted["terminal_recovery_state"] == "unrecoverable:tuck"
    assert inverted["final_roll_rad"] <= -3.0
    assert inverted["roll_extreme"]["max_abs_roll_rad"] == pytest.approx(
        3.142, abs=0.01)


def test_traced_null_holds_the_rest_without_engagement():
    trial = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_null_trace_20260928Tf60")
    assert trial["enable_fall_recovery"] is False
    assert trial["attempt_start_s"] is None
    assert trial["final_height_m"] == pytest.approx(0.139, abs=0.005)
    settled = trial["settled"]
    assert settled["roll_rad_max"] - settled["roll_rad_min"] <= 0.002
    assert all(value <= 15.0
               for value in settled["foot_force_median_n"].values())
    assert settled["max_effort_nm"] == 0.0


def test_traced_success_control_loads_both_front_feet_and_rights():
    trial = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_placed_trace_20260928Tpitch1.4")
    assert trial["attempt_start_s"] == pytest.approx(0.1)
    peaks = trial["tuck_window_peak_foot_force_n"]
    assert peaks["FL"] >= 40.0 and peaks["FR"] >= 40.0
    assert peaks["RL"] <= 1.0 and peaks["RR"] <= 1.0
    assert trial["roll_extreme"]["max_abs_roll_rad"] <= 0.01
    assert abs(trial["post_strike_contact_free"]["roll_delta_rad"]) <= 0.01
    assert trial["terminal_recovery_state"] == "succeeded:stand"
    assert trial["final_height_m"] >= 0.30


def test_strike_threshold_separates_tipping_from_righting():
    tipping = [analyzer.summarize_trial(EVIDENCE / name) for name in (
        "fall_ladder_delayed_trace_20260928Td1.0",
        "fall_ladder_delayed_trace_20260928Td1.0b")]
    success = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_placed_trace_20260928Tpitch1.4")
    assert all(trial["strike"]["peak_force_n"] >= analyzer.STRIKE_CLASSIFY_N
               for trial in tipping)
    assert success["strike"]["peak_force_n"] < analyzer.STRIKE_CLASSIFY_N
