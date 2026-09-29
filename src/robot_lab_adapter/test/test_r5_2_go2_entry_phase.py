"""Hermetic contracts for the get-up ladder entry-phase tooling.

The tooling answers what the ladder's *first* phase was, on every recorded
ladder trial, from the trial's own ``recovery_transitions`` record -- so the
grouping spans both eras and never depends on a trial's name. It was written
for the pose-keyed entry (a roll-dominant trunk under 1.0 rad of pitch entered
at the braced push, the ``fall_ladder_brace_*`` runs), which was measured and
then reverted: on a matched settled side rest the braced push drives 0.75-0.82
kN against the fold's 0.47-0.54 kN and *both* entry poses inverted 2 of 2, so
the entry pose is not what decides the outcome. These tests pin the analyzer's
extraction and the measured claims the tooling exists to support.
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
ANALYZER = ROOT / "docs/status/evidence/r52-go2-policy-2026-09-25/analyze_entry_phase.py"
EVIDENCE = ROOT / "docs/status/evidence/r52-go2-policy-2026-09-25"
_spec = importlib.util.spec_from_file_location("r52_entry_phase_analyzer", ANALYZER)
assert _spec and _spec.loader
analyzer = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = analyzer
_spec.loader.exec_module(analyzer)


def _sample(sim_s: float, roll: float, *, roll_rate: float = 0.0,
            fl: float = 0.0, fr: float = 0.0, rl: float = 0.0, rr: float = 0.0,
            state: str = "waiting") -> dict:
    return {
        "sim_s": sim_s, "roll_rad": roll, "pitch_rad": 0.0,
        "tilt_rad": abs(roll), "xyz": [0.0, 0.0, 0.139],
        "recovery_state": state,
        "foot_force_n": {"FL": fl, "FR": fr, "RL": rl, "RR": rr},
    }


def _ramp(start_s: float, roll_at: float, rate: float, count: int = 11,
          step: float = 0.02, **kwargs) -> list[dict]:
    """Samples reaching ``roll_at`` on the last one, moving at ``rate`` rad/s.

    Samples are anchored on the *last* one and the earlier ones back-extrapolated
    from it, so the same rest attitude is reached both slowly (a trunk settled
    on its side) and quickly (one already rolling past it).
    """
    return [_sample(start_s + index * step,
                    roll_at + rate * (index - (count - 1)) * step, **kwargs)
            for index in range(count)]


def _write_trial(root: Path, *, trace: list[dict], transitions: list[dict],
                 recovery: bool = True, trace_joints: bool = True,
                 **manifest) -> None:
    (root / "manifest.json").write_text(json.dumps(
        {"enable_fall_recovery": recovery, "trace_joints": trace_joints}
        | manifest))
    (root / "probe.json").write_text(json.dumps({
        "trace": trace, "recovery_transitions": transitions,
        "final_recovery_state": transitions[-1]["state"] if transitions else None,
    }))


def test_summarize_reports_entry_phase_rest_rate_and_strike(tmp_path):
    _write_trial(tmp_path, trace=[
        # A settled rest: the pose is on its side and it is *not* moving.
        _sample(4.50, -0.52, fr=3.9, rl=4.9),
        _sample(4.60, -0.52, fr=3.9, rl=4.9),
        # The braced push: the down-side pair (roll < 0 -> FL/RL) strikes.
        _sample(4.64, -0.60, fl=205.0, rl=110.0, state="attempting:roll"),
        _sample(4.70, -0.95, state="attempting:roll"),
        # The trunk goes over inside the 1.5 s outcome window.
        _sample(6.00, -3.14, state="unrecoverable:roll"),
    ], transitions=[
        {"sim_s": 4.00, "state": "waiting"},
        {"sim_s": 4.64, "state": "attempting:roll"},
        {"sim_s": 5.20, "state": "unrecoverable:roll"},
    ])
    trial = analyzer.summarize_trial(tmp_path)
    assert trial["attempt_start_s"] == 4.64
    assert trial["entry_phase"] == "roll"
    assert trial["settled_entry"] is True
    assert trial["rest"]["roll_rad"] == pytest.approx(-0.52)
    assert trial["rest"]["roll_rate_rad_s"] == pytest.approx(0.0, abs=1e-6)
    assert trial["rest"]["loaded_feet"] == ["FR", "RL"]
    assert trial["entry_strike"]["peak_force_n"]["FL"] == pytest.approx(205.0)
    assert trial["entry_strike"]["loaded_feet"] == ["FL", "RL"]
    assert trial["roll_extreme_after_entry"]["inverted_within_window"] is True
    assert trial["terminal_recovery_state"] == "unrecoverable:roll"


def test_the_same_pose_at_speed_is_not_a_settled_rest(tmp_path):
    """Pose alone cannot tell a rest from a trunk already rolling past it."""
    slow = tmp_path / "slow"
    fast = tmp_path / "fast"
    for path, rate in ((slow, 0.05), (fast, -5.0)):
        path.mkdir()
        _write_trial(path, trace=_ramp(4.40, -0.52, rate) + [
            _sample(4.64, -0.60, state="attempting:tuck"),
            _sample(4.80, -1.20, state="attempting:tuck"),
        ], transitions=[
            {"sim_s": 4.00, "state": "waiting"},
            {"sim_s": 4.64, "state": "attempting:tuck"},
        ])
    settled = analyzer.summarize_trial(slow)
    rolling = analyzer.summarize_trial(fast)
    assert settled["rest"]["roll_rad"] == pytest.approx(
        rolling["rest"]["roll_rad"], abs=0.05)
    assert settled["settled_entry"] is True
    assert rolling["settled_entry"] is False
    assert abs(rolling["rest"]["roll_rate_rad_s"]) >= analyzer.ENTRY_MOVING_RAD_S


def test_the_entry_phase_is_never_guessed_from_the_trial_name(tmp_path):
    named_for_the_fold = tmp_path / "fall_ladder_tuck_20260929Troll0.9d1.0"
    named_for_the_fold.mkdir()
    _write_trial(named_for_the_fold, trace=[
        _sample(1.40, 0.512, rr=390.0, state="attempting:roll"),
        _sample(1.50, 0.90, state="attempting:roll"),
    ], transitions=[
        {"sim_s": 0.10, "state": "waiting"},
        {"sim_s": 1.40, "state": "attempting:roll"},
    ])
    trial = analyzer.summarize_trial(named_for_the_fold)
    assert trial["entry_phase"] == "roll"
    assert trial["phase_sequence"] == ["attempting:roll"]


def test_a_run_where_the_fall_never_latched_has_no_entry(tmp_path):
    _write_trial(tmp_path, trace=[
        _sample(3.00, 0.05), _sample(9.00, 0.02),
    ], transitions=[{"sim_s": 0.004, "state": "idle"}])
    trial = analyzer.summarize_trial(tmp_path)
    assert trial["attempt_start_s"] is None
    assert trial["entry_phase"] is None
    assert trial["rest"] is None
    assert trial["settled_entry"] is None


def test_the_matched_settled_rest_inverts_for_both_entry_poses():
    """The refutation: the entry pose is not what decides the outcome.

    Same harness -- a placed 0.9 rad roll with a 1.0 s start delay and no
    perturbation force, so the attempt begins from the *measured* rest -- two
    repeats each per entry pose, and identical manifests across the four runs.
    The braced push drives the pair the trunk rests on harder (0.75-0.82 kN
    against the fold's 0.47-0.54 kN) and both poses tip the trunk over.
    """
    braced = [analyzer.summarize_trial(EVIDENCE / name) for name in (
        "fall_ladder_brace_placed_20260929Troll0.9d1.0",
        "fall_ladder_brace_placed_20260929Troll0.9d1.0b")]
    fold = [analyzer.summarize_trial(EVIDENCE / name) for name in (
        "fall_ladder_tuck_placed_20260929Troll0.9d1.0",
        "fall_ladder_tuck_placed_20260929Troll0.9d1.0b")]
    assert [trial["entry_phase"] for trial in braced] == ["roll", "roll"]
    assert [trial["entry_phase"] for trial in fold] == ["tuck", "tuck"]
    # The comparison is only a comparison if the four runs share a harness.
    harness = {(trial["spawn_roll_rad"], trial["spawn_pitch_rad"],
                trial["start_delay_s"], trial["force_n"], trial["traced"])
               for trial in braced + fold}
    assert harness == {(0.9, 0.0, 1.0, 0.0, True)}
    for trial in braced + fold:
        assert trial["settled_entry"] is True
        assert trial["rest"]["roll_rad"] == pytest.approx(0.512, abs=0.005)
        assert trial["rest"]["roll_rate_rad_s"] == pytest.approx(0.0, abs=0.1)
        assert trial["roll_extreme_after_entry"]["inverted_within_window"] is True
        # Roll > 0: the trunk rests on the right pair, and both strokes drive it.
        peaks = trial["entry_strike"]["peak_force_n"]
        assert peaks["FR"] > 150.0 and peaks["RR"] > 150.0
    # Both are unrecoverable, each reported at the phase it gave up in: the
    # fold never got past attempting:tuck, the braced push past attempting:roll.
    assert [trial["terminal_recovery_state"] for trial in braced] == [
        "unrecoverable:roll", "unrecoverable:roll"]
    assert [trial["terminal_recovery_state"] for trial in fold] == [
        "unrecoverable:tuck", "unrecoverable:tuck"]
    assert all(trial["entry_fold"]["peak_sum_n"] >= 700.0 for trial in braced)
    assert all(400.0 <= trial["entry_fold"]["peak_sum_n"] <= 600.0
               for trial in fold)


def test_the_real_delayed_entry_is_the_same_strike_for_either_pose():
    """The real 60 N delayed case: the entry strike is a down-side strike of
    the same size whether the pose is the fold or the push, and only the fold's
    population owns a run that stopped instead of inverting."""
    braced = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_brace_delayed_20260929Td1.0")
    fold = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_delayed_trace_20260928Td1.0")
    assert (braced["entry_phase"], fold["entry_phase"]) == ("roll", "tuck")
    for trial in (braced, fold):
        assert trial["settled_entry"] is True
        assert trial["rest"]["roll_rad"] == pytest.approx(-0.52, abs=0.005)
    assert braced["entry_strike"]["peak_force_n"]["FL"] == pytest.approx(
        208.1, abs=1.0)
    assert fold["entry_strike"]["peak_force_n"]["FL"] == pytest.approx(
        198.2, abs=1.0)
    assert braced["roll_extreme_after_entry"]["inverted_within_window"] is True
    assert fold["roll_extreme_after_entry"]["inverted_within_window"] is False
    assert fold["roll_extreme_after_entry"]["max_abs_roll_rad"] == pytest.approx(
        1.72, abs=0.05)


def test_the_working_pitch_control_keeps_its_tuck_first_ladder():
    """A pitch-dominant trunk is not what the entry was ever keyed for, and the
    get-up it reaches is the one the recorded control measured."""
    trial = analyzer.summarize_trial(
        EVIDENCE / "fall_ladder_brace_placed_20260929Tpitch1.4")
    assert trial["entry_phase"] == "tuck"
    assert trial["phase_sequence"] == [
        "attempting:tuck", "attempting:roll", "attempting:tuck",
        "attempting:crouch", "attempting:stand", "succeeded:stand"]
    assert trial["final_height_m"] == pytest.approx(0.329, abs=0.005)
    assert trial["terminal_recovery_state"] == "succeeded:stand"


def test_the_ballistic_lateral_family_is_caught_moving():
    """60 N no-delay runs read a roll close to a settled rest's, with a measured
    roll rate of 1-2 rad/s -- no entry pose is a rest for this family."""
    for name in ("fall_ladder_brace_20260929Tf60",
                 "fall_ladder_brace_20260929Tf60b"):
        trial = analyzer.summarize_trial(EVIDENCE / name)
        assert trial["settled_entry"] is False
        assert abs(trial["rest"]["roll_rate_rad_s"]) >= analyzer.ENTRY_MOVING_RAD_S
        assert trial["terminal_recovery_state"] == "unrecoverable:roll"


def test_a_failed_recording_is_skipped_by_the_analyzer(tmp_path, monkeypatch):
    """Two repeats were lost to this host's CycloneDDS range (ROS domain 233
    and up: a multicast port out of range), and the harness kept their logs and
    returncodes -- with no ``probe.json``, so the analyzer must skip them
    instead of reading an empty result."""
    for name in ("fall_ladder_brace_delayed_20260929Td1.0c",
                 "fall_ladder_brace_delayed_20260929Td1.0d"):
        failed = EVIDENCE / name
        assert (failed / "returncodes.json").exists()
        assert not (failed / "probe.json").exists()
        assert "is out of range" in (failed / "probe.log").read_text()
        (tmp_path / name).symlink_to(failed)
    valid = tmp_path / "fall_ladder_synthetic"
    valid.mkdir()
    _write_trial(valid, trace=[_sample(1.0, 0.5, state="attempting:tuck")],
                 transitions=[{"sim_s": 0.5, "state": "waiting"},
                              {"sim_s": 1.0, "state": "attempting:tuck"}])
    out = tmp_path / "summary.json"
    monkeypatch.setattr(sys, "argv", [
        "analyze_entry_phase.py", "--root", str(tmp_path), "--out", str(out)])
    analyzer.main()
    summary = json.loads(out.read_text())
    assert summary["trial_count"] == 1
    assert [trial["trial"] for trial in summary["trials"]] == [
        "fall_ladder_synthetic"]
