#!/usr/bin/env python3
"""What the ladder's *entry* phase is, and what it costs, from traced trials.

The entry used to be the fold-all tuck for every fallen trunk. For one measured
increment the entry was keyed on attitude instead (``go2_locomotion`` grew a
``recovery_entry_phase``: a roll-dominant trunk carrying less than 1.0 rad of
pitch started at the roll phase's braced push). That keying was recorded in the
``fall_ladder_brace_*`` trials and then refuted: on a settled side rest the
braced push is a 0.75-0.82 kN strike and a matched tuck entry is a 0.47-0.54 kN
strike, and *both* inverted 2 of 2, so the entry pose does not decide the
outcome. The code is back to the tuck entry.

This script reads the entry from each trial's own ``recovery_transitions``
record, so its grouping spans both eras and never depends on a trial's name.
Per trial (``manifest.json`` + ``probe.json``) it reports:

- the phase the attempt actually entered, and the sequence of ``attempting:*``
  phases it then ran;
- the rest it engaged from: signed roll and pitch, the roll *rate* over the
  ~0.1 s before the entry (both ends of that window pre-attempt, so the entry
  stroke cannot leak into the rate), and the per-foot load. The rate is the
  mechanism, not the pose: a trunk rolling at several rad/s is falling, one at
  a tenth of that is resting on its side, and both can read the same 0.52 rad
  of roll;
- the entry window's contact pattern: peak force per foot in the first 0.15 s
  and 0.4 s after the entry. The tuck's fold shows up as one down-side foot
  taking the whole load; the braced push drives the down-side *pair*;
- the outcome: whether the trunk's roll reached the inverted tilt inside 1.5 s
  of the entry, and the terminal state.

Two recording facts bound the claims. Trials recorded before 2026-09-29 have no
``trunk_force_n`` in their trace, because the trunk's own ground contact was not
published then; for those the trunk's support is unmeasured, and only the newer
traced runs (e.g. ``trunk_contact_20260929Troll0.9``) carry it. And
``trace_joints`` is off for the runs that predate the traced era, which limits
those runs to the 0.1 s trace (no per-foot force series) -- their entries are
still classified, with the contact fields reported as null.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

#: Foot order of ``/go2/foot_contact_forces`` (probe_stance.py contact callback).
FOOT_ORDER = ("FL", "FR", "RL", "RR")
#: A trunk rolling at or above this is falling, not resting on its side. The two
#: measured populations are ~0.0 rad/s (a settled rest) and 1.8-2.0 rad/s (the
#: ballistic 60 N family at its latch), far enough apart that the exact value
#: does not decide any measured trial.
ENTRY_MOVING_RAD_S = 0.5
#: Window over which the pre-entry roll rate is measured.
RATE_SPAN_S = 0.1
#: Window in which the entry's foot strike/load is searched (matches
#: analyze_tuck_forces.py, so both scripts classify the same transient).
STRIKE_WINDOW_S = 0.15
#: Longer window covering the whole tuck phase (the fold runs for 0.4 s).
FOLD_WINDOW_S = 0.4
#: Window after the entry in which the trunk's roll extreme is measured.
OUTCOME_WINDOW_S = 1.5
#: A foot is unloaded when every force in a window reads at or below this.
UNLOAD_N = 1.0
#: Roll magnitude at or past this is the inverted rest.
INVERTED_TILT_RAD = 3.0


def _samples(trace: list[dict]) -> list[dict]:
    return sorted(trace, key=lambda sample: sample["sim_s"])


def _attempt_start(transitions: list[dict]) -> float | None:
    return next((entry["sim_s"] for entry in transitions
                 if entry["state"].startswith("attempting")), None)


def _phase_sequence(transitions: list[dict]) -> list[str]:
    """Every ``attempting:*`` and terminal state the attempt ran, in order."""
    return [entry["state"] for entry in transitions
            if entry["state"].startswith(("attempting", "succeeded", "failed",
                                          "unrecoverable"))]


def _rate_before(samples: list[dict], sim_s: float,
                 span_s: float = RATE_SPAN_S) -> float | None:
    """Signed roll rate (rad/s) over ``span_s`` before the attempt began.

    Both ends of the window are pre-attempt: a sample taken once the entry
    stroke is already driving the legs measures the stroke, not the rest the
    attempt engaged from.
    """
    before = [s for s in samples
              if s["sim_s"] <= sim_s - span_s and s.get("roll_rad") is not None]
    at = [s for s in samples
          if s["sim_s"] <= sim_s and s.get("roll_rad") is not None
          and not (s.get("recovery_state") or "").startswith("attempting")]
    if not before or not at:
        return None
    start, end = before[-1], at[-1]
    dt = end["sim_s"] - start["sim_s"]
    if dt <= 0.0:
        return None
    return round((end["roll_rad"] - start["roll_rad"]) / dt, 2)


def _window_loads(samples: list[dict], start_s: float,
                  span_s: float) -> dict | None:
    """Peak per-foot force and the loaded feet inside one window."""
    window = [s for s in samples
              if start_s <= s["sim_s"] <= start_s + span_s
              and s.get("foot_force_n")]
    if not window:
        return None
    peaks = {foot: round(max(sample["foot_force_n"].get(foot, 0.0)
                             for sample in window), 2)
             for foot in FOOT_ORDER}
    loaded = [foot for foot in FOOT_ORDER if peaks[foot] > UNLOAD_N]
    return {"peak_force_n": peaks, "peak_sum_n": round(sum(peaks.values()), 2),
            "loaded_feet": loaded, "loaded_count": len(loaded)}


def _entry_rest(samples: list[dict], attempt_s: float) -> dict | None:
    """The last traced sample before the attempt's first phase transition."""
    before = [s for s in samples
              if s["sim_s"] < attempt_s
              and not (s.get("recovery_state") or "").startswith("attempting")]
    rest = before[-1] if before else None
    if rest is None:
        return None
    forces = rest.get("foot_force_n") or {}
    return {
        "sim_s": rest["sim_s"],
        "roll_rad": rest.get("roll_rad"),
        "pitch_rad": rest.get("pitch_rad"),
        "tilt_rad": rest.get("tilt_rad"),
        "foot_force_n": {foot: forces.get(foot) for foot in FOOT_ORDER},
        "sum_force_n": (round(sum(forces.get(foot, 0.0)
                                  for foot in FOOT_ORDER), 2)
                        if forces else None),
        "loaded_feet": ([foot for foot in FOOT_ORDER
                         if (forces.get(foot) or 0.0) > UNLOAD_N]
                        if forces else None),
        "roll_rate_rad_s": _rate_before(samples, attempt_s),
    }


def summarize_trial(trial_dir: Path) -> dict:
    """Extract the entry phase, the rest it engaged, and the outcome."""
    manifest = json.loads((trial_dir / "manifest.json").read_text())
    result = json.loads((trial_dir / "probe.json").read_text())
    samples = _samples(result.get("trace", []))
    transitions = result.get("recovery_transitions", [])
    attempt_s = _attempt_start(transitions)
    entry_phase = None
    for entry in transitions:
        if entry["state"].startswith("attempting"):
            entry_phase = entry["state"].split(":", 1)[1]
            break
    rest = _entry_rest(samples, attempt_s) if attempt_s is not None else None
    strike = fold = extreme = None
    if attempt_s is not None:
        strike = _window_loads(samples, attempt_s, STRIKE_WINDOW_S)
        fold = _window_loads(samples, attempt_s, FOLD_WINDOW_S)
        after = [s for s in samples
                 if attempt_s <= s["sim_s"] <= attempt_s + OUTCOME_WINDOW_S
                 and s.get("roll_rad") is not None]
        if after:
            peak = max(after, key=lambda s: abs(s["roll_rad"]))
            extreme = {
                "max_abs_roll_rad": round(abs(peak["roll_rad"]), 3),
                "max_abs_roll_offset_s": round(peak["sim_s"] - attempt_s, 3),
                "roll_at_window_end_rad": after[-1]["roll_rad"],
                "inverted_within_window":
                    abs(peak["roll_rad"]) >= INVERTED_TILT_RAD,
            }
    final = samples[-1] if samples else {}
    rate = rest["roll_rate_rad_s"] if rest else None
    return {
        "trial": trial_dir.name,
        "enable_fall_recovery": manifest.get("enable_fall_recovery"),
        "force_n": manifest.get("force_n"),
        "start_delay_s": manifest.get("fall_recovery_start_delay_s"),
        "spawn_pitch_rad": manifest.get("spawn_pitch_rad"),
        "spawn_roll_rad": manifest.get("spawn_roll_rad"),
        "traced": bool(manifest.get("trace_joints")),
        "attempt_start_s": attempt_s,
        "entry_phase": entry_phase,
        "phase_sequence": _phase_sequence(transitions),
        "rest": rest,
        "settled_entry": (None if rate is None
                          else abs(rate) < ENTRY_MOVING_RAD_S),
        "entry_strike": strike,
        "entry_fold": fold,
        "roll_extreme_after_entry": extreme,
        "terminal_recovery_state": result.get("final_recovery_state"),
        "max_tilt_rad": result.get("max_tilt_rad"),
        "final_height_m": result.get("final_height_m"),
        "final_roll_rad": final.get("roll_rad"),
        "final_xyz_z_m": (final.get("xyz")[2] if final.get("xyz") else None),
    }


def _outcome(trial: dict) -> dict:
    return {
        "entry_phase": trial["entry_phase"],
        "rest_roll_rad": trial["rest"]["roll_rad"] if trial["rest"] else None,
        "rest_roll_rate_rad_s":
            trial["rest"]["roll_rate_rad_s"] if trial["rest"] else None,
        "rest_sum_force_n": trial["rest"]["sum_force_n"] if trial["rest"] else None,
        "entry_strike_peak_n": (trial["entry_strike"] or {}).get("peak_force_n"),
        "entry_strike_peak_sum_n": (trial["entry_strike"] or {}).get("peak_sum_n"),
        "entry_fold_peak_sum_n": (trial["entry_fold"] or {}).get("peak_sum_n"),
        "roll_extreme_rad":
            (trial["roll_extreme_after_entry"] or {}).get("max_abs_roll_rad"),
        "inverted_within_1_5_s":
            (trial["roll_extreme_after_entry"] or {}).get("inverted_within_window"),
        "terminal": trial["terminal_recovery_state"],
        "final_height_m": trial["final_height_m"],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path,
                        default=Path(__file__).resolve().parent)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    trial_dirs = sorted(
        path for path in args.root.glob("fall_ladder_*") if path.is_dir())
    trials = [
        summarize_trial(path) for path in trial_dirs
        if (path / "manifest.json").exists() and (path / "probe.json").exists()]
    attempted = [t for t in trials if t["attempt_start_s"] is not None]
    by_phase: dict[str, list[dict]] = {}
    for trial in attempted:
        by_phase.setdefault(trial["entry_phase"] or "unknown", []).append(trial)
    settled = [t for t in attempted if t["settled_entry"]]
    moving = [t for t in attempted if t["settled_entry"] is False]
    summary = {
        "task": "R5.2 Go2 get-up ladder entry-phase analysis",
        "root": str(args.root.resolve()),
        "trial_count": len(trials),
        "attempted_count": len(attempted),
        "trials": trials,
        "interpretation": {
            "entry_phase_counts": {
                phase: len(group) for phase, group in sorted(by_phase.items())},
            "entries_from_a_settled_rest": {
                trial["trial"]: _outcome(trial) for trial in settled},
            "entries_already_moving": {
                trial["trial"]: _outcome(trial) for trial in moving},
            "moving_threshold_rad_s": ENTRY_MOVING_RAD_S,
            "grain": (
                "The entry phase is read from the recovery_transitions record "
                "and the rest's roll rate from the traced roll ~0.1 s before "
                "that transition: pose alone cannot tell a trunk resting on "
                "its side from one already rolling past it."),
            "limitations": [
                "Commanded efforts are PD outputs, not measured joint "
                "torques; per-link contact wrenches would be needed to "
                "apportion an entry's push between the joints of a leg.",
                "The trunk's own ground contact is published as "
                "/go2/trunk_contact_forces only from 2026-09-29; for every "
                "earlier trial the trunk's support during an entry is "
                "inferred from its roll rate, not measured.",
                "Trials recorded before the traced era sample at 0.1 s, so "
                "their rates are coarser and their contact fields are null.",
                "A trial where the perturbation never latched the fall "
                "detector has no entry at all; it is reported with an "
                "attempt_start_s of null, not as a successful entry.",
            ],
        },
    }
    out_path = args.out or (args.root / "entry_phase.json")
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    header = ("trial", "on", "entry", "rest roll", "rate", "settled", "peakN",
              "foldN", "extreme", "terminal")
    print(("%-46s %3s %6s %9s %7s %7s %8s %7s %8s %s") % header)
    for trial in trials:
        rest = trial["rest"] or {}
        strike = trial["entry_strike"] or {}
        fold = trial["entry_fold"] or {}
        extreme = trial["roll_extreme_after_entry"] or {}
        print("%-46s %3s %6s %9s %7s %7s %8s %7s %8s %s" % (
            trial["trial"],
            "yes" if trial["enable_fall_recovery"] else "no",
            trial["entry_phase"] or "-",
            rest.get("roll_rad", "-"),
            rest.get("roll_rate_rad_s", "-"),
            ("yes" if trial["settled_entry"] else "no")
            if trial["settled_entry"] is not None else "-",
            strike.get("peak_sum_n", "-"),
            fold.get("peak_sum_n", "-"),
            extreme.get("max_abs_roll_rad", "-"),
            trial["terminal_recovery_state"]))
    print("attempted %d of %d trials; entry phases %s"
          % (len(attempted), len(trials),
             {phase: len(group) for phase, group in sorted(by_phase.items())}))
    print("summary written to %s" % out_path)


if __name__ == "__main__":
    main()
