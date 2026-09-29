#!/usr/bin/env python3
"""Which contact tips the get-up ladder's tuck entry, from traced trials.

Each ``fall_ladder_*trace*`` trial directory is read from its ``manifest.json``
and ``probe.json``. These are the only ladder trials recorded with
``trace_joints`` on and a 0.02 s trace interval, so all twelve joints'
commanded effort, position and velocity, all four foot forces and the signed
trunk roll exist at ~50 Hz around the engagement. Three things are compared:

- the two delayed 60 N runs whose tuck engages on the settled side-rest
  (the tipping case, one ending inverted and one stopping mid-roll);
- the matched null with recovery disabled (the same rest, no engagement);
- the placed 1.4 rad pitch run where the same command rights the trunk
  (the success control).

Attribution is at *foot* granularity: ``/go2/foot_contact_forces`` is the
finest contact telemetry this harness publishes (four values, no per-link
wrenches), so "which joint's ground reaction tips it" resolves to which foot
struck the ground and what that leg's joints were doing on the same sample.
Commanded efforts are PD outputs, not measured torques, and during the sweep
every joint is clamp- or damping-limited at once, so the strike cannot be
apportioned between the three joints of the striking leg from these records.
"""
from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

#: Foot order of ``/go2/foot_contact_forces`` (probe_stance.py contact callback).
FOOT_ORDER = ("FL", "FR", "RL", "RR")
#: Joint names of the effort command topic (probe_stance efforts callback).
JOINT_NAMES = tuple(
    f"{leg}_{kind}_joint" for leg in FOOT_ORDER for kind in ("hip", "thigh", "calf"))
#: Rest window before the first ``attempting:*`` transition.
REST_WINDOW_S = 0.3
#: Engagement window in which the strike is searched.
STRIKE_WINDOW_S = 0.15
#: Window after the strike in which the last ground touch is looked for; the
#: tipping trials show a small follow-up contact (tens of N) inside this span
#: before all four feet read 0.0 N for the rest of the motion.
KICK_WINDOW_S = 0.2
#: A foot is "free" when every force reads at or below this after the strike;
#: the free-roll samples read exactly 0.0 N in both tipping trials.
UNLOAD_N = 1.0
#: Window after the engagement in which the trunk's roll extreme is measured;
#: both tipping trials reach their extreme inside it (one reaching pi).
OUTCOME_WINDOW_S = 1.5
#: A tuck-window contact at or above this classifies the run as tipping; it
#: separates the 151-198 N strikes of the tipping runs from the <=70 N paired
#: front-foot ramp of the placed-pitch success control.
STRIKE_CLASSIFY_N = 100.0


def _attempt_start(transitions: list[dict]) -> float | None:
    return next((entry["sim_s"] for entry in transitions
                 if entry["state"].startswith("attempting")), None)


def _rate(before: dict, after: dict) -> float | None:
    """Signed roll rate (rad/s) between two trace samples."""
    dt = after["sim_s"] - before["sim_s"]
    if dt <= 0:
        return None
    return round((after["roll_rad"] - before["roll_rad"]) / dt, 2)


def _foot_median(samples: list[dict]) -> dict:
    return {foot: round(statistics.median(
        sample["foot_force_n"][foot] for sample in samples), 2)
        for foot in FOOT_ORDER}


def summarize_trial(trial_dir: Path) -> dict:
    """Extract the engagement/rest/outcome picture of one traced trial."""
    manifest = json.loads((trial_dir / "manifest.json").read_text())
    result = json.loads((trial_dir / "probe.json").read_text())
    trace = sorted(result.get("trace", []), key=lambda sample: sample["sim_s"])
    transitions = result.get("recovery_transitions", [])
    attempt_s = _attempt_start(transitions)
    final = trace[-1]
    summary = {
        "trial": trial_dir.name,
        "ros_domain_id": manifest.get("ros_domain_id"),
        "enable_fall_recovery": manifest.get("enable_fall_recovery"),
        "trace_joints": manifest.get("trace_joints"),
        "trace_interval_s": manifest.get("trace_interval_s"),
        "attempt_start_s": attempt_s,
        "terminal_recovery_state": (
            transitions[-1]["state"] if transitions else None),
        "final_roll_rad": final.get("roll_rad"),
        "final_height_m": final["xyz"][2],
        "trace_points": len(trace),
    }
    if attempt_s is None:
        settled = [sample for sample in trace
                   if sample["sim_s"] >= final["sim_s"] - 3.0]
        summary["settled"] = {
            "roll_rad_min": min(sample["roll_rad"] for sample in settled),
            "roll_rad_max": max(sample["roll_rad"] for sample in settled),
            "foot_force_median_n": _foot_median(settled),
            "foot_force_max_n": {foot: round(max(
                sample["foot_force_n"][foot] for sample in settled), 2)
                for foot in FOOT_ORDER},
            "max_effort_nm": max(sample["max_effort_nm"] for sample in settled),
        }
        return summary
    rest = [sample for sample in trace
            if attempt_s - REST_WINDOW_S <= sample["sim_s"] < attempt_s]
    window = [sample for sample in trace
              if attempt_s <= sample["sim_s"] <= attempt_s + STRIKE_WINDOW_S]
    if not rest or not window:
        summary["incomplete_trace"] = True
        return summary
    engage = window[0]
    strike_sample = max(
        window, key=lambda sample: max(sample["foot_force_n"].values()))
    strike_foot = max(strike_sample["foot_force_n"],
                      key=strike_sample["foot_force_n"].get)
    strike_index = window.index(strike_sample)
    before = window[strike_index - 1] if strike_index else rest[-1]
    kick_window = [sample for sample in trace
                   if strike_sample["sim_s"] < sample["sim_s"]
                   <= strike_sample["sim_s"] + KICK_WINDOW_S]
    last_contact = strike_sample
    for sample in kick_window:
        if max(sample["foot_force_n"].values()) > UNLOAD_N:
            last_contact = sample
    free = []
    for sample in trace:
        if sample["sim_s"] <= last_contact["sim_s"]:
            continue
        if max(sample["foot_force_n"].values()) > UNLOAD_N:
            break
        free.append(sample)
    extreme = max((sample for sample in trace
                   if attempt_s <= sample["sim_s"]
                   <= attempt_s + OUTCOME_WINDOW_S),
                  key=lambda sample: abs(sample["roll_rad"]))
    summary.update({
        "rest": {
            "roll_rad": rest[-1]["roll_rad"],
            "height_m": rest[-1]["xyz"][2],
            "foot_force_median_n": _foot_median(rest),
            "max_effort_nm": max(sample["max_effort_nm"] for sample in rest),
        },
        "strike": {
            "foot": strike_foot,
            "peak_force_n": round(strike_sample["foot_force_n"][strike_foot], 2),
            "second_foot_force_n": round(sorted(
                strike_sample["foot_force_n"].values())[-2], 2),
            "secondary_contact_peak_n": round(max(
                (max(sample["foot_force_n"].values()) for sample in kick_window),
                default=0.0), 2),
            "offset_after_engage_s": round(
                strike_sample["sim_s"] - attempt_s, 3),
            "roll_rate_before_rad_s": _rate(rest[-1], before),
            "roll_rate_at_strike_rad_s": _rate(before, strike_sample),
            "roll_rad_at_strike": strike_sample["roll_rad"],
            "height_m_at_strike": strike_sample["xyz"][2],
        },
        "post_strike_contact_free": {
            "last_contact_offset_s": round(
                last_contact["sim_s"] - attempt_s, 3),
            "span_s": (round(free[-1]["sim_s"] - last_contact["sim_s"], 3)
                       if free else 0.0),
            "roll_delta_rad": (round(free[-1]["roll_rad"]
                                     - last_contact["roll_rad"], 3)
                               if free else 0.0),
        },
        "roll_extreme": {
            "max_abs_roll_rad": round(abs(extreme["roll_rad"]), 3),
            "offset_after_engage_s": round(extreme["sim_s"] - attempt_s, 3),
        },
        "engage_effort_nm": engage.get("effort", {}),
        "striking_leg_effort_at_engage_nm": {
            kind: engage.get("effort", {}).get(f"{strike_foot}_{kind}_joint")
            for kind in ("hip", "thigh", "calf")},
        "peak_joint_velocity_rad_s": {
            joint: round(max(abs(sample["v"].get(joint, 0.0))
                             for sample in window), 2)
            for joint in JOINT_NAMES},
        "tuck_window_peak_foot_force_n": {
            foot: round(max(sample["foot_force_n"][foot]
                            for sample in window), 2)
            for foot in FOOT_ORDER},
    })
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, required=True,
        help="evidence directory holding fall_ladder_*trace* trial directories")
    parser.add_argument(
        "--out", type=Path, default=None,
        help="where to write the JSON summary (default: <root>/tuck_forces.json)")
    args = parser.parse_args()
    trials = []
    for path in sorted(args.root.glob("fall_ladder_*trace*")):
        manifest_path = path / "manifest.json"
        if not path.is_dir() or not manifest_path.exists() \
                or not (path / "probe.json").exists():
            continue
        if json.loads(manifest_path.read_text()).get("trace_joints"):
            trials.append(summarize_trial(path))
    flips = [trial for trial in trials
             if trial["attempt_start_s"] is not None
             and trial.get("strike") is not None
             and trial["strike"]["peak_force_n"] >= STRIKE_CLASSIFY_N]
    nulls = [trial for trial in trials if trial["attempt_start_s"] is None]
    summary = {
        "task": "R5.2 Go2 tuck-entry contact attribution analysis",
        "root": str(args.root.resolve()),
        "trial_count": len(trials),
        "trials": trials,
        "interpretation": {
            "tipping_trials": {
                trial["trial"]: {
                    "strike_foot": trial["strike"]["foot"],
                    "strike_peak_force_n": trial["strike"]["peak_force_n"],
                    "strike_offset_after_engage_s":
                        trial["strike"]["offset_after_engage_s"],
                    "roll_rate_before_rad_s":
                        trial["strike"]["roll_rate_before_rad_s"],
                    "roll_rate_at_strike_rad_s":
                        trial["strike"]["roll_rate_at_strike_rad_s"],
                    "contact_free_roll_span_s":
                        trial["post_strike_contact_free"]["span_s"],
                    "contact_free_roll_delta_rad":
                        trial["post_strike_contact_free"]["roll_delta_rad"],
                    "max_abs_roll_rad":
                        trial["roll_extreme"]["max_abs_roll_rad"],
                    "max_abs_roll_offset_s":
                        trial["roll_extreme"]["offset_after_engage_s"],
                    "terminal": trial["terminal_recovery_state"],
                }
                for trial in flips},
            "null_controls": {
                trial["trial"]: trial["settled"] for trial in nulls},
            "grain": (
                "Contact telemetry is per foot, not per joint: the tipping "
                "reaction is a single down-side foot strike while every joint "
                "is clamp- or damping-limited, so the strike cannot be "
                "apportioned between the striking leg's hip/thigh/calf from "
                "the command topic."),
            "limitations": [
                "Commanded efforts are PD outputs at ~0.02 s sampling, not "
                "measured joint torques; per-link contact wrenches would be "
                "needed to split the strike between joints.",
                "The trunk-ground contact force is not published, so the "
                "trunk's own support during the flip is unmeasured.",
                "Run-to-run variation is real: the two tipping trials end "
                "differently (one inverted, one stopped mid-roll), so the "
                "shared claim is the strike pattern, not the outcome.",
            ],
        },
    }
    out_path = args.out or (args.root / "tuck_forces.json")
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    header = ("trial", "on", "attempt", "strike", "peakN", "offset",
              "free-span", "free-roll", "terminal")
    print(("%-46s %3s %8s %6s %6s %7s %9s %9s %s") % header)
    for trial in trials:
        strike = trial.get("strike") or {}
        free = trial.get("post_strike_contact_free") or {}
        print("%-46s %3s %8s %6s %6s %7s %9s %9s %s" % (
            trial["trial"],
            "yes" if trial["enable_fall_recovery"] else "no",
            trial["attempt_start_s"],
            strike.get("foot"),
            strike.get("peak_force_n"),
            strike.get("offset_after_engage_s"),
            free.get("span_s"),
            free.get("roll_delta_rad"),
            trial["terminal_recovery_state"]))
    print("summary written to %s" % out_path)


if __name__ == "__main__":
    main()

