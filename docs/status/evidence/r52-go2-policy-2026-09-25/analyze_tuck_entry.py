#!/usr/bin/env python3
"""What the get-up ladder's engagement commanded, from the recorded traces.

Each ``fall_ladder_*`` trial directory is read from its ``manifest.json`` and
``probe.json``. For every trial with a recovery attempt the summary reports the
*rest* the attempt engaged from (the last trace sample before the first
``attempting:*`` transition), the effort command sampled at engagement, and how
the trunk moved afterwards; trials without an attempt report the rest they held
instead. This is the comparison behind the open question about phase 1: the
discriminator between inversion and righting is the rest pose, not the size of
the engagement spike.

Two recording facts bound what this script can claim:

- the probe samples the effort command topic *instantaneously* every ~0.1 s
  (``probe_stance.py`` records ``latest_tau`` per sample), so a sampled
  engagement value is not a peak; the run-wide ``max_command_nm`` is a peak but
  carries no timestamp;
- ``trace_joints`` is off for every ladder trial, so only the RR calf angle is
  traced and ``tuck_calf_error_rad`` is the only per-joint waypoint error the
  records support.

The tuck calf target is duplicated from ``go2_locomotion.TUCK_POSE`` on purpose:
this script must run with no ROS and no installed adapter, and the test suite
pins the two values equal.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

#: ``TUCK_POSE["calf"]`` from go2_locomotion.py (see the test that pins it).
TUCK_CALF_TARGET_RAD = -2.70
#: A terminal tilt at or past this is the inverted rest the ladder ends on.
INVERTED_TILT_RAD = 3.0


def _sample_at(trace: list[dict], sim_s: float) -> dict | None:
    """Last trace sample at or before ``sim_s``."""
    last = None
    for sample in trace:
        if sample["sim_s"] <= sim_s:
            last = sample
    return last


def summarize_trial(trial_dir: Path) -> dict:
    """Extract rest/engagement/outcome of one trial directory."""
    manifest = json.loads((trial_dir / "manifest.json").read_text())
    result = json.loads((trial_dir / "probe.json").read_text())
    trace = sorted(result.get("trace", []), key=lambda sample: sample["sim_s"])
    transitions = result.get("recovery_transitions", [])
    attempt_s = next(
        (entry["sim_s"] for entry in transitions
         if entry["state"].startswith("attempting")), None)
    rest = engage = None
    if attempt_s is not None:
        before = [s for s in trace
                  if s["sim_s"] < attempt_s
                  and not (s.get("recovery_state") or "").startswith("attempting")]
        after = [s for s in trace if s["sim_s"] >= attempt_s]
        rest = before[-1] if before else None
        engage = after[0] if after else None
    elif trace:
        rest = trace[-1]
    waiting_efforts = [sample.get("max_effort_nm", 0.0) for sample in trace
                       if sample.get("recovery_state") == "waiting"]
    tilt_plus_04 = None
    if attempt_s is not None:
        sample = _sample_at(trace, attempt_s + 0.4)
        tilt_plus_04 = sample["tilt_rad"] if sample else None
    final = trace[-1] if trace else None
    rest_calf = rest.get("rr_calf_rad") if rest else None
    return {
        "trial": trial_dir.name,
        "enable_fall_recovery": manifest.get("enable_fall_recovery"),
        "force_n": manifest.get("force_n"),
        "perturbation_axis": manifest.get("perturbation_axis"),
        "spawn_pitch_rad": manifest.get("spawn_pitch_rad"),
        "spawn_roll_rad": manifest.get("spawn_roll_rad"),
        "attempt_start_s": attempt_s,
        "rest": None if rest is None else {
            "sim_s": rest["sim_s"],
            "tilt_rad": rest["tilt_rad"],
            "height_m": rest["xyz"][2],
            "rr_calf_rad": rest_calf,
            "sampled_effort_nm": rest.get("max_effort_nm"),
        },
        "waiting_sample_count": len(waiting_efforts),
        "waiting_max_sampled_effort_nm": max(waiting_efforts, default=None),
        "engage": None if engage is None else {
            "sim_s": engage["sim_s"],
            "tilt_rad": engage["tilt_rad"],
            "height_m": engage["xyz"][2],
            "rr_calf_rad": engage.get("rr_calf_rad"),
            "sampled_effort_nm": engage.get("max_effort_nm"),
        },
        "tuck_calf_error_rad": (
            None if rest_calf is None
            else round(TUCK_CALF_TARGET_RAD - rest_calf, 3)),
        "tilt_at_engage_plus_0_4_s": tilt_plus_04,
        "final_tilt_rad": final["tilt_rad"] if final else None,
        "final_height_m": final["xyz"][2] if final else None,
        "terminal_recovery_state": (
            transitions[-1]["state"] if transitions else None),
        "inverted": (final is not None
                     and final["tilt_rad"] >= INVERTED_TILT_RAD),
    }



def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, required=True,
        help="evidence directory holding fall_ladder_* trial directories")
    parser.add_argument(
        "--out", type=Path, default=None,
        help="where to write the JSON summary (default: <root>/tuck_entry.json)")
    args = parser.parse_args()
    trial_dirs = sorted(
        path for path in args.root.glob("fall_ladder_*") if path.is_dir())
    trials = [
        summarize_trial(path) for path in trial_dirs
        if (path / "manifest.json").exists() and (path / "probe.json").exists()]
    attempted = [trial for trial in trials if trial["attempt_start_s"] is not None]
    inverted = [trial for trial in attempted if trial["inverted"]]
    engaged_efforts = [
        trial["engage"]["sampled_effort_nm"] for trial in attempted
        if trial["engage"] is not None
        and trial["engage"]["sampled_effort_nm"] is not None]
    summary = {
        "task": "R5.2 Go2 fall-recovery ladder engagement analysis",
        "root": str(args.root.resolve()),
        "trial_count": len(trials),
        "attempted_count": len(attempted),
        "inverted_count": len(inverted),
        "trials": trials,
        "interpretation": {
            "engagement_sampled_effort_nm": {
                trial["trial"]: trial["engage"]["sampled_effort_nm"]
                for trial in attempted if trial["engage"] is not None},
            "engagement_sampled_effort_range_nm": (
                [min(engaged_efforts), max(engaged_efforts)]
                if engaged_efforts else []),
            "settled_rest_trials": {
                trial["trial"]: {
                    "rest_tilt_rad": trial["rest"]["tilt_rad"],
                    "rest_height_m": trial["rest"]["height_m"],
                    "final_tilt_rad": trial["final_tilt_rad"],
                    "final_height_m": trial["final_height_m"],
                    "terminal": trial["terminal_recovery_state"],
                }
                for trial in trials
                if trial["rest"] is not None
                and trial["rest"]["tilt_rad"] is not None
                and trial["rest"]["tilt_rad"] <= 0.6},
            "limitations": [
                "Sampled efforts are instantaneous command samples every "
                "~0.1 s, not peaks; clamp episodes between samples are invisible.",
                "Only the RR calf angle is traced (trace_joints is off for "
                "every ladder trial), so no other joint's waypoint error or "
                "effort can be attributed from these records.",
                "Rest pose differs between runs of the same configuration; "
                "compare rest fields, not trial names.",
            ],
        },
    }
    out_path = args.out or (args.root / "tuck_entry.json")
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    header = ("trial", "on", "rest tilt", "rest h", "rest calf",
              "engage s", "engage Nm", "tilt+0.4", "final tilt", "terminal")
    print(("%-46s %3s %9s %7s %9s %9s %10s %8s %10s %s") % header)
    for trial in trials:
        rest = trial["rest"] or {}
        engage = trial["engage"] or {}
        print("%-46s %3s %9s %7s %9s %9s %10s %8s %10s %s" % (
            trial["trial"],
            "yes" if trial["enable_fall_recovery"] else "no",
            rest.get("tilt_rad"),
            rest.get("height_m"),
            rest.get("rr_calf_rad"),
            engage.get("sim_s", trial["attempt_start_s"]),
            engage.get("sampled_effort_nm"),
            trial["tilt_at_engage_plus_0_4_s"],
            trial["final_tilt_rad"],
            trial["terminal_recovery_state"]))
    print("attempted %d, inverted %d" % (len(attempted), len(inverted)))
    print("summary written to %s" % out_path)


if __name__ == "__main__":
    main()
