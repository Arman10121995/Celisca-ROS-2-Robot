#!/usr/bin/env python3
"""Summarize matched Go2 fall-recovery ladder trials.

Each trial directory written by ``run_perturbation_trial.sh`` is read from its
``manifest.json`` and ``probe.json``. The summary reports what the *measured*
trace shows: whether the fall latch ever fired, at what tilt and tilt rate, how
far the get-up ladder advanced, and whether the robot ended up. Nothing is
inferred from the process staying alive.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

#: Ladder phases in the order ``FallRecovery`` enters them.
LADDER_PHASES = ("tuck", "roll", "crouch", "stand")


def _tilt_at(trace: list[dict], sim_s: float) -> float | None:
    """Tilt at ``sim_s``, linearly interpolated between trace samples."""
    before = None
    for sample in trace:
        if sample["sim_s"] <= sim_s:
            before = sample
            continue
        if before is None:
            return sample["tilt_rad"]
        dt = sample["sim_s"] - before["sim_s"]
        if dt <= 0:
            return sample["tilt_rad"]
        frac = (sim_s - before["sim_s"]) / dt
        return before["tilt_rad"] + frac * (sample["tilt_rad"] - before["tilt_rad"])
    return before["tilt_rad"] if before else None


def _sample_at(trace: list[dict], sim_s: float) -> dict | None:
    """Last trace sample at or before ``sim_s``."""
    last = None
    for sample in trace:
        if sample["sim_s"] <= sim_s:
            last = sample
    return last


def _tilt_rate(trace: list[dict], sim_s: float, window_s: float = 0.2) -> float | None:
    """Mean tilt rate (rad/s) over the ``window_s`` after ``sim_s``.

    A slope across the two samples that bracket an instant is not usable on a
    tumbling body: the trunk's tilt oscillates as it bounces, so a 0.1 s
    bracket can report a *reducing* tilt while the body is in fact rolling over.
    Averaging over a fixed window gives a rate that matches the observed
    motion.
    """
    start = _tilt_at(trace, sim_s)
    end = _tilt_at(trace, sim_s + window_s)
    if start is None or end is None:
        return None
    return (end - start) / window_s


def summarize_trial(trial_dir: Path) -> dict:
    """Extract the measured ladder behaviour of one trial directory."""
    manifest = json.loads((trial_dir / "manifest.json").read_text())
    result = json.loads((trial_dir / "probe.json").read_text())
    returncodes = json.loads((trial_dir / "returncodes.json").read_text())
    trace = result.get("trace", [])
    transitions = result.get("recovery_transitions", [])
    latches = [
        entry["sim_s"] for entry in result.get("fallen_transitions", [])
        if entry.get("fallen")]
    latch_s = min(latches) if latches else None
    phases = [entry["state"] for entry in transitions if entry["state"] != "idle"]
    terminal = transitions[-1]["state"] if transitions else "idle"
    final_tilt = trace[-1]["tilt_rad"] if trace else None
    final_height = result.get("final_height_m")
    latch_sample = _sample_at(trace, latch_s) if latch_s else None
    if not latches:
        verdict = "no_fall"
    elif terminal.split(":")[0] in ("succeeded", "recovered"):
        verdict = "recovered"
    elif final_tilt is not None and final_tilt < 0.7:
        verdict = "failed"
    else:
        verdict = "unrecoverable"
    return {
        "trial": trial_dir.name,
        "force_n": manifest.get("force_n"),
        "perturbation_axis": manifest.get("perturbation_axis"),
        "pulse_s": manifest.get("pulse_s"),
        "spawn_z_m": manifest.get("spawn_z_m"),
        "spawn_pitch_rad": manifest.get("spawn_pitch_rad"),
        "spawn_roll_rad": manifest.get("spawn_roll_rad"),
        "duration_s": manifest.get("duration_s"),
        "enable_fall_recovery": manifest.get("enable_fall_recovery"),
        "fall_recovery_timeout_s": manifest.get("fall_recovery_timeout_s"),
        "probe_returncode": returncodes["probe_returncode"],
        "launch_returncode": returncodes["launch_returncode"],
        "fallen_latched": bool(latches),
        "fallen_latch_s": latch_s,
        "tilt_at_latch_rad": _tilt_at(trace, latch_s) if latch_s else None,
        "height_at_latch_m": latch_sample["xyz"][2] if latch_sample else None,
        "tilt_rate_at_latch_rad_s": _tilt_rate(trace, latch_s) if latch_s else None,
        "max_tilt_rad": result.get("max_tilt_rad"),
        "final_tilt_rad": final_tilt,
        "min_height_m": result.get("min_height_m"),
        "final_height_m": result.get("final_height_m"),
        "safe_stop_seen": result.get("perturbation", {}).get("safe_stop_seen"),
        "final_safety_state": result.get("final_safety_state"),
        "ladder_transitions": [
            {"sim_s": entry["sim_s"], "state": entry["state"]}
            for entry in transitions],
        "phases_reached": phases,
        "reached_stand": any(label.endswith("stand") for label in phases),
        "terminal_recovery_state": terminal,
        "verdict": verdict,
        "probe_result": str((trial_dir / "probe.json").resolve()),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, required=True,
        help="evidence directory holding fall_ladder_* trial directories")
    parser.add_argument(
        "--out", type=Path, default=None,
        help="where to write the JSON summary (default: <root>/ladder_sweep.json)")
    args = parser.parse_args()
    trial_dirs = sorted(
        path for path in args.root.glob("fall_ladder_*") if path.is_dir())
    trials = [
        summarize_trial(path) for path in trial_dirs
        if (path / "manifest.json").exists()
        and (path / "probe.json").exists()
        and (path / "returncodes.json").exists()]
    trials.sort(key=lambda trial: (trial["force_n"] or 0.0))
    toppled = [trial for trial in trials if trial["fallen_latched"]]
    quiet = [trial for trial in trials if not trial["fallen_latched"]]
    summary = {
        "task": "R5.2 Go2 fall-recovery ladder sweep",
        "root": str(args.root.resolve()),
        "trial_count": len(trials),
        "trials": trials,
        "interpretation": {
            "no_fall_forces_n": [trial["force_n"] for trial in quiet],
            "toppled_forces_n": [trial["force_n"] for trial in toppled],
            "tilt_rate_at_latch_rad_s": {
                trial["trial"]: trial["tilt_rate_at_latch_rad_s"]
                for trial in toppled},
            "reached_stand": [
                trial["trial"] for trial in trials if trial["reached_stand"]],
            "ended_standing": [
                trial["trial"] for trial in trials
                if trial["final_tilt_rad"] is not None
                and trial["final_tilt_rad"] < 0.35
                and trial["final_height_m"] is not None
                and trial["final_height_m"] >= 0.25],
            "verdicts": {trial["trial"]: trial["verdict"] for trial in trials},
            "limitations": [
                "One lateral axis (body y) and one pulse width per force; other "
                "axes and widths are not covered.",
                "The ladder is engaged only after the debounced fall latch, so a "
                "trial can only exercise it while the robot is already down.",
                "A force that does not latch ``fallen`` says nothing about the "
                "ladder; it is a control showing the ladder is not self-triggering.",
            ],
        },
    }
    out_path = args.out or (args.root / "ladder_sweep.json")
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    for trial in trials:
        print(json.dumps({
            "trial": trial["trial"],
            "force_n": trial["force_n"],
            "spawn_pitch_rad": trial["spawn_pitch_rad"],
            "spawn_roll_rad": trial["spawn_roll_rad"],
            "fallen_latch_s": trial["fallen_latch_s"],
            "tilt_at_latch_rad": trial["tilt_at_latch_rad"],
            "tilt_rate_at_latch_rad_s": trial["tilt_rate_at_latch_rad_s"],
            "phases_reached": trial["phases_reached"],
            "final_tilt_rad": trial["final_tilt_rad"],
            "final_height_m": trial["final_height_m"],
            "verdict": trial["verdict"],
        }))
    print("summary written to %s" % out_path)


if __name__ == "__main__":
    main()
