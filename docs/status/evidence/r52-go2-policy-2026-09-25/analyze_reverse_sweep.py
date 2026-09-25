#!/usr/bin/env python3
"""Summarize matched Go2 reverse-command sweep artifacts."""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_trials(manifest_path: Path) -> list[dict]:
    manifest = json.loads(manifest_path.read_text())
    root = manifest_path.parent
    trials = []
    for meta_path in sorted(root.glob("*.meta.json")):
        meta = json.loads(meta_path.read_text())
        result_path = Path(meta["probe_result"])
        if not result_path.is_absolute():
            result_path = (root / result_path).resolve()
        if not result_path.exists():
            trials.append({"meta": meta, "error": "missing probe result"})
            continue
        result = json.loads(result_path.read_text())
        motion = result.get("motion", {})
        command = float(meta["command_vx_mps"])
        window = float(manifest["drive_end_s"]) - float(manifest["drive_start_s"])
        delta_x = float(motion.get("drive_delta_x_m", 0.0))
        observed_vx = delta_x / window if window > 0 else None
        ratio = observed_vx / command if command and observed_vx is not None else None
        trials.append({
            "command_vx_mps": command,
            "reverse_command_map": meta.get("reverse_command_map", "feedforward"),
            "ros_domain_id": meta["ros_domain_id"],
            "probe_returncode": meta["probe_returncode"],
            "launch_returncode": meta["launch_returncode"],
            "sim_duration_s": result.get("sim_duration_s"),
            "drive_delta_x_m": delta_x,
            "observed_vx_mps": observed_vx,
            "tracking_ratio": ratio,
            "yaw_change_rad": result.get("yaw_change_rad"),
            "stop_delta_xy_m": motion.get("stop_delta_xy_m"),
            "max_tilt_rad": result.get("max_tilt_rad"),
            "xy_drift_m": result.get("xy_drift_m"),
            "max_command_nm": result.get("max_command_nm"),
            "effort_messages": result.get("effort_messages"),
            "contact_messages": result.get("contact_messages"),
            "peak_foot_force_n": result.get("peak_foot_force_n"),
            "probe_result": str(result_path),
            "probe_log": meta["probe_log"],
            "launch_log": meta["launch_log"],
        })
    manifest["trials"] = trials
    return trials


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    trials = load_trials(args.manifest)
    successful = [trial for trial in trials if "error" not in trial]
    summary = {
        "task": "R5.2 Go2 reverse-calibration sweep",
        "reverse_command_map": trials[0].get("reverse_command_map", "feedforward") if trials else None,
        "source_manifest": str(args.manifest.resolve()),
        "trial_count": len(trials),
        "complete_count": len(successful),
        "trials": trials,
        "interpretation": {
            "dead_zone_candidates": [
                trial["command_vx_mps"] for trial in successful
                if abs(trial["drive_delta_x_m"]) < 0.10],
            "reverse_motion_commands": [
                trial["command_vx_mps"] for trial in successful
                if trial["drive_delta_x_m"] < -0.10],
            "safety_or_launch_failures": [
                trial for trial in trials
                if "error" in trial or trial.get("probe_returncode", 1) != 0
                or trial.get("launch_returncode", 1) != 0],
            "limitations": [
                "A displacement threshold is a screening metric, not a velocity-tracking qualification.",
                "Each command is run sequentially with a distinct ROS domain; repeat seeds remain required.",
                "Terrain, fall recovery and navigation are outside this sweep.",
            ],
        },
    }
    args.out.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({
        "trial_count": summary["trial_count"],
        "complete_count": summary["complete_count"],
        "dead_zone_candidates": summary["interpretation"]["dead_zone_candidates"],
        "reverse_motion_commands": summary["interpretation"]["reverse_motion_commands"],
    }, indent=2))


if __name__ == "__main__":
    main()
