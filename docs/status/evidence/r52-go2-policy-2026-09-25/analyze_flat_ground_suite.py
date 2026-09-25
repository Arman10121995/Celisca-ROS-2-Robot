#!/usr/bin/env python3
"""Summarize a matched Go2 flat-ground forward/reverse/turn suite."""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def _load_trials(manifest_path: Path) -> list[dict]:
    manifest = json.loads(manifest_path.read_text())
    root = manifest_path.parent
    trials = []
    meta_by_case = {}
    for meta_path in root.glob("*.meta.json"):
        meta = json.loads(meta_path.read_text())
        meta_by_case[meta.get("case")] = (meta_path, meta)
    ordered = []
    for case in manifest.get("cases", []):
        name = case.get("case") if isinstance(case, dict) else None
        if name in meta_by_case:
            ordered.append(meta_by_case.pop(name))
    ordered.extend(sorted(meta_by_case.values(), key=lambda item: item[0].name))
    for meta_path, meta in ordered:
        result_path = Path(meta["probe_result"])
        if not result_path.is_absolute():
            result_path = (root / result_path).resolve()
        if not result_path.exists():
            trials.append({"case": meta.get("case"), "error": "missing probe result"})
            continue
        result = json.loads(result_path.read_text())
        motion = result.get("motion", {})
        window = float(manifest["drive_end_s"]) - float(manifest["drive_start_s"])
        drive_x = float(motion.get("drive_delta_x_m", 0.0))
        drive_yaw = float(motion.get("drive_delta_yaw_rad", 0.0))
        command_x = float(meta.get("command_vx_mps", 0.0))
        command_yaw = float(meta.get("command_wz_radps", 0.0))
        observed_vx = drive_x / window if window > 0 else None
        observed_wz = drive_yaw / window if window > 0 else None
        trial = {
            "case": meta["case"],
            "command_vx_mps": command_x,
            "command_wz_radps": command_yaw,
            "drop_command_after_drive": bool(meta.get("drop_command_after_drive")),
            "reverse_command_map": meta.get("reverse_command_map"),
            "ros_domain_id": meta["ros_domain_id"],
            "probe_returncode": meta["probe_returncode"],
            "launch_returncode": meta["launch_returncode"],
            "sim_duration_s": result.get("sim_duration_s"),
            "drive_delta_x_m": drive_x,
            "drive_delta_yaw_rad": drive_yaw,
            "observed_vx_mps": observed_vx,
            "observed_wz_radps": observed_wz,
            "stop_delta_xy_m": motion.get("stop_delta_xy_m"),
            "stop_delta_yaw_rad": motion.get("stop_delta_yaw_rad"),
            "max_tilt_rad": result.get("max_tilt_rad"),
            "xy_drift_m": result.get("xy_drift_m"),
            "max_command_nm": result.get("max_command_nm"),
            "effort_messages": result.get("effort_messages"),
            "contact_messages": result.get("contact_messages"),
            "peak_foot_force_n": result.get("peak_foot_force_n"),
            "probe_result": str(result_path),
            "probe_log": meta["probe_log"],
            "launch_log": meta["launch_log"],
        }
        if meta["case"] == "forward_stop":
            trial["screening_pass"] = drive_x > 0.30 and abs(float(motion.get("stop_delta_xy_m", 99.0))) < 0.15
        elif meta["case"] == "reverse_stop":
            trial["screening_pass"] = drive_x < -0.30 and abs(float(motion.get("stop_delta_xy_m", 99.0))) < 0.15
        elif meta["case"] == "turn_stop":
            trial["screening_pass"] = abs(drive_yaw) > 0.50 and abs(float(motion.get("stop_delta_yaw_rad", 99.0))) < 0.20
        elif meta["case"] == "forward_command_loss":
            trial["screening_pass"] = drive_x > 0.30 and abs(float(motion.get("stop_delta_xy_m", 99.0))) < 0.20
        else:
            trial["screening_pass"] = abs(drive_x) < 0.10 and abs(drive_yaw) < 0.10
        trials.append(trial)
    return trials


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    trials = _load_trials(args.manifest)
    summary = {
        "task": "R5.2 Go2 flat-ground forward/reverse/turn suite",
        "source_manifest": str(args.manifest.resolve()),
        "trial_count": len(trials),
        "complete_count": sum("error" not in trial for trial in trials),
        "screening_pass_count": sum(bool(trial.get("screening_pass")) for trial in trials),
        "trials": trials,
        "interpretation": {
            "screening_is_not_qualification": True,
            "limitations": [
                "Screening thresholds are bounded diagnostics, not acceptance limits.",
                "The suite uses one sequential trial per case; repeat seeds remain required.",
                "Terrain, fall recovery and navigation are outside this suite.",
                "A command-loss case proves bounded settling after publisher loss, not recovery from a fall.",
            ],
        },
    }
    args.out.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({
        "trial_count": summary["trial_count"],
        "complete_count": summary["complete_count"],
        "screening_pass_count": summary["screening_pass_count"],
        "cases": {trial["case"]: trial.get("screening_pass") for trial in trials},
    }, indent=2))


if __name__ == "__main__":
    main()
