#!/usr/bin/env python3
"""Run the fresh-PD timing diagnostic on the compiled common BHL plant."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent))
from probe_contact_parity import build_common  # noqa: E402
import probe_intra_interval_pd as intra  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--native-report", type=Path, required=True)
    args = parser.parse_args()
    common = build_common()

    def common_loader(_robot_dir, _policy):
        return None, common, mujoco.MjData(common)

    intra.load_model = common_loader
    common_result = {"model": "compiled common URDF + nav_empty",
                     "modes": [intra.run("policy_humanoid", fresh_pd=False),
                               intra.run("policy_humanoid", fresh_pd=True)]}
    native = json.loads(args.native_report.read_text())
    result = {"common": common_result, "native": native,
              "comparison_note": "Same policy, startup, commands, gains and 2 kHz physics; only compiled plant differs."}
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    for label, report in (("common", common_result), ("native", native)):
        for mode in report["modes"]:
            print(label, "fresh_pd=", mode["fresh_pd"],
                  "dyaw_8_to_13=", mode["windows"]["8-13"].get("dyaw"),
                  "spread_8_to_13=", mode["windows"]["8-13"].get("mean_effort_spread"))


if __name__ == "__main__":
    main()
