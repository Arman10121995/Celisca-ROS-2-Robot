# Actual R10 marine artifacts

The producer uses `SimulationLauncherGui`, its real generated command and
native Marine callbacks. It launches an actual Fortress plant and records
received `/marine/state`; no body trajectories or results are synthesized.

SSD root: `/workspace/molar/robot_lab_runtime/r104-marine-2026-10-15/`.

- `gui-marine-smoke.py`: exact bounded producer and owned cleanup.
- `tank-gui-1/`: failed inherited `auto` spawn conversion, retained unchanged.
- `tank-gui-2/{report.json,gui-console.log}`: actual second run, 227 samples,
  seven milestones, source hashes, command, final state and exit code.
- `source-snapshot-live/`: the six measured plant/controller/control files,
  SHA-256 checked against that actual report.
- `final-build.log`: affected-package installation; `final-contracts.log`
  records the seven focused checks separately from the live run.
- `gui-publication.json`: final GUI selection/default/autofill checks without
  starting another plant; not physical mission evidence.

The compact [receipt](../../marine-receipt-2026-10-15.json) carries real
milestones and metrics. Large raw logs stay on SSD. See
[scope and gaps](../../marine-2026-10-15.md); no full marine mission,
independent navigation or other-world/backend certificate is granted.
