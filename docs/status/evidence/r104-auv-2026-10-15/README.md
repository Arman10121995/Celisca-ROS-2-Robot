# Native AUV and marine convention artifacts

SSD root: `/workspace/molar/robot_lab_runtime/r104-auv-2026-10-15/`.

- `fuel-metadata.json`, `tethys-v3.zip`, `source-v3/`: actual primary model
  response/archive, SHA-256 and CC-BY source; originals also retained in repo.
- `gui-auv-smoke.py`: bounded real normal GUI producer, generated command,
  actual Marine latches/depth/turn/Stop and owned cleanup. Each run snapshots
  its actual source bytes before launching.
- `tank-gui-1/`, `tank-gui-2/`, `reef-gui-1/`: actual earlier measurements,
  source hashes and failed/incomplete guidance/cleanup diagnostics retained.
- `reef-gui-2/`: corrected positive added-mass native plant; actual source
  body/joints/propeller/fins and cruise depth/turn/Stop measurements.
- `rov-sign-correction/`: separate ROV GUI diagnostic after the same physical
  added-mass convention correction. The original ROV receipt is not relabeled.
- `LiftDrag-ign-gazebo6.cc`, `Hydrodynamics-ign-gazebo6.cc`: inspected primary
  implementation sources, not a newly built or patched Gazebo library.
- `contracts.log`: retained first semantic-comparison whitespace assertion
  failure; original source-byte hash still matched. `final-contracts.log`
  records focused source/force/water/geometry checks separately from missions.
- `marine-publication-build.log`, `gui-publication.json`: installation and
  actual final GUI/preview/command checks, without another qualification run.

[Receipt](../../auv-receipt-2026-10-15.json) records exact per-run sources,
commands, real measurements and publication differences. No independent
underwater navigation, all-map/backend, collision-free or full-release
acceptance is granted by these short implementation diagnostics.
