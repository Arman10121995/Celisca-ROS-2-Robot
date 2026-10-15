# Actual multi-leg implementation artifacts

SSD root: `/workspace/molar/robot_lab_runtime/r106-gaits-2026-10-15/`.

- `gui-gait-smoke.py`: real `SimulationLauncherGui` command, native Gait
  ownership and actual latched Drive callbacks; bounded launch/owned cleanup.
- `octopod-gui-1/`: failed inherited string/numeric spawn arithmetic.
- `octopod-gui-2/`: real motion but superseded wrong trunk mass and incomplete
  cleanup observation; not source-mass acceptance.
- `octopod-gui-3/`: corrected 4.64 kg source model; actual stance, input and Stop.
- `hexapod-gui-1/`: actual licensed model stance, input and Stop.
- `centipede-gui-1/`: actual twelve-leg/five-spine body motion and Stop.
- `centipede-gui-2/`: repeat after advancing/matched timestamp and physical
  reset barriers. Each directory retains `report.json` and `gui-console.log`.
- `source-snapshots/`: hash-verified measured files, including reconstructed
  earlier controller/GUI bytes only where they match the recorded hashes.
- `gait-contracts.log`: original exact-mass assertion/serialization failure.
- `gait-contracts-final.log`: eight focused checks, separate from GUI missions.
- `final-build.log`, `publication-build.log`: affected package installation.
- `gui-publication.json`: final source-stance previews and GUI/default/command
  checks without another physical plant.
- `upstream/hexapod_ros2/`: pinned original checkout. Retained model assets and
  MIT license are installed in the repository; no user download UI is needed.

[Receipt](../../gaits-receipt-2026-10-15.json) records commands, per-run source
hashes, actual states/metrics and final-candidate differences. Large raw reports
stay on SSD. These diagnostics do not qualify terrain, independent estimation,
saved SLAM/navigation, all maps/backends or broad release acceptance.
