# R9.1: asset attribution check (2026-10-13)

`scripts/asset_attribution.py` checks the declared license of each
registry asset against the license evidence in the tree:

- robots and environments (`robot_lab_registry` config);
- the provisioned policies.

It finds each asset's files and walks up to the nearest
`LICENSE`/`COPYING`, then identifies the license from the file's text.
Without such a file it falls back to the owning package's `package.xml`.

The table is in [attribution.md](attribution.md) and the rows in
`attribution.json`.

## Fixed

**Unitree descriptions were declared MIT; they are BSD-3-Clause.** This
covered 14 registry robots (A1, Aliengo, Aliengo-Z1, B1, B2, B2W, G1, Go1,
Go2, Go2W, H1, H1-2, Laikago, Z1).

- The only license shipped with the Unitree models
  (`_upstream/Awesome-URDFs/LICENSE`) is the BSD 3-Clause License of
  Unitree Robotics.
- The registry now says BSD-3-Clause.
- The copies under `src/robot_lab_robots/unitree/` had no license file at
  all; BSD-3-Clause requires the notice to travel with redistributed
  source. The license is now vendored there, with a NOTICE.md explaining
  it.

## Resolved later the same day

The five entries first left open are settled from the evidence in the
tree. The checker now reports robots 25 agree, 0 differ, 1 unverified.

| Entry | Was declared | Now | Evidence |
|---|---|---|---|
| assem12ros_29 | MIT | BSD-3-Clause | Its own package.xml says BSD (an exporter template, author "TODO"); the collection's only license file is Unitree's BSD-3-Clause. Nothing supports MIT. The original author is unknown. |
| berkeley_humanoid_lite | BSD-3-Clause | **CC-BY-SA-4.0** | The lab's meshes, MJCF, USD and SCAD are byte-identical to Berkeley-Humanoid-Lite-Assets, and its two URDFs are modified copies. That repository is CC BY-SA 4.0 (its LICENCE); the BHL *software* is MIT. The repository field pointed at an unrelated project. The LICENCE and a NOTICE.md are now vendored in `src/robot_lab_robots/berkeley_humanoid_lite/`. ShareAlike applies to redistributed adaptations. |
| quadrotor_sitl | GPL-3.0 | Apache-2.0 | The description is first-party and mesh-free (its own header says so). GPL-3.0 is ArduPilot's license; ArduPilot SITL runs as a separate process. |
| h1 | BSD-3-Clause | BSD-3-Clause | The registry path pointed at a missing file; it now points at the H1 URDF in Awesome-URDFs, under Unitree's BSD-3-Clause. |
| px4_x500 | BSD-3-Clause | BSD-3-Clause (unverified in-tree) | PX4 provides the model at runtime from its checkout outside the workspace. `/workspace/molar/px4/PX4-Autopilot/Tools/simulation/gz/LICENSE` is BSD 3-Clause; the registry comment records it. |

The checker gained the CC BY-SA 4.0 pattern. The first table is kept in
git history (`e68e0fc`).

## Also checked

- All 26 environments agree: Apache-2.0, the lab's own worlds.
- The three policies have license evidence:
  - rl_sar: Apache-2.0 file;
  - go2_recovery_nju: MIT file;
  - go2_velocity_flat: the adapter package's Apache-2.0 manifest.

  Their registry entries carry no license field.
