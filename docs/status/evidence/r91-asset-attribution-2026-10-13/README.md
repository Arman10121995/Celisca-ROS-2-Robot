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

## Still differing or unverified (left as declared, for an owner to decide)

| Entry | Declared | Found | Why it is open |
|---|---|---|---|
| assem12ros_29 | MIT | BSD-3-Clause (Unitree's license at the Awesome-URDFs root) | The wychar_labs model has no license of its own; the collection's root license is Unitree's |
| berkeley_humanoid_lite | BSD-3-Clause | Apache-2.0 (the lab package manifest) | The upstream BHL license file is not vendored |
| quadrotor_sitl | GPL-3.0 | Apache-2.0 (the lab package manifest) | The source of the model's GPL claim is not in the tree |
| h1 | BSD-3-Clause | - | Its registry asset path does not resolve |
| px4_x500 | BSD-3-Clause | - | No asset paths in the registry (PX4 provides the model at runtime) |

## Also checked

- All 26 environments agree: Apache-2.0, the lab's own worlds.
- The three policies have license evidence:
  - rl_sar: Apache-2.0 file;
  - go2_recovery_nju: MIT file;
  - go2_velocity_flat: the adapter package's Apache-2.0 manifest.

  Their registry entries carry no license field.
