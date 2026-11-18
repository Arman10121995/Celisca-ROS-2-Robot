# R3.6: Kinova Jaco 2 controllable with lab servos (November 18, 2026)

**Status: implemented, experimental.** It passes the arm screen on MuJoCo.
On PyBullet, five of six joints pass; joint 4 does not. Display-only
profiles drop from 65 to 64.

## What changed

**Continuous joints with a declared source range.** robot_assets' Kinova
(`j2s6s200`) has three continuous joints, and each declares ±2π in its
`<limit>`. Until now, `urdf_lab_servos` and the articulation contract
refused every continuous joint.

- Now a continuous joint becomes a position channel only when its source
  `<limit>` declares a finite lower < upper
  (`urdf_lab_servos.has_source_range`).
- Fetch's and PR2's continuous joints declare no range and stay refused;
  `display_reasons` counts them as before.

**Joints whose range excludes zero spawn at their home.** Plants spawn
every joint at 0. Kinova's joints 2, 3 and 5 start their ranges at 0.82,
0.33 and 0.52 rad.

- At 0 the plant starts out of bounds. The contract latched "Measured
  articulation outside source bounds" and refused every command (screen 2,
  kept).
- `URDFArticulation.spawn_positions` lists exactly those joints with their
  clamped home. The MuJoCo and PyBullet spawners put them there at spawn
  and on reset.
- Every other joint still spawns at 0, so TIAGo, Panda and the other arms
  are unchanged.

**Overlay** (`installed_overlays.yaml`, `asset_kinova`):

- the same lab-servo rule as UR5/KUKA (kp 40 × effort, kd 8 × √effort,
  ≤ 0.35 rad/s);
- `fixed_base: true` (its mounting block has no `world` anchor);
- lab home: the middle of each limited joint's range (π, the straight-up
  pose) and 0 on the continuous joints.

## Measured (`scripts/screen_urdf_arm.py`, Display, `nav_empty`)

| Run | Backend | Result | Worst steady error | Stop | Lost heartbeat |
|---|---|---|---|---|---|
| 1 | MuJoCo | failed: launch refused (contract rejected continuous joints) | — | — | — |
| 2 | MuJoCo | failed: spawned out of range, every command rejected | — | — | — |
| **3** | MuJoCo | **passed** | 0.028 rad (joint 3) | holds | holds |
| 1 | PyBullet | joint 4 fails; others pass | 0.082 rad (joint 4) | holds | holds |
| 2 (traced) | PyBullet | joint 4 fails; others pass | 0.082 rad (joint 4) | holds | holds |

**PyBullet joint 4** (forearm roll, 20 N m), from the trace:

- it lags the rate-limited reference by up to 0.25 rad;
- it carries on past the 0.5 rad target to 0.58 and stays there;
- the same happens on the way back (−0.085).

A bare PyBullet script with the same URDF, motor settings and 240 Hz step
converges without overshoot, so the bridge path differs somewhere; not
found yet. Bullet gives the inertia-less end-effector link a default
1 kg m² inertia, a possible cause, not confirmed.

## Limits

- One screen per backend after the fixes; Isaac and Gazebo are not run.
- The Jaco's fingers are fixed in this URDF, so there is no gripper.
- Joint jogging only: no pick and place.
