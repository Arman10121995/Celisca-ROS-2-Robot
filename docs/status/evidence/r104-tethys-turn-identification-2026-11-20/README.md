# R10.4: why the ported Tethys turns 30 % slower than Gazebo (November 20, 2026)

**Status: identified.** The ported Tethys plant (MuJoCo, PyBullet, Isaac)
applies added mass as Fossen does: it adds to the hull's inertia.
Fortress 6.18 applies the opposite sign to the pinned SDF's added-mass
values, and so *removes* most of the hull's sway and yaw inertia. Steady
turn rates agree: about 0.045 rad/s ported and 0.050 rad/s in Gazebo,
over the last 8 s of the probe turn. The difference is the spin-up:
Gazebo turns 0.163 rad in the first 4 s, the port 0.046. The port's default
stays Fossen-correct.

## What Fortress executes

Read from gz-sim `ignition-gazebo6_6.18.0`, `src/systems/hydrodynamics/`
(Apache-2.0):

- `kTotalWrench = -Ma·ν̇ - Cmat·ν + D·ν`, with `Cmat = buildCoriolisMatrix(Ma, ν)`.
  That function returns −C_A (Fossen 2011, eq. 6.43), and ν̇ is a backward
  finite difference.
- The applied wrench is `-kTotalWrench`, which is **+Ma·ν̇** − C_A(ν)·ν − D·ν,
  rotated to the world frame.
- `Ma` is the SDF values as written (`xDotU`, …). The pinned Tethys
  `model.sdf` gives them **positive**: 4.998, 129.483, 129.483, 0, 34.296,
  34.296. With Fortress's +Ma·ν̇, the effective inertias are:

  | Axis | Rigid (port's summed body) | Port (Fossen, + added) | Fortress (− added) |
  |---|---|---|---|
  | surge mass | 152.1 kg | 157.1 kg | 147.1 kg |
  | sway / heave mass | 152.1 kg | 281.5 kg | **22.6 kg** |
  | pitch / yaw inertia | 42.6 kg·m² | 76.9 kg·m² | **8.3 kg·m²** |

- The Coriolis term is Fossen's −C_A(ν)·ν, the port's existing `fossen`
  option.

## Measured (`r104` `auv_probe.py`, `marine_tank`, 1.5 m deep; final depth controller of 11-20)

| Plant | Turn phase (25 s) | Notes |
|---|---|---|
| Gazebo (reference) | 1.188 rad | constant rate from the start (0.163 rad in 4 s) |
| MuJoCo `off` (default) | 0.865 rad | slow spin-up (0.046 rad in 4 s), then ~0.045 rad/s |
| MuJoCo `fossen` (−C_A ν, Fossen added mass) | 0.958 rad, then **envelope exceeded** (dived 0.8 m) | as on 11-19; the new depth hold does not stop it |
| MuJoCo `flipped` | −0.209 rad (turns the wrong way) | as on 11-19 |
| MuJoCo `fortress` (new, +Ma·ν̇ and −C_A ν, as Gazebo executes) | **diverged** 7.6 s into the turn (QACC/QVEL NaN) | the lagged +Ma·ν̇ with 20 % effective inertia is unstable at the 2 ms plant step |

`marine_tethys_plant.py` keeps `fortress` as a labelled diagnostic option
(`tethys_coriolis:=fortress`), with this explanation in the source. The
default (`off`) and its measured probes are unchanged; a test pins the
signs.

## Consequence

- The port is the physically consistent model, since added mass makes a
  body harder to accelerate. The Gazebo reference vehicle is light in sway
  and yaw because of the SDF's sign convention.
- A port that matched Gazebo's numbers would need those reduced inertias
  set on the body itself, with weight compensated, rather than a lagged
  force. That would copy a sign quirk, so it is not done.
- Cross-backend Tethys comparisons should expect the slower spin-up on
  the ported plants.

## Limits

- Identified from the source and from the probe's turn traces. Fortress is
  not re-run with corrected SDF signs.

Traces: `robot_lab_runtime/tethys-coriolis-20261120` and the 11-20 probe
regressions in `robot_lab_runtime/auv-long-20261120/probe-*` (hashes in
`runtime-sha256.txt`).
