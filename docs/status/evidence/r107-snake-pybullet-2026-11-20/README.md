# R10.7: the passive-wheel snake on PyBullet, and a speed calibration (November 20, 2026)

**Status: implemented, experimental; navigation not passed.** The
self-made `lab_snake` now walks, maps and saves a map on PyBullet; until
now it ran on MuJoCo only. While doing this, its serpenoid gait was found
to ignore the speed command: it ran 0.33–0.40 m/s for 0.03–0.06 m/s
commands, on both backends. Speed now follows the command. Saved-map Nav2
goals fail on both backends, for the reasons below.

## Changes

- **PyBullet rolling adapter**:
  - `gait_plant.GaitPlant` takes the MuJoCo plant's named contact groups
    (`gait_contact_config`, URDF collision names mapped to links), so each
    passive wheel reports its own normal force. The launch passes them,
    and `gait_simulator.launch.py` now allows the snake on PyBullet.
  - Wheel links get Bullet friction 1.0. The URDF gives them none;
    MuJoCo's and Gazebo's default is 1.0, Bullet's 0.5.
  - Contact groups average each substep's normal force over the publish
    window, capped at the last 50 substeps.
  - Floor placement uses the wheels when there are no foot probes.
- **Speed calibration** (`serpentine_model.phase_rate`, `gait_controller`,
  `lab_snake` `cycle_s` 2.4 → 10.6 s):
  - The wave phase now advances in proportion to the command intensity;
    the amplitude law is unchanged.
  - Measured on MuJoCo at a 2.4 s cycle, speed was 4.4× the command
    (0.08 → 0.353 m/s, 0.03 → 0.133 m/s), so the cycle was scaled by 4.4.
  - Achieved speeds: MuJoCo 0.08 → **0.074 m/s** and 0.03 → **0.025 m/s**;
    PyBullet 0.08 → **0.085 m/s**.
- **Rolling odometry** (`serpentine_odometry.py`):
  - The validity state is published at least every 0.2 wall s. Ten samples
    took about 0.55 s at PyBullet's 0.36× real time, against the
    controller's 0.6 s freshness limit.
  - Unobserved travel is bounded at twice the configured top speed
    (0.16 m/s) instead of 0.5 m/s. The 0.5 constant predates the
    calibration and exhausted the 0.1 m bound after a few invalid samples.
- **Mapping workload** (`extension_workload.py`). The drive brakes at
  1.1 × the requested travel. The PyBullet snake's head settled back to
  0.198 of 0.2 m after a brake at 0.2. The saving check still requires
  the requested distance.
- **Tests:** `test_snake_pybullet_contacts.py` checks, on Bullet DIRECT:
  the wheel groups, friction 1.0, the capped window, and three or more
  supports. The serpentine contract tests pass with the new cycle.

## Measured

Manual walks, `nav_empty`, truth (`walk-*.json`; `snake_odometry_probe.py`
also records `/gait/odometry_state`):

| Backend | Cycle | Command | Speed | Odometry samples invalid | Odometry ÷ truth travel |
|---|---|---|---|---|---|
| MuJoCo | 2.4 s (before) | 0.06 m/s | ~0.40 m/s, hit the wall at 6 m | — | — |
| PyBullet | 2.4 s (before) | 0.06 m/s | 0.34 m/s, hit the wall | — | — |
| MuJoCo | 2.4 s, frequency ∝ command | 0.08 / 0.03 m/s | 0.353 / 0.133 m/s | 19 % / 1 % | — |
| MuJoCo | **10.6 s** | 0.08 / 0.03 m/s | **0.074 / 0.025 m/s** | 5 % / 0 % | 0.93 / 0.99 |
| PyBullet | **10.6 s** | 0.08 m/s | **0.085 m/s** | 9 % | **0.65** |

Saved-map goals (`qualify_gait_saved_map.py`, `nav_empty`, 0.5 m ahead,
0.15 m contract):

| Backend | Version | Map | Navigation |
|---|---|---|---|
| MuJoCo | before calibration | 50,092 cells | **not passed**: Nav2 "Goal succeeded"; truth 0.23 m past the goal (overshoot) |
| PyBullet | before calibration, no averaging | 37,448 | not passed: arm rejected; rolling odometry inconsistent |
| PyBullet | + averaging | 41,276 | not passed: "Failed to make progress"; odometry stale at 0.36× real time |
| MuJoCo | calibrated | 32,159 / 33,127 / 33,008 | **not passed**: navigation interrupted by uncertainty (before the bound change), then twice "Goal succeeded" by Nav2 but 0.17–0.22 m short by truth |
| PyBullet | calibrated | mapping did not reach 0.2 m (twice: 120 s window; brake at 0.198 m) | — |
| PyBullet | + brake margin | **38,589 cells (first PyBullet snake map)** | **not passed**: canceled by "rolling odometry inconsistent/stale" 0.15 m from the goal, after 0.59 m |

On 10-30 the snake passed MuJoCo installed- and acquired-map goals (0.113
and 0.106 m). Before this calibration it already failed the 11-20
qualifier, by overshooting.

## What remains

- **PyBullet odometry.** Travel during invalid samples is not integrated,
  and on PyBullet those samples fall in the fast phases of the wave. The
  estimate ends 35 % short, and navigation is interrupted.
- **MuJoCo estimate.** Nav2's estimate reaches the goal 0.17–0.22 m before
  truth does. The odometry is within 7 %, so the AMCL correction in
  `nav_empty`, with only far walls in view, is the next suspect.
- **Earlier certificates.** The 10-30 MuJoCo snake certificates predate the
  calibration and were not re-run.

## Reproduce

```bash
GAIT_CASE_DIRECTORY=<dir> GAIT_BACKEND=pybullet GAIT_ROBOT=lab_snake GAIT_YAW=0 GAIT_SPEED=.08 GAIT_WALK_S=10 \
  python3 docs/status/evidence/r107-snake-pybullet-2026-11-20/snake_odometry_probe.py
python3 scripts/qualify_gait_saved_map.py --output <dir> --simulator pybullet --timeout 900 --map-timeout 400 --robots lab_snake
```

Logs are on SSD under `robot_lab_runtime/snake-pybullet-20261120` (hashes in
`runtime-sha256.txt`).
