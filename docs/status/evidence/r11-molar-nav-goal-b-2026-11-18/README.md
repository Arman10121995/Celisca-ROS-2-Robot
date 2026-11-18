# R11 MOLAR Nav2 on CELISCA floor 1: near-obstacle goal B passes (November 18, 2026)

**Status: implemented, experimental.** Two runs, both passing every step.

**Run:** `CHECK_ARGS="--base base_link --robot-radius 0.49"
scripts/sim_nav_mission.sh gazebo molar_hri celisca_floor_1_detailed`,
seed 1. The goals are the same as in the failed run of November 15.

## Why goal B failed before

- The AMCL estimate was already 0.26 m and 4.7° off truth at the end of
  route A, and 0.49 m off during goal B.
- From the wrong pose, Nav2's planner placed MOLAR's footprint inside an
  obstacle: "Starting point in lethal space", in the launch log of the
  [November 15 run](../r11-molar-nav-2026-11-15/README.md). It aborted after
  41.7 s.

## Change

MOLAR's AMCL overlay (`robot_lab_localization/config/robots/molar_hri.yaml`):

- updates every 0.05 m / 0.05 rad (was 0.1 / 0.1);
- 180 beams (was 120);
- `alpha1` and `alpha4` 0.2 (was 0.05): it trusts odometry rotation less.

MOLAR has wheel-only odometry (no IMU is simulated) and a front-only ±105°
scan.

## Measured (errors from simulator truth)

| Step | Nov 15 (before) | Run 1 | Run 2 |
|---|---|---|---|
| Cancel after 1 m | passed | passed | passed |
| Route A (9.58 m path): error to goal | 0.183 m, 11.8° | 0.076 m, 6.2° | 0.122 m, 8.1° |
| Localization error at A | 0.26 m | 0.13 m | 0.17 m |
| **Goal B** (0.60 m clearance, 5.56 m path) | **aborted** 3.3 m short | **succeeded**, 0.091 m, −4.6° | **succeeded**, 0.089 m, −4.4° |
| Localization error at B | 0.49 m | 0.14 m | 0.16 m |

Reports: `gazebo_celisca1_run1.json`, `gazebo_celisca1_run2.json`.

## Limits

- Two runs, one seed, one floor (Gazebo).
- Localization still drifts by 0.13–0.17 m. The real robot's rear scanner
  and IMU are not simulated (that needs the real sensor layout).
- Runs were measured while Go2 stairs training loaded the CPU.
- Launch logs, traces and TF bags are on SSD under
  `robot_lab_runtime/molar-nav-20261118`.
