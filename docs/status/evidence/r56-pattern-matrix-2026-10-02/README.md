# R5.6 pattern × backend Nav2 screen matrix — 2026-10-02 (session 2, owner: codex)

Extends `r56-functional-2026-10-02` (crab + MuJoCo patterns + Gazebo mecanum)
with the remaining clear-map cells and the first pattern/mecanum obstacle-route
screens. **A passing screen is not qualification**: each route is a single
one-shot measurement of goal attainment, body-truth error and heading, not a
repeated statistical or physical-robot result.

## Scope completed in this session

- **12 new clear-map screens** on `nav_empty` (goal offset (1,1)/90°, DWB,
  via-topic, body truth `/odom/ground_truth`): Gazebo 4WS ackermann / in_phase /
  pivot, PyBullet 4WS ackermann / in_phase / pivot, Isaac 4WS ackermann /
  in_phase / pivot, MuJoCo/PyBullet/Isaac mecanum.
- **3 new obstacle-route screens** on `nav_obstacle` (spawn (−7,−7) → goal
  (−3,−4)/0°, a route whose straight line cuts through the `obstacle_04`
  footprint at (−5.5,−5.5), so a detour is required): Gazebo 4WS in_phase,
  MuJoCo 4WS in_phase, Gazebo mecanum.

Together with the prior-session screens this gives **every cell at least one
passing DWB screen**: all 16 clear-map 4WS cells (4 patterns × 4 backends) and
the mecanum clear-map cell on all four backends.

## New screens (all single-run, serial, one simulator at a time)

| report | domain | route | outcome / wall | final truth xy | truth err / yaw err |
|---|---|---|---|---|---|
| `gazebo-4ws-ackermann-dwb-nav.json` | 202 | nav_empty (1,1)/90° | succeeded / 11.4 s | (0.863, 0.814) | 0.231 m / −13.85° |
| `gazebo-4ws-inphase-dwb-nav.json` | 203 | nav_empty (1,1)/90° | succeeded / 12.0 s | (0.790, 0.894) | 0.235 m / −14.21° |
| `gazebo-4ws-pivot-dwb-nav.json` | 204 | nav_empty (1,1)/90° | succeeded / 10.0 s | (0.841, 0.817) | 0.242 m / −14.67° |
| `pybullet-4ws-inphase-dwb-nav.json` | 205 | nav_empty (1,1)/90° | succeeded / 24.8 s | (0.837, 0.965) | 0.167 m / −14.32° |
| `pybullet-4ws-pivot-dwb-nav.json` | 206 | nav_empty (1,1)/90° | succeeded / 19.9 s | (0.919, 0.852) | 0.169 m / −13.50° |
| `pybullet-4ws-ackermann-dwb-nav.json` | 207 | nav_empty (1,1)/90° | succeeded / 21.0 s | (0.904, 0.842) | 0.184 m / −13.33° |
| `mujoco-mecanum-dwb-nav.json` | 208 | nav_empty (1,1)/90° | succeeded / 13.5 s | (0.886, 0.810) | 0.222 m / −14.06° |
| `pybullet-mecanum-dwb-nav.json` | 209 | nav_empty (1,1)/90° | succeeded / 23.1 s | (0.906, 0.812) | 0.210 m / −14.43° |
| `isaac-4ws-inphase-dwb-nav.json` | 210 | nav_empty (1,1)/90° | succeeded / 86.3 s | (0.859, 0.925) | 0.165 m / −15.40° |
| `isaac-4ws-pivot-dwb-nav.json` | 211 | nav_empty (1,1)/90° | succeeded / 67.1 s | (0.890, 0.834) | 0.207 m / −14.64° |
| `isaac-4ws-ackermann-dwb-nav.json` | 212 | nav_empty (1,1)/90° | succeeded / 72.6 s | (0.960, 0.862) | 0.152 m / −14.66° |
| `isaac-mecanum-dwb-nav.json` | 213 | nav_empty (1,1)/90° | succeeded / 75.8 s | (0.880, 0.799) | 0.233 m / −14.05° |
| `mujoco-4ws-inphase-obstacle-nav.json` | 214 | nav_obstacle (−7,−7)→(−3,−4)/0° | succeeded / 34.3 s | (−3.124, −4.187) | 0.224 m / −2.16° |
| `gazebo-4ws-inphase-obstacle-nav.json` | 215 | nav_obstacle (−7,−7)→(−3,−4)/0° | succeeded / 23.8 s | (−3.118, −4.172) | 0.209 m / −0.17° |
| `gazebo-mecanum-obstacle-nav.json` | 216 | nav_obstacle (−7,−7)→(−3,−4)/0° | succeeded / 25.4 s | (−3.043, −4.148) | 0.154 m / 12.92° |

Every report has `outcome: succeeded`, `motion_source: /odom/ground_truth`
(independent body truth, not AMCL), and an empty `.err` file. Localization
estimate vs truth agree to ≈0.06 m at the goal, so AMCL converged on both maps.

### Prior-session cells folded into the matrix

From `docs/status/evidence/r56-functional-2026-10-02/`: Gazebo crab
(16.0 s) and Gazebo mecanum; PyBullet crab (24.8 s); MuJoCo crab
(17.9 s) + in_phase/ackermann/pivot; Isaac crab-fixed (76.1 s, 0.210 m /
−16.13°). The failed unconstrained-RPP crab route
(`mujoco-4ws-crab-diagonal-nav.json`) is preserved there and remains the reason
parallel patterns run constrained DWB.

## Controller configuration actually loaded

`navigation.launch.py` selects the DWB critic set from the motion model:
`omni_parallel` (crab, in_phase) loads
`robot_lab_controller::ParallelSteeringCritic` + RotateToGoal/Oscillation/
BaseObstacle/PathDist/GoalDist; `ackermann` motion model (ackermann, pivot)
loads the default DWB critics (GoalAlign/PathAlign included). Each launch log
records the exact `Using critic` lines and the bt_navigator
`Goal succeeded` — see `navigation-log-excerpts.txt`. The parallel critic
rejects simultaneous translation/yaw, so it is present only in the
crab/in_phase runs, as designed.

## Method

```bash
source scripts/ssd_env.sh
export ROS_DOMAIN_ID=<202..216>   # distinct per trial, serial execution
export TIMEOUT=300                # 240 on gazebo clear-map runs
export CHECK_ARGS='--via-topic --offset-x 1.5 --offset-y 1.5 --goal-yaw-deg 90'
scripts/sim_nav_check.sh <backend> four_wheel_steer_car nav_empty \
  /workspace/molar/robot_lab_runtime/r56-patterns-2026-10-02/<name>.json \
  steering_mode:=<ackermann|in_phase|pivot|crab>
# obstacle routes: map nav_obstacle, CHECK_ARGS='--via-topic --offset-x 4 --offset-y 3 --goal-yaw-deg 0'
# mecanum runs: robot mecanum_car, no steering_mode
```

- Harness: `scripts/sim_nav_check.sh` + `scripts/sim_nav_check.py`
  (goal picked relative to the localized start; success = NavigateToPose
  terminal state; truth from `/odom/ground_truth`).
- Raw artifacts (JSON reports, full launch logs, run logs, `source-hashes.txt`)
  at `/workspace/molar/robot_lab_runtime/r56-patterns-2026-10-02/` on the SSD.
  This directory holds the 15 JSON reports, `source-hashes.txt` and the log
  excerpts; the multi-megabyte launch logs stay on the SSD only.
- No source files changed in this session; `source-hashes.txt` records the
  hashed harness/controller/bringup files for provenance.
- Clean-up verified after every trial: no orphan sim/launch processes.

## Caveats (why this is not qualification)

1. **Systematic terminal offset**: clear-map screens cluster at 0.15–0.24 m and
   13–15° truth error regardless of backend/pattern — a consistent controller/
   tolerance behaviour, not noise. Nav2 accepts within its goal checker; precise
   final heading remains open (the 90° goal finishes ≈75°).
2. **One shot per cell**: no repeats, no variance, no regression threshold.
3. **Obstacle routes are one-shot**: the route requires detouring around
   `obstacle_04`, and the action succeeded with the expected motion
   (in_phase routes with ~0.0–0.17 rad body-yaw travel, i.e. lateral
   translation around the obstacle; the mecanum route used 3.49 rad heading
   travel), but trajectories/contacts were not independently instrumented, so
   obstacle clearance margin is unmeasured.
4. **Still open in R5.6 scope**: repeated final-heading screens, command-loss /
   reset recovery, Localization/2D-SLAM/RGB-D SLAM missions, physical mecanum
   travel qualification (a simulator Nav2 screen says nothing about the
   physical robot).
5. Screens were taken from launches started by this harness only; no
   GUI-saved manifest reproducibility run was performed here.

