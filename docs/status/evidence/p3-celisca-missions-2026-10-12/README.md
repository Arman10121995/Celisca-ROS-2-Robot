# P3: Celisca navigation missions, plain and furnished floors (2026-10-12)

Jetson AGX Orin, ROS 2 Humble, headless. One mission per cell:
4 simulators x {Labbot, Bumperbot} x {celisca_floor_1, celisca_floor_1_furniture,
celisca_floor_2, celisca_floor_2_furniture}.

The mission (`scripts/sim_nav_mission.sh`, `scripts/sim_nav_mission.py`) has four steps:

1. Start a detour goal A (path at least 1.5x the straight line).
2. Cancel it after 1 m and require the robot to stop.
3. Resume A.
4. Go to a second goal B close to an obstacle.

Goals A and B must finish as Nav2 `succeeded`. Each final pose is then
measured against simulator truth (`/odom/ground_truth`) after one settled
simulated second. Goals come from a seeded draw on each map (seed 1).

## Result: 32 of 32 cells pass

Full table: [matrix.md](matrix.md). Per-cell steps and goals: `matrix.json`.

| Measure (32 cells) | Median | Max |
|---|---:|---:|
| Goal A final error vs truth | 0.069 m | 0.112 m |
| Goal B (near obstacle) final error vs truth | 0.054 m | 0.139 m |
| Goal heading error (A and B) | | 3.0 deg |
| Stop after cancel (simulated) | | 0.40 s |

On furnished floor 1, every cell rounded the wall end that had stopped the
earlier runs. Goal A was at (-5.14, -1.77); PyBullet Labbot's was at
(-5.16, -1.77).

## What was wrong, in the order found

| Problem | Evidence | Change |
|---|---|---|
| The furnished maps shipped the plain floor's PGM, with no furniture. AMCL matched scans against a map that lacked it. Nav2 reported success up to 5.4 m from the goal. | [dwb_default_runs/plain_floor_map_for_furniture.md](dwb_default_runs/plain_floor_map_for_furniture.md): 5 of 16 furnished cells passed | `src/robot_lab_maps/tools/furnish_occupancy_map.py` slices the furnished world's collision geometry into the base grid (`map_source.json`). MuJoCo scan endpoints on the new map lie within one cell (0.023 m) of an occupied cell. On the plain map, 44% lie more than 0.10 m away. |
| The furnished floor-1 start was 0.23 m from furniture collision geometry. | `test_celisca_spawn_has_nav_clearance` | Spawn and initial pose moved to (0.35, 1.17) in `sim_maps.yaml`. |
| The Bumperbot costmap radius (0.10 m) was smaller than its wheels (0.118 m). It wedged on furniture. | | `robot_radius: 0.12` |
| DWB stopped at a wall end on furnished floor 1. This happened in every backend, for both robots, and in all three Labbot diagnostic runs. | [dwb_default_runs/matrix.md](dwb_default_runs/matrix.md): 19 of 28 cells passed; [dwb_stall_diagnosis/](dwb_stall_diagnosis/) | Labbot and Bumperbot now default to Nav2 Regulated Pure Pursuit (`robots.yaml default_algorithms`; DWB stays selectable). Both costmaps use `inflation_radius 0.55` for these robots. |
| A leftover PX4 flight's Gazebo Harmonic server sent discovery traffic that Fortress cannot parse. Every Gazebo run logged "Unknown message type" about 6,500 times per second, about 1M lines per mission, and some navigation log lines were lost. | Earlier Gazebo logs have none | Fixed in `cc3010a`: separate Harmonic discovery ports, flight run id and reaper, and isolated measurement scripts. All Gazebo cells here logged 0 such lines. |

### The DWB stall, recorded

`dwb_stall_eval.py` captured DWB's `/evaluation` once the robot had been
still for 5 s:

- **0.3 m inflation band** (`dwb_evaluation_inflation_0.30.json`). The robot
  stopped 0.25 m from the wall end, just outside the band. The path rounded
  the tip 0.27 m away, only 0.09 m outside Labbot's 0.18 m inscribed
  radius. Faster forward samples hit the inscribed zone. The slow samples
  reachable from rest scored the band's edge (BaseObstacle raw 196-205,
  about 4 points) against about 1 point of progress. Rotation in place
  scored 36.0 against at least 39.4 for any forward sample, so DWB rotated
  until the progress checker failed.
- **0.55 m band** (`dwb_evaluation_inflation_0.55.json`, run D). The robot
  still stopped. GoalDist stayed at 30 for every sample up to 0.28 m/s.
  The Oscillation critic ruled out the right turns. Rotation scored 37.9
  against at least 38.2 forward. The wider band alone does not fix DWB.

Same route (spawn to goal A), MuJoCo Labbot, inflation 0.55:

| Local planner | Outcome | Truth error | Wall time | File |
|---|---|---:|---:|---|
| DWB (A, B, C at 0.30; D at 0.55) | aborted, 4 of 4 | 1.45-1.86 m short | 81-148 s | `labbot_f1f_mujoco_{A..D}.json` |
| Regulated Pure Pursuit | succeeded | 0.046 m | 46 s | `labbot_f1f_mujoco_E_pure_pursuit.json` |
| MPPI | succeeded, after one progress recovery | 0.097 m | 101 s | `labbot_f1f_mujoco_E_mppi_controller.json` |

## Conditions and limits

- **Background load.** An orphaned PX4 SITL with its Harmonic `gz sim`
  (pid 117158/117586) ran throughout, using about 2.2 cores. It was started
  12:05 from a desktop session. This session was not allowed to stop it, so
  it needs a manual `kill`. Missions are timed in simulated time; Gazebo
  runs used their own discovery ports.
- **Coverage.** One seeded mission per cell. These are not repeated trials,
  and DWB is not tuned.
- **Behaviour change.** The RPP default and the 0.55 m inflation apply to
  Labbot and Bumperbot only. Earlier DWB evidence stays historical.

## Reproduce

```bash
source scripts/ssd_env.sh
scripts/sim_nav_mission_matrix.sh $ROBOT_LAB_RUNTIME_ROOT/p3 "mujoco pybullet gazebo isaac" \
  "labbot bumperbot" "celisca_floor_1 celisca_floor_1_furniture celisca_floor_2 celisca_floor_2_furniture"
python3 scripts/nav_mission_matrix_report.py $ROBOT_LAB_RUNTIME_ROOT/p3 --markdown matrix.md
```

Raw logs, truth/command traces and the DWB rosbag stay in
`/workspace/molar/robot_lab_runtime/claude-p3/` (matrix, matrix_dwb_runs, diag).
