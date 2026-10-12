# R4.3: Bumperbot two-planner comparison (predeclared, 2026-10-12)

## Design

The design is declared in `experiments.yaml` (`bumperbot_planner_comparison`
v2.0.0) and `comparison.make_bumperbot_comparison`. Everything is fixed
except the global planner:

| Setting | Value |
|---|---|
| Arms | Smac 2D A* (`a_star_planner`) and NavFn's Dijkstra wavefront (`navfn_planner`) |
| Robot | Bumperbot |
| Map | nav_maze |
| Simulator | Gazebo, with isolated transport partition and discovery ports |
| Local planner | Regulated Pure Pursuit (`local_planning:=pure_pursuit`), stock parameters |
| Goals | Three seeded goals per trial (`scripts/sim_nav_goals.py`) |
| Measurement | Against Gazebo truth |
| Seeds | Tuning 7 and 9; evaluation 11, 23, 37, 41, 53 |
| Timeout | 300 s per goal |
| Tolerance | 0.25 m |

Each goal is drawn from the previous goal; the first is drawn from the
map's spawn at (-7, -7). Both arms therefore get the same goals for a
seed.

Version 1.0.0 was never runnable:

- It compared navfn with dijkstra, but both ids select the same NavFn
  plugin.
- It used TEB, which is not built in this workspace.
- It used small_office, which has no map.

Its "measured" threshold justifications had no runs behind them.

## Tuning trials (seeds 7, 9)

Results are in [tuning/report.md](tuning/report.md). Per-goal records are
in `tuning/*_seed*.json`. The manifest pins revision `2859641`.

| Planner | Goals | Success | Sim time median (s) | Path median (m) | Final error median (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| a_star_planner | 6 | 6/6 | 46.95 | 8.898 | 0.058 | 0.086 |
| navfn_planner | 6 | 5/6 | 48.28 | 9.320 | 0.056 | 0.215 |

All six goal positions were identical across arms. On the five goals both
arms reached:

- A*'s time was 0.95x NavFn's (median; range 0.84-1.02).
- A*'s path length was 0.99x NavFn's.

NavFn failed on seed 7, goal 2, a 9 m route. `ComputePathToPose` succeeded
but returned a plan with 0 poses, so the controller exceeded its patience
twice and the navigation aborted after 11 s.

The thresholds below were set from these tuning trials before any
evaluation trial ran:

| Threshold | Value | Basis |
|---|---|---|
| `max_path_length_m` | 13.0 m | Paths were 5.91-9.98 m; the sampler bounds the grid path at 10 m |
| `max_goal_sim_seconds` | 100 s | Goals took 27.7-67.3 s |
| `min_footprint_clearance_m` | 0.0 m | Measured 0.086-0.98 m; any overlap fails a goal |
| Final error | 0.25 m (`goal_tolerance_m`) | Measured at most 0.095 m |

### First tuning attempt, discarded

[tuning_v1_goals_from_stop/](tuning_v1_goals_from_stop/) drew each goal
from where the robot had stopped. Small stop differences changed the
sampled goals, so only 3 of 6 goals matched across planners, and its
paired ratios compare different routes. It is kept as the reason for the
fixed sequence. It is not comparison evidence.

## Evaluation trials

The five evaluation seeds are pending; they run after this design commit.

## Reproduce

```bash
source scripts/ssd_env.sh
GOALS=3 ROBOT_RADIUS=0.12 scripts/planner_comparison.sh <dir> gazebo bumperbot nav_maze \
  global_planning "a_star_planner navfn_planner" "11 23 37 41 53" local_planning:=pure_pursuit
python3 scripts/planner_comparison_report.py <dir> --markdown report.md
```
