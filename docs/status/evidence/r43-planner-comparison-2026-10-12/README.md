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

NavFn failed on seed 7, goal 2, a 9 m route. The controller received a
plan with 0 poses and exceeded its patience twice, so the navigation
aborted after 11 s. Evaluation found the same signature, a stale planner
robot pose (below).

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

## Evaluation trials (seeds 11, 23, 37, 41, 53)

The thresholds were committed in `98da1c7` before the evaluation started.
The evaluation ran at revision `e194ea7` (`evaluation/manifest.json`).
Results are in [evaluation/report.md](evaluation/report.md), with per-goal
records in `evaluation/*_seed*.json`.

| Planner | Goals | Success | Sim time median (s) | Path median (m) | Final error median / max (m) | Min clearance (m) |
|---|---|---|---|---|---|---|
| a_star_planner (Smac 2D A*) | 15 | 14/15 | 37.97 | 6.781 | 0.089 / 0.178 | 0.261 |
| navfn_planner (NavFn Dijkstra) | 15 | 15/15 | 38.90 | 6.756 | 0.074 / 0.139 | 0.330 |

All 15 goals were identical between the arms. On the 14 goals both arms
reached:

- A*'s time was 0.978x NavFn's (median; range 0.89-1.08).
- A*'s path length was 0.991x NavFn's (range 0.954-1.013).

On this maze the two searches give practically the same routes and
times. NavFn keeps slightly more clearance (median minimum 0.39 m against
0.33 m).

Every successful goal is within all predeclared thresholds:

- path at most 10.02 m (limit 13 m);
- at most 60.9 s (limit 100 s);
- clearance at least 0.261 m (limit > 0);
- final error at most 0.178 m (limit 0.25 m).

**The one failure is not the planner's.** A* seed 11, goal 2 aborted:
the controller received transformed plans with 0 poses and exceeded its
patience twice. The bag shows why:

- From about 11.5 simulated seconds onward, every plan from
  planner_server started at (-5.82, -7.02), where the robot had been
  then, while AMCL kept tracking the robot. When goal 2 started, the
  robot was at (3.70, -5.17).
- Plan stamps kept advancing, so the planner process was running but
  its TF view of the robot had frozen.
- Regulated Pure Pursuit keeps path poses within 1.5 m of the robot. The
  new 9 m path passed nowhere near it, so the transformed plan was empty.

The same signature (third goal, 0-pose plan) ended NavFn's seed-7 goal 2
in tuning. The evidence points to TF updates no longer reaching
planner_server. In Humble, `Costmap2DROS::getRobotPose` does not check how
old the pose is, so the fault is silent. It is not specific to either
search. It is counted as a failure of the arm it
happened in.

Diagnosed 2026-10-13: an ABBA deadlock in the installed tf2/tf2_ros 0.25.23
([ros2/geometry2#992](https://github.com/ros2/geometry2/issues/992)). A
costmap MessageFilter and the TransformListener thread lock the same two
mutexes in opposite order, so the listener thread blocks forever. Upstream
fixed it in 0.25.24, which apt installed here on 2026-10-13 (a workspace
overlay, `scripts/build_tf2_deadlock_fix.sh`, carried it until then);
see the [global planning tutorial](../../../tutorials/global_planning_comparison.md).
The protocol rerun with 0.25.24 passed 30/30 goals with no stale-transform
signature ([tf2-0.25.24-2026-10-13/](tf2-0.25.24-2026-10-13/README.md)).

## Reproduce

```bash
source scripts/ssd_env.sh
GOALS=3 ROBOT_RADIUS=0.12 scripts/planner_comparison.sh <dir> gazebo bumperbot nav_maze \
  global_planning "a_star_planner navfn_planner" "11 23 37 41 53" local_planning:=pure_pursuit
python3 scripts/planner_comparison_report.py <dir> --markdown report.md
```
