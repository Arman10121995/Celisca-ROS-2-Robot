# Local Planning Comparison: five controllers on one maze

Measured October 12, 2026, at revision `10f1e51`. The evidence and
per-goal records are in
[r77-local-planner-comparison-2026-10-12](../status/evidence/r77-local-planner-comparison-2026-10-12/README.md).

**Category:** local_planning
**Type:** measured comparison (Gazebo)

## What is compared

Five local planners each drive Bumperbot. Everything else is held fixed:

- map nav_maze, simulator Gazebo;
- global planner Smac 2D A*, AMCL localization;
- 0.55 m inflation and the 0.12 m footprint.

| Selection (`local_planning:=`) | Implementation | How it drives |
|---|---|---|
| `pure_pursuit` | Nav2 Regulated Pure Pursuit plugin | Lookahead on the global path; regulates speed by curvature and cost; rotates to heading |
| `dwb_local_planner` | Nav2 DWB plugin | Samples velocities; scores rollouts with critics (path/goal distance and alignment, obstacle cost) |
| `mppi_controller` | Nav2 MPPI plugin | Samples 2000 noisy control sequences over 2.8 s; weighted-average optimal control |
| `regulated_pure_pursuit` | robot_lab pure pursuit (`nav2_local_planner`) | Same law as above: curvature `2y/L^2`, curvature speed limit, rotate to heading |
| `follow_the_gap` | robot_lab follow-the-gap (`nav2_local_planner`) | Steers to the open scan gap closest to the path direction; speed follows free space ahead |

The two robot_lab planners serve Nav2's `FollowPath` action through
`robot_lab_algorithms.nav2_adapters`. The navigator calls them, so they
drive the robot. Before October 12 they ran beside Nav2 and only
published to side topics.

**Goals.** Each trial sends three goals from a seeded generator. Each
goal is drawn from the previous goal; the first is drawn from the spawn.
Every planner gets the same 15 goals (`identical_goal_positions` in the
report). Five seeds are used: 11, 23, 37, 41, 53.

**Measurement.** Outcome is the Nav2 result. Time is simulated seconds.
Path length and the footprint's minimum clearance to the static map are
measured from Gazebo truth. Final error is the truth position after one
settled simulated second.

## Run

```bash
source scripts/ssd_env.sh
GOALS=3 ROBOT_RADIUS=0.12 scripts/planner_comparison.sh <dir> gazebo bumperbot nav_maze \
  local_planning "pure_pursuit dwb_local_planner mppi_controller regulated_pure_pursuit follow_the_gap" \
  "11 23 37 41 53" global_planning:=a_star_planner
python3 scripts/planner_comparison_report.py <dir> --markdown report.md
```

Each planner/seed is one launch, with a rosbag of truth, `/cmd_vel`,
`/plan` and AMCL. One trial is about 5 minutes on the Jetson.

## Results

| Planner | Success | Sim time median (s) | Path median (m) | Mean speed median (m/s) | Final error median (m) | Min clearance median / worst (m) |
|---|---|---|---|---|---|---|
| pure_pursuit (Nav2 RPP) | 15/15 | 38.88 | 6.756 | 0.175 | 0.080 | 0.327 / 0.241 |
| regulated_pure_pursuit (robot_lab) | 15/15 | 29.28 | 6.739 | 0.241 | 0.075 | 0.310 / 0.241 |
| follow_the_gap (robot_lab) | 15/15 | 28.62 | 6.712 | 0.239 | 0.090 | 0.304 / **0.130** |
| dwb_local_planner (Nav2 DWB) | 14/15 | 32.82 | 6.973 | 0.216 | 0.082 | 0.330 / 0.234 |
| mppi_controller (Nav2 MPPI) | **7/15** | 21.86 | 8.276 | 0.364 | 0.081 | 0.341 / **0.030** |

Ratios on the same goals (median of the first planner's time over the
second's):

- robot_lab pure pursuit vs Nav2 RPP: time 0.76.
- follow-the-gap vs robot_lab pure pursuit: time 0.99, path 0.99.
- DWB vs Nav2 RPP: time 0.89.
- DWB vs robot_lab pure pursuit: time 1.13.

## What the numbers say

- **Nav2 RPP is the most conservative.** It never failed, but it was the
  slowest (median 0.175 m/s). Its curvature and cost regulation slow it
  in the maze's turns and narrow passages.
- **The robot_lab pure pursuit tracks as reliably and about 25% faster.**
  It has the same footprint clearance; it only applies curvature slowdown
  below a 0.9 m turn radius.
- **Follow-the-gap succeeded on every goal but cut closest to walls**
  (0.130 m minimum clearance). It steers by the scan's open gaps, not by
  the costmap.
- **DWB failed one goal** (seed 23): it stopped and the progress checker
  aborted twice. This is the same failure mode diagnosed on furnished
  Celisca ([evidence](../status/evidence/p3-celisca-missions-2026-10-12/README.md)).
- **MPPI reached only 7 of 15 goals.** Its runs ended with "Failed to make
  progress", and on seed 41 also with "Collision Ahead" during the
  back-up recovery. It came within 0.030 m of a mapped wall.

  This is not a fair verdict on MPPI. Its sampler uses Nav2's default
  vx_max of 0.5 m/s, while the other planners are limited to about
  0.3 m/s, and its critic weights were not tuned for Bumperbot. It shows
  that the shipped MPPI configuration is not ready for this robot.

## Limits

- One robot, one map, five seeds, three goals each. These are
  distributions on this task, not general rankings.
- Each plugin keeps its own speed parameters. MPPI's 0.5 m/s cap is the
  main imbalance.
- The robot_lab python TEB and MPPI nodes are placeholders and are not
  selectable as planners (`algorithm_dispatch.yaml`). `teb_local_planner`
  is not built in this workspace.

## Related tutorials

- [Tutorial index](README.md)
- [Global planning comparison](global_planning_comparison.md)
- [Numerical learning notes](local_planning.md)
- [Measured-run and failure workflow](../WORKFLOW.md)

*Part of R9.2: Seven Real Comparison Tutorials. See [ROADMAP.md](../../ROADMAP.md).*
