# R7.6 / R9.2: eight global planners on the R4.3 protocol (2026-10-12/13)

This comparison uses the R4.3 protocol:

| Setting | Value |
|---|---|
| Robot | Bumperbot |
| Map | nav_maze |
| Simulator | Gazebo, with isolated discovery ports |
| Controller | Nav2 Regulated Pure Pursuit |
| Seeds | 11, 23, 37, 41, 53 |
| Goals | Three per seed, the same 15 for every planner |
| Measurement | Gazebo truth |
| Revision | `10f1e51` (`manifest.json`) |

The eight planners:

- Nav2: Smac 2D A*, NavFn, Smac Hybrid-A* and Smac Lattice;
- robot_lab, through the ComputePathToPose adapter: RRT, RRT*, PRM and
  the Voronoi roadmap.

The results are in [report.md](report.md) and `report.json`, which
includes paired ratios on identical goals. Per-seed goal records are in
`*_seed*.json`. Bags, launch logs and truth traces are on the SSD in
`/workspace/molar/robot_lab_runtime/claude-global-cmp/`.

The discussion and limits are in the tutorial:
[global_planning_comparison.md](../../../tutorials/global_planning_comparison.md).

Success:

| Planner | Goals reached |
|---|---|
| A* | 15/15 |
| NavFn | 15/15 |
| PRM | 15/15 |
| Voronoi | 14/15 |
| Lattice | 13/15 (several long arrival loops) |
| RRT* | 9/15 |
| RRT | 8/15 |
| Hybrid-A* | 7/15 (six 300 s timeouts while circling) |

Some of these failures came after controller_server's TF view froze,
which is not the planner's doing. They are Voronoi 1 (its only failure),
Lattice 2 (both), RRT* 3, RRT 2 and Hybrid-A* 1, from
`python3 scripts/tf_freeze_goals.py <dir>` on the launch logs. They
are still counted in the success column. The tutorial explains the
freeze.

In this run the robot_lab planners blocked cells only at the inscribed
radius (`blocked_at: 99`). The default is now 80.

## Rerun with `blocked_at: 80` (2026-10-13, revision `b8e8ba5`)

The four robot_lab planners were rerun on the same protocol and goals:
[margin80/report.md](margin80/report.md), with per-seed records in
`margin80/`.

| Planner | Goals reached |
|---|---|
| Voronoi | 15/15 |
| RRT* | 14/15 |
| RRT | 14/15 |
| PRM | 12/15 |

- RRT and RRT* both failed seed 11 goal 2, where neither found a path.
- All three PRM failures (seed 37) came after a controller_server TF
  freeze. `margin80/prm_planner_seed37.tf_freeze.txt` shows that a fresh
  listener still received `map -> odom` from AMCL.
