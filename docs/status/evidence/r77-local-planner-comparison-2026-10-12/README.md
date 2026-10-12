# R7.7 / R9.2: five local planners on the R4.3 protocol (2026-10-12)

This comparison uses the R4.3 protocol:

| Setting | Value |
|---|---|
| Robot | Bumperbot |
| Map | nav_maze |
| Simulator | Gazebo, with isolated discovery ports |
| Global planner | Smac 2D A* |
| Seeds | 11, 23, 37, 41, 53 |
| Goals | Three per seed, the same 15 for every planner |
| Measurement | Gazebo truth |
| Revision | `10f1e51` (`manifest.json`) |

The five local planners:

- Nav2 `pure_pursuit` (Regulated Pure Pursuit);
- Nav2 `dwb_local_planner`;
- Nav2 `mppi_controller`;
- robot_lab `regulated_pure_pursuit`, through the FollowPath adapter;
- robot_lab `follow_the_gap`, through the FollowPath adapter.

The results are in [report.md](report.md) and `report.json`, which
includes paired ratios on identical goals. Per-seed goal records are in
`*_seed*.json`. Bags, launch logs and truth traces are on the SSD in
`/workspace/molar/robot_lab_runtime/claude-local-cmp/`.

The discussion and limits are in the tutorial:
[local_planning_comparison.md](../../../tutorials/local_planning_comparison.md).

Summary:

- Success: Nav2 RPP 15/15, robot_lab pure pursuit 15/15, follow-the-gap
  15/15, DWB 14/15 (one progress-checker stop), MPPI 7/15.
- MPPI's runs ended in progress failures and collision-checked recovery
  aborts. It ran with Nav2's default 0.5 m/s vx_max and untuned critics.
