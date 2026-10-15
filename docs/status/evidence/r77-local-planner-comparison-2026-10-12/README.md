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
- Three MPPI failures (all of seed 41) came after controller_server's TF
  view froze, 29 s into the first goal (`scripts/tf_freeze_goals.py`).
  The freeze also hit five global-planner trials; it is not the
  controller's doing.

## MPPI rerun at vx_max 0.3 m/s (2026-10-13, revision `b8e8ba5`)

Same protocol and goals, with MPPI's sampler capped at Bumperbot's
0.3 m/s: [mppi_vx03/report.md](mppi_vx03/report.md).

- Result: **7/15 again**, with no TF freeze.
- All eight failed goals ended 0.05-0.19 m from the goal and
  within 4.5° of its heading. MPPI gets close, then does not settle into
  the 0.07 m / 0.03 rad arrival window before the progress checker aborts.
- The first run's failures look the same, apart from the frozen seed 41.

## MPPI with arrival tuning (2026-10-13, revision `4e93bcc`, negative)

Same protocol (vx_max 0.3), with stronger goal critics: GoalCritic weight
10, GoalAngleCritic weight 8, both at power 1, and narrower sampling
(vx_std 0.1, wz_std 0.3). Results in [mppi_arrival/report.md](mppi_arrival/report.md).

- Result: **6/15**, with a median of 85 s per goal instead of about 40 s.
  There was no stale-transform signature.
- Eight of the nine failures again ended 0.09-0.31 m from the goal; one
  (seed 41, first goal) ended 3.5 m away.
- The tuning was reverted (`7a8a3cf`). Settling into a 0.07 m / 0.03 rad
  window is not a matter of these two critics.

Cross-track error from the latest plan for all five first-run planners is
in [tracking.md](tracking.md).
