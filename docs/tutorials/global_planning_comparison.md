# Global Planning Comparison: eight planners on one maze

Measured October 12-13, 2026, at revision `10f1e51`. The evidence and
per-goal records are in
[r76-global-planner-comparison-2026-10-12](../status/evidence/r76-global-planner-comparison-2026-10-12/README.md).
The predeclared two-planner subset (A* vs NavFn) is
[R4.3](../status/evidence/r43-planner-comparison-2026-10-12/README.md).

**Category:** global_planning
**Type:** measured comparison (Gazebo)

## What is compared

Eight global planners each plan for Bumperbot. Everything else is held
fixed:

- map nav_maze, simulator Gazebo;
- controller Nav2 Regulated Pure Pursuit;
- AMCL localization;
- 0.55 m inflation, 0.12 m footprint.

| Selection (`global_planning:=`) | Implementation | Search |
|---|---|---|
| `a_star_planner` | Nav2 Smac 2D | A* on the costmap grid, cost-aware |
| `navfn_planner` | Nav2 NavFn | Dijkstra wavefront (potential field) |
| `hybrid_a_star_planner` | Nav2 Smac Hybrid-A* | Dubins arcs, 0.4 m minimum turning radius, forward only |
| `state_lattice_planner` | Nav2 Smac Lattice | Precomputed motion primitives (0.5 m turning radius, diff variant) |
| `rrt_planner` | robot_lab RRT (`nav2_global_planner`) | Goal-biased tree, every edge collision-checked |
| `rrt_star_planner` | robot_lab RRT* | Best-parent choice and rewiring, anytime within a 1 s budget |
| `prm_planner` | robot_lab PRM | 600 free samples, k = 12 nearest-neighbour roadmap, Dijkstra |
| `voronoi_planner` | robot_lab Voronoi roadmap | Follows the generalized Voronoi diagram (maximum clearance) |

The robot_lab planners serve Nav2's `ComputePathToPose` through
`robot_lab_algorithms.nav2_adapters`. They plan on the same inflated
global costmap, and in this run a cell was blocked at the inscribed
radius.

**Goals.** Each trial sends three seeded goals; each is drawn from the
previous goal, the first from the spawn. Every planner gets the same 15
goals. Five seeds are used: 11, 23, 37, 41, 53.

**Measurement.** Outcome is the Nav2 result. Time is simulated seconds.
Path length and minimum footprint clearance come from Gazebo truth. A
clearance of 0 or less means the footprint overlapped a mapped obstacle.

## Run

```bash
source scripts/ssd_env.sh
GOALS=3 ROBOT_RADIUS=0.12 scripts/planner_comparison.sh <dir> gazebo bumperbot nav_maze \
  global_planning "a_star_planner navfn_planner hybrid_a_star_planner state_lattice_planner rrt_planner rrt_star_planner prm_planner voronoi_planner" \
  "11 23 37 41 53" local_planning:=pure_pursuit
python3 scripts/planner_comparison_report.py <dir> --markdown report.md
```

## Results

| Planner | Success | Failures after a TF freeze | Sim time median (s) | Path median (m) | Path / A* (paired) | Min clearance median / worst (m) |
|---|---|---|---|---|---|---|
| a_star_planner | 15/15 | - | 38.48 | 6.781 | 1.00 | 0.307 / 0.271 |
| navfn_planner | 15/15 | - | 39.60 | 6.773 | 1.01 | 0.395 / 0.330 |
| prm_planner | 15/15 | - | 38.58 | 6.779 | 1.00 | 0.172 / **-0.020** |
| voronoi_planner | 14/15 | 1 (its only failure) | 46.25 | 8.381 | 1.16 | **0.530** / 0.380 |
| state_lattice_planner | 13/15 | 2 (both failures) | 44.26 | 9.533 | 1.37 | 0.241 / 0.030 |
| rrt_star_planner | 9/15 | 3 | 33.12 | 6.794 | 1.01 | 0.184 / -0.020 |
| rrt_planner | 8/15 | 2 | 56.23 | 7.444 | 1.19 | 0.038 / -0.070 |
| hybrid_a_star_planner | 7/15 | 1 | 30.94 | 6.830 | 1.03 | 0.230 / -0.020 |

"Path / A*" is the inverse of the paired A*/planner path ratio on the
goals both reached.

**Failures after a TF freeze.** In five trials (one launch each), Nav2's
controller_server stopped receiving transforms partway through. From then
on it logged "Transform data too old when converting from map to odom",
while AMCL kept publishing. The controller then used a frozen robot pose:
the robot rotated in place until the progress checker aborted, and later
goals in the same launch failed at once.

These goals are counted as failures of the planner that was running. The
column shows how many of each planner's failures came after a freeze;
the cause is not the planner. Counts come from `scripts/tf_freeze_goals.py`
on the launch logs. One Hybrid-A* goal had already been circling for
175 s when its freeze began. The cause of the freeze is still open (see
below).

## What the numbers say

- **Grid search wins on a maze.** A* and NavFn reach every goal along
  nearly identical routes.
- **PRM is close behind.** A 600-sample roadmap found the same routes
  (path ratio 1.00), but its straight roadmap edges cut closer to walls;
  the worst footprint clearance is -0.020 m.
- **The Voronoi roadmap does what it is for.** It keeps the most
  clearance (median minimum 0.53 m), at the price of 16% longer paths and
  slower travel. Its one failure (seed 53, goal 3) came after a TF
  freeze: the robot rotated in place on a frozen pose. This entry
  previously called it a stop near a wall; the launch log shows otherwise.
- **The lattice and Hybrid-A\* planners are designed for curvature-limited
  robots.** On a differential base in narrow corridors they produce
  arrival loops:
  - Lattice "successes" include detours of 13-51 m to goals 7 m away by
    grid. Both lattice failures came after a TF freeze.
  - Hybrid-A\* ran out the 300 s goal timeout on 6 goals, after driving
    up to about 60 m in circles near the goal.

  Prefer them for the car-like robots, where the lab already selects
  them.
- **Plain RRT and RRT\* struggle in a maze.** Even without the frozen
  goals (2 of RRT's 7 failures, 3 of RRT\*'s 6), they fail the most:
  - Within the time budget they sometimes find no path.
  - Their paths sit on the inscribed boundary, so RPP's collision check
    stops on them ("detected collision ahead").
  - RRT\*'s rewiring gives near-A\* path lengths when it succeeds; RRT's
    paths are about 19% longer.

The robot_lab sampling planners now default to a ~7 cm margin inside the
inflation band (`blocked_at: 80`). A rerun with that margin is queued
and will be added to the evidence. The table above is the first run, at
the inscribed boundary.

The same freeze ended goals elsewhere:

- R4.3 A* seed 11: planner_server froze, and its plans started where the
  robot used to be.
- MPPI seed 41 in the [local planning comparison](local_planning_comparison.md).
- A MuJoCo car mission.

In all, it happened in 9 of about 90 recent Nav2 launches. Comparison bags
now record `/tf`, and `scripts/tf_freeze_probe.sh` snapshots a fresh
listener and the server threads at the next occurrence.

## Limits

- One robot, one map, five seeds, three goals each. These are
  distributions for this task, not general rankings.
- The robot_lab planners plan only on the costmap's blocked and free
  cells. They do not smooth their paths and do not trade path cost
  against clearance the way Smac and NavFn do.

## Related tutorials

- [Tutorial index](README.md)
- [Local planning comparison](local_planning_comparison.md)
- [Planning notes](planning.md)
- [Measured-run and failure workflow](../WORKFLOW.md)

*Part of R9.2: Seven Real Comparison Tutorials. See [ROADMAP.md](../../ROADMAP.md).*
