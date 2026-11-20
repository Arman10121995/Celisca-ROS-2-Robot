# R7: `dijkstra_planner` dispatches its own Dijkstra (November 20, 2026)

**Status: implemented, experimental.** Until now `dijkstra_planner` ran the
same Nav2 NavFn plugin as `navfn_planner`, so the two ids were one method.
It now runs `robot_lab_planning::DijkstraPlanner`, a separate Nav2 planner
plugin. On the R7.6 protocol it reached 15 of 15 goals, as NavFn did.

## What changed

- **`robot_lab_planning::DijkstraPlanner`** (rewritten; the old version was
  teaching code):
  - eight-connected uniform-cost search on the global costmap, with no
    heuristic;
  - a step costs its length × (1 + 3·c/252), where c is the cell's cost;
  - cells at or above 253 (inscribed and lethal) are blocked; diagonal
    steps between two blocked cells are refused;
  - unknown cells follow `allow_unknown`;
  - the start cell may lie in the inflation band;
  - dense distance and parent arrays, instead of a linear search over the
    visited list;
  - an off-map start or goal, or no path, raises
    `nav2_core::PlannerException`. The old version returned an empty path.
  - Smoothing is optional (`use_smoother`, off) and never shuts the node
    down. The old `activate()` called `rclcpp::shutdown()` when no
    `smooth_path` server was up.
- **Dispatch:** `algorithm_dispatch.yaml`, the adapter's
  `GLOBAL_PLANNER_PLUGINS` and the registry entry name the new plugin.
- **Tests:** `test_dijkstra_search` (gtest, 8 cases on small grids);
  the dispatch and method-record tests are updated. The method records now
  list `dijkstra_planner` as corrected, not a duplicate of NavFn.

## Measured (R7.6 protocol: Gazebo, Bumperbot, `nav_maze`, RPP, seeds 11/23/37/41/53, 3 goals each)

Both planners ran fresh on this host, on the same goals
([report.md](report.md), `report.json`, per-seed `*_seed*.json`).

| Planner | Success | Sim time median (s) | Path median (m) | Final error median (m) | Min clearance median / worst (m) |
|---|---|---|---|---|---|
| `dijkstra_planner` (robot_lab) | **15/15** | 38.24 | 7.041 | 0.065 | 0.372 / **0.261** |
| `navfn_planner` (Nav2) | 15/15 | 38.84 | 6.841 | 0.083 | 0.412 / 0.030 |

Paired results on the 15 goals (Dijkstra ÷ NavFn):

- time ratio: median 0.994, range 0.79–1.12;
- path ratio: median 1.003, range 0.90–1.05.

- **Clearance.** One NavFn goal came within 0.030 m of a mapped obstacle.
  Dijkstra's worst was 0.261 m, because its cost term keeps plans out of
  the inflation band. The median clearance is a little lower, since its
  grid paths hug lower-cost corners.
- **Errors.** No `PlannerException` was logged, and no TF-freeze probe
  fired.

## Limits

- One robot, one map, one controller; Gazebo only.
- The manifest names HEAD `3f000d1b`; the plugin change was in the working
  tree and is committed with this receipt.
- Planning time per request was not measured.
- `robot_lab_planning::AStarPlanner` is still teaching code and is not
  dispatched: `a_star_planner` runs Nav2 Smac 2D.

## Reproduce

```bash
source scripts/ssd_env.sh
GOALS=3 ROBOT_RADIUS=0.12 scripts/planner_comparison.sh <dir> gazebo bumperbot nav_maze \
  global_planning "dijkstra_planner navfn_planner" "11 23 37 41 53" local_planning:=pure_pursuit
python3 scripts/planner_comparison_report.py <dir> --markdown report.md
```

Bags, launch logs and truth traces are on SSD under
`robot_lab_runtime/dijkstra-cmp-20261120` (hashes in `runtime-sha256.txt`).
