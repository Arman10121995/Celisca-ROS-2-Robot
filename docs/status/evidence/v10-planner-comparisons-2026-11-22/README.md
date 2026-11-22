# V10: global and local planner comparisons at one revision (November 22, 2026)

**Status: 50 of 50 trials ran; 145 of 150 goals were reached.** Both
comparisons used the R7.6/R7.7 protocol, all at revision 52964578 (installed
build), between 03:56 and 06:06.

| Setting | Value |
|---|---|
| Robot, map, simulator | Bumperbot, nav_maze, Gazebo |
| Seeds | 11, 23, 37, 41, 53; three goals each, the same 15 goals for every method |
| Start | (−7, −7); each goal starts where the previous one ended |
| Measurement | Gazebo truth (`final_error_truth_m`, trace) |
| Global category | fixed `local_planning:=pure_pursuit` (Nav2 Regulated Pure Pursuit), as in October |
| Local category | fixed `global_planning:=a_star_planner` |
| Order | seed by seed, methods interleaved within a seed, one shared physics lock hold per trial |

Unlike October, every method ran at the same revision in one round. October's
methods ran at `10f1e51`, `b8e8ba5` and `4e93bcc`, and Dijkstra ran on
November 20 at `3f000d1`.

## Global planners ([global/report.md](global/report.md))

| Planner | Goals reached | Sim time median | Path median | Worst footprint clearance |
|---|---|---|---|---|
| A* (Smac 2D) | **15/15** | 38.3 s | 6.80 m | 0.261 m |
| Dijkstra (robot_lab) | **15/15** | 38.6 s | 7.02 m | 0.261 m |
| PRM | **15/15** | 41.9 s | 7.17 m | −0.008 m |
| RRT | 14/15 | 53.7 s | 7.49 m | −0.020 m |
| RRT* | 14/15 | 45.4 s | 6.76 m | −0.049 m |

Paired on the goals both reached, A* took 0.98× Dijkstra's time, 0.92×
PRM's, 0.74× RRT's and 0.98× RRT*'s. Its path was 0.94× RRT's and about
equal to the others. Full ratios are in `global/report.json`.

- **RRT** failed seed 11, goal 1, at (3.78, −5.17): "rrt_planner found no
  path". This is the goal that both RRT and RRT* failed in the October 13
  `margin80` rerun.
- **RRT*** reached that goal this time. It failed seed 41, goal 2, at
  (−6.47, 5.83), also with no path found.
- Compared with October 13 (margin80): RRT 14/15 and RRT* 14/15 then too.
  PRM rose from 12/15 to 15/15; its three October failures followed a
  controller TF freeze. No trial in this round froze.

**Clearance.** The worst clearance is the trace's distance to the nearest
occupied map cell, minus the 0.12 m robot radius, at 0.05 m cells. Four goals
went below zero: 2 RRT, 1 RRT* and 1 PRM. On those, the sampled paths cut
corners within one cell of a wall. Contact is not measured, and no goal was
lost to it.

## Local planners ([local/report.md](local/report.md))

| Controller | Goals reached | Sim time median | Path median | Worst footprint clearance |
|---|---|---|---|---|
| robot_lab `regulated_pure_pursuit` | **15/15** | 29.4 s | 6.77 m | 0.261 m |
| robot_lab `follow_the_gap` | **15/15** | 28.3 s | 6.74 m | 0.163 m |
| Nav2 `pure_pursuit` (Regulated Pure Pursuit plugin) | **15/15** | 39.6 s | 6.78 m | 0.234 m |
| Nav2 `mppi_controller` (with the Arrival fallback) | **15/15** | 74.8 s | 6.88 m | 0.184 m |
| Nav2 `dwb_local_planner` | 14/15 | 32.4 s | 6.94 m | 0.261 m |

The robot_lab methods serve Nav2 FollowPath through the adapter, as in October.

Paired against robot_lab `regulated_pure_pursuit`: Follow-the-Gap took
0.99× its time, DWB 1.13×, the Nav2 plugin 1.32× and MPPI 2.56×.

- **DWB** failed seed 23, goal 2, at (−2.52, −4.62), ending 1.20 m short
  with "Failed to make progress". October's DWB failure was the same goal,
  1.18 m short.
- **MPPI** was 7/15 in October. All eight October failures came within
  0.05–0.19 m of the goal and did not settle into the 0.07 m / 0.03 rad
  window. Since October 16 (`99e2e784`), MPPI on a differential base has
  had an Arrival fallback: the Nav2 Regulated Pure Pursuit plugin finishes
  on the same path after MPPI's attempt fails. **15/15 counts MPPI
  together with the fallback**; MPPI alone (`arrival_controller:=false`)
  was not re-run.
- Here the progress checker aborted 12 times, on 11 of the 15 goals.
  MPPI ran for 42–64 s before each first abort, and Arrival then
  finished in 12–28 s. On seed 11, goal 0, a second abort came 30 s after
  the first, before the goal succeeded. That is the 2.6× time.
- The [October 16 measurement](../open-items-2026-10-16/README.md) of the
  same fallback reached 15/15 too, but with 9 aborts and a median of 33 s
  per goal, against 74.8 s now. MPPI's parameters and its tree are
  unchanged since then (`git diff 5e81ea69..52964578` on the controller
  and Bumperbot configs and the MPPI tree). The slowdown is not yet
  explained, and its launch logs are not on SSD to compare.

## Limits

- One robot, one map and one simulator; 15 goals per method.
- Only the five methods of each category that run on this Bumperbot
  protocol today were repeated. NavFn, Hybrid-A*, Lattice and Voronoi are
  not part of this round; their October results stand.
- The clearance is measured against the map, not by contact sensing.

## Reproduce

```bash
docs/status/evidence/v10-planner-comparisons-2026-11-22/run_round.sh
python3 scripts/planner_comparison_report.py <run>/global --markdown report.md > report.json
```

Per-seed goal records and reports are copied under `global/` and `local/`.
Bags, launch logs and truth traces are on SSD under
`robot_lab_runtime/v10-planner-cmp-20261122`. Their hashes are in
`runtime-sha256.txt`. `progress.txt` lists each trial's exit status.
