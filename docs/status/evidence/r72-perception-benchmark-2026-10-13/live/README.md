# R7.2 perception on simulator scans (2026-10-13)

The [offline benchmark](../README.md) clusters synthetic scenes of round
obstacles and named *"runs on Gazebo `/scan` and depth clouds"* as its next
step. This is the 2D-laser half of that: the same
`robot_lab_algorithms.perception` kernels run on the real 360-beam `/scan`
streams the R7.3 simulator runs recorded, on the task a ground robot
actually faces in nav_maze — split the scan at the corners between wall
segments. (The depth-cloud half still needs a recorded cloud; the bags hold
`/scan` only.)

Bumperbot drove the R4.3 goals on nav_maze (A*, seeds 23/37/53, Regulated
Pure Pursuit, AMCL), once in Gazebo and once in MuJoCo; the bags are the
R7.3 live-scan bags (`EXTRA_BAG_TOPICS=/scan scripts/planner_comparison.sh`,
kept on the SSD in `robot_lab_runtime/claude-r73-live-{gazebo,mujoco}/`).

**Ground truth is the map.** For each scan `scripts/r7_2_live_scan_replay.py`
ray-casts the nav_maze occupancy grid from the recorded `/odom/ground_truth`
pose (the ray-cast reproduces the recorded ranges to 0.010 m median). A
*true* boundary is two adjacent valid reference beams more than 0.20 m apart
in the world — a wall corner — with max-range beams breaking adjacency. Each
kernel clusters the real scan; a *predicted* boundary is two adjacent valid
beams it puts in different clusters. Boundaries are matched within 2 beams
and scored precision / recall / F1. The breakpoint / eps / tolerance is set
to the maze wall-segment scale (0.30 m), not the round-obstacle scale of the
synthetic scene. Every 40th scan after a 20-scan warm-up: 14-20 scans per
run, ~20 true segments each. Code as in `07a2cd0`.

## Results (boundary precision / recall / F1, both simulators)

| Method | Gazebo P / R / F1 | MuJoCo P / R / F1 | Clusters / scan | ms p50 |
|---|---|---|---|---|
| Obstacle detector (single linkage) | 0.851 / 0.543 / **0.663** | 0.862 / 0.553 / **0.674** | ~12 | 2.1 |
| Scan clusterer, fixed breakpoint | 0.796 / 0.544 / 0.646 | 0.862 / 0.553 / 0.674 | ~13 | 0.33 |
| Scan clusterer, adaptive breakpoint | 0.632 / 0.213 / 0.319 | 0.702 / 0.194 / 0.304 | ~7 | 0.59 |
| DBSCAN | 0.659 / 0.147 / 0.240 | 0.586 / 0.106 / 0.180 | ~6 | 2.6 |
| Euclidean clustering | 0.659 / 0.147 / 0.240 | 0.586 / 0.106 / 0.180 | ~6 | 2.2 |

Per-seed numbers are in the `results.json` files; both simulators agree
seed-for-seed (F1 within ~0.05).

## What it shows

- **Precision is high, recall is the ceiling.** The single-linkage obstacle
  detector and the fixed scan clusterer split ~85% of the boundaries they
  predict correctly, but find only ~54% of the true wall corners. The task
  is genuinely hard: ~20 real segments per scan, many short, some seen at
  grazing angles where the beams are nearly collinear and no proximity rule
  can see the corner.
- **Adaptive breakpoints hurt here.** Borges-Aldon's range-scaled threshold
  is tuned for round obstacles at varying range; on a maze of long walls it
  merges the corners it should split, dropping recall to ~0.2.
- **DBSCAN and Euclidean clustering fragment the walls.** With `min_points`
  3 they reach only ~6 clusters per scan against ~20 true segments — most of
  the wall stays one loose cluster — so recall is ~0.1-0.15. The connected-
  component clusterers on raw 2D points do not respect scan adjacency.
- **The cheapest method is nearly the best.** The fixed scan clusterer is
  ~6x faster than DBSCAN (0.33 ms vs 2.6 ms) and matches the single-linkage
  detector, because it clusters in beam order, which is exactly what a
  wall-corner split needs.
- **Limits.** One robot, one map, both simulators replayed offline (the live
  nodes publish the same clusters, but their timing in a running stack is
  not measured here). The depth-cloud stratum of R7.2 (ground removal,
  3D clustering) is not covered — the bags have no point cloud.

## Reproduce

```bash
python3 scripts/r7_2_live_scan_replay.py --map nav_maze --out <out>/gazebo \
  /workspace/molar/robot_lab_runtime/claude-r73-live-gazebo/*.bag
python3 scripts/r7_2_live_scan_replay.py --map nav_maze --out <out>/mujoco \
  /workspace/molar/robot_lab_runtime/claude-r73-live-mujoco/*.bag
```

The bags stay on the SSD in `robot_lab_runtime/claude-r73-live-{gazebo,mujoco}/`.
