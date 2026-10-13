# R7.3: scan-to-map localizers on nav_maze (2026-10-13)

Status: **implemented, experimental**. The localizers are measured on scans
ray-cast on the real nav_maze map along robot trajectories recorded in
Gazebo, and on recorded Gazebo and MuJoCo scans against the AMCL that ran
with them ([live/](live/README.md)).

## What was built

`robot_lab_algorithms.scan_matching` (revision `42d1ccb`); the localization
node ids now run it:

| Selection (`localization:=`) | Method |
|---|---|
| `icp_localization` | ICP, point to point: nearest map surface cells (KD-tree), closed-form 2D rigid alignment (SVD), shrinking gate |
| `plicp_localization` (new) | ICP, point to line: Gauss-Newton on distances to map normals (PCA of 8 neighbours), robust weights |
| `ndt_localization` | 2D NDT: per-cell Gaussians on 0.5 m cells, reweighted Gauss-Newton with a line search |
| `correlative_localization` (new) | Correlative scan matching: coarse-to-fine search of +-0.3 m, +-10° on a Gaussian likelihood field |
| `dead_reckoning` | The odometry increments alone (baseline) |

**How the nodes run.** Each node:

- subscribes to `/map`, `/scan` and the `odom -> base` transform;
- takes its start pose from `/initialpose` or the first `/amcl_pose`;
- predicts with the odometry increment, then aligns the scan;
- refuses corrections larger than 0.5 m;
- publishes `PoseWithCovarianceStamped` on `/robot_lab/<method>_pose`,
  beside Nav2 AMCL (it does not publish TF).

The map is reduced to its wall-surface cells, those touching free space.
Interior cells of thick walls had biased NDT inward by 3-4 cm.

**What the ids ran before.**

- `dead_reckoning` had no subscriptions.
- ICP estimated rotation about the map origin with brute-force matching.
- A Python "AMCL" and an "RGB-D SLAM" stand-in were selectable. They are
  retired in favour of Nav2 AMCL and RTAB-Map.

## Benchmark (`scripts/r7_3_localization_benchmark.py`)

**Truth.** Three Bumperbot trajectories driven in Gazebo on nav_maze (the
R4.3 evaluation traces, A* seeds 23, 37 and 53), resampled to 5 Hz:
19.6, 17.8 and 25.3 m. They are stored in
[trajectories.json](trajectories.json).

**Scans.** 360 beams ray-cast on the nav_maze occupancy map that Nav2 uses,
with 0.02 m range noise, 3% dropouts and a 12 m range.

**Odometry.** The truth increments with +3% distance scale, +2% rotation
scale, a heading drift of 0.5°/m and white noise. Every localizer gets the
same input (seeds 1, 2).

| Localizer | Position RMSE | Max error (mean of runs) | Heading RMSE | Lost (> 0.5 m) | Compute p50 / max p99 |
|---|---|---|---|---|---|
| ICP point to point | **0.007 m** | 0.016 m | 0.21° | 0% | 2.7 / 6.6 ms |
| ICP point to line | 0.008 m | 0.017 m | 0.23° | 0% | **1.8 / 2.7 ms** |
| NDT | 0.008 m | 0.018 m | 0.24° | 0% | 3.7 / 6.0 ms |
| Correlative | 0.024 m | 0.104 m | 0.72° | 0% | 7.6 / 8.8 ms |
| Dead reckoning | 0.549 m | 0.892 m | 3.30° | **40.9%** | 0.07 ms |

Per-run numbers are in [results.json](results.json) and the table in
[report.md](report.md).

## What it shows

- **All four scan matchers hold the pose to about a centimetre,** while
  the biased odometry alone drifts past 0.5 m on 41% of the scans.
- **Point-to-line ICP is the fastest** (1.8 ms median, against 2.7 ms for
  point to point) at the same accuracy.
- **Correlative matching is resolution-limited.**
  - Its 0.025 m / 0.5° fine grid sets a floor of 2-3 cm.
  - Once, on seed 37, it picked a neighbouring corridor-like optimum
    (max 0.30 m) before recovering.
  - It needs no initial convergence basin and is the most robust to a bad
    guess, but here the odometry guess is always good.
- **Limits.**
  - The map and the scans come from the same grid. The scans are
    noise-perturbed, but there is no map-world mismatch (furniture,
    glass, moving people) and no beam divergence.
  - Simulator scans: see below.

## On simulator scans ([live/](live/README.md))

Recorded Gazebo and MuJoCo runs (Bumperbot, nav_maze, three runs each)
replayed through the same matchers:

- the matchers hold **3.6-5.4 cm**, against **7.9-8.8 cm** for the AMCL that
  ran live on the same scans;
- their error is a fixed offset of 2-5 cm, the same in both simulators,
  which points at the map grid's registration rather than noise;
- with the benchmark's biased odometry they change by less than 1 mm,
  while the odometry alone drifts to 0.33-0.38 m.

Seeded at a wrong pose, point-to-point ICP and correlative matching recover
from shifts of up to 2 m along the corridor (point-to-line ICP reliably up to
1 m), and all four from 15-30° turns. NDT stops 0.26-0.29 m
off. Only correlative matching recovers from a 1.5 m shift across the
corridor (2/3). None recovers from 90° or 180°: these are local trackers,
and global localization stays AMCL's job (live/README.md, "Recovery").
slam_toolbox on the same data is not covered.

## Reproduce

```bash
python3 scripts/r7_3_localization_benchmark.py --out <dir> --traces <gazebo trace.json> ...
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/robot_lab_algorithms/test/test_scan_matching.py
```
