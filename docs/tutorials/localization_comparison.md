# Localization Comparison: scan-to-map matchers on nav_maze

Measured October 13, 2026, at revision `42d1ccb`. The evidence and raw
results are in
[r73-localization-benchmark-2026-10-13](../status/evidence/r73-localization-benchmark-2026-10-13/README.md).

**Category:** localization
**Type:** measured comparison on recorded trajectories with modelled scans

## What is compared

A robot knows the map and its starting pose. Each localizer keeps the pose
from then on:

1. Predict with the wheel-odometry increment since the last scan.
2. Align the new laser scan to the map, starting from that prediction.

| Selection (`localization:=`) | Method | Idea |
|---|---|---|
| `icp_localization` | ICP, point to point | Pair each scan point with the nearest wall cell, solve the best rigid motion in closed form, repeat with a shrinking pairing gate |
| `plicp_localization` | ICP, point to line | Pair the same way, but minimise the distance along the wall normal, so points can slide along a wall |
| `ndt_localization` | NDT | Replace the wall cells by one Gaussian per 0.5 m cell; climb the summed likelihood with Gauss-Newton |
| `correlative_localization` | Correlative scan matching | Score every pose on a grid (±0.3 m, ±10°, then a finer grid) against a likelihood field of distances to walls |
| `dead_reckoning` | Odometry only | The baseline: what happens without a scan |

The equations, parameters and sources are in
`src/robot_lab_algorithms/config/r7_method_subrecords.yaml` (localization)
and the module `robot_lab_algorithms.scan_matching`. Nav2 AMCL, slam_toolbox
and RTAB-Map are separate selections and are not part of this run.

## Run

Benchmark (a few minutes; no simulator):

```bash
python3 scripts/r7_3_localization_benchmark.py --out <dir> \
  --traces <R4.3 Gazebo trace.json> ...
```

On a robot, in a mode with a map:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav simulator:=gazebo \
  robot_model:=bumperbot map_name:=nav_maze localization:=plicp_localization
```

The node starts from `/initialpose` (the RViz 2D Pose Estimate) or the
first `/amcl_pose`. It publishes `/robot_lab/<method>_pose` and does not
publish TF. In the GUI, pick the method in the **Localization** slot.

## The benchmark

- **Truth:** three Bumperbot drives through nav_maze, recorded in Gazebo
  (17.8-25.3 m each).
- **Scans:** 360 beams, ray-cast on the occupancy map Nav2 uses, with 2 cm
  noise and 3% dropped beams.
- **Odometry:** deliberately biased: distances 3% long, turns 2% long, and a
  heading drift of 0.5° per metre.

Every localizer gets exactly the same scans and odometry (two noise seeds
per trajectory).

## Results

| Localizer | Position RMSE | Max error (mean over runs) | Heading RMSE | Time lost (> 0.5 m) | Compute per scan, median |
|---|---|---|---|---|---|
| ICP, point to point | **7 mm** | 16 mm | 0.21° | 0% | 2.7 ms |
| ICP, point to line | 8 mm | 17 mm | 0.23° | 0% | **1.8 ms** |
| NDT | 8 mm | 18 mm | 0.24° | 0% | 3.7 ms |
| Correlative | 24 mm | 104 mm | 0.72° | 0% | 7.6 ms |
| Dead reckoning | 549 mm | 892 mm | 3.30° | 41% | 0.07 ms |

## What the numbers say

- **Any scan matcher removes the odometry drift.** The biased odometry alone
  is more than half a metre off for 41% of the run. All four matchers stay
  within 2 cm on average.
- **Point-to-line ICP is the best value.** It is as accurate as
  point-to-point and a third faster per scan. Minimising the distance
  along the wall normal lets points slide along walls, which is known to
  converge in fewer iterations (Censi 2008).
- **NDT needs a careful map.** Built from every occupied cell, the Gaussians
  of thick walls sat inside the wall and pulled the pose 3-4 cm off. Built
  from the wall surfaces only, it matches ICP.
- **Correlative matching is coarse but robust.** Its fine grid (2.5 cm,
  0.5°) limits its accuracy to about 2-3 cm. It needs no convergence basin,
  so it is the one to use after a large odometry error or a kidnapping. In
  one run its error reached 30 cm at a similar-looking spot before it
  came back.

## On simulator scans

The same matchers replayed recorded Gazebo and MuJoCo runs: the robot's
own scans and odometry, with the AMCL that ran live as a reference
([evidence](../status/evidence/r73-localization-benchmark-2026-10-13/live/README.md)).

| Localizer | Gazebo | MuJoCo | With biased odometry |
|---|---|---|---|
| ICP, point to point | **3.7 cm** | **3.6 cm** | unchanged |
| ICP, point to line | 4.6 cm | 4.5 cm | unchanged |
| NDT | 4.7 cm | 4.6 cm | unchanged |
| Correlative | 5.1 cm | 5.3 cm | unchanged |
| AMCL (live) | 7.9 cm | 8.8 cm | - |
| Odometry alone | 3.4 cm | 0.7 cm (truth) | 33-38 cm |

- Real scans cost the matchers a few centimetres, mostly a fixed offset
  between the map grid and the simulated walls.
- They still beat AMCL on the same scans, and they do not care how bad
  the odometry is.
- Gazebo's wheel odometry is so good on a flat floor that it alone is as
  accurate here. MuJoCo feeds the simulator's truth as odometry unless
  `odometry_source:=wheels` is set.

## Limits

- The ray-cast benchmark matches against the same map it scans. The
  recorded runs add real simulator scans, but both use one robot on one
  flat map.
- The localizers start at the true pose; global localization and
  recovery from a wrong pose are not tested.
- slam_toolbox's localization mode is not compared yet.

## Related tutorials

- [Tutorial index](README.md)
- [Numerical learning notes](localization.md)
- [Sensor fusion comparison](sensor_fusion_comparison.md)
- [Measured-run and failure workflow](../WORKFLOW.md)

*Part of R9.2: Seven Real Comparison Tutorials. See [ROADMAP.md](../../ROADMAP.md).*
