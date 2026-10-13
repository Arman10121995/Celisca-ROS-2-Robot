# Perception Comparison: obstacle detection, ground removal and clustering

Measured October 13, 2026, at revision `45e8ecc`. The evidence and raw
results are in
[r72-perception-benchmark-2026-10-13](../status/evidence/r72-perception-benchmark-2026-10-13/README.md).

**Category:** perception
**Type:** measured comparison on synthetic scenes with known objects

## What is compared

Three perception jobs, each with known answers:

1. **Find obstacles in a laser scan.** Group the scan points into clusters;
   small clusters are obstacles.
2. **Remove the ground from a depth cloud**, so that only objects remain.
3. **Split the remaining points into objects.**

| Selection (`perception:=`) | Job | Idea |
|---|---|---|
| `obstacle_detector` | 1 | Join any two scan points closer than a distance (single linkage) |
| `scan_clusterer` | 1 | Walk along the scan; start a new cluster when two neighbouring beams jump apart. With `adaptive:=true`, the allowed jump grows with range |
| `pointcloud_segmenter` | 2 | Everything lower than a height threshold is ground |
| `ransac_ground_removal` | 2 | Fit the floor plane to random point triples; keep the plane most points agree with |
| `euclidean_clusterer` | 3 | Join points closer than a tolerance (PCL's Euclidean clustering) |
| `dbscan_clusterer` | 3 | Join points that have enough neighbours within a radius (DBSCAN) |

The equations, parameters and sources are in
`src/robot_lab_algorithms/config/r7_method_subrecords.yaml` (perception)
and the module `robot_lab_algorithms.perception`.

## Run

Benchmark (about a minute; no simulator):

```bash
python3 scripts/r7_2_perception_benchmark.py --out <dir>
```

On a robot, the scan methods read `/scan` and the cloud methods read
`/oakd/points`. They publish markers or clouds under `/perception/`:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav simulator:=gazebo \
  robot_model:=bumperbot map_name:=nav_obstacle perception:=scan_clusterer
```

The scan clusterer reads its `adaptive` parameter (default `false`) at
start-up.

In the GUI, pick the method in the **Perception** slot.

## The benchmark

**Laser.** 100 rooms of 8 × 6 m, each with six round obstacles (10-25 cm
radius) at random places. A 360-beam scan is taken from a random spot,
with 1 cm noise. A cluster smaller than 0.7 m counts as a detection when
its centre is near a real obstacle.

**Depth cloud.** 20 clouds of about 6,000 points: a floor tilted by 2°, a
wall, and four boxes of 20-50 cm.

Every method sees the same scenes.

## Results

**Obstacles in a laser scan:**

| Method | Precision (detections that are real) | Recall (obstacles found) | Time per scan |
|---|---|---|---|
| obstacle_detector | 0.79 | **0.998** | 1.2 ms |
| scan_clusterer, fixed jump | 0.77 | 0.996 | **0.8 ms** |
| scan_clusterer, adaptive | **0.86** | 0.989 | 1.1 ms |
| DBSCAN | 0.79 | **0.998** | 1.4 ms |
| Euclidean | 0.79 | **0.998** | 1.2 ms |

**Ground removal:**

| Method | Precision | Recall (ground found) | Time per cloud |
|---|---|---|---|
| pointcloud_segmenter (below 0.1 m) | 0.95 | 0.43 | **1 ms** |
| ransac_ground_removal | 0.93 | **1.00** | 38 ms |

**Object clustering:** both `euclidean_clusterer` and `dbscan_clusterer`
found all 80 boxes, in 16 ms per cloud.

## What the numbers say

- **Almost every visible obstacle is found by every method.** The
  difference is in false detections.
- **False detections come from the walls.** The rooms hold nothing else,
  so a false detection is a short piece of wall (cut off by an obstacle's
  shadow, or seen at a grazing angle) or a second piece of one obstacle.
  The adaptive scan clusterer tolerates bigger gaps far away, so it cuts
  walls less: 86% of its detections are real, against 77% for the fixed
  jump.
- **A cluster's centre is not the obstacle's centre.** The scan only sees
  the near side, so the centre of the points sits about 3 cm inside the
  near surface. Add the radius along the beam to find the true centre.
- **A height threshold fails on any slope.** On a 2° floor, the ground
  further than 2.9 m rises above 0.1 m and is taken for obstacles. RANSAC
  fits the real floor plane and finds all of it, but costs 35 times more.
- **Clustering separates well-spaced boxes either way.** The two methods
  differ only on sparse surfaces, which DBSCAN breaks into more pieces.

## Limits

- The scenes are synthetic: perfect circles and boxes, no missing returns
  from dark or shiny surfaces, no motion.
- A run on Gazebo scans and depth clouds is the next step.

## Related tutorials

- [Tutorial index](README.md)
- [Numerical learning notes](perception.md)
- [Measured-run and failure workflow](../WORKFLOW.md)

*Part of R9.2: Seven Real Comparison Tutorials. See [ROADMAP.md](../../ROADMAP.md).*
