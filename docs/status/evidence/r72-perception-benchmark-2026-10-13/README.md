# R7.2: perception kernels on synthetic scenes (2026-10-13)

Status: **implemented, experimental**. The kernels are measured on seeded
synthetic laser scans and depth clouds with known objects, and on the real
`/scan` streams the simulator runs recorded ([live](live/README.md)). The
depth-cloud half on simulator data is still pending.

## What was built

`robot_lab_algorithms.perception` (revision `45e8ecc`; the node parameter
`adaptive` was added later on 2026-10-13):

| Selection (`perception:=`) | Input | Method |
|---|---|---|
| `obstacle_detector` | `/scan` | Single-linkage clusters of scan points (KD-tree neighbourhoods) |
| `scan_clusterer` | `/scan` | Breakpoints between consecutive beams; fixed distance, or adaptive with `adaptive:=true` (Borges and Aldon 2004) |
| `euclidean_clusterer` | `/oakd/points` | Region growing on a KD-tree (the PCL Euclidean cluster extraction) |
| `dbscan_clusterer` | `/oakd/points` | DBSCAN on a KD-tree |
| `pointcloud_segmenter` | `/oakd/points` | Ground / non-ground by a height threshold |
| `ransac_ground_removal` | `/oakd/points` | RANSAC plane, near-horizontal hypotheses only, least-squares refit |

`laser_scan_to_pointcloud` and `costmap_2d_observation` (Nav2's obstacle
layer) are separate upstream selections.

**What changed.**

- Euclidean clustering, DBSCAN and the obstacle detector were O(N²) pure
  Python. They now use KD-tree neighbourhoods, with the same results on
  the 38 existing tests.
- The scan clusterer gained adaptive breakpoints; a test checks that a
  far wall at a grazing angle stays whole.
- RANSAC ground removal:
  - skips hypotheses that are not near horizontal;
  - counts inliers vectorised, with a seeded random generator;
  - refits the plane to the inliers by least squares.

## Benchmark (`scripts/r7_2_perception_benchmark.py`)

### Laser obstacle detection

100 seeded scenes. Each scene is an 8 × 6 m room with six round obstacles
(radius 0.10-0.25 m), seen by a 360-beam scan with 0.01 m noise.

- **Candidates:** clusters of at least 3 points with an extent below
  0.7 m.
- **True detection:** a candidate whose centroid lies within (radius
  + 0.15 m) of a visible obstacle (one hit by at least 3 beams).
- **Thresholds:** 0.15 m for every method; 0.1 m base for the adaptive
  clusterer.

| Method | Precision | Recall | Centroid to surface | ms p50 / max |
|---|---|---|---|---|
| obstacle_detector | 0.794 | **0.998** | -0.031 m | 1.16 / 1.36 |
| scan_clusterer, fixed | 0.766 | 0.996 | -0.030 m | **0.79** / 0.83 |
| scan_clusterer, adaptive | **0.862** | 0.989 | -0.031 m | 1.12 / 1.17 |
| DBSCAN (eps 0.15, 3 samples) | 0.794 | **0.998** | -0.031 m | 1.40 / 1.57 |
| Euclidean (0.15 m, 3 points) | 0.794 | **0.998** | -0.031 m | 1.20 / 1.38 |

### Ground removal and object clustering (depth cloud)

20 seeded clouds of about 6,000 points. Each has a ground plane tilted 2°
with 0.01 m noise, a wall, and four boxes of 0.2-0.5 m.

| Ground removal | Precision | Recall | ms p50 |
|---|---|---|---|
| pointcloud_segmenter (height < 0.1 m) | 0.947 | 0.429 | **1.1** |
| ransac_ground_removal | 0.929 | **1.000** | 38.3 |

Object clustering runs on the true non-ground points (boxes and wall). An
object counts as found when one cluster holds at least 80% of its points
with at least 80% purity. Parameters: RANSAC 0.04 m inlier distance and
200 iterations; Euclidean 0.12 m tolerance and 20 points minimum; DBSCAN
eps 0.12 m and 8 samples.

| Object clustering | Boxes found | Clusters per scene | ms p50 |
|---|---|---|---|
| euclidean_clusterer | 80/80 | 11.6 | 15.8 |
| dbscan_clusterer | 80/80 | 15.6 | 15.8 |

Raw numbers are in [results.json](results.json) and [report.md](report.md).

## What it shows

- **On a clean scan, the point-clustering methods agree.** Single
  linkage, DBSCAN with 3 samples and Euclidean clustering on the same
  0.15 m radius gave identical results.
- **Precision is set by the walls.** The rooms hold only walls and
  obstacles. A false candidate is therefore either a wall piece under
  0.7 m (cut off by obstacle shadows or seen at a grazing angle) or a
  second fragment of one obstacle.
  - The adaptive breakpoint raises its threshold with range, so it splits
    far walls less: precision 0.86 against 0.77 for the fixed threshold.
  - The larger far threshold can also join an obstacle to a neighbour or
    to the wall behind it (obstacles keep at least 0.35 m from both).
    That is the likely cause of the small recall drop to 0.99; the
    benchmark does not record it.
- **A centroid is not the obstacle centre.** A scan sees only the near
  side, so the cluster centroid sits about 3 cm inside the near surface.
  A tracker must add roughly the radius along the beam.
- **A height threshold cannot follow a tilted floor.** The 2° slope lifts
  the floor beyond 2.9 m above 0.1 m, so the threshold finds only 43% of
  the ground. RANSAC finds all of it, at 35 times the cost.
- **Both clusterers find every box.** They differ on the wall, which is
  sparse: 600 points over 7.5 m², about as far apart as the 0.12 m
  radius. DBSCAN, which also needs 8 neighbours per core point, most
  likely breaks it into more pieces: 15.6 clusters per scene against
  11.6.
- **Limits.** The scenes are synthetic: perfect circles and boxes, no
  reflectance dropouts, no motion blur or multi-echo returns. The 2D-laser
  half of the next step — real Gazebo/MuJoCo `/scan` — is in
  [live](live/README.md). The depth-cloud half — a live Gazebo RGB-D stream
  — is in [depth-live](depth-live/README.md). It found that the cloud
  nodes worked in the camera's optical frame, where neither ground method
  found the floor; they now segment in the robot frame (`cdb8b34`).

## Reproduce

```bash
python3 scripts/r7_2_perception_benchmark.py --out <dir>
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/robot_lab_algorithms \
  python3 -m pytest -q src/robot_lab_algorithms/test/test_r7_perception_algorithms.py
```
