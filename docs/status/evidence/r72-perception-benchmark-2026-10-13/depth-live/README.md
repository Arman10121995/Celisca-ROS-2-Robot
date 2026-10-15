# R7.2 on simulator depth clouds: Gazebo RGB-D, scored live (2026-10-15)

`scripts/r7_2_depth_live.py` ran beside a Gazebo Bumperbot drive: the
26 m nav_obstacle loop of `scripts/slam_mapping_check.sh`, as its
`SIDECAR`.

**Sampling.** Every 2 s of sim time it took the latest `/oakd/points` cloud
(320 × 240 RGB-D, camera optical frame) and thinned it to at most 8000
points.

**Frames.** It moved the cloud into `base_footprint` through TF, then into
the world with the simulator's truth pose.

**Labels:**
- ground: world z within 3 cm of the floor (the floor points' median z was
  0.000 m);
- obstacle: higher, and within 0.12 m (in x, y) of an occupied cell of the
  nav_obstacle reference map; the object is that cell's connected
  component (13 in the map);
- other: everything else. Only 0-542 points over the whole run, left out.

The same robot_lab_algorithms kernels as the nodes, at the nodes'
settings. Run of 11:51 (`4a9fc63`, 53 clouds, 390,337 labelled points);
an earlier run at 11:32 without the optical-frame rows gave the same
numbers within 0.001 ([first](gazebo_bumperbot_depth_first.json)).

## Ground removal (per point)

| Method | Frame | Precision | Recall | ms p50 |
|---|---|---|---|---|
| pointcloud_segmenter (height 0.1 m) | robot (z up) | 0.972 | 0.999 | **2.1** |
| ransac_ground_removal (0.04 m, 200 iterations) | robot (z up) | **0.987** | **1.000** | 43 |
| pointcloud_segmenter, as the node ran before | camera optical | - (no ground found) | 0.000 | 1.5 |
| ransac_ground_removal, as the node ran before | camera optical | 0.122 | 0.028 | 22 |

**The nodes did not work on a depth camera until this fix.**
- In the optical frame z is depth and y points down. The height threshold
  therefore kept nothing as ground.
- RANSAC's near-vertical normal test chose walls facing the camera, so it
  labelled 3% of the floor as ground, with 12% precision.
- Both nodes now move each cloud into `target_frame` (default
  `base_footprint`) before segmenting (`cdb8b34`).

On this flat floor, upright, both find essentially all of the ground.
RANSAC also leaves out more obstacle bases (precision 0.987 against
0.972) at 20 times the cost. The synthetic benchmark's 2° floor is where a
height threshold breaks.

## Object clustering (labelled obstacle points)

An object view counts when at least 60 of its points are in the cloud; it
is found when one cluster holds at least 80% of them with at least 80%
purity.

| Method | Object views found | Clusters per cloud | ms p50 |
|---|---|---|---|
| euclidean_clusterer (0.12 m, 20 points) | **100/138** (72%) | 6.1 | 35 |
| dbscan_clusterer (eps 0.12 m, 8 samples) | 86/138 (62%) | 6.7 | 35 |

The misses are mostly long wall components seen in pieces through gaps
(a wall run is one reference object, but the camera sees separate
stretches of it). DBSCAN's density requirement splits sparse far
surfaces more often.

## Limits

- One drive, one robot, one flat world, Gazebo only (the MuJoCo and Isaac
  RGB-D bridges were not scored).
- Labels come from the 2D reference map and floor height. Overhangs and
  objects absent from the 2D map would be mislabelled; here "other"
  stayed at 0-0.1% of the points.
