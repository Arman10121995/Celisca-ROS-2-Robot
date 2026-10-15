# R7.2 depth_image_to_pointcloud on all four simulators (2026-10-15)

Status: **implemented, experimental**. The perception id
`depth_image_to_pointcloud` was cataloged only: depth_image_proc is not
installed here. `robot_lab_algorithms.perception` now has the same pinhole
back-projection (`ee85db4`):

- input: `/oakd/depth/image_raw` (32FC1 metres or 16UC1 millimetres) and
  `/oakd/rgb/camera_info`;
- output: `/perception/depth_points`, in the depth image's frame;
- parameters: `stride` (default 2: every second row and column),
  `min_range`/`max_range` (0.05-10 m), `frame_id` (override).

`perception:=depth_image_to_pointcloud` starts it through the dispatch.

## How it was measured (`v1/`, 12:02-12:15, `083f212` tree with `ee85db4`'s code)

Bumperbot drove the 26 m nav_obstacle loop of
`scripts/slam_mapping_check.sh` in each simulator. Its `SIDECAR` ran
three processes:

1. the node: stride 1 in Gazebo, the default stride 2 elsewhere;
2. `scripts/r7_2_depth_projection_check.py`, which counted images and
   clouds and timed each cloud from its image's arrival. In Gazebo it also
   compared the node's cloud with Gazebo's own `/oakd/points` of the same
   stamp;
3. `scripts/r7_2_depth_live.py --topic /perception/depth_points`, which
   scored ground removal and clustering on the node's cloud against
   simulator truth, as for Gazebo's own cloud
   ([depth-live](../depth-live/README.md)).

## Results

| Simulator | Depth images | Clouds | Latency p50 / p95 | Points per cloud | Floor z (median) |
|---|---|---|---|---|---|
| Gazebo (30 Hz, stride 1) | 3177 | 2861 | 6.6 / 10.3 ms | 55,116 | +1.5 mm |
| MuJoCo (5 Hz) | 523 | 522 | 4.4 / 7.5 ms | 13,566 | +2.7 mm |
| PyBullet (5 Hz) | 478 | 477 | 3.7 / 5.6 ms | 13,665 | +3.0 mm |
| Isaac (5 Hz) | 518 | 507 | 3.3 / 7.1 ms | 13,752 | +1.5 mm |

- Gazebo's missing clouds were not split into start-up and drops in this
  run (the counter was added for the next runs). In PyBullet and Isaac,
  2 images came before the first cloud.
- In Isaac, 9 images during the drive got no cloud.
- The latency maximum was 127 ms in Gazebo and at most 21 ms elsewhere.

Ground removal and clustering on the node's cloud (`r7_2_depth_live.py`,
about 50 clouds per run):

| Simulator | Height threshold P / R | RANSAC P / R | Euclidean objects | DBSCAN objects |
|---|---|---|---|---|
| Gazebo | 0.973 / 0.999 | 0.983 / 1.000 | 115/132 | 90/132 |
| MuJoCo | 0.955 / 1.000 | 0.980 / 1.000 | 103/127 | 88/127 |
| PyBullet | 0.974 / 1.000 | 0.979 / 1.000 | 93/118 | 77/118 |
| Isaac | 0.974 / 0.998 | 0.984 / 1.000 | 103/129 | 85/129 |

- Gazebo's own cloud, in the earlier run, scored 0.972 / 0.999 and
  0.987 / 1.000 with 100/138 and 86/138.
- These are the first depth-cloud scores for MuJoCo, PyBullet and Isaac,
  which publish no cloud of their own.
- Every simulator's floor lies within 3 mm of z = 0, and the obstacles on
  the reference map's walls.
- RANSAC is again the more precise ground remover in every backend, at
  about 20 times the cost.

## Agreement with Gazebo's own cloud (Gazebo, 85 stamps)

| Distance from each node point to Gazebo's nearest point | Median over stamps | Worst stamp |
|---|---|---|
| Median | 3.7 mm | 4.8 mm |
| 95th percentile | 16.2 mm | 20.5 mm |
| Maximum | 26.8 mm | 31.9 mm |

The two clouds are the same pixels, but they differ systematically. The
largest distance, 31.9 mm at the 10 m range limit, is exactly
0.5 × √2 × 10 m / fx (fx = 221.77). That is half a pixel in both image
axes.

- Gazebo's camera info gives the principal point as (160, 120), half the
  image size.
- Renderers put pixel centres at u + 0.5, so an image centred on the
  optical axis has its principal point at (159.5, 119.5) in the ROS and
  OpenCV convention.
- The floor sitting 1.5-3 mm high in every backend fits the same offset.
  The lab's MuJoCo, PyBullet and Isaac bridges also publish (160, 120),
  from `robot_lab_utils.camera_model.intrinsics`.

The rerun below checks this directly.

## Limits

- One drive per simulator, one flat world, Bumperbot's OAK-D model
  (320 × 240).
- Latency is wall-clock on the loaded host, with the simulator and the
  scorers running.
