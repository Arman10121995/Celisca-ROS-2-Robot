# R7.2 feature_detection: feature-based RGB-D visual odometry (2026-10-15)

Status: **implemented, experimental**. The perception id
`feature_detection` named ORB-SLAM3, whose build here carries an OpenCV ABI
mismatch, so it was cataloged only.

`robot_lab_algorithms.visual_odometry` (`c16139b`, `a691fae`) is the lab's
own front end, using OpenCV and NumPy:

1. detect and describe features in each grey frame;
2. match them with a keyframe's features (two nearest neighbours, Lowe
   ratio 0.8);
3. lift the keyframe's matches to 3D with its depth image;
4. solve the camera motion with PnP inside RANSAC (2 px, 200 iterations).

A new keyframe is taken after 0.25 m, 10° or fewer than 40 inliers. A frame
with fewer than 12 inliers, or an implausible step (over 0.5 m or 30°), is
lost: the pose holds and the frame becomes the keyframe.

Five detectors (`detector:=`):

| Detector | Features | Descriptor, distance |
|---|---|---|
| `orb` (default) | FAST corners on an 8-level pyramid | rBRIEF, Hamming |
| `akaze` | nonlinear scale space | M-LDB, Hamming |
| `brisk` | AGAST corners over scales | BRISK, Hamming |
| `sift` | difference of Gaussians | SIFT, L2 |
| `gftt_orb` | Shi-Tomasi corners | rBRIEF, Hamming |

`perception:=feature_detection` starts the node through the dispatch. It
publishes:

- the base pose as `nav_msgs/Odometry` on `/perception/visual_odometry`,
  in frame `visual_odom` (identity at the first frame);
- the keyframe's features as a cloud on `/perception/features`.

It does not publish TF.

## Unit checks

`test_r7_visual_odometry.py` ray-casts a room whose walls and floor carry a
random texture and moves a camera 0.30 m and 15° through it. Every detector
recovers the path:

- AKAZE, BRISK, SIFT and Shi-Tomasi end 1.6-5.2 mm off.
- ORB ends 2.9 cm off. FAST corners on blurred noise localize worst; over
  five textures and paths ORB ended 1-7 cm off, the Shi-Tomasi corners with
  the same descriptor 0.5-1.4 cm.

## On the nav_obstacle loop, four simulators (`nav_obstacle/`, round 23)

**Recording.** Bumperbot drove the 26 m nav_obstacle loop of
`scripts/slam_mapping_check.sh` with `perception:=feature_detection`. The
RGB-D stream, simulator truth and wheel odometry were recorded:

- Gazebo at 30 Hz (3187 frames);
- MuJoCo, PyBullet and Isaac at 5 Hz (about 500 frames each).

**Replay.** `scripts/r7_2_feature_odometry_replay.py` ran all five
detectors over the same frames. Results are in
[report.md](nav_obstacle/report.md) and
[results.json](nav_obstacle/results.json).

**Every detector failed in every simulator.**
- Position RMSE was 3.5-7.1 m and heading RMSE 38-111°.
- 222-2571 frames were lost: 27% of a run at best, 99% at worst.
- A wheel-odometry baseline was available in Gazebo (0.10 m RMSE) and
  Isaac (2.9 m, heading RMSE 21°).
- Final errors near zero in some Isaac rows only mean those methods never
  moved: the loop ends at the start.

**Why.** [rgb_depth_frames.png](nav_obstacle/rgb_depth_frames.png) shows
an RGB frame and its depth for Gazebo, MuJoCo, PyBullet and Isaac, from
top to bottom.

- The nav_obstacle walls and floor are flat-shaded. The only features are
  wall edges and corners: 13-83 per frame on average outside Isaac.
- MuJoCo's lighting leaves the scene nearly black (grey standard deviation
  18, against 52-63 elsewhere).
- Isaac's RGB carries per-pixel noise, like unconverged ray tracing. Its
  detectors fill their 800-1000 feature budgets with noise that does not
  repeat between frames. Isaac RGB edges correlate 0.15 with depth edges;
  the other simulators reach 0.57-0.89.
- The drive turns on the spot at 0.8 rad/s. At 5 Hz that is 9° per frame,
  with only a few corners in view.

The live node, recorded in the same runs, lost tracking on the same scale:
674 of 3000 Gazebo frames by its own log.

This world cannot rank the detectors. A textured world follows (round 25,
small_house).

## Limits

- Frame-to-keyframe odometry only: no bundle adjustment, no loop closure.
- The Gazebo camera info's principal point is half a pixel off
  ([depth-node](../depth-node/README.md)); the replay uses it as given.
