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

This world cannot rank the detectors.

**Wheel baselines.** A second replay
([nav_obstacle/wheel_baseline](nav_obstacle/wheel_baseline/report.md))
starts the wheel odometry at its first sample, which comes after the first
camera frame. Wheel odometry then holds:

- 0.18 m RMSE in MuJoCo;
- 4.1 m in PyBullet, where it loses 66° of heading RMSE;
- (from the first replay) 0.10 m in Gazebo and 2.9 m in Isaac.

## In a textured house (`small_house/`, round 25, 16:37-16:54)

**Setup.** Bumperbot drove a 32.6 m loop through small_house, with
waypoints (-6, 3), (6, 3) and (6, -3). The world uses the AWS RoboMaker
residential models. Frames in [rgb_frames.png](small_house/rgb_frames.png),
Gazebo, MuJoCo and Isaac from top to bottom:

- Gazebo renders the models' textures: wood floor, patterned bedding.
- MuJoCo draws them untextured and dim.
- Isaac again adds per-pixel noise.

The replay ran over every frame
([report](small_house/report.md)), and over every sixth Gazebo frame,
which is 5 Hz like the other simulators ([every6](small_house/every6/report.md)).

| Gazebo, 28.4 m | Position RMSE | Final | Drift | Heading RMSE | Lost | Inliers | ms |
|---|---|---|---|---|---|---|---|
| orb, 30 Hz | **0.54 m** | 0.81 m | **2.9%** | 12° | 3 | 138 | 8.3 |
| gftt_orb, 30 Hz | 0.97 m | 2.56 m | 9.0% | 23° | 1 | 45 | **4.3** |
| brisk, 30 Hz | 1.25 m | 2.59 m | 9.1% | 20° | 12 | 85 | 12.6 |
| akaze, 30 Hz | 7.95 m | 8.10 m | 28% | 101° | 180 | 61 | 21.9 |
| sift, 30 Hz | 5.70 m | 8.29 m | 29% | 93° | 123 | 46 | 31.3 |
| brisk, 5 Hz | **1.18 m** | 2.14 m | 7.5% | 17° | 12 | 79 | 12.9 |
| gftt_orb, 5 Hz | 2.74 m | 2.45 m | 8.6% | 22° | 7 | 41 | 4.6 |
| orb, 5 Hz | 3.26 m | 3.59 m | 13% | 34° | 7 | 120 | 8.4 |
| wheel odometry | 0.08 m | 0.16 m | 0.6% | 1.3° | | | |

- **With texture, the odometry works.** ORB at 30 Hz loses 3 of 3755
  frames and ends 0.81 m off after 28.4 m (2.9%).
- **The live node agrees.** Its recorded `/perception/visual_odometry`
  in the same run scores the same: 0.54 m RMSE, 0.81 m final, 12°
  heading RMSE.
- **Errors come from heading.** A few degrees lost in the in-place turns
  (0.8 rad/s) carry into every later metre.
- **AKAZE and SIFT lose the turns.** They lose 123-180 frames, mostly
  while turning; each lost frame holds the pose while the robot turns.
- **At 5 Hz BRISK is best.** It barely changes (1.18 m), while ORB's error
  grows sixfold (3.26 m).
- Wheel odometry beats every visual method here (0.08 m). The visual
  odometry is a separate, drift-prone estimate, not a replacement.

**MuJoCo** got stuck in the house. It drove only 6.2 m in 501 s, with 67
back-offs and 137 of 148 route points skipped, and its slam map matched
the reference at precision 0.12.
- Its house does not match the 2D map in the drive's path (mean distance
  to the reference walls 1.2 m). This was not investigated further.
- The wheels spun against the obstacle: wheel odometry 5.5 m RMSE,
  against 0.29-1.0 m for the visual methods (BRISK 0.29 m).

**Isaac** drove the whole loop (slam 1.8 cm RMSE), but its noisy RGB
defeats every detector again: 3.4-9.9 m RMSE.

## What it shows

- The visual odometry needs texture, and among the lab's worlds and
  simulators only Gazebo's textured models give it.
- ORB is the right default at camera rate. BRISK holds up best when
  frames are far apart. The Shi-Tomasi corners are the cheapest
  (4.3 ms/frame) and rarely lose tracking (1 frame at 30 Hz), but drift
  three times more.
- Isaac's RGB noise and MuJoCo's untextured, dim scenes are simulator
  rendering issues that block visual methods there. They are recorded for
  the backend owners.

## Limits

- One drive per world and simulator.
- Frame-to-keyframe odometry only: no bundle adjustment, no loop closure.
- The Gazebo camera info's principal point is half a pixel off
  ([depth-node](../depth-node/README.md)); the replay uses it as given.
