# R7.2 feature-based RGB-D visual odometry on recorded simulator frames

Revision `91931ed`. Each method starts at the true pose; errors are planar, against simulator truth.

## mujoco_bumperbot_nav_obstacle.json.tf.bag

522 of 523 RGB-D pairs (every 1), 104 s, 25.4 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 4.915 | 10.073 | 39.66 | 102.98 | 295 | 46.6 | 22 | 3.4 |
| wheel odometry (baseline) | 0.184 | 0.429 | 1.69 | 1.54 | - | - | - | - |

## pybullet_bumperbot_nav_obstacle.json.tf.bag

477 of 478 RGB-D pairs (every 1), 97 s, 25.6 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 3.627 | 7.608 | 29.68 | 91.51 | 297 | 83.0 | 17 | 3.8 |
| wheel odometry (baseline) | 4.140 | 9.355 | 36.5 | 66.01 | - | - | - | - |
