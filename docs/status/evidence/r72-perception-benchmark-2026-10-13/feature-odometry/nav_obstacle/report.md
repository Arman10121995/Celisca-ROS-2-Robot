# R7.2 feature-based RGB-D visual odometry on recorded simulator frames

Revision `d65fc2e`. Each method starts at the true pose; errors are planar, against simulator truth.

## mujoco_bumperbot_nav_obstacle.json.tf.bag

522 of 523 RGB-D pairs (every 1), 104 s, 25.4 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 4.915 | 10.073 | 39.66 | 102.98 | 295 | 46.6 | 22 | 3.2 |
| akaze | 4.104 | 9.015 | 35.49 | 100.38 | 326 | 21.5 | 17 | 21.8 |
| brisk | 3.953 | 8.217 | 32.35 | 108.02 | 342 | 29.7 | 15 | 1.6 |
| sift | 4.422 | 2.503 | 9.85 | 106.13 | 477 | 13.1 | 12 | 29.9 |
| gftt_orb | 4.171 | 3.057 | 12.04 | 107.95 | 435 | 30.1 | 12 | 2.4 |

## pybullet_bumperbot_nav_obstacle.json.tf.bag

477 of 478 RGB-D pairs (every 1), 97 s, 25.6 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 3.627 | 7.608 | 29.68 | 91.51 | 297 | 83.0 | 17 | 3.9 |
| akaze | 4.591 | 7.191 | 28.06 | 100.12 | 282 | 46.8 | 14 | 22.3 |
| brisk | 3.866 | 6.138 | 23.95 | 85.67 | 326 | 81.8 | 16 | 3.1 |
| sift | 4.968 | 2.383 | 9.3 | 96.09 | 410 | 30.4 | 11 | 30.2 |
| gftt_orb | 4.364 | 4.429 | 17.28 | 97.21 | 342 | 66.0 | 15 | 3.3 |

## isaac_bumperbot_nav_obstacle.json.tf.bag

527 of 528 RGB-D pairs (every 1), 105 s, 25.4 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 3.894 | 7.217 | 28.47 | 90.78 | 222 | 800.0 | 17 | 19.6 |
| akaze | 4.379 | 9.402 | 37.09 | 101.19 | 265 | 229.9 | 13 | 34.0 |
| brisk | 5.907 | 0.226 | 0.89 | 110.72 | 525 | 1000.0 | 6 | 62.4 |
| sift | 5.876 | 0.121 | 0.48 | 110.73 | 524 | 320.3 | 7 | 34.7 |
| gftt_orb | 5.919 | 0.070 | 0.28 | 110.96 | 524 | 638.0 | 6 | 16.6 |
| wheel odometry (baseline) | 2.862 | 6.385 | 25.19 | 21.08 | - | - | - | - |

## gazebo_bumperbot_nav_obstacle.json.tf.bag

3187 of 3187 RGB-D pairs (every 1), 105 s, 25.4 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 7.073 | 14.701 | 57.8 | 38.44 | 860 | 49.7 | 28 | 3.8 |
| akaze | 4.956 | 11.235 | 44.17 | 105.38 | 1614 | 29.4 | 22 | 20.4 |
| brisk | 4.682 | 9.565 | 37.61 | 66.90 | 1004 | 41.7 | 21 | 2.7 |
| sift | 3.467 | 5.208 | 20.48 | 98.60 | 2571 | 16.8 | 14 | 29.6 |
| gftt_orb | 4.567 | 5.041 | 19.82 | 90.30 | 1250 | 46.3 | 18 | 3.3 |
| wheel odometry (baseline) | 0.102 | 0.047 | 0.18 | 1.09 | - | - | - | - |
