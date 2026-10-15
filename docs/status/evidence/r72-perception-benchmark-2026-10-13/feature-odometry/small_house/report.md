# R7.2 feature-based RGB-D visual odometry on recorded simulator frames

Revision `91931ed`. Each method starts at the true pose; errors are planar, against simulator truth.

## gazebo_bumperbot_small_house.json.tf.bag

3755 of 3755 RGB-D pairs (every 1), 124 s, 28.4 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 0.540 | 0.813 | 2.86 | 12.06 | 3 | 539.0 | 138 | 8.3 |
| akaze | 7.951 | 8.104 | 28.49 | 100.61 | 180 | 154.1 | 61 | 21.9 |
| brisk | 1.254 | 2.594 | 9.12 | 19.63 | 12 | 384.2 | 85 | 12.6 |
| sift | 5.702 | 8.294 | 29.16 | 93.01 | 123 | 140.3 | 46 | 31.3 |
| gftt_orb | 0.972 | 2.559 | 9.0 | 22.65 | 1 | 130.6 | 45 | 4.3 |
| wheel odometry (baseline) | 0.081 | 0.163 | 0.57 | 1.27 | - | - | - | - |

## isaac_bumperbot_small_house.json.tf.bag

622 of 623 RGB-D pairs (every 1), 124 s, 28.2 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 7.673 | 9.542 | 33.79 | 122.39 | 116 | 842.1 | 26 | 20.9 |
| akaze | 9.935 | 13.030 | 46.14 | 119.41 | 98 | 453.2 | 25 | 35.0 |
| brisk | 3.431 | 0.200 | 0.71 | 102.64 | 613 | 1000.0 | 7 | 82.3 |
| sift | 3.694 | 4.941 | 17.5 | 74.64 | 445 | 496.5 | 10 | 40.1 |
| gftt_orb | 4.828 | 8.893 | 31.49 | 95.93 | 392 | 630.8 | 11 | 16.7 |
| wheel odometry (baseline) | 0.747 | 0.216 | 0.76 | 9.73 | - | - | - | - |

## mujoco_bumperbot_small_house.json.tf.bag

2508 of 2509 RGB-D pairs (every 1), 501 s, 6.2 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 0.900 | 1.263 | 20.27 | 16.19 | 260 | 203.2 | 70 | 5.3 |
| akaze | 0.663 | 1.030 | 16.53 | 19.07 | 747 | 59.8 | 45 | 19.0 |
| brisk | 0.285 | 0.423 | 6.79 | 13.30 | 164 | 159.8 | 56 | 6.3 |
| sift | 1.002 | 1.395 | 22.38 | 11.53 | 421 | 72.6 | 38 | 23.1 |
| gftt_orb | 0.999 | 1.271 | 20.39 | 4.35 | 108 | 109.5 | 47 | 3.9 |
| wheel odometry (baseline) | 5.525 | 8.140 | 130.61 | 102.81 | - | - | - | - |
