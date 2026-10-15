# R7.2 feature-based RGB-D visual odometry on recorded simulator frames

Revision `91931ed`. Each method starts at the true pose; errors are planar, against simulator truth.

## gazebo_bumperbot_small_house.json.tf.bag

626 of 3755 RGB-D pairs (every 6), 124 s, 28.4 m driven.

| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | Inliers | ms |
|---|---|---|---|---|---|---|---|---|
| orb | 3.263 | 3.593 | 12.63 | 34.18 | 7 | 539.0 | 120 | 8.4 |
| akaze | 10.208 | 9.416 | 33.11 | 107.95 | 36 | 154.1 | 55 | 20.6 |
| brisk | 1.182 | 2.139 | 7.52 | 16.90 | 12 | 383.7 | 79 | 12.9 |
| sift | 11.563 | 10.891 | 38.29 | 125.87 | 41 | 140.8 | 39 | 30.3 |
| gftt_orb | 2.741 | 2.448 | 8.61 | 22.06 | 7 | 130.8 | 41 | 4.6 |
| wheel odometry (baseline) | 0.081 | 0.163 | 0.57 | 1.27 | - | - | - | - |
