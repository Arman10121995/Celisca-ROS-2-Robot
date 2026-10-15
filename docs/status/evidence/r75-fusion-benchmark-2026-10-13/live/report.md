# R7.5 replay of simulator IMU and wheel odometry

Revision `815e54e`. Attitude filters at their node defaults, started at the true attitude; tilt error = roll and pitch against truth after 5 s.

## gazebo_bumperbot_nav_obstacle.json.tf.bag

IMU 100 Hz; true trunk tilt RMS 0.59 deg, max 0.60 deg.

| Attitude filter | Tilt RMS (deg) | Tilt max (deg) | Roll / pitch bias (deg) |
|---|---|---|---|
| complementary | 0.17 | 0.41 | -0.04 / -0.05 |
| mahony | 0.24 | 0.48 | -0.06 / 0.00 |
| madgwick | 0.26 | 1.26 | -0.05 / -0.02 |
| eskf | 0.20 | 0.69 | -0.06 / -0.02 |

Pose over 25.4 m:

| Pose filter | Position RMSE (m) | Final (m) | Heading RMSE (deg) |
|---|---|---|---|
| wheel_imu_ekf | 0.013 | 0.023 | 0.10 |
| wheel_imu_ukf | 0.047 | 0.013 | 0.10 |
| wheel_dead_reckoning | 0.098 | 0.043 | 1.05 |

## gazebo_unitree_go2_nav_obstacle.json.tf.bag

IMU 200 Hz; true trunk tilt RMS 2.90 deg, max 6.23 deg.

| Attitude filter | Tilt RMS (deg) | Tilt max (deg) | Roll / pitch bias (deg) |
|---|---|---|---|
| complementary | 0.25 | 1.14 | -0.07 / -0.06 |
| mahony | 0.32 | 1.21 | -0.08 / -0.07 |
| madgwick | 0.53 | 1.30 | -0.01 / -0.34 |
| eskf | 0.78 | 2.76 | -0.14 / -0.13 |
