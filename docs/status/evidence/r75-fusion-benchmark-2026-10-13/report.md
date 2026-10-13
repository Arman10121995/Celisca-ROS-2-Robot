# R7.5 sensor fusion benchmark (modelled sensors)

Revision `303e01a`, molar1 aarch64, Python 3.10.12, NumPy 1.26.4; seeds [1, 2, 3].

## Attitude (mean over evaluation seeds)

Gains chosen on seed 0: complementary tau=4, mahony kp=0.25, madgwick beta=0.02, eskf linear_accel_std=16.

| Filter | Overall RMS after 10 s (deg) | Settle to 2 deg (s) | Steady RMS roll / pitch (deg) | Steady max (deg) | RMS / max under 2 m/s^2 (deg) | Yaw drift (deg/min) | Compute p50 / p99 (ms) |
|---|---|---|---|---|---|---|---|
| complementary | 1.90 | 11.3633 | 0.82 / 2.25 | 3.39 | 2.69 / 6.18 | 40.11 | 0.039 / 0.054 |
| mahony | 1.49 | 15.18 | 0.81 / 1.48 | 6.29 | 2.46 / 4.75 | -0.56 | 0.085 / 0.107 |
| madgwick | 2.03 | 15.6967 | 1.69 / 1.83 | 8.46 | 2.96 / 6.82 | 39.19 | 0.043 / 0.057 |
| eskf | 0.46 | 4.3433 | 0.45 / 0.46 | 1.86 | 0.53 / 0.98 | -24.18 | 0.168 / 0.195 |

## Planar pose over 300 s (mean over seeds)

| Method | Position RMSE (m) | Final error (m) | Max error (m) | Heading RMSE (deg) | Gyro bias estimate (true 0.010) | Compute p50 / p99 (ms) |
|---|---|---|---|---|---|---|
| dead_reckoning | 17.77 | 40.32 | 40.32 | 50.18 | - | 0.001 / 0.002 |
| wheel_imu_ekf | 0.42 | 0.93 | 1.38 | 1.62 | 0.0099 | 0.073 / 0.092 |
| ekf_gnss | 0.20 | 0.17 | 0.48 | 1.83 | 0.0099 | 0.073 / 0.128 |
| ukf_gnss | 0.20 | 0.17 | 0.48 | 1.83 | 0.0099 | 0.227 / 0.348 |
