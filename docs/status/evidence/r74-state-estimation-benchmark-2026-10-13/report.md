# R7.4 state estimation benchmark (noisy position fixes)

Revision `65ae417`, molar1 aarch64; seeds [1, 2, 3]; fixes at 10 Hz with 0.3 m noise (raw fix RMSE 0.425 m).

accel_std chosen on seed 0: cv_kalman 0.5, ctrv_ekf 0.35, ctrv_ukf 0.35, particle_filter 0.5, imm 0.35, window_smoother 0.35.

| Estimator | Position RMSE (m) | Speed RMSE (m/s) | Heading RMSE, v > 0.1 / v > 0.3 (deg) | Turn-rate RMSE (rad/s) | Compute p50 / p99 (ms) |
|---|---|---|---|---|---|
| cv_kalman | 0.167 | 0.112 | 23.2 / 16.7 | 0.261 | 0.057 / 0.069 |
| ctrv_ekf | 0.164 | 0.111 | 25.8 / 17.3 | 0.234 | 0.169 / 0.192 |
| ctrv_ukf | 0.169 | 0.122 | 29.0 / 23.1 | 0.271 | 0.229 / 0.267 |
| particle_filter | 0.191 | 0.128 | 32.6 / 24.3 | 0.335 | 0.223 / 0.362 |
| imm | 0.166 | 0.111 | 25.1 / 19.6 | 0.213 | 0.448 / 0.483 |
| window_smoother | 0.187 | 0.136 | 26.8 / 18.5 | 0.261 | 0.220 / 0.247 |
