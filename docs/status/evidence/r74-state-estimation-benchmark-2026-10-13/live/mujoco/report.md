# R7.4 live replay: state estimators on simulator AMCL fixes

Revision `681939c`, molar1 aarch64. Input: the `/amcl_pose` stream recorded in the R7.3 simulator runs, scored against `/odom/ground_truth`. First 10 s skipped as warm-up.

| Estimator | Position RMSE (m) | p95 (m) | max (m) | mean gap (s) | compute p50 (ms) |
|---|---|---|---|---|---|
| amcl_raw | 0.0897 | 0.1306 | 0.1548 | None | None |
| cv_kalman | 0.0929 | 0.1419 | 0.1644 | 0.6 | 0.052 |
| ctrv_ekf | 0.0952 | 0.1462 | 0.1908 | 0.6 | 0.0956 |
| ctrv_ukf | 0.1208 | 0.2092 | 0.3245 | 0.6 | 0.122 |
| particle_filter | 0.2414 | 0.5807 | 0.7894 | 0.6 | 0.0953 |
| imm | 0.0986 | 0.155 | 0.1953 | 0.6 | 0.2125 |
| window_smoother | 0.093 | 0.1426 | 0.1647 | 0.6 | 0.2214 |
