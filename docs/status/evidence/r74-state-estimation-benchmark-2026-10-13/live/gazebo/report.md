# R7.4 live replay: state estimators on simulator AMCL fixes

Revision `681939c`, molar1 aarch64. Input: the `/amcl_pose` stream recorded in the R7.3 simulator runs, scored against `/odom/ground_truth`. First 10 s skipped as warm-up.

| Estimator | Position RMSE (m) | p95 (m) | max (m) | mean gap (s) | compute p50 (ms) |
|---|---|---|---|---|---|
| amcl_raw | 0.0799 | 0.115 | 0.1411 | None | None |
| cv_kalman | 0.0827 | 0.1192 | 0.1436 | 0.6157 | 0.0519 |
| ctrv_ekf | 0.0839 | 0.1268 | 0.1584 | 0.6157 | 0.0949 |
| ctrv_ukf | 0.1128 | 0.1866 | 0.2892 | 0.6157 | 0.1214 |
| particle_filter | 0.2843 | 0.6822 | 0.9726 | 0.6157 | 0.1153 |
| imm | 0.0862 | 0.1312 | 0.1712 | 0.6157 | 0.211 |
| window_smoother | 0.0831 | 0.1191 | 0.1543 | 0.6157 | 0.223 |
