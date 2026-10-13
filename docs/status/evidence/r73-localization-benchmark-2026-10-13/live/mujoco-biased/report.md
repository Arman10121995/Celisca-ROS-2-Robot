# R7.3 replay of simulator scans (MuJoCo, biased odometry)

Revision `63319f1`; map nav_maze; 3 runs; 5 scans/s replayed; localizers start at truth; odometry with the benchmark bias model (seed 1).

| Localizer | Position RMSE (m) | p95 (m) | Max (m) | Heading RMSE (deg) | Lost (>0.5 m) | Compute p50 / max p99 (ms) |
|---|---|---|---|---|---|---|
| dead_reckoning | 0.328 | 0.504 | 0.517 | 2.88 | 10.0% | 0.01 / 0.09 |
| icp_point_to_point | 0.036 | 0.060 | 0.063 | 0.61 | 0.0% | 2.58 / 17.63 |
| icp_point_to_line | 0.045 | 0.079 | 0.082 | 0.82 | 0.0% | 2.96 / 15.59 |
| ndt | 0.046 | 0.085 | 0.092 | 0.88 | 0.0% | 4.92 / 42.79 |
| correlative | 0.054 | 0.081 | 0.093 | 1.15 | 0.0% | 8.48 / 163.63 |
