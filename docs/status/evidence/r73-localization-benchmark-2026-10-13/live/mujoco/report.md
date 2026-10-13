# R7.3 replay of simulator scans (MuJoCo, odometry as recorded)

Revision `63319f1`; map nav_maze; 3 runs; 5 scans/s replayed; localizers start at truth; odometry as recorded.

| Localizer | Position RMSE (m) | p95 (m) | Max (m) | Heading RMSE (deg) | Lost (>0.5 m) | Compute p50 / max p99 (ms) |
|---|---|---|---|---|---|---|
| dead_reckoning | 0.007 | 0.011 | 0.011 | 0.08 | 0.0% | 0.01 / 0.06 |
| icp_point_to_point | 0.036 | 0.060 | 0.062 | 0.60 | 0.0% | 1.85 / 7.87 |
| icp_point_to_line | 0.045 | 0.079 | 0.082 | 0.82 | 0.0% | 1.69 / 7.03 |
| ndt | 0.046 | 0.086 | 0.089 | 0.88 | 0.0% | 3.86 / 13.04 |
| correlative | 0.053 | 0.068 | 0.089 | 1.13 | 0.0% | 7.23 / 18.96 |
| amcl_live | 0.088 | 0.136 | 0.156 | 1.58 | 0.0% | - |
