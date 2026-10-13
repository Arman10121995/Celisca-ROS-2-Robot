# R7.3 replay of simulator scans (Gazebo, odometry as recorded)

Revision `676f1ba`; map nav_maze; 3 runs; 5 scans/s replayed; localizers start at truth; odometry as recorded.

| Localizer | Position RMSE (m) | p95 (m) | Max (m) | Heading RMSE (deg) | Lost (>0.5 m) | Compute p50 / max p99 (ms) |
|---|---|---|---|---|---|---|
| dead_reckoning | 0.034 | 0.051 | 0.053 | 0.36 | 0.0% | 0.01 / 0.04 |
| icp_point_to_point | 0.037 | 0.060 | 0.065 | 0.64 | 0.0% | 2.03 / 8.48 |
| icp_point_to_line | 0.046 | 0.079 | 0.086 | 0.87 | 0.0% | 2.28 / 7.66 |
| ndt | 0.047 | 0.086 | 0.094 | 0.93 | 0.0% | 4.44 / 13.83 |
| correlative | 0.051 | 0.074 | 0.094 | 1.16 | 0.0% | 7.59 / 10.04 |
| amcl_live | 0.079 | 0.113 | 0.142 | 1.49 | 0.0% | - |
