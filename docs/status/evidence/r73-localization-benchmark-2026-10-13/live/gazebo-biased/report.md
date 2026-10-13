# R7.3 replay of simulator scans (Gazebo, biased odometry)

Revision `676f1ba`; map nav_maze; 3 runs; 5 scans/s replayed; localizers start at truth; odometry with the benchmark bias model (seed 1).

| Localizer | Position RMSE (m) | p95 (m) | Max (m) | Heading RMSE (deg) | Lost (>0.5 m) | Compute p50 / max p99 (ms) |
|---|---|---|---|---|---|---|
| dead_reckoning | 0.384 | 0.579 | 0.601 | 2.58 | 16.3% | 0.01 / 0.03 |
| icp_point_to_point | 0.037 | 0.060 | 0.066 | 0.64 | 0.0% | 2.71 / 8.72 |
| icp_point_to_line | 0.046 | 0.079 | 0.086 | 0.87 | 0.0% | 3.09 / 8.38 |
| ndt | 0.047 | 0.085 | 0.095 | 0.93 | 0.0% | 4.97 / 13.33 |
| correlative | 0.051 | 0.078 | 0.093 | 1.15 | 0.0% | 7.35 / 7.87 |
