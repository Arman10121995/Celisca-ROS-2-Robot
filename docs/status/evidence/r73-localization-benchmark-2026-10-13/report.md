# R7.3 localization benchmark (nav_maze, ray-cast scans)

Revision `42d1ccb`, molar1 aarch64; 3 trajectories (19.55, 17.84, 25.27 m), seeds [1, 2].

| Localizer | Position RMSE (m) | Max error (m) | Final error (m) | Heading RMSE (deg) | Lost (>0.5 m) | Compute p50 / max p99 (ms) |
|---|---|---|---|---|---|---|
| dead_reckoning | 0.549 | 0.892 | 0.797 | 3.30 | 40.9% | 0.07 / 0.11 |
| icp_point_to_point | 0.007 | 0.016 | 0.006 | 0.21 | 0.0% | 2.72 / 6.56 |
| icp_point_to_line | 0.008 | 0.017 | 0.006 | 0.23 | 0.0% | 1.78 / 2.65 |
| ndt | 0.008 | 0.018 | 0.008 | 0.24 | 0.0% | 3.65 / 6.04 |
| correlative | 0.024 | 0.104 | 0.021 | 0.72 | 0.0% | 7.57 / 8.79 |
