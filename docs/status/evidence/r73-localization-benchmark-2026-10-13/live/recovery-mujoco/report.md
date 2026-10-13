# R7.3 recovery: scan matchers seeded at a wrong pose (mujoco)

Revision `10cb62c`; map nav_maze; 3 run(s); 2 scans/s; each matcher seeded at the wrong pose and replayed. "recovered" = position < 0.25 m and heading < 10 deg at some scan.

## Gate 0.5 m (recovered fraction, per offset)

| Method | x+0.3_y+0.0 | x+0.5_y+0.0 | x+0.7_y+0.0 | x+1.0_y+0.0 | x+1.5_y+0.0 | x+2.0_y+0.0 | x+0.0_y+0.7 | x+0.0_y+1.5 | yaw+15 | yaw+30 | yaw+90 | yaw+180 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dead_reckoning | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 |
| icp_point_to_point | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 3/3 | 3/3 | 0/3 | 0/3 |
| icp_point_to_line | 3/3 | 3/3 | 3/3 | 3/3 | 1/3 | 1/3 | 3/3 | 0/3 | 3/3 | 3/3 | 0/3 | 0/3 |
| ndt | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 3/3 | 0/3 | 3/3 | 3/3 | 0/3 | 0/3 |
| correlative | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 2/3 | 3/3 | 3/3 | 0/3 | 0/3 |

## Gate 5 m (recovered fraction, per offset)

| Method | x+0.3_y+0.0 | x+0.5_y+0.0 | x+0.7_y+0.0 | x+1.0_y+0.0 | x+1.5_y+0.0 | x+2.0_y+0.0 | x+0.0_y+0.7 | x+0.0_y+1.5 | yaw+15 | yaw+30 | yaw+90 | yaw+180 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dead_reckoning | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 |
| icp_point_to_point | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 0/3 | 3/3 | 3/3 | 0/3 | 0/3 |
| icp_point_to_line | 3/3 | 3/3 | 3/3 | 3/3 | 1/3 | 1/3 | 3/3 | 0/3 | 3/3 | 3/3 | 0/3 | 0/3 |
| ndt | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 0/3 | 3/3 | 0/3 | 3/3 | 3/3 | 0/3 | 0/3 |
| correlative | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 2/3 | 3/3 | 3/3 | 0/3 | 0/3 |

