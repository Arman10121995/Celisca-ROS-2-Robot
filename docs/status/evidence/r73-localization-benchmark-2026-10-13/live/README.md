# R7.3 on simulator scans: recorded Gazebo and MuJoCo runs (2026-10-13)

The offline benchmark ray-cast its scans from the map itself. Here the
scans come from the simulators: Bumperbot drove the R4.3 goals on nav_maze
(A*, seeds 23/37/53, three goals each, Regulated Pure Pursuit, AMCL), once
in Gazebo and once in MuJoCo. Each run was recorded with its `/scan`
(`EXTRA_BAG_TOPICS=/scan scripts/planner_comparison.sh`, 12:16-12:33).

`scripts/r7_3_live_scan_replay.py` then replays every run offline:

- each scan matcher starts at the truth pose of the first scan and, at 5
  scans per second, predicts with the recorded `odom -> base_footprint`
  increment and aligns the scan to the nav_maze map Nav2 used;
- the laser pose in the robot comes from `/tf_static` (the laser is
  mounted backwards, yaw 3.14);
- AMCL, which ran live in the same run, is scored at the same scan times
  (its last `/amcl_pose` moved forward by the odometry since);
- "biased" repeats the replay with the offline benchmark's odometry error
  model (distance +3%, rotation +2%, heading drift 0.5°/m, seeded noise).
  AMCL ran on the true odometry, so it is not in those tables.

Three runs per simulator, 17.8-25.3 m and 385-515 replayed scans each.
Code as in `63319f1`.

## Results (position RMSE against simulator truth)

| Localizer | Gazebo | Gazebo, biased odometry | MuJoCo | MuJoCo, biased odometry | Compute p50 |
|---|---|---|---|---|---|
| ICP, point to point | **0.037 m** | **0.037 m** | **0.036 m** | **0.036 m** | 2.0 ms |
| ICP, point to line | 0.046 m | 0.046 m | 0.045 m | 0.045 m | 2.0 ms |
| NDT | 0.047 m | 0.047 m | 0.046 m | 0.046 m | 4.2 ms |
| Correlative | 0.051 m | 0.051 m | 0.053 m | 0.054 m | 7.4 ms |
| AMCL (ran live) | 0.079 m | - | 0.088 m | - | - |
| Odometry alone | 0.034 m | 0.384 m (lost 16%) | 0.007 m | 0.328 m (lost 10%) | - |

The scan matchers were never more than 0.11 m from truth, AMCL never more
than 0.19 m. Full tables:
[gazebo](gazebo/report.md), [gazebo-biased](gazebo-biased/report.md),
[mujoco](mujoco/report.md), [mujoco-biased](mujoco-biased/report.md).
Per-run numbers, including each localizer's mean error vector, are in
the `results.json` files.

## What it shows

- **On real simulator scans, the scan matchers hold 3.6-5.4 cm**: 1.5-2.4
  times closer than the AMCL that ran live on the same scans (7.9-8.8
  cm).
- **Their remaining error is mostly a fixed offset.** The mean error
  vector is nearly the same in every run and in both simulators:
  - both ICPs and NDT sit 2.2-3.8 cm off in x;
  - correlative matching sits 3-5 cm off in y;
  - AMCL sits 5-8 cm off in y.

  That points at how the 5 cm map grid registers against the simulated
  walls rather than at the matchers' noise.
- **The matchers do not depend on the odometry.** With the biased
  odometry they change by less than 1 mm, while the odometry alone drifts
  to 0.33-0.38 m RMSE and more than 0.5 m away for 10-16% of the time.
- **Good odometry alone beats them here.** Gazebo's wheel odometry drifts
  only 3.4 cm over about 20 m on this flat floor. In MuJoCo the odometry
  is the simulator's truth by default (`odometry_source:=truth`; the
  `wheels` option exists), hence 0.7 cm.
- **Limits.** One robot and one map. The localizers start at the true
  pose, so global localization and recovery are not tested. The replay
  runs offline: the live nodes publish the same estimates, but their
  timing in a running stack is not measured here.

## Reproduce

```bash
EXTRA_BAG_TOPICS=/scan GOALS=3 ROBOT_RADIUS=0.12 scripts/planner_comparison.sh <dir> gazebo bumperbot nav_maze \
  global_planning a_star_planner "23 37 53" local_planning:=pure_pursuit
python3 scripts/r7_3_live_scan_replay.py --map nav_maze --out <out> <dir>/*.bag
python3 scripts/r7_3_live_scan_replay.py --map nav_maze --odom-bias --out <out-biased> <dir>/*.bag
```

The bags stay on the SSD in `robot_lab_runtime/claude-r73-live-{gazebo,mujoco}/`.
