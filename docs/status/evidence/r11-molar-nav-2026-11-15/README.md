# R11 MOLAR Nav2 on a CELISCA floor in Gazebo: first measured route (November 15, 2026)

**Status:** implemented, experimental. One run.

**Run:** `CHECK_ARGS="--base base_link --robot-radius 0.49" scripts/sim_nav_mission.sh gazebo molar_hri celisca_floor_1_detailed`.

The mission (`sim_nav_mission.py`, seed 1) chooses its goals from the
published map. It is judged by `/odom/ground_truth`.

| Step | Result |
|---|---|
| Route A: 9.58 m grid path (6.79 m straight, multi-turn) | **succeeded**; 0.183 m and 11.8° from the goal by truth (limits 0.30 m / 15°); localization estimate 0.109 m from the goal |
| Cancel after 1.0 m | **passed**: canceled, stopped within 0.7 simulated s |
| Resume route A | **succeeded** (the row above) |
| Near-obstacle goal B (0.60 m clearance, 5.56 m path) | **failed**: aborted 3.3 m short; the estimate had drifted 0.49 m from truth |

Overall `passed: false`, because goal B failed. The route, cancel and resume
steps pass.

`gazebo_celisca1_route.json` is the report. The truth and command trace stays
on SSD (`robot_lab_runtime/molar-nav-20261115/gazebo_celisca1_r49.json.trace.json`;
hash in `trace-sha256.txt`).

## What had to be fixed first

Before this, MOLAR had never run Nav2 in Gazebo. `be016bd0` listed the modes,
but each attempt below failed. The failed reports are on SSD; the
0.25 m-radius run is kept here as `gazebo_celisca1_radius025_failed.json`.

**Mode check refused the robot.** `robots.yaml` declared the feature `lidar`
instead of `lidar_2d` for `molar_hri` and `probot_l`, so the mode check
refused Gazebo nav.

- Fix: renamed to `lidar_2d`.

**No odometry.** The shared wheel controller (`robot_lab_controller`) is not
defined by MOLAR's description, so nothing published odometry.

- Fix: in every Gazebo mode, `simulated_robot.launch.py` now spawns the
  robot's own controllers (`molar_base_controller`, the position and roller
  groups) and `molar_gazebo_control`, as Display already did.
- Spawners use a 60 s switch timeout: the base controller missed the 5 s
  default on the detailed floor.
- `molar_base_controller` runs with its odom TF off, so the EKF owns
  odom → base_link.
- `molar_gazebo_control relay_odom` republishes its odometry on
  `/robot_lab_controller/odom`.

**Wrong base frame in the checker.** It defaults to `base_footprint`; MOLAR
is rooted at `base_link`, so it now passes `--base base_link`.

**Self-hits wrecked localization.** The Gazebo front scan was 360° from a
mount 0.34 m ahead of the base: 216 of 540 beams beyond ±110° hit MOLAR's own
body at 0.17–0.34 m. With that scan, the first route ended 2.3 m and 130° off.

- Fix: the simulated front scanner now covers ±105° (315 samples, the same
  resolution).
- The rear arc belongs to the hardware's rear scanner, which is not
  simulated.
- MuJoCo, PyBullet and Isaac cast their scan skipping the robot, so they were
  unaffected.

**Goals chosen for a smaller robot.** With the default 0.25 m radius, route A
went through gaps MOLAR (0.75 × 0.62 m footprint, 0.49 m circumscribed) does
not fit: "failed to make progress", then "start in lethal space".

## Remaining

- Goal B: near-obstacle precision and localization drift late in the run.
- Repeats; floors 2 and 3.
- MuJoCo, PyBullet and Isaac nav runs.
- A station transfer at a CELISCA station and multi-floor operation through
  the fleet lift.
