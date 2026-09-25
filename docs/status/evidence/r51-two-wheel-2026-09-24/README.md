# R5.1 Bumperbot and Labbot qualification (2026-09-24)

Named backend: MuJoCo, ROS 2 Humble on Jetson AGX Orin. All launches used
`robot_lab_bringup/simulated_robot.launch.py`, `mode:=loc` for drive and
`mode:=nav` for Nav2, `gui:=false`, `start_rviz:=false`, unique ROS domains.
The drive probe publishes the GUI's `/key_vel` at 10 Hz; `twist_mux` relays
it to `/robot_lab_controller/cmd_vel_unstamped`.

| Robot | Forward command 0.2 m/s for 4 sim s | Turn command 0.5 rad/s for 2 sim s | Reverse command -0.15 m/s for 2 sim s | Stop and watchdog |
|---|---:|---:|---:|---|
| Bumperbot | 0.713 m | 0.961 rad | -0.291 m | all stops below 0.05 m/s; watchdog velocity ~0 |
| Labbot | 0.632 m | 0.755 rad | -0.245 m | all stops below 0.05 m/s; watchdog velocity ~0 |

The two `*_limited_drive.json` files include start/end truth poses for every phase,
the stop checks, and a 1.5 wall-second command-loss watchdog check. Both
have `passed: true`. `/odom/ground_truth` is recorded separately from
`/odometry/filtered` (396 and 405 estimate messages respectively); the
mux output was also observed. The differential wheel-target model now applies
the catalog's 1.0/0.8 m/s linear caps and 2.0/1.5 m/s² acceleration caps for
Bumperbot/Labbot, plus bounded angular speed and acceleration. The unit test
checks clamping, slewing, and reset. The live probes check motion and watchdog
after those bounds are applied; they do not excite the upper speed limits.
This is simulator qualification, not physical hardware qualification.

The same limited Bumperbot probe also passed on PyBullet: 0.754 m forward,
0.706 rad turn, -0.298 m reverse and a zero-speed watchdog. Its report is
`bumperbot_pybullet_limited_drive.json`.

| Robot | `nav_empty` | `nav_obstacle` |
|---|---|---|
| Bumperbot | succeeded, 0.272 m map-frame goal error | succeeded, 0.273 m |
| Labbot | succeeded, 0.274 m | succeeded, 0.277 m |

The four `*_limited_nav.json` reports and `.launch.log` files contain the
Nav2 result, selected start/goal, truth final pose, and stack lifecycle on
the final bounded-drive code. The clear map started at `(-0.5, -0.5)` and
the obstacle map at `(-7, -7)`; each selected a free goal 2 m ahead.
These missions demonstrate clear and obstacle-map navigation on one backend;
they do not establish every map/backend pair or a zero-contact count.

The golden static harness now processes the actual installed xacros for both
robots. Labbot's xacro includes the shared OAK-D RGB-D sensor, and its live
MuJoCo launch log reports a 320×240, 5 Hz RGB-D camera; the old golden fixture
incorrectly declared no depth sensor. `test_r5_1_qualification.py`: 41
passed. The 2026-09-24 GitHub ROS 2 CI run `35989437894` passed all tiers;
the scheduled full suite `35992128283` also passed. `probe_drive.py` and the JSON files make the live checks
repeatable; the initial probe failures (an integer ROS field and a wrong
estimate topic) were repaired before the fresh runs above.
