# TurtleBot 4 workflows — October 6, 2026

Forty named actual GUI/physics screens are retained: Standard and Lite on
Gazebo Fortress, MuJoCo, PyBullet and native Isaac Sim, each with Localization,
2D SLAM, 3D SLAM, clear-map Nav2 and blocked-direct-path Nav2. Source baseline
is `61335fe` plus each pre-trial source/installed stage. Both normal installed
profiles now enable these measured backend modes; normal selectors,
compatible algorithm defaults and command autofill are checked separately.
This qualifies named bounded screens, not full roadmap acceptance or all maps.

## What was measured

Localization/mapping use `nav_empty`: independently observed forward/reverse,
both turns, publisher loss, monotonic reset, restored spawn, post-reset motion,
fresh estimate and resumed sensors/mapping. GUI Save Map produces real YAML/
PGM or an intact SQLite database plus finite PCD points. All sixteen final map
exporters exited after saving; clean launch exit and plant cleanup precede the
next trial. Twelve non-Isaac mapping trials were repeated after the cleanup
fix; Isaac modes were repeated after the pose-derived odometry correction.

Nav2 goals use the RViz relay topic. Acceptance is at most 0.15 m / 5° physical
body error, fresh truth, a one-simulation-second settling window and at least
0.02 m swept clearance with a conservative 0.185 m robot radius. The obstacle
route's direct path intersects the selected box, so a clear corridor alone
cannot enable Navigation. No contact telemetry or cancellation/second-goal
qualification is inferred from swept geometry.

| Variant | Backend | Clear goal error (m / °) | Obstacle goal error (m / °) | Minimum obstacle clearance (m) |
|---|---|---|---|---:|
| standard | gazebo | 0.041 / +0.50 | 0.042 / +2.58 | 0.184 |
| standard | mujoco | 0.106 / -0.23 | 0.026 / -0.89 | 0.169 |
| standard | pybullet | 0.112 / -0.42 | 0.028 / -0.51 | 0.196 |
| standard | isaac | 0.075 / -0.82 | 0.015 / -0.52 | 0.181 |
| lite | gazebo | 0.043 / +0.61 | 0.056 / +2.06 | 0.185 |
| lite | mujoco | 0.135 / -0.63 | 0.014 / -0.42 | 0.170 |
| lite | pybullet | 0.110 / +0.55 | 0.038 / -1.28 | 0.179 |
| lite | isaac | 0.105 / +0.56 | 0.019 / -0.18 | 0.204 |

Native Isaac Lite 3D SLAM tracks within 0.01504 m, resumes tracking within
0.00154 m after reset, and saves six keyframes / 1,435 finite PCD points.
Standard saves six keyframes / 1,796 points. The original SDK velocity was
nonzero while the physical Lite body was stationary; integrating it drifted
the EKF about 0.3 m. Successive actual root poses now supply ideal body twist
and IMU angular rate. This fixes the solver-bias mismatch while retaining
ideal ground-truth simulation input; it does not establish noisy hardware
odometry or independent sensor-only localization.

Gazebo uses a maximum 2 ms step for Create 3 compliance, preserving selected
world geometry, name and plugins in an SSD derivative. The Standard clear-goal
trial was repeated against the current helper before mode promotion. Isaac
headless camera sampling is decoupled from 60 Hz physics; visible rendering
is unchanged. GUI console drawing is bounded while full output is retained
on SSD. Map export uses a wall deadline and normal node/context destruction;
Stop also terminates tracked auxiliary export processes.

## Provenance and remaining work

Each case has the measured report/workflow or navigation JSON, producer and
pre-trial source manifest. [manifest.json](manifest.json) retains raw artifact
paths, sizes, hashes and negative reports. Normal mode enablement verifies the
source pin, drive/sensor configuration, backend code, navigation parameters
and numeric observations through `asset_support.py`. It does not trust a
PASS label or transfer one backend's result to another.

Failed Gazebo coarse-step odometry, Isaac slow-timeout and overlapping-clock
trials, failed neighbor-ICP hypotheses, and the original SDK-bias diagnostic
remain on SSD. The ICP overlay was removed: the successful correction is
pose-derived ideal twist. Original failures are not replaced by positive
report files. Other maps, longer routes, cancellation/second goals, measured
contacts, docking/hazards, realistic noise, full visuals and hardware remain
open. The all-map performance requirement also remains open; see the measured
[RTF and sensor screens](../turtlebot4-sensors-2026-10-06/README.md).

Use [the TurtleBot 4 guide](../../../tutorials/turtlebot4.md). The current
normal GUI check and producer are retained alongside this evidence.
