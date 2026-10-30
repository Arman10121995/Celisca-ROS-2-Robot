# Robot Lab tutorials

October 30: [native-form delivery](../status/native-forms-2026-10-30.md)
adds snake acquired-map navigation, a surface vessel and articulated flipper
controls. The guides below distinguish runnable workflows from named missions.

New: [native dexterous-hand and coupled-gripper controls](native-hand-controls.md)
for four experimental MuJoCo profiles. Use measured contacts and source effort
feedback; object-grasp qualification remains separate.

Updated October 14, 2026. Thirteen [native fixed arm planning candidates](native-arm-planning.md)
have normal GUI Plan/Execute/autofill; UR5e and SO101 each have one scoped
MuJoCo/nav_empty reach. Broader arm/object/backend acceptance remains open.
Earlier controls/workflows have an
[implementation checkpoint](../status/implementation-2026-10-09.md), with
physical validation deferred. Earlier named measurements remain in the
[extension checkpoint](../status/continuation-2026-10-08.md). Start with [current status](../status/CURRENT_STATUS.md) and the
[done/remaining checklist](../status/CHECKLIST.md). The project remains partial;
a guide, imported model or numerical example does not qualify every mission.

## Operator guides

| Guide | Working scope and limits |
|---|---|
| [Amphibious rover](amphibious-rover.md) | Sprung native wheels/propellers, actual shore round-trip goals, hybrid sensing/RGB-D mapping and source-aware Plan/Execute |
| [Fleet facility](fleet-infrastructure.md) | Physical doors/lift, neutral Facility controls, actual docking and explicitly unknown unqualified battery energy; multi-floor transport remains |
| [GUI workspace](gui-workspace.md) | Launch control column, categories/types, compatible defaults/autofill, limits and sidebar navigation |
| [Registry 3D](registry-3d.md) | Native embedded source geometry, grouped variants/components and camera controls; static inspection |
| [TurtleBot3](turtlebot3.md) | Three official models, measured four-backend Drive/localization/2D SLAM/export/navigation; RGB-only 3D SLAM remains unavailable |
| [TurtleBot4](turtlebot4.md) | Standard/Lite, measured four-backend Drive/localization/2D/3D SLAM/export/navigation; other maps and vendor behavior remain |
| [Husky](husky.md) | Original four-wheel skid geometry, declared lab sensor/motor kit and evidence-gated modes; exact backend/map screens remain separate from wider readiness |
| [Native Panda](panda_arm.md) | Measured prior joint/Hand/static planning; new experimental Servo and measured object attach/detach await validation |
| [Native fixed arm planning](native-arm-planning.md) | 13 experimental KDL/OMPL/native trajectory candidates; actual UR5e/SO101 reaches, authored frames and explicit pending qualification |
| [Native controls and Stretch](native-controls.md) | Unitree exact-model policies; 28 fixed actuator candidates, Cartesian/teach-replay; Stretch base/sensors and explicit higher workflows |
| [Concurrent experiments](concurrent-experiments.md) | Real bounded subprocess queue, actual telemetry and owned cancel/reset; mission qualification remains separate |
| [Surface vessel](surface-vessel.md) | Complete native catamaran, latched Marine controls, harbor map acquisition and planar route/settling |
| [Wheeled snake](wheeled-snake.md) | Complete passive-wheel snake, native Drive, actual 2D/3D acquisition and installed/acquired-map Nav2 endpoints; terrain/backends remain |
| [Tracked vehicle](tracked-vehicle.md) | Native contact-surface belts and fixed/actuated flipper variants; Tracks controls, real joint settling/Drive and separate navigation scope |
| [PX4 X500](px4_x500.md) | Gazebo Harmonic flight, manual Drive/altitude, hold/goals/land in named worlds; wider flight-world/planning matrix remains |
| [Mapped drone routes](mapped-drone-planning.md) | RTAB-Map + registered depth supplements static planning; async execution rechecks, stale/blockage Hold, actual flight acceptance pending |
| [Go2 policy](go2.md) | Bounded MuJoCo policy trials and their recorded failures; terrain/recovery/navigation remain unqualified |

Use these commands from the existing workspace root:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_lab_gui robot_lab_gui
```

Reopen a GUI started before rebuilding. Drive, Arm, Hand and Drone pages are
inside Launch's right-hand control column. Use the exact source variant and
backend required by each guide; variants do not inherit controller support.

## Numerical learning material

The [learning index](index.md) explains the checked numerical APIs and their
limits. [Perception](perception.md), [planning](planning.md),
[localization](localization.md), [state estimation](state_estimation.md),
[sensor fusion](sensor_fusion.md), [local planning](local_planning.md) and
[control](control.md) contain educational material and implementation notes.
Some examples are sketches or approximations; ROS node initialization and
real input/output must be qualified separately.

## Measured comparisons

R7 breadth and R9.2 measured comparisons remain **partial**. **Global
planning** (eight planners) and **local planning** (five controllers) are
measured comparisons on the same seeded goals, with per-goal truth
metrics and failures (2026-10-12/13). **Control** (six path-tracking
controllers) is measured on a model plant and in Gazebo, and **sensor
fusion** (five attitude and three pose filters) on modelled sensors,
offline and over ROS (2026-10-13). **Perception** (six kernels on
synthetic scenes), **localization** (four scan matchers and dead
reckoning on nav_maze trajectories recorded in Gazebo) and **state
estimation** (six filters on noisy position fixes) are measured offline
with known truth (2026-10-13). All four also have simulator-data runs:
localization on recorded Gazebo and MuJoCo scans, perception on recorded
scans and on live depth clouds, state estimation on recorded AMCL fixes
and sensor fusion on recorded Gazebo IMU and wheel odometry (2026-10-15).

| Category | Measured comparison |
|---|---|
| Perception | [Obstacles, ground, clusters and depth clouds](perception_comparison.md) |
| Localization | [Scan matchers on nav_maze trajectories](localization_comparison.md) |
| State estimation | [Six filters on noisy position fixes](state_estimation_comparison.md) |
| Sensor fusion | [Attitude and pose filters](sensor_fusion_comparison.md) |
| Global planning | [Eight planners on nav_maze](global_planning_comparison.md) |
| Local planning | [Five controllers on nav_maze](local_planning_comparison.md) |
| Control | [Six path trackers, model plant and Gazebo](control_comparison.md) |

The older generated `comparison_*.md` drafts now only point to these pages.
Historical generated scores and placeholder commands were withdrawn by the
[October 2 audit](../status/audit-2026-10-02.md). A fair comparison still needs
actual executables/plugins, matched inputs/parameters/seed budgets, real
recordings, independently checked metrics and reproducible plots.

## Run documentation and source checks

```bash
source scripts/ssd_env.sh
bash scripts/test_tiers.sh fast
```

This checks source/numerical/configuration contracts. It does not execute
Markdown as Python or produce the drafts' proposed robot comparisons. For
ROS-dependent checks, source the workspace and follow [Testing](../TESTING.md).
Use the [workflow](../WORKFLOW.md), [roadmap](../../ROADMAP.md) and
[agent handoff](../AGENT_HANDOFF.md) to turn a draft into measured evidence.
