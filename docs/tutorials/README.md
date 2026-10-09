# Robot Lab tutorials

Updated October 9, 2026. New controls/workflows have an
[implementation checkpoint](../status/implementation-2026-10-09.md), with
physical validation deferred. Earlier named measurements remain in the
[extension checkpoint](../status/continuation-2026-10-08.md). Start with [current status](../status/CURRENT_STATUS.md) and the
[done/remaining checklist](../status/CHECKLIST.md). The project remains partial;
a guide, imported model or numerical example does not qualify every mission.

## Operator guides

| Guide | Working scope and limits |
|---|---|
| [GUI workspace](gui-workspace.md) | Launch control column, categories/types, compatible defaults/autofill, limits and sidebar navigation |
| [Registry 3D](registry-3d.md) | Native embedded source geometry, grouped variants/components and camera controls; static inspection |
| [TurtleBot3](turtlebot3.md) | Three official models, measured four-backend Drive/localization/2D SLAM/export/navigation; RGB-only 3D SLAM remains unavailable |
| [TurtleBot4](turtlebot4.md) | Standard/Lite, measured four-backend Drive/localization/2D/3D SLAM/export/navigation; other maps and vendor behavior remain |
| [Husky](husky.md) | Original four-wheel skid geometry, declared lab sensor/motor kit and evidence-gated modes; exact backend/map screens remain separate from wider readiness |
| [Native Panda](panda_arm.md) | Measured prior joint/Hand/static planning; new experimental Servo and measured object attach/detach await validation |
| [Native controls and Stretch](native-controls.md) | Unitree exact-model policies; 28 fixed actuator candidates, Cartesian/teach-replay; Stretch base/sensors and explicit higher workflows |
| [Concurrent experiments](concurrent-experiments.md) | Real bounded subprocess queue, actual telemetry and owned cancel/reset; mission qualification remains separate |
| [PX4 X500](px4_x500.md) | Gazebo Harmonic flight, manual Drive/altitude, hold/goals/land in named worlds; wider flight-world/planning matrix remains |
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

## Comparison drafts

R7 breadth and R9.2 measured comparisons remain **partial**. The following
pages describe planned experiments and artifact requirements. They do not
provide five measured runnable methods per category or performance rankings.

| Category | Experiment draft | Comparison notes |
|---|---|---|
| Perception | [Obstacle detection](perception_comparison.md) | [Method comparison](comparison_perception.md) |
| Localization | [Pose estimation](localization_comparison.md) | [Method comparison](comparison_localization.md) |
| State estimation | [Filter performance](state_estimation_comparison.md) | [Method comparison](comparison_state_estimation.md) |
| Sensor fusion | [Sensor integration](sensor_fusion_comparison.md) | [Method comparison](comparison_sensor_fusion.md) |
| Global planning | [Path planning](global_planning_comparison.md) | [Method comparison](comparison_global_planning.md) |
| Local planning | [Collision avoidance](local_planning_comparison.md) | [Method comparison](comparison_local_planning.md) |
| Control | [Motion control](control_comparison.md) | [Method comparison](comparison_control.md) |

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
