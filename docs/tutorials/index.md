# Robot Lab learning examples

Updated October 7, 2026. [Current status](../status/CURRENT_STATUS.md) and
[the operator index](README.md) collect the GUI and named robot workflows.
These learning examples demonstrate numerical APIs and an opt-in Go2 workflow. They are not universal
simulator benchmarks or evidence that five alternatives per category are
working. The historical P7.4 tutorial delivery means these documents exist; it
does not qualify the platform.

The [roadmap](../../ROADMAP.md) defines implementation and qualification work.
An agent continuing that work must start with the
[agent handoff](../AGENT_HANDOFF.md), not infer readiness from these examples.

## Prerequisites and scope

Run commands from the workspace root containing `src/robot_lab_algorithms`.
The snippets use checked-out Python source, not a potentially stale `install/`
copy. No simulator, robot, hardware controller, benchmark recorder, or output
file is started by these examples.

```bash
test -d src/robot_lab_algorithms/robot_lab_algorithms
export PYTHONPATH="$PWD/src/robot_lab_algorithms${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
```

Perception, planning, state estimation, and sensor fusion call ordinary Python
objects and need only Python 3 for these particular snippets. Their modules
contain optional ROS imports/fallbacks; succeeding without ROS proves only
that the numerical method ran, not that dependencies or node adapters work.
The localization example is different: `DeadReckoning` inherits a ROS node, so
that tutorial explicitly requires a sourced ROS 2 environment and initializes
and shuts down `rclpy`.

Do not run bootstrap or simulator installation scripts merely to execute the
pure numerical examples. For workspace setup, follow the current root
[README](../../README.md) and its limitations.

## Tutorials by category

| Category | Tutorial | What it actually demonstrates |
|---|---|---|
| Perception | [Point and scan clustering](perception.md) | Two simple grouping methods on equivalent synthetic geometry |
| Planning | [Planning prototypes](planning.md) | Seeded RRT output and limitations of a greedy ridge walker |
| Localization | [Dead reckoning](localization.md) | Manually supplied body-twist integration; ROS initialization is required |
| State Estimation | [Diagonal filter](state_estimation.md) | Constant-velocity prediction plus independent scalar corrections |
| Sensor Fusion | [Complementary tilt filter](sensor_fusion.md) | The current filter's numerical response and axis-convention caveat |
| Locomotion policy | [Go2 flat-ground policy](go2.md) | Opt-in MuJoCo stance, command-loss stop, forward/turn and reverse dead-zone evidence; not terrain/navigation qualification |

There are no completed tutorial comparisons for local planning or control.
The Go2 tutorial is a bounded runtime evidence walkthrough, not a replacement
for a fair algorithm comparison. These documents do not constitute five
algorithms in any category.

## What must precede a full comparison

A fair experiment must use a common task, compatible robot/simulator/sensors,
the same input dataset or controlled simulation, explicit frame and timestamp
conventions, a fixed parameter protocol, multiple recorded seeds when random,
and independently validated metrics. Each tutorial identifies relevant metrics
and missing work. Timing a small snippet or counting waypoints is not a
performance ranking.

The registry CLI can list and describe catalog entries after the relevant
packages have been built and sourced:

```bash
ros2 run robot_lab_registry robot-lab list experiments
ros2 run robot_lab_registry robot-lab validate -c src/robot_lab/robot_lab_registry/config --cross-references
```

These are inventory checks, not experiment execution or complete compatibility
qualification. Runtime compatibility remains exact-cell specific, and legacy demonstration
benchmark outputs contain placeholder behavior.
Do not publish their output as measured comparative results until the roadmap's
execution and measurement acceptance criteria are met.

- [TurtleBot 4 Standard/Lite](turtlebot4.md): installed official models, measured
  four-backend Drive, sensors, localization, mapping/export and navigation
  screens; longer routes, other maps and vendor firmware remain.
- [Registry 3D preview](registry-3d.md): native embedded geometry, complete
  parent families and nested source variants/components; orbit/pan/zoom/Fit.
- [GUI workspace](gui-workspace.md): Launch setup and command beside a separate
  control column; robot categories/types, limits and sidebar navigation.
- [TurtleBot 3 Burger/Waffle/Waffle Pi](turtlebot3.md): source-backed wheels
  and LDS, measured four-backend GUI Drive and evidence-gated modes.
- [Native Panda Arm controls](panda_arm.md): actual MuJoCo joint/Home/Stop/
  Hand and MoveIt Plan/Execute, measured static-world TCP and physical cube
  grasp; Servo, attached-payload planning and other backends remain.

- [PX4 X500 Flight](px4_x500.md): actual Gazebo Harmonic flight, altitude/Drive,
  hold/goals/land and bounded command loss in the measured worlds.
