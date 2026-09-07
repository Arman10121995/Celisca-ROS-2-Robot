# Robot Lab learning examples

These tutorials demonstrate numerical APIs already present in the repository.
They are not simulator benchmarks, live ROS algorithm integrations, or evidence
that five alternatives per category are working. The historical P7.4 tutorial
delivery means these documents exist; it does not qualify the platform.

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

There are no completed tutorial comparisons for local planning or control.
These five documents do not constitute five algorithms in any category.

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
ros2 run robot_lab_registry robot-lab validate --cross-references
```

These are inventory checks, not experiment execution or complete compatibility
qualification. Current validation has known gaps, component choices are not
fully wired into launch, and benchmark outputs include placeholder behavior.
Do not publish their output as measured comparative results until the roadmap's
execution and measurement acceptance criteria are met.
