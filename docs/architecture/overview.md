# Robot Lab architecture: current implementation and target

Documentation baseline: source commit `dff388f`, audited 2026-09-07. This page
separates code that exists from architecture still to implement. It is not a
claim that every described combination runs successfully.

Start with [the support matrix](../status/support-matrix.md) and
[the audit](../status/audit-2026-09-07.md). Implementation order, task ownership,
and acceptance criteria belong in [ROADMAP.md](../../ROADMAP.md); an agent
resuming work must also read [AGENT_HANDOFF.md](../AGENT_HANDOFF.md).

## Goal and present gap

The goal is a unified learning and evaluation platform for mobile, legged,
humanoid, and aerial robots in 2D and 3D environments. Users should independently
select perception, localization, state estimation, sensor fusion, global
planning, local planning, and control algorithms, then compare measured
performance under the same scenario conditions. Five genuinely distinct,
runnable choices per category is a target, not a claim established by catalog
counts.

The current repository has a substantial Bumperbot-oriented ROS stack, robot and
map assets, four simulator launch adapters, a desktop GUI, algorithm examples,
and benchmark/reporting foundations. Its main execution path still uses legacy
mode profiles. Independent composition is incomplete, runtime contracts are
inconsistent, and benchmark results contain placeholders. One reproducible,
measured end-to-end experiment is the next integration milestone.

## Actual source-tree and package map

There are 26 discoverable ROS packages at the audited baseline, including optional
ORB-SLAM3. Nested vendor descriptions are assets of the enclosing
`robot_lab_robots` package, not individually supported robot stacks.

```text
src/
  robot_lab/
    robot_lab_registry/     # catalogs, schemas, validation, query CLI
    robot_lab_benchmark/    # result records, runner/reporting foundations
  robot_lab_adapter/        # incomplete selectors/composition/legacy adapters
  robot_lab_bringup/        # current profile-driven launch orchestration
  robot_lab_description/    # Gazebo/display launch and description support
  robot_lab_robots/         # consolidated first-party and vendored robot assets
  robot_lab_maps/           # worlds, occupancy maps, arena tools and metadata
  robot_lab_models/         # shared world models
  robot_lab_pybullet/       # PyBullet bridge and spawner
  robot_lab_mujoco/         # MuJoCo bridge and spawner
  robot_lab_isaac/          # ROS bridge plus external Isaac runtime process
  robot_lab_algorithms/     # numerical examples and partial ROS adapters
  robot_lab_gui/            # Tkinter launch/control center
  robot_lab_*/              # ROS stack, hardware utilities and examples below
  ORB_SLAM3/                # optional external-library wrapper
```

| Layer/packages | Current responsibility | Boundary to maintain |
|---|---|---|
| `robot_lab_registry` | YAML catalogs, schemas, references, CLI, partial compatibility | Describe and validate; do not start ROS or simulator processes |
| `robot_lab_adapter`, `robot_lab_bringup` | Selectors, launch fragments, profile resolution, simulator dispatch | Own composition, readiness, namespaces and parameters; not algorithm mathematics |
| `robot_lab_description`, `robot_lab_robots` | Descriptions, Gazebo/display startup, sensor/control assets | Own physical model and declared interfaces; not benchmark verdicts |
| `robot_lab_maps`, `robot_lab_models` | SDF worlds, occupancy data, static/dynamic assets, generators | Own geometry, transforms, landmarks and provenance |
| `robot_lab_pybullet`, `robot_lab_mujoco`, `robot_lab_isaac` | Additional engine loading, stepping and ROS bridges | Own engine-specific import, physics, sensors, ground truth and reset |
| `robot_lab_mapping` | SLAM Toolbox and RTAB-Map launch/configuration | Adapt mapping to explicit sensor/frame contracts |
| `robot_lab_localization` | AMCL, EKF, odometry/IMU processing and examples | Publish estimates with explicit TF ownership |
| `robot_lab_navigation` | Nav2 configuration, lifecycle and behavior-tree launch | Compose navigation plugins without hiding selected algorithms |
| `robot_lab_planning`, `robot_lab_motion` | Standalone planners and path followers | Separate global paths, local obstacle handling and actuation |
| `robot_lab_controller`, `robot_lab_utils` | Wheel control, joystick/multiplexer, mapping/cleaning, safety | Translate commands, enforce limits and stop safely |
| `robot_lab_algorithms` | Additional numerical algorithms and entry points | One functioning contract per implementation; no empty-node integrations |
| `robot_lab_benchmark` | Records, execution helpers, reporting and regression utilities | Measure real runs; never manufacture success or ground truth |
| `robot_lab_gui` | Profiles, catalog browser, drive/map tools, monitoring | Use the same resolver/runner as CLI; no private support rules |
| `robot_lab_vacuum_cleaning` | Additional basic vacuum controller | Reconcile duplication with cleaning logic in `robot_lab_controller` |
| `robot_lab_firmware`, `robot_lab_msgs` | Serial/Arduino interface and shared messages | Separate hardware operation from simulation-only workflows |
| `robot_lab_cpp_examples`, `robot_lab_py_examples`, `orbslam3` | Teaching examples and optional external integration | Keep optional dependencies out of the reference build's critical path |

## Current execution paths

### Main profile-driven route

```text
GUI or ros2 launch robot_lab_bringup simulated_robot.launch.py
  → load robot profile + map profile + mode profile
  → check a subset of profile requirements
  → choose Gazebo / PyBullet / MuJoCo / Isaac launch adapter
  → launch the mode's localization, mapping and/or navigation nodes
  → sensors → pose estimate → planner → path follower → command → robot
```

The implementation is
[simulated_robot.launch.py](../../src/robot_lab_bringup/launch/simulated_robot.launch.py).
Its configuration sources are:

- [Robot launch profiles](../../src/robot_lab_robots/config/robots.yaml).
- [Environment launch profiles](../../src/robot_lab_bringup/config/sim_maps.yaml).
- [Mode profiles](../../src/robot_lab_bringup/config/sim_modes.yaml).
- Package-specific controller, localization, mapping and Nav2 configuration.

| Mode | Current launch intent | Qualification caveat |
|---|---|---|
| `display` | Gazebo selection uses RViz; other selections include their simulator viewer | Non-Gazebo display forwards `gui=true`; not universally physics-free or headless-safe |
| `loc` | Known-map AMCL plus local EKF and drive controls | Needs compatible map, sensor topics, clock and TF |
| `slam` | SLAM Toolbox plus local state estimation | Needs working scan/odometry; not qualified on every backend |
| `3d_slam` | RTAB-Map with RGB, depth, camera info and optional points | Non-Gazebo bridges lack required RGB-D despite mode allowlists |
| `nav` | Known-map localization plus Nav2 | Strongest route is Bumperbot/Gazebo; no fresh mission recertification in this audit |

Configured Nav2 defaults are SmacPlanner2D and Regulated Pure Pursuit; do not infer
the active implementation from the registry inventory. Gazebo uses the
`ros2_control` layer; other bridges implement their own command subscriptions and
state publication. The legacy EKF expects `/robot_lab_controller/odom`, while
those bridges publish `/odom`. The joystick multiplexer targets
`robot_lab_controller/cmd_vel_unstamped`, while the bridges subscribe to
`/cmd_vel`. These need deliberate adapters and tests.

### Registry/composition route: not working orchestration

Separate [registry catalogs](../../src/robot_lab/robot_lab_registry/config)
describe robots, environments, algorithms, scenarios and experiments. At the
baseline they contain 20 robots, 26 environments, 43 algorithms, 18 scenarios and
15 experiments. These are metadata counts, not successful-run counts.

Current gaps that a new agent must not mistake for completed integration:

- Registry and launch-profile IDs differ, for example `go2` versus `unitree_go2`;
  there are 20 catalog robots but 17 main launch profiles.
- `robot-lab launch` prints configuration; execution is explicitly unimplemented.
- `select_robot.launch.py` describes the robot rather than spawning it.
- `select_components.launch.py` fails construction by concatenating strings with
  `LaunchConfiguration` objects.
- The main launch file declares `algorithm` but does not apply its value. GUI
  selection therefore does not prove a different algorithm ran.
- Capability validation is disabled; category/simulator mismatches can pass
  registry composition validation. Main-launch simulator-name validation does
  not repair this separate validation path.

## Target architecture to implement

This section is a future design contract, not existing guarantees. Implement it
in the dependency order recorded in the roadmap.

```text
GUI / CLI / CI
       ↓
one experiment resolver and compatibility validator
       ↓
immutable manifest: robot + backend + environment + scenario
  + algorithms by category + parameters + seed + artifact policy
       ↓
bringup lifecycle and readiness gates
       ↓
simulator/robot adapters ↔ typed ROS algorithm adapters
       ↓
scenario outcome + isolated ground truth + recorded measurements
       ↓
validated result → reports and regression comparisons
```

### Canonical entities and single source of truth

| Entity | Required definition and evidence |
|---|---|
| Robot | Stable ID/aliases, class, description, inertias/collisions, joints/limits, sensors, frames, command/state contracts, backend capabilities, provenance, per-combination evidence |
| Simulator | Engine/version, imports/features, physics step, clock/reset/seed semantics, sensors, commands, ground truth/contacts, readiness and shutdown |
| Environment | ID, world per supported backend, scale/origin, dimensionality, occupancy/geometry correspondence, spawn zones, goals, dynamics, reference paths and provenance |
| Algorithm | Category, distinct method, real executable/plugin, typed I/O, parameters, required sensors/frames, classes, resource limits and evidence |
| Scenario | Task, initialization, goal/stopping conditions, timeouts, collision/fall/flight criteria, perturbations and requested metrics |
| Experiment | Pinned selections/overlays, seed, limits, versions, recording policy and complete resolved manifest |
| Qualification | Exact composition, revision, toolchain/machine, command, test level, outcome, artifacts and known limits |

Use one canonical resolver for CLI, GUI and CI. During migration, support explicit
aliases and translate legacy modes into resolved presets; never silently drop a
selection or maintain separate compatibility rules. A Gazebo world must not
implicitly become qualified for another engine.

### Typed runtime contracts

Document exact topic names, message types, frames, QoS, units, rates, timeouts and
publishers before implementing adapters. These are target requirements; proposed
names are not all present in current code.

| Boundary | Required contract |
|---|---|
| Clock | One `rosgraph_msgs/msg/Clock` publisher per isolated simulation clock domain; consistent consumers; tested stepping/reset |
| Command | Namespaced velocity, joint trajectory/effort or aerial setpoint appropriate to the class; one arbitrated actuation route, limits, stale-command timeout |
| Measurements | Namespaced wheel odometry, IMU, scan, image/depth/camera-info or cloud; explicit frame, calibration, timestamp, covariance and noise/dropout policy |
| Estimated state | Namespaced `nav_msgs/msg/Odometry` and/or pose; one owner of each estimated TF edge |
| Frames | Explicit world/map/odom/body/sensor chain; isolate frame IDs, not merely node/topic namespaces |
| Path/local command | `nav_msgs/msg/Path` with common costmap/obstacle and command interfaces; feasibility and failure outputs |
| Goal/task | Common action/status; Nav2 actions for compatible mobile tasks and explicit adapters for other classes |
| Ground truth | Dedicated `ground_truth/*` state/contact streams, excluded from estimators unless explicitly testing a truth-fed baseline |
| Readiness | Expected type, rate, timestamp progress, finite values, TF connectivity, active lifecycle and process health—not topic existence alone |

All three non-Gazebo spawners currently publish `builtin_interfaces/msg/Time` on
`/clock`, use absolute topics, publish `odom → base_footprint` TF, and derive
odometry from simulator state. They do not satisfy this target. Separate raw
measurements, estimates and simulator truth before comparing filters.

Specify compatible QoS for every pair, including sensors, static metadata and
TF. Put numerical tolerances for timestamp skew, rates and transform age in each
qualification case. Clock reset must reset dependent filters, buffers and task
state; it cannot silently continue a run.

### Lifecycle, isolation and failure handling

```text
resolve → validate → start → ready → reset/seed → ready → record/run
                                                    ↓
                                      success / failure / timeout / abort
                                                    ↓
                                       stop → collect → verify artifacts
```

- Validate before starting processes. Unknown simulator, wrong category, missing
  sensors and unsupported command interfaces are hard errors, not warnings.
- Readiness proves actual physics, advancing time, sensors, TF, controllers and
  required actions. An offline fallback is diagnostic, not a successful run.
- Apply and record seeds in every random component; seeding only a Python generator
  is insufficient. Verify initial pose and world state after reset.
- Run until explicit success/failure/timeout/cancellation. Wall-clock and simulated
  time limits serve different purposes and both matter.
- Isolate process groups, ROS domain/backend transport, topics, TF frames and
  artifact paths. Stop only owned processes; do not broadly kill ROS/simulators
  while another agent or user may be working.
- Clean up on all exits. Test sequential runs and simultaneous isolated robots
  before claiming repeatable reset or multi-robot support.

### Benchmark records and fair comparisons

Schema, normalization, reporting and regression helpers exist. Fixed placeholder
metrics and success after a wait are not benchmark evidence.

Future records must include the resolved manifest/hash, revision/dirty state,
dependency/backend versions, hardware, simulation settings, seeds actually
applied, timestamps, outcome/reason, metrics with units/source, and links/checksums
for logs/recordings. Missing metrics are unavailable with a reason, never invented
zeros. Label synthetic fixtures so reports cannot confuse them with measured runs.

Measure success, trajectory error, simulated/wall duration, path length,
footprint-aware clearance, contacts, falls/flight violations, resource use and
real-time factor as appropriate. Hold initial conditions, sensor model and compute
limits consistent; record tuning budgets, repeat over seeds, and compare raw
metrics/distributions before aggregate scores.

## Extension procedure for agents

Preserve existing assets. Do not add catalog entries first and declare a category
complete. Attach reproducible evidence to each tested combination.

### Add a robot

1. Add licensed description, collisions/inertias and class-specific limits under
   `robot_lab_robots`; check Xacro/URDF and mesh resolution after installation.
2. Define sensors, frames, I/O and explicit backend support. Register canonical
   IDs/aliases without another independent profile interpretation.
3. Implement the backend/control adapter and safe reset/spawn. Wheel control does
   not establish legged locomotion or flight support.
4. Add static/install tests, then real headless sensor/command smoke checks.
5. Complete a class-appropriate task: mobile goal, legged traversal, humanoid
   balance/walking or takeoff–waypoints–landing. Record limits and failures.
6. Promote only the tested configuration; qualify other backends/maps separately.

### Add an environment

1. Add provenance and backend-specific assets; check units, transforms, collisions,
   dimensions and installed resource resolution.
2. Supply occupancy maps where appropriate, with generation/provenance and geometry
   agreement tests; 3D geometry alone is not a valid 2D navigation map.
3. Define free-space spawn/goals, reference paths, reset and deterministic dynamics.
   Distinguish occlusion from noise/dropout models.
4. Test loading and a compatible scenario per advertised backend. Do not inherit
   qualification from a related world.

### Add an algorithm

1. Choose a distinct method and state assumptions, I/O and failure behavior. Keep
   utilities/relays separate from comparative algorithm breadth.
2. Add analytic/reference tests, including degenerate/adversarial cases; label
   educational approximations honestly.
3. Implement a real installed ROS node/plugin: consume inputs, execute the method,
   publish outputs, expose parameters and obey clock/namespace contracts.
4. Register compatibility and connect the common resolver. Test that switching the
   ID changes the executable/plugin and effective parameters.
5. Add input-to-output and mission checks, then repeated measured comparisons with
   a reference under the same conditions.

### Add or qualify a simulator backend

1. Implement installation/runtime discovery without host-specific paths. Missing
   optional engines must produce an explicit unavailable result.
2. Test model/world import and real physics independently from the ROS bridge.
3. Implement typed clock, sensors, commands, TF policy, truth, contacts, seeds/reset,
   readiness and shutdown.
4. Verify message traffic and command response, then complete the same mobile
   reference scenario as the primary backend.
5. Qualify RGB-D, terrain, flight and extra classes separately. Engine boot or a
   generic falling-body test is not navigation qualification.

## Validation and documentation policy

Progress from schema/references to assets/install, launch construction, numerical
correctness, ROS contracts, scenario smoke and repeated measured benchmarks.
Each level proves only its own scope.

The recorded audit found 485 passing selected source tests and one failure, plus
five passing selected backend tests. It did not certify full missions, GUI,
all optional engines or hardware. CI has branch-coverage and ignored-failure gaps.
Use the linked audit for exact limits, not a blanket passing-platform badge.

When completing a roadmap task, update evidence, support matrix and machine status
together. Keep historical counts dated. Verify licenses per asset/dependency;
the repository license does not override third-party licenses. Record unresolved
blockers and the exact next check in the handoff so another agent can resume.
