# Robot Lab architecture: current implementation and target

Documentation updated October 9, 2026. Exact source stages and current checks
are in the [extension checkpoint](../status/continuation-2026-10-08.md). The [current status](../status/CURRENT_STATUS.md),
[ledger](../status/platform-status.yaml) and
[support matrix](../status/support-matrix.md) distinguish implemented wiring,
named measured missions and remaining qualification. Historical audits and
source-stage reports retain their original scope.

## Goal and present gap

The target is a reproducible learning platform with independently selected
robots, simulators, maps and seven algorithm categories. Five substantive,
ROS-connected and fairly measured alternatives in every category remain a
qualification goal. Metadata, numerical examples and static previews do not
close it.

The current system includes profile-driven four-backend bringup, shared GUI/CLI
resolution, physical wheel drives, measured TurtleBot3/TurtleBot4 modes, native
Panda joint/Hand/MoveIt control, and actual PX4 X500 flight on Gazebo Harmonic.
Launch has a separate right-hand control column and structural robot filters;
Registry has grouped variants/components and native embedded 3D inspection.
These workflows have exact plant/world/backend/source evidence. Remaining
asset/controllers/world missions, algorithm comparisons, concurrency and
clean-host reproduction are partial.

The current user priority is R3.6 and R6.5–R6.7, then R5.10 and R5.7–R5.9,
before the older wheel/legged/comparison roadmap. Follow actual owners and
[AGENT_HANDOFF](../AGENT_HANDOFF.md); preserve separately owned terrain and
occupancy work.

## October 9 implementation additions

New candidate controls preserve native source plants: exact-model Unitree
TorchScript observations/PD torques, generic authored position channels,
local Jacobian Servo and taught measured articulation sequences. Measured
Panda object state feeds acknowledged MoveIt collision attachment diffs;
physical grasp remains native contact dynamics. Stretch has separate wheel
transmission control and base/articulation ownership, source-mounted sensor
publishers and explicit sensor-gated mapping/Nav2 composition. Truth-bootstrap
EKF inputs are not independent state-estimation accuracy claims.

Static aerial geometry planning runs asynchronously so FCU setpoints remain
responsive. PX4 executes bounded waypoint setpoints from actual FCU state and
retains manual/Stop/Hold/Land overrides. The benchmark coordinator launches
actual owned subprocesses with domain/partition isolation and real telemetry;
process/trace results remain separate from mission success. Cached coplanar
collision reduction preserves validated planar unions and source visual/raycast
assets; physical speed/contact measurements remain pending.

Read [the implementation checkpoint](../status/implementation-2026-10-09.md)
for exact files/commands, still-missing initial features and deferred validation.
These new integrations have not inherited historical source certificates or
full robot/map/backend qualification.

## Actual source-tree and package map

There are 26 discoverable ROS packages; current CI builds 25 and excludes
optional ORB-SLAM3. Nested vendor descriptions are assets of the enclosing
`robot_lab_robots` package, not individually supported robot stacks.

```text
src/
  robot_lab/
    robot_lab_registry/     # catalogs, schemas, validation, query CLI
    robot_lab_benchmark/    # result records, runner/reporting foundations
  robot_lab_adapter/        # shared resolver, selectors and class adapters
  robot_lab_bringup/        # profile-driven launch orchestration
  robot_lab_description/    # Gazebo/display launch and description support
  robot_lab_robots/         # consolidated first-party and vendored robot assets
  robot_lab_maps/           # worlds, occupancy maps, arena tools and metadata
  robot_lab_models/         # shared world models
  robot_lab_pybullet/       # PyBullet bridge and spawner
  robot_lab_mujoco/         # MuJoCo bridge and spawner
  robot_lab_isaac/          # ROS bridge plus external Isaac runtime process
  robot_lab_utils/          # shared contracts, source geometry, grouping/taxonomy
  robot_lab_algorithms/     # numerical examples and ROS adapters
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
| `robot_lab_gui` | Launch control column, category/type filters, native Registry 3D, profiles/maps/status/monitoring | Use the same resolver/runner as CLI; no private support rules |
| `robot_lab_vacuum_cleaning` | Additional basic vacuum controller | Reconcile duplication with cleaning logic in `robot_lab_controller` |
| `robot_lab_firmware`, `robot_lab_msgs` | Serial/Arduino interface and shared messages | Separate hardware operation from simulation-only workflows |
| `robot_lab_cpp_examples`, `robot_lab_py_examples`, `orbslam3` | Teaching examples and optional external integration | Keep optional dependencies out of the reference build's critical path |

## Current execution paths

### Main profile-driven route

```text
GUI or ros2 launch robot_lab_bringup simulated_robot.launch.py
  → load robot profile + map profile + mode profile
  → resolve exact source profile, backend, mode, algorithms and runtime gates
  → choose Gazebo / PyBullet / MuJoCo / Isaac launch adapter
  → launch the compatible localization/mapping/navigation, native-arm or Flight stack
  → sensors → pose estimate → planner → path follower → command → robot
```

The implementation is
[simulated_robot.launch.py](../../src/robot_lab_bringup/launch/simulated_robot.launch.py).
Its configuration sources are:

- [Core robot profiles](../../src/robot_lab_robots/config/robots.yaml) and pinned SSD installed extension profiles.
- [Grouping](../../src/robot_lab_bringup/config/asset_groups.yaml) and
  [structural taxonomy](../../src/robot_lab_bringup/config/robot_taxonomy.yaml); neither grants controller support.
- [Measured extension support](../status/asset-runtime-support.yaml) with exact source/numeric guards.
- [Environment launch profiles](../../src/robot_lab_bringup/config/sim_maps.yaml).
- [Mode profiles](../../src/robot_lab_bringup/config/sim_modes.yaml).
- Package-specific controller, localization, mapping and Nav2 configuration.

| Mode | Current path | Qualification boundary |
|---|---|---|
| `display` | Selected robot/world and requested backend/RViz viewers; either robot or world may be omitted | Source imports and rest/display behavior remain asset/backend-specific; native Panda additionally has its guarded arm workflow |
| `loc` | Known-map localization and compatible state estimation/Drive | Named T3/T4 and mobile screens; passive legged localization does not supply a gait |
| `slam` | SLAM Toolbox, odometry and actual map export | Named T3/T4 and selected 4WS/mecanum mapping/reset screens; full matrix remains |
| `3d_slam` | RTAB-Map RGB/depth/info and real database/PCD export | Named T4 and selected wheeled RGB-D screens; RGB-only T3 cameras remain unavailable |
| `nav` | Map/localization/Nav2 with robot-specific overlays | Named clear/obstacle missions; other maps and repeated avoidance/contact tasks remain |
| `flight` | PX4 X500 FCU/ROS2 control on Gazebo Harmonic | Actual `nav_empty`/`nav_obstacle` flight/altitude/goals/land; wider world/aerial autonomy remains |

Robot and steering overlays choose compatible planners/controllers. Differential
bases use their measured Nav2 defaults; car turning constraints and parallel
4WS critics are explicit. The GUI command and shared resolver apply those
choices before startup.

The `/key_vel` Drive route reaches the multiplexer and each selected controller.
Non-Gazebo bridges accept the post-mux command with a raw command fallback;
Gazebo uses the active `ros2_control` interfaces. Controller odometry and
`/odom/ground_truth` remain separate. Sensors use authored link frames; filter/
TF ownership and reset must be verified per exact cell. Robot reset preserves
monotonic simulation time and reseeds estimators from measured state.

Native Panda uses original MuJoCo position/finger actuators, guarded actions
and a source-derived static MoveIt scene. PX4 Flight uses its FCU and actual
X500 plant; wheel controllers and passive model holds do not implement either
workflow. See [Panda](../tutorials/panda_arm.md),
[PX4](../tutorials/px4_x500.md) and the
[GUI workspace](../tutorials/gui-workspace.md).

### Shared resolver and GUI composition route

The registry catalogs describe robots, environments, algorithms, scenarios and
experiments. The current CLI resolver and GUI composition layer share typed
validation, legacy aliases, resolved manifests, planner plugin selection and
unsupported-combination diagnostics. This is an implemented composition path,
not proof that every catalog cell can execute a mission. GUI execution and
class-specific readiness still require the exact workflow and evidence below.

Current boundaries that remain important:

- Catalog and legacy profile counts still differ; aliases are explicit rather
  than silently merged.
- Dry-run by default proves manifest construction and validation only. `--execute`
  or a GUI Run action must still pass readiness, task and cleanup checks.
- A planner selector changing a Nav2 plugin is not a benchmark comparison.
- Absolute topics, frame IDs, simulator truth and estimator inputs must be
  checked for each exact robot/backend/map cell.

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

The non-Gazebo spawners now publish `rosgraph_msgs/msg/Clock` on `/clock`,
expose simulator truth separately from controller odometry, and no longer
publish the estimator-owned `odom → base_footprint` edge. Commands and
watchdogs are adapter-specific, so topic names, QoS and class controllers still
require explicit tests. Do not infer estimator quality from simulator truth.

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

Exact `091d388` CI passes the core build, 933 fast checks with four skips/four
deselections, 20 physics checks, 134 integration checks with twelve skips and
registry validation. Final local GUI and physical Drive/Panda checks have
separate measured artifacts. [Testing](../TESTING.md) and
[current status](../status/CURRENT_STATUS.md) retain the scopes; full mission,
concurrency, hardware and clean-host qualification remain open.

When completing a roadmap task, update evidence, support matrix and machine status
together. Keep historical counts dated. Verify licenses per asset/dependency;
the repository license does not override third-party licenses. Record unresolved
blockers and the exact next check in the handoff so another agent can resume.
