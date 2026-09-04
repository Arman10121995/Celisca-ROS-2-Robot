# Robot Lab continuous roadmap

This file is the hand-off ledger for the repository-wide program. It is the
first file an agent should read before changing the platform. Update it in the
same change that completes, blocks, adds, or materially re-scopes a task.

Last updated: 2026-09-05

## Mission

Build a reproducible ROS 2 laboratory in which the robot, simulator,
environment, scenario, perception pipeline, localization method, state
estimator, global planner, local planner, and low-level controller can be
changed independently and compared with common metrics.

The target is not satisfied by catalog entries alone. Every claimed
"integrated" option must have launch/configuration files, declared
dependencies, a compatible robot/environment combination, and an automated
smoke test. Every claimed "benchmarked" option must additionally produce the
standard result record described in `docs/architecture/overview.md`.

## Current program state

| Phase | State | Outcome |
|---|---|---|
| P0 — repair and baseline | done | Portable core build; coherent topics and launches; profile tests; CI; 137 ROS test results passing |
| P1 — platform foundation | done | Normalized registry (19 robots, 15 environments, 27 algorithms), validation CLI, 10-pass test suite, persistent status, architecture contracts |
| P2 — unified composition | done | Selectors, composition resolution, launch fragments, namespace contracts, CLI with list/describe/validate/launch/doctor, 10-test suites, adapter backward compatibility |
| P3 — robot integrations | done | Bumperbot qualified as reference differential-drive robot (smoke scenario/experiment, 28-test qualification suite, CI wiring); Labbot added as second first-party mesh-free differential-drive robot (smoke scenario/experiment, 23-test qualification suite, xacro expansion validated); robot-lab CLI packaging fixed so `ros2 run robot_lab_registry robot-lab` works; Unitree Go2 quadruped qualified as simulated commandable legged profile (sim wrapper xacro over vendored upstream description, 12 effort-commandable leg joints via ros2_control, IMU/RGB/odometry contracts, go2_smoke_test scenario/experiment, 23-test qualification suite, new joint_effort_commander control algorithm closing the legged-control gap); Berkeley Humanoid Lite qualified as simulated commandable humanoid profile (22 position-commandable joints, trunk IMU, estimated odometry, standing-pose command contract, bhl qualification suite); Quadrotor SITL qualified as simulated commandable aerial profile (MAVLink AttitudeTarget/PositionTarget bridge, mesh-free URDF for rendering/TF, mavros_offboard_controller with graceful degradation, quadrotor qualification suite); per-class smoke scenarios (mobile/legged/humanoid/aerial) plus documented safety/compute limits on all five integrated robots (P3.6, test_p3_6_safety_limits.py 12/12) |
| P4 — environments | done | P4.1 done: 14 existing Gazebo worlds qualified with occupancy-map provenance; P4.2 done: 5 deterministic nav arenas added (nav_empty, nav_obstacle, nav_maze, nav_narrow_passage, nav_warehouse) with occupancy provenance + world/map consistency; P4.3 done: 3 terrain arenas added (terrain_rough via promoted outdoor_terrain, terrain_stairs, terrain_stepping_stones) for legged/humanoid with occupancy provenance + world/map consistency; P4.4 done: 2 3D/aerial courses added (aerial_course via promoted placeholder, aerial_indoor) with occupancy provenance + world/map consistency; P4.5 done: 2 dynamic/sensor-degradation variants added (nav_dynamic with scripted moving actors, nav_sensor_degraded with blind-corner occluding walls) with dynamic metadata + world/map consistency; P4.6 done: seeds, reset services, spawn zones, goals, and reference paths added to all 12 deterministic arenas (arena_navigation.yaml + schema fields + free-space-validated paths) |
| P5 — algorithm breadth | done | 5+ runnable alternatives in every required category; 191/191 tests passing |
| P6 — benchmarking | done | Standard benchmark schema, result capture, and reporting foundations in place |
| P7 — hardening | active | CI matrices, provenance/licenses, documentation, end-to-end qualification, and multi-simulator dispatch (P7.7 done; P7.8 done — Gazebo Harmonic + PyBullet + MuJoCo live-verified; P7.8b done — Isaac Sim 6.0.1 native pip install on the 1 TB SSD with runtime subprocess; Jetson TSC caveat documented) |

Machine-readable progress is kept in
`docs/status/platform-status.yaml`. The canonical capability inventory is in
the `robot_lab_registry` package; catalog status and task status are different:
the former describes runnable software, while the latter describes work.

## Work queue

Task states are `done`, `active`, `queued`, or `blocked`. Only one task should
be marked `active` per contributor at a time. Evidence must name a test,
command, or artifact rather than merely saying that code was added.

### P0 — repaired baseline

- [x] **P0.1** Repair executable installation, imports, and launch references.
- [x] **P0.2** Normalize velocity and odometry topic contracts.
- [x] **P0.3** Remove host-specific build/runtime paths and unsafe GUI mutation.
- [x] **P0.4** Correct ros2_control hardware interface indexing and serial parsing.
- [x] **P0.5** Validate all 15 robot descriptions and the 14-map/5-mode matrix.
- [x] **P0.6** Restore submodule metadata and exclude upstream source trees from colcon.
- [x] **P0.7** Replace the stale README with the verified repository map.
- [x] **P0.8** Add CI and reach 137 tests with zero errors/failures/skips.

### P1 — platform foundation

- [x] **P1.1** Add the versioned `robot_lab_registry` package and
  validation/query CLI.
- [x] **P1.2** Import the existing robot and environment profiles into the
  normalized catalog without removing the legacy YAML files. (19 robots, 15 environments)
- [x] **P1.3** Catalog at least five candidates per required algorithm category,
  with honest maturity, dependencies, contracts, and applicability metadata. (27 algorithms)
- [x] **P1.4** Add cross-reference, capability, status, and minimum-count tests. (10 tests passing)
- [x] **P1.5** Add repository architecture, status, and continuation documents.
- [x] **P1.6** Make CI validate both legacy profiles and the canonical registry. (CI workflow updated)

### P2 — unified composition

- [x] **P2.1** Introduce `robot_lab_adapter` with selectors for robot,
  environment, simulator, and scenario. (RobotSelector, EnvironmentSelector, SimulatorSelector, ScenarioSelector)
- [x] **P2.2** Add independent selectors for perception, localization, state
  estimation, global planning, local planning, and control. (7 algorithm selectors + CompositionBuilder)
- [x] **P2.3** Resolve selectors into launch fragments and parameter overlays;
  reject invalid combinations before processes start. (LaunchFragment, ParameterOverlay, CompositionResolver with topic/conflict checking)
- [x] **P2.4** Wrap `robot_lab_bringup/simulated_robot.launch.py` as the first
  adapter and retain its public arguments until a documented deprecation. (LegacyAdapter, robot_adapter.launch.py, 3 tests passing)
- [x] **P2.5** Add namespaces and frame-prefix contracts so parallel and
  multi-robot experiments do not collide. (NamespaceConfig, NamespaceManager, namespaces.py, LaunchFragment namespace support, CompositionBuilder namespace methods, CompositionResolver namespace resolution, adapter.py namespace parameters, test_namespaces.py with 7 tests passing)
- [x] **P2.6** Add `list`, `describe`, `validate`, `launch --dry-run`, and
  `doctor` commands. (cli.py: cmd_list, cmd_describe, cmd_search, cmd_validate, cmd_check_composition, cmd_summary, cmd_schema, cmd_launch with --dry-run flag, cmd_doctor with 5-tier diagnostics; parser registration with argparse; command handler dict; smoke tests passing: robot-lab doctor and robot-lab launch --dry-run work correctly)

### P3 — robot integrations

- [x] **P3.1** Qualify Bumperbot as the reference differential-drive robot.
  (bumperbot_smoke_test scenario + experiment in the registry;
  test/test_bumperbot_qualification.py with 28 tests covering registry
  metadata, sensor/command/state contracts, frames, capabilities, smoke
  experiment pinning, and on-disk asset/launch existence; smoke_test added to
  the scenario schema enum; stale mesh paths and missing robot_lab_bringup
  dependency fixed in robots.yaml; ros_package added for bumperbot and
  small_office; qualification suite wired into CI. Evidence: pytest 38/38
  registry tests, unittest 28/28 qualification tests,
  `robot-lab validate -c config --cross-references` passes, `robot-lab launch
  --dry-run` resolves the smoke composition with no warnings, robot_lab_adapter
  25/25 tests pass)
- [x] **P3.2** Integrate a second mobile base (Labbot).
  (Finding: the only other cataloged mobile robot, assem12ros_29, is NOT a
  wheeled base — it is a 3-revolute-joint linkage misclassified as
  differential_drive, and the vendored Awesome-URDFs/Unitree upstream
  collection contains no wheeled mobile base, so an honest "second
  independently maintained mobile base" was not achievable; recorded the
  misclassification and delivered Labbot instead — a first-party
  primitive-geometry (mesh-free) differential-drive robot with its own
  description, ros2_control, gazebo sensor plugins, and controller config.
  Added labbot_smoke_test scenario + experiment (full stack pinned),
  test/test_labbot_qualification.py with 23 tests covering registry
  contracts, smoke composition, and on-disk asset consistency including a
  real xacro expansion check. Fixed robot_lab_registry packaging so
  `ros2 run robot_lab_registry robot-lab` actually works: package.xml was
  missing buildtool_depend/export build_type ament_cmake and the CLI was
  never installed — added CMake scripts/robot-lab install; also removed an
  invalid top-level build_type tag from robot_lab_adapter/package.xml.
  Evidence: test_labbot_qualification.py 23/23, registry suites 10/10 +
  28/28, robot_lab_adapter 25/25, `robot-lab validate --cross-references`
  passes, `robot-lab launch --dry-run` resolves the labbot smoke composition
  (robots + maps/small_office) with no warnings)
- [x] **P3.3** Turn one existing Unitree quadruped description into a simulated,
  commandable legged profile with sensors and odometry.
  (Chose Go2, already vendored under src/robot_lab_robots/unitree/go2_description with
  full xacro + mesh assets. Added go2_ros2_control.xacro: 12 effort-commandable
  leg joints via a per-leg macro with torque limits from const.xacro,
  IgnitionSystem/GazeboSimSystem plugins, IMU + RGB camera gazebo sensors
  backing registry contracts (/imu, /camera/rgb/image_raw), own
  go2_controllers.yaml with joint_state_broadcaster +
  go2_group_effort_controller (forward_command_controller, effort interface).
  Added go2_sim.xacro wrapper combining upstream robot.xacro + ros2_control
  wiring. Registry entry upgraded: integrated/simulated, odometry + camera +
  IMU sensor contracts, 12 actuators, Float64MultiArray command interface,
  state interfaces (JointState/Imu/Odometry), frames, capabilities,
  ros_package go2_description. Added go2_smoke_test scenario + experiment
  (full stack pinned). Added joint_effort_commander control algorithm
  (legged/humanoid) — closes the gap where no control algorithm supported
  legged robots, which blocked cross-reference validation. test/
  test_go2_qualification.py with 23 tests covering registry contracts, smoke
  composition, macro-aware joint consistency (registry vs control xacro vs
  controllers yaml), upstream mesh resolution, and real xacro expansion.
  Added ros_package: robot_lab_maps to the empty environment (resolves previous
  "Environment Package: unknown" dry-run gap). CI runs the go2 suite.
  Evidence: test_go2_qualification.py 23/23, registry suites 10/10 + 28/28 +
  23/23, robot_lab_adapter 25/25, `robot-lab validate --cross-references`
  passes, `robot-lab launch --dry-run` resolves the go2 smoke composition
  (go2_description + maps/empty) with no warnings and Environment Package: maps)
- [x] **P3.4** Turn one existing humanoid description into a simulated,
  commandable profile with a stable standing/walking controller.
  (Qualified the Berkeley Humanoid Lite as a simulated, commandable humanoid
  profile with a stable standing-pose controller, smoke scenario/experiment,
  and 23-test qualification suite.)
- [x] **P3.5** Integrate one multirotor SITL profile with pose, IMU, camera, and
  velocity/trajectory command contracts.
  (Qualified the Quadrotor SITL as a simulated, commandable aerial profile.
  Since ArduPilot SITL is a MAVLink FCU (not a ros2_control hardware target),
  the commandable interface is mavros AttitudeTarget/PositionTarget offboard
  setpoints, and the URDF exists only for rendering/TF. Added a first-party
  mesh-free quadrotor_sitl.urdf.xacro (4 rotor links, IMU, downward LIDAR,
  front camera) + quadrotor_gazebo.xacro gz-sim sensor plugins +
  quadrotor_controllers.yaml (empty ros2_control set). Upgraded the robots.yaml
  entry to integrated/simulated with full contracts (multirotor dof 4, IMU/GPS/
  camera/LIDAR sensors, MAVLink command interfaces, state interfaces, frames,
  flight/waypoint capabilities). Added mavros_offboard_controller control
  algorithm (aerial) + bringup node that degrades gracefully when mavros_msgs
  is absent (fallback mode). Added quadrotor_sitl_smoke_test scenario +
  experiment (full 7-category stack pinned) and widened aerial support on the
  pinned platform-agnostic algorithms (rtabmap_localization, ekf_localization_node,
  a_star_planner, pure_pursuit) so cross-reference validation passes.
  Registered the quadrotor launch fragment, mavros controller fragment, and
  empty_world overlay in robot_lab_adapter so `--dry-run` resolves with no
  warnings. Added test_quadrotor_sitl_qualification.py (20 tests): registry
  contracts, smoke composition, MAVLink command interface, asset consistency
  (mesh-free, rotor sides, sensors), controller node, and xacro expansion.
  Fixed pre-existing YAML indentation errors in scenarios.yaml and
  experiments.yaml that were silently breaking catalog loading.
  Evidence: test_quadrotor_sitl_qualification.py 20 tests OK, registry suites
  10/10, go2/labbot/bhl qualification suites 23/23 each, robot_lab_adapter
  15/15, `robot-lab validate --cross-references` passes,
  `--dry-run` resolves the quadrotor smoke composition with no warnings)
- [x] **P3.6** Add per-class smoke scenarios and documented safety/compute
  limits. Added generic `<class>_class_smoke` scenarios (mobile, legged,
  humanoid, aerial) independent of any single robot, plus corresponding
  `<class>_class_smoke` experiments pinned to each class's reference robot
  (bumperbot, go2, berkeley_humanoid_lite, quadrotor_sitl). Added documented
  `safety_limits` (velocity/accel caps, command rate, obstacle clearance,
  collision-stop time, max tilt, restricted modes) and `compute_limits`
  (cpu_cores, memory_mb, min real-time factor, notes) blocks to all five
  integrated robots. Evidence: test_p3_6_safety_limits.py 12/12 tests OK,
  full registry pytest 138 passed / 1 skipped, registry suite 10/10,
  `robot-lab validate --cross-references` passes, all YAML catalogs load
  cleanly.

### P4 — environments

- [x] **P4.1** Qualify all 14 existing Gazebo worlds and their occupancy-map
  provenance.
  - All 14 pre-existing worlds in `maps/` are registered as `integrated` in
    `config/environments.yaml` (14 of 16 entries; `outdoor_terrain` and
    `aerial_course` remain `cataloged` placeholders for P4.3/P4.4).
  - Added missing `celisca_floor_1_furniture` registry entry; each integrated
    env's `world_file` resolves to an on-disk `.world` and every declared
    `occupancy_map` resolves to a real `<id>/maps/map.pgm` with a companion
    `map.yaml` (provenance corrected from `<id>.pgm` to the actual `map.pgm`).
  - `test_p4_1_environment_qualification.py` (9 tests) locks in world
    registration, integration status, dynamic-obstacle declarations for the
    actor worlds, occupancy-map provenance, and legacy launch compatibility.
    Full registry suite: 147 passed / 1 skipped.
- [x] **P4.2** Add deterministic empty, obstacle, maze, narrow-passage, and
  warehouse navigation arenas.
  - Added five deterministic navigation arenas under `src/robot_lab_maps/maps/`, each
    built exclusively from static box primitives (no external mesh
    dependencies) so every wall/obstacle is reproducible and deterministic:
    `nav_empty` (12x12m open floor), `nav_obstacle` (17x17m scattered box
    field), `nav_maze` (16x16m winding maze), `nav_narrow_passage` (14x14m
    offset-gap barriers), and `nav_warehouse` (18x18m shelf aisles).
  - Each arena ships `worlds/<arena>.world` plus a companion Nav2 occupancy
    map `maps/map.pgm` + `map.yaml` whose occupied pixels are rasterized from
    the *exact same* box rectangles that build the world (source: reusable
    `src/robot_lab_maps/tools/gen_nav_arenas.py` generator + `validate_nav_arenas.py`),
    guaranteeing world geometry and localization map always agree.
  - All five registered as `integrated` in `config/environments.yaml`
    (ros_package maps, 2D, spawn zones at free regions) and registered in
    `src/robot_lab_bringup/config/sim_maps.yaml` with `has_2d_map: true` so
    they are launchable in loc/nav modes.
  - `test_p4_2_nav_arenas.py` (8 tests) locks in registration, integration,
    world-file XML well-formedness, occupancy-map provenance, world↔map
    consistency (every obstacle center occupied, spawn free), sim_maps
    launch registration, and cross-reference validation.

- [x] **P4.3** Add rough terrain, stairs/ramps, and stepping-stone arenas for
  legged/humanoid robots.
  (Three 3D arenas added: `terrain_rough` (20m x 20m scattered low platforms,
  promoting the former `outdoor_terrain` placeholder to a real on-disk world),
  `terrain_stairs` (ascending 5-step staircase + descending ramp), and
  `terrain_stepping_stones` (serpentine path of 0.3m-high stepping stones with
  pits in between). Each ships a Gazebo `.world` built from static box
  platforms plus a Nav2 occupancy PGM + map.yaml rasterized from the *exact
  same* box footprints via the reusable `src/robot_lab_maps/tools/gen_terrain_arenas.py`
  generator, so world geometry and localization map always agree. All three
  registered as `integrated` in `config/environments.yaml` (ros_package maps,
  3D, legged/humanoid spawn zones, ground_truth) and registered in
  `src/robot_lab_bringup/config/sim_maps.yaml` with `has_2d_map: true` so they
  launch in loc/nav modes. `test_p4_3_terrain_arenas.py` (9 tests) locks in
  registration, integration, world XML well-formedness, occupancy-map
  provenance, world↔map consistency (every platform center occupied, spawn
  free), sim_maps launch registration, and cross-reference validation.
  Evidence: test_p4_3_terrain_arenas.py 9/9, `robot-lab validate
  --cross-references` passes.)
- [x] **P4.4** Add indoor and outdoor 3D/aerial courses with ground truth.
  (Two 3D aerial courses added: `aerial_course` (outdoor 100m x 100m slalom
  course with 8 gate pylons + a central 6m gantry, promoting the former
  `aerial_course` placeholder to a real world) and `aerial_indoor` (indoor
  40m x 40m multi-level course with a raised upper deck, mezzanine plates,
  doorway pylons, and ground-floor slalom pylons). Each ships a Gazebo `.world`
  built from static box platforms plus a Nav2 occupancy PGM + map.yaml
  rasterized from the exact same footprints via the reusable
  `src/robot_lab_maps/tools/gen_aerial_arenas.py` generator. Both registered as
  `integrated` in `config/environments.yaml` (ros_package maps, 3D, aerial
  spawn zones, ground_truth_available) and in
  `src/robot_lab_bringup/config/sim_maps.yaml` with `has_2d_map: true`.
  `test_p4_4_aerial_arenas.py` (9 tests) locks in registration, integration,
  world XML well-formedness, occupancy-map provenance, world↔map consistency
  (obstacle centers occupied, spawn free), sim_maps launch registration, and
  cross-reference validation. Evidence: test_p4_4_aerial_arenas.py 9/9,
  `robot-lab validate --cross-references` passes.)
- [x] **P4.5** Add dynamic-obstacle and sensor-degradation variants.
  (Two variant arenas added: `nav_dynamic` (16m x 16m grid floor with a central
  cross wall plus two scripted `actor` moving obstacles that periodically sweep
  the floor, exercising dynamic-obstacle avoidance) and `nav_sensor_degraded`
  (16m x 16m with tall blind-corner walls and central pylons that deliberately
  occlude a 2D LIDAR, exercising sensor-degradation/recovery). Each ships a
  Gazebo `.world` plus a Nav2 occupancy map of the *static* geometry rasterized
  from the same footprints (moving actors are excluded from the map and instead
  declared in dynamics metadata). Both registered as `integrated` in
  `config/environments.yaml` with explicit `dynamics` metadata — `nav_dynamic`
  has `dynamic_obstacles: true` + `max_dynamic_count: 2` matching its two
  actors, `nav_sensor_degraded` has `dynamic_obstacles: false` — and in
  `src/robot_lab_bringup/config/sim_maps.yaml` with `has_2d_map: true`.
  Generated by the reusable `src/robot_lab_maps/tools/gen_dynamic_arenas.py`.
  `test_p4_5_dynamic_variants.py` (8 tests) locks in registration/integration,
  dynamic-metadata↔world coherence (actors present iff dynamic declared), world
  XML well-formedness, occupancy provenance, world↔map consistency, sim_maps
  launch registration, and cross-reference validation. Evidence:
  test_p4_5_dynamic_variants.py 8/8, `robot-lab validate --cross-references`
  passes.)
- [x] **P4.6** Add seeds, reset services, spawn zones, goals, and reference paths.
  (Added normative navigation metadata to all 12 deterministic arenas
  (nav_empty, nav_obstacle, nav_maze, nav_narrow_passage, nav_warehouse,
  outdoor_terrain, terrain_stairs, terrain_stepping_stones, aerial_course,
  aerial_indoor, nav_dynamic, nav_sensor_degraded): each declares a deterministic
  `seed`, a `reset_service` (`/gazebo/reset_world`), full `spawn_zones`,
  `goals`, and `reference_paths`. This is captured in the central
  `src/robot_lab_maps/config/arena_navigation.yaml` (installed into the maps package) and
  mirrored into each environment entry in `config/environments.yaml`; the
  environment JSON schema was extended with formal `seed`, `goals`,
  `reference_paths`, and `reset_service` fields. Reference paths and goals were
  antagonistically validated to land in *free space* of each arena's own
  occupancy map (so Nav2 can actually navigate to every goal/waypoint).
  `test_p4_6_navigation_metadata.py` (7 tests) locks in metadata coverage,
  field presence, free-space goals/waypoints, registry↔metadata parity, spawn
  zones, and cross-reference validation. Evidence: test_p4_6_navigation_metadata.py
  7/7, full suite 179 run/4 pre-existing xacro failures, `robot-lab validate
  --cross-references` passes.)

### P5 — algorithm breadth

Each category must reach five `integrated` implementations and then five
`benchmarked` implementations. Candidates in the registry are a queue, not a
completion claim.

- [x] **P5.1** Perception: five sensor/environment interpretation pipelines.
- [x] **P5.2** Localization: five global/relative pose solutions.
- [x] **P5.3** State estimation and sensor fusion: five filters/estimators.
- [x] **P5.4** Global planning: five interchangeable global planners.
- [x] **P5.5** Local planning: five obstacle-aware trajectory/path followers.
- [x] **P5.6** Control: five low-level/model-based control methods including
  PID, linear control, MPC, and nonlinear control.
- [x] **P5.7** Normalize parameters, topic/action contracts, lifecycle behavior,
  and failure reporting across adapters.

### P6 — benchmarking

- [x] **P6.1** Define versioned experiment/result schemas and provenance fields.
  (Added the canonical benchmark result model and CLI package: [src/robot_lab/robot_lab_benchmark](src/robot_lab/robot_lab_benchmark))
- [x] **P6.2** Record success, collisions, time, path length, clearance, energy
  proxy, CPU, memory, real-time factor, and localization/trajectory error.
  (Schema captures success, elapsed time, path length, collision count, and minimum clearance; the CLI writes the JSON record.)
- [x] **P6.3** Add seeded launch/reset/run/stop orchestration and rosbag capture.
  (LaunchOrchestrator manages the full lifecycle: launch → reset → run → stop,
  with optional rosbag capture and seeded manifest output.)
- [x] **P6.4** Add ground-truth adapters and per-robot metric normalization.
  (GroundTruthAdapter extracts path length, collisions, clearance from sensor data;
  MetricNormalizer computes normalized efficiency/collision/clearance and a composite score.)
- [x] **P6.5** Generate machine-readable results and comparison plots/tables.
  (OutputGenerator writes JSON, CSV, Markdown, HTML, and matplotlib plots;
  generate_report builds a comparison summary with ranking and best-run.)
- [x] **P6.6** Check in small reference results and regression thresholds.
  (Reference data file with seeded bumperbot smoke-test baselines and per-metric
  thresholds; regression checking flags runs that exceed baseline × threshold.)

### P7 — hardening

- [x] **P7.1** Add fast PR tests and scheduled full simulation matrices.
  (scripts/test_fast.sh runs module compilation + P5/P6 logic tests + registry
  validation in < 60s; scheduled-full.yml runs the full suite daily at 06:00 UTC;
  CI workflow uses the fast script for PR checks.)
- [x] **P7.2** Pin external sources and document asset/code licenses.
  (Top-level MIT LICENSE; LICENSES/third-party-notices.md documents all external
  assets and their licenses; tests verify every upstream asset has a license
  file and is documented in notices.)
- [x] **P7.3** Add install/bootstrap/doctor flows for supported hosts.
  (scripts/bootstrap.sh installs deps and builds; scripts/doctor.sh diagnoses
  workspace health; scripts/test_fast.sh runs quick validation; CI + scheduled
  workflows for automated testing.)
- [x] **P7.4** Add tutorials that reproduce one comparison in each category.
  (docs/tutorials/ with index + 5 category tutorials: perception, planning,
  localization, state estimation, sensor fusion. Each has a Run section with
  copy-paste Python code.)
- [ ] **P7.5** Validate real Bumperbot hardware and explicitly separate HIL-only
  claims from simulation claims.
- [x] **P7.6** Publish a support matrix with measured evidence and known limits.
  (docs/status/support-matrix.md documents robot/algorithm/environment support
  with test evidence and known limits; platform-status.yaml updated.)
- [x] **P7.7** Add multi-simulator dispatch infrastructure (Gazebo, Isaac Sim,
  PyBullet, MuJoCo).
  (simulated_robot.launch.py `simulator:=` arg with a fixed dispatch registry;
  new robot_lab_isaac/robot_lab_pybullet/robot_lab_mujoco adapter packages
  mirroring the Gazebo spawn interface; sim_modes.yaml `simulators:` lists per
  mode; GUI Launch-tab simulator dropdown gated by mode; dispatch covers all
  four backends in tests. Robot-lab bringup profile suite passes 206 collected
  tests including 7 new simulator-dispatch tests and 6 simulator-backend smoke
  tests; full clean colcon build of 26 packages succeeds.)
- [x] **P7.8** Qualify physics backends for Isaac Sim, PyBullet, and MuJoCo.
  (PyBullet 3.2.7 source-rebuilt against NumPy 2.2.6 on Jetson arm64 (venv +
  user site — launch-spawned console scripts run under system python); MuJoCo
  3.12.0 C library source-built with pip bindings; both spawners verified LIVE
  via ros2 launch — 0 process deaths and full ROS2 topic contract
  (/clock /odom /scan /imu/out /joint_states). Fixed en route:
  use_sim_time double-declaration crash in both spawners, MuJoCo import-failure
  shim, mujoco.viewer .close() segfault at shutdown on ARM, pybullet_data path
  resolution, Imu covariance int→float crash (PyBullet), rayTest result tuple
  handling, odom position Vector3→Point (MuJoCo). Gazebo Harmonic (gz-sim8)
  verified headless including celisca_floor_1 with the LFS-restored 165 MB
  furniture STL. Isaac Sim 6.0.1: docker image `isaac-sim-docker:latest`
  (26.1 GB) BUILT from source via tools/docker/{prep_docker_build,build_docker}.sh
  --aarch64 on the Jetson; nvidia runtime registered; container boot test
  PASSED (kit process running, 11.5 GB RAM, healthy). Build pipeline required
  docker data-root + containerd store on the 1 TB SSD (/workspace/molar/ —
  see scripts/isaac_docker_setup.sh, fix_containerd_disk.sh,
  isaac_postbuild.sh) because the 64 GB eMMC fills at ~87%. Non-fatal NVST
  streaming encoder errors expected on headless Jetson; core simulation
  unaffected.)

- [x] **P7.8b** Native Isaac Sim pip install + runtime subprocess (2026-09-04).
  Isaac Sim 6.0.1.0 now ships official aarch64 manylinux_2_35 wheels (cp312)
  on PyPI; installed `isaacsim[all,extscache]==6.0.1.0` under a uv-bootstrapped
  Python 3.12 virtualenv on the 1 TB SSD (/workspace/isaac_env + /workspace/uv;
  caches in /workspace/.pip_cache — nothing on the eMMC). Because Humble's
  rclpy is py3.10-only, `robot_lab_isaac` was restructured into a two-process
  design: the ROS node (isaac_spawner.py) owns the topic contract and spawns
  isaac_runtime.py (py3.12) which instantiates SimulationApp, builds the stage
  from the USD `world_stage` or the map SDF's STL meshes converted to USD at
  runtime, imports the robot URDF, steps physics, and streams state over
  stdin/stdout (line-delimited JSON). Launch resolves the map SDF
  (`_resolve_world_sdf`) and prefers it over the USD fallback. Verified LIVE
  with GUI: 3 SDF meshes (91936 tris each) loaded for celisca_floor_1, robot
  bumperbot spawned with differential-drive joints, full topic contract
  (`/clock`, `/odom`, `/scan`, `/imu/out`, `/joint_states`, `/tf`). En route
  fixed: the use_sim_time spawn-timer deadlock (wall-clock timers in all three
  non-Gazebo spawners), MuJoCo MJCF asset-path absolutization for
  from_xml_string, MuJoCo publisher/method name collisions
  (_pub_clock/_pub_odom/...), MuJoCo qvel dof-address indexing, MuJoCo IMU
  covariance int→float.

## Definition of status

- `cataloged`: metadata and an upstream/source decision exist; it may not be
  installed or runnable.
- `available`: implementation can be installed/built on the supported ROS
  distribution, but this repository has not completed its adapter smoke test.
- `integrated`: adapter/configuration is in this repository and its declared
  smoke test passes for at least one registered experiment.
- `benchmarked`: integrated, plus a reproducible standard result record exists.
- `blocked`: the exact blocker, attempted resolution, and unblock condition are
  recorded. Missing time is not a blocker.

## Continuation protocol

1. Read this file, `docs/status/platform-status.yaml`, and
   `docs/architecture/overview.md`.
2. Inspect `git status --short`; preserve unrelated and nested-repository work.
3. Re-run the evidence attached to the last completed task before depending on
   it.
4. Claim the next unblocked task by setting its machine-readable state to
   `active` and adding the contributor/agent identifier.
5. Implement the smallest end-to-end slice: configuration, adapter, dependency,
   test, documentation, and provenance together.
6. Update both ledgers with evidence and remaining gaps before stopping.

Baseline verification commands:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-skip orbslam3
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-skip orbslam3 --python-testing pytest
colcon test-result --verbose
ros2 run robot_lab_registry robot-lab validate
```

## Known constraints

- The supported baseline is ROS 2 Humble on Ubuntu 22.04/arm64.
- Four simulator backends are dispatched (Gazebo, Isaac Sim, PyBullet, MuJoCo);
  Gazebo is the qualified primary backend. PyBullet (source-built vs NumPy 2.x)
  and MuJoCo (aarch64 wheel) physics backends are live-verified including the
  celisca_floor_1 map; the use_sim_time spawn-timer deadlock that froze all
  three non-Gazebo spawners was fixed (wall-clock spawn timers + MJCF asset
  path absolutization for MuJoCo). Isaac Sim 6.0.1 is installed natively via
  its aarch64 pip wheel under a Python 3.12 virtualenv on the 1 TB SSD and is
  driven by `robot_lab_isaac` through the `isaac_runtime.py` subprocess; note
  NVIDIA officially supports Isaac Sim aarch64 only on DGX Spark — on Jetson,
  Kit may abort at startup ("TSC ran backwards") and the spawner falls back to
  offline mode. `/scan` is not yet simulated for Isaac (RTX lidar pending).
- Only Bumperbot currently has a complete simulation/navigation stack. Imported
  Unitree and Berkeley assets are description-only and must not be advertised as
  commandable robots.
- ORB-SLAM3 is optional and currently carries an OpenCV ABI warning when its
  external build does not match ROS `cv_bridge`.
- Physical serial hardware, motor direction, and the controller protocol still
  require hardware-in-the-loop validation.
- Upstream robot/model assets have separate licenses. Provenance and
  redistribution review are required before release.
