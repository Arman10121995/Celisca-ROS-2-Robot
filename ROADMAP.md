# Robot Lab: implementation roadmap and continuation plan

Updated: 2026-09-07. Runtime audit baseline: `dff388f`.

This is an implementation specification, not a list of promised features.
[Machine-readable status](docs/status/platform-status.yaml) owns task state,
ownership, dependencies and the next task. This file owns scope and acceptance
criteria. Update both together. Start each session with
[AGENT_HANDOFF](docs/AGENT_HANDOFF.md).

## Goal and completion boundary

Build a reproducible ROS 2 platform for learning and comparing algorithms across
mobile, legged, humanoid and aerial robots, in 2D and 3D environments. Preserve
existing assets and the Bumperbot workflow. Robot, simulator, environment,
scenario, perception, localization, state estimation, sensor fusion, global
planning, local planning and control must be selectable independently **where
their physical and data contracts are compatible**.

The goal is complete only when all of the following are evidenced:

- At least two mobile bases and one quadruped, humanoid and multirotor complete
  their own class-appropriate simulated tasks. Manipulator support is an extension;
  retain existing manipulator assets without claiming commandable support.
- Existing worlds remain available. Navigation, terrain, stairs, stepping stones,
  aerial, moving-obstacle and degraded-sensor tasks have verified geometry,
  runtime resets and recorded seeds. A 2D projection is not a 3D map.
- Each of seven algorithm categories has at least five distinct, runnable,
  mathematically defensible implementations and reproducible comparisons.
  Include PID, linear/model-based control, MPC and nonlinear control.
- One resolved experiment manifest drives CLI and GUI execution, recording the
  actual plugins, parameters, assets, simulator, scenario and seeds.
- Results measure success, contact events, simulated time, path/trajectory
  quality, clearance, estimation error, compute use, real-time factor and a
  defined effort/energy proxy where applicable. Missing metrics remain unavailable.
- Truth is separate from measurements; failed trials are retained; a clean
  checkout can reproduce the comparison.
- Every advertised backend/robot/mode combination has its own qualification
  evidence. Four adapters do not imply universal cross-product support.

There is no universal "best" algorithm. Compare within declared input/model
strata and publish accuracy, robustness, cost and failure tradeoffs. Republishers,
command relays and parameter variants do not count as distinct algorithms.
Physical HIL is a separate hardware-dependent milestone, not a prerequisite for
a simulation-only release. Reduced-scope releases must explicitly list exclusions.

## Achieved foundation and historical reconciliation

The [audit](docs/status/audit-2026-09-07.md) records 26 ROS packages, 20 robot
entries, 26 environments, 43 algorithms, 18 scenarios and 15 experiments.
Selected tests passed 490 cases and failed one; this is not mission qualification.
Useful work includes descriptions/worlds, arena generators, the Bumperbot-oriented
mode launcher, Nav2/AMCL/EKF/SLAM configurations, catalogs/query tools, GUI,
numerical examples, backend code and result/reporting helpers.

Catalog maturity labels have **not** been changed in this documentation revision.
They overstate runtime support; R3.1 reconciles metadata and its tests together.

| Legacy phase | Audited state | Retained work / remaining obligation | Recovery |
|---|---|---|---|
| P0 baseline | Partial; reverify | Hardware parsing/profile tests exist; installs, paths, topics and CI need repair | R1, R2, R9 |
| P1 foundation | Partial | Catalog/schema/query code exists; compatibility checks insufficient | R3.1, R3.2 |
| P2 composition | Partial | Selector/fragment classes exist; actual launch and applied choices incomplete | R3.3–R3.5 |
| P3 robots | Partial | Richer assets exist; Go2/BHL display profiles and unproven flight/locomotion | R5 |
| P3.4 humanoid | Partial, no current owner asserted | Standing-related code is not balance/walking qualification; check live ownership | R5.3 |
| P4 environments | Assets/static checks implemented | Geometry/generators retained; runtime reset, actors and 3D traversal unqualified | R6 |
| P5 algorithms | Partial | Counts/kernels exist; empty/broken ROS entry points and simplified methods | R7 |
| P6 benchmarking | Partial | Reporting helpers exist; placeholder metrics and false-success paths | R4 |
| P7 hardening | Partial | Scripts/CI/backend code exist; blanket qualification claims withdrawn | R1, R8, R9 |
| P7.5 hardware | Blocked | Equipment, safe setup and explicit operation authorization required | R9.4 |
| P7.7/P7.8/P7.8b | Dispatch/runtime code exists | Historical startup reports are not current mission evidence | R2, R8 |

Old IDs remain valid for locating commits/tests. Historical narrative is available
with `git show dff388f:ROADMAP.md`; do not copy its green counts or host-specific
installation reports into current support claims.

## Order, ownership and state rules

The default sequence is R0 → R1 → R2 → R3 → R4 → R5/R6 → R7 → R8 → R9.
Dependencies below allow safe parallel work: R1.1/R1.2, independent robot-class
lanes after R5.1, environment work, and R7 categories after R7.1. Prefer the
smallest complete experiment over adding more catalogs or GUI controls.

- Task states: `queued`, `active`, `partial`, `blocked`, `done`.
- `partial` means useful work exists but acceptance has not passed.
- A blocked task needs the exact prerequisite, attempted checks and unblock action.
- Claim ownership in YAML before changing shared files; one coordinator owns ledgers.
- Source maturity is separate: cataloged → available → integrated → benchmarked.
  Integration requires installed code plus a real input-to-output runtime test on
  a named robot/backend/task; benchmarking additionally requires measured artifacts.
- Read dependency acceptance before relying on a `done` label. If evidence fails,
  reopen the task and record it; do not silently weaken its tests.
- No runtime implementation has been completed by the R0 documentation revision.

## Ordered task specifications

Each task includes scope, files, dependencies and a completion test. The YAML
ledger is authoritative for current state/owner. Paths containing `*` are scope
hints, not instructions to mechanically rewrite every package.

## R0 — Documentation recovery

### R0.1 — Reconcile audited baseline

Dependencies: none.

- Files: `docs/status/audit-2026-09-07.md`.
- Implement: Preserve revision, commands, exclusions, results and negative probes. Separate inventory, historical reports, static tests and live evidence.
- Acceptance: Counts match the catalogs; findings link to source; no fabricated logs, clean-build or mission qualification claim.

### R0.2 — Publish truthful documentation and agent work plan

Dependencies: `R0.1`.

- Files: `README.md`, `ROADMAP.md`, `docs/`, `src/robot_lab/robot_lab_registry/test/test_p6_benchmarking.py`.
- Implement: Rewrite current state and target architecture; add durable task ownership/dependencies and continuation protocol. Replace documentation assertions demanding stale green numbers with scoped evidence checks.
- Acceptance: Links and YAML parse; task dependencies resolve without cycles; docs tests pass; no runtime repair is marked complete.

## R1 — Trustworthy development baseline

### R1.1 — Repair installed entry points and portable paths

Dependencies: `R0.2`.

- Files: `src/*/setup.py`, `src/*/setup.cfg`, `src/*/CMakeLists.txt`, `src/robot_lab_navigation/config/bt_navigator.yaml`, `src/robot_lab_controller/robot_lab_controller/`.
- Implement: Fix benchmark executable ROS placement and audit every advertised executable/plugin/launch against the installed index. Resolve assets via package shares; use configurable output directories outside source/install; preserve existing assets.
- Acceptance: Clean isolated build of 25 non-ORB packages; installed --help and launch-generation checks pass without source import hacks or developer home paths; optional dependencies reported separately.

### R1.2 — Separate and repair test tiers

Dependencies: `R0.2`.

- Files: `src/robot_lab/robot_lab_registry/test/`, `src/robot_lab_adapter/test/`, `src/robot_lab_bringup/test/`, `scripts/test_fast.sh`.
- Implement: Separate numerical, ROS-node, launch-contract, physics, mission and hardware tests. Fix the DeadReckoning initialization failure. Mock subprocesses in unit orchestration tests; replace count-only qualification progressively.
- Acceptance: Unit suite runs without accessing a shared ROS graph; optional engines produce explicit skips; numerical assertions remain strong; ROS nodes initialize and tear down correctly.

### R1.3 — Make CI bootstrap and doctor trustworthy

Dependencies: `R1.1`, `R1.2`.

- Files: `.github/workflows/`, `scripts/bootstrap.sh`, `scripts/doctor.sh`, `scripts/test_fast.sh`.
- Implement: Include master/current development branches; remove ignored required-test failures and invalid test paths. Make dependency/build failures nonzero. Separate project bootstrap from privileged host changes; pin required dependency inputs.
- Acceptance: Intentional build/test/dependency failure makes the required job fail; clean required lane passes; optional lanes report reasons. Do not call a job a simulation matrix until it executes missions.

## R2 — Simulator and ROS contracts

### R2.1 — Fix simulation clocks

Dependencies: `R1.2`.

- Files: `src/robot_lab_pybullet/python/robot_lab_pybullet/pybullet_spawner.py`, `src/robot_lab_mujoco/python/robot_lab_mujoco/mujoco_spawner.py`, `src/robot_lab_isaac/python/robot_lab_isaac/isaac_spawner.py`, `src/robot_lab_bringup/test/`.
- Implement: Publish rosgraph_msgs/msg/Clock wrapping Time in all three adapters. Keep startup/readiness on wall time and measurements on simulation time; declare dependencies.
- Acceptance: A real use_sim_time subscriber advances, pauses and resets; types match; startup works at time zero; reset does not create negative-duration measurements. Engine import tests are insufficient.

### R2.2 — Unify commands odometry sensors and TF

Dependencies: `R2.1`, `R1.1`.

- Files: `src/robot_lab_controller/`, `src/robot_lab_localization/`, `src/robot_lab_bringup/`, `src/robot_lab_pybullet/`, `src/robot_lab_mujoco/`, `src/robot_lab_isaac/`.
- Implement: Reconcile /odom versus /robot_lab_controller/odom, mux output versus backend /cmd_vel, sensor topics/QoS and TF ownership. Separate perfect ground truth from measurements and estimates. Add command watchdogs and bounded arbitration.
- Acceptance: Commands reach only selected simulated robot with correct signs; stale commands stop; EKF receives data; one publisher owns each TF edge; covariance/timestamps and sensor frames are correct.

### R2.3 — Gate readiness capabilities and failures

Dependencies: `R2.2`.

- Files: `src/robot_lab_bringup/config/sim_modes.yaml`, `src/robot_lab_bringup/launch/`, `src/robot_lab_gui/`, `src/robot_lab_pybullet/`, `src/robot_lab_mujoco/`, `src/robot_lab_isaac/`.
- Implement: Expose ready/health/reset contracts. Gate modes by actual backend sensor/actuator support. Offline fallback is a diagnostic mode, never successful physics. Replace blind sleeps with bounded readiness checks.
- Acceptance: Missing engine/camera/LiDAR, reset failure and process death yield structured errors and scoped cleanup. Unsupported modes fail before launch; headless is honored even for non-Gazebo display.

## R3 — Strict composition and execution

### R3.1 — Reconcile registry profiles and aliases

Dependencies: `R1.1`.

- Files: `src/robot_lab/robot_lab_registry/config/`, `src/robot_lab/robot_lab_registry/robot_lab_registry/schemas.py`, `src/robot_lab_robots/config/robots.yaml`, `src/robot_lab_bringup/config/`, `src/robot_lab_adapter/robot_lab_adapter/launch_fragments.py`.
- Implement: Establish canonical IDs and migration aliases (go2/unitree_go2, outdoor_terrain/display name). Audit all 43 algorithm implementations including PID/coverage/aerial references and reversed contracts. Downgrade unsupported maturity without deleting assets.
- Acceptance: One inventory agrees across GUI/CLI/docs; available package/plugin references resolve; integrated labels name scoped runtime evidence. Replace tests that require false labels with evidence checks.

### R3.2 — Enforce typed composition compatibility

Dependencies: `R3.1`, `R2.3`.

- Files: `src/robot_lab/robot_lab_registry/robot_lab_registry/validation.py`, `src/robot_lab/robot_lab_registry/robot_lab_registry/schemas.py`, `src/robot_lab_adapter/`.
- Implement: Validate explicit simulator, algorithm category, sensors, dimensions, kinematics, command interfaces, dependencies, topics and TF ownership. Restore capability checks and use one validator in CLI and GUI.
- Acceptance: Unknown simulator, AMCL in global_planning and missing required LiDAR are rejected; valid combinations pass with actionable diagnostics; 2D planners cannot gain flight capability by a metadata label.

### R3.3 — Execute resolved experiments through CLI

Dependencies: `R3.2`.

- Files: `src/robot_lab/robot_lab_registry/robot_lab_registry/cli.py`, `src/robot_lab_adapter/`, `src/robot_lab_bringup/`.
- Implement: Fix LaunchConfiguration concatenation. Replace describe-only robot selection and dry-run-only launch with one resolver/executor. Support seven independent selectors, robot/backend/world/scenario, spawn/reset, parameters and seed; preserve safe dry-run and legacy aliases.
- Acceptance: Dry-run emits concrete resolved manifest without processes; live execution uses it; switching between two implemented planners changes the active plugin and behavior; no duplicate hardcoded stack starts.

### R3.4 — Connect GUI to the common resolver

Dependencies: `R3.3`.

- Files: `src/robot_lab_gui/`.
- Implement: Replace ignored algorithm argument with full composition controls; filter by validator, show maturity/readiness and unsupported reasons. Save/load resolved manifests and migrate old profiles; monitor actual running choices.
- Acceptance: CLI and GUI resolve identical saved profiles; selection changes active component; stop/close only clean owned processes and never delete user artifacts; headless GUI-adjacent logic tests pass.

### R3.5 — Isolate namespaces frames and simulator instances

Dependencies: `R3.3`, `R2.2`.

- Files: `src/robot_lab_adapter/robot_lab_adapter/namespaces.py`, `src/robot_lab_bringup/`, `src/robot_lab_gui/`, `src/robot_lab_*/`.
- Implement: Separate robot from experiment namespaces; remove accidental absolute-topic coupling. One clock belongs to each simulator instance; independent simulators require isolated ROS discovery/partitions.
- Acceptance: Two simulated robots have independent commands/sensors/TF; independent experiments cannot share reset services/clocks/result directories; stopping/resetting one cannot affect the other.

## R4 — Measured reference experiment

### R4.1 — Implement scenario lifecycle and truthful outcomes

Dependencies: `R3.3`, `R2.3`.

- Files: `src/robot_lab/robot_lab_benchmark/`, `src/robot_lab/robot_lab_registry/config/scenarios.yaml`.
- Implement: Implement validate→launch→ready→reset/seed→initialize pose→send task→observe→stop→record. Task completion determines success, not sleep duration; launch/reset failures abort. Use unique result IDs and real rosbag2 recording.
- Acceptance: Success, collision, timeout, no path, lost state, process death and cancellation have distinct terminal records; unit tests mock processes; integration tests prove cleanup and prevent artifact overwrite.

### R4.2 — Measure metrics and preserve provenance

Dependencies: `R4.1`.

- Files: `src/robot_lab/robot_lab_benchmark/robot_lab_benchmark/`.
- Implement: Remove hardcoded distance/collision/clearance. Measure contacts, footprint-aware clearance, trajectory distance and timestamp/frame-aligned truth/estimation error; collect CPU/memory/RTF and defined effort proxy. Record manifest/hash, revision/dirty state, dependencies, asset hashes, seeds, budgets, tolerances and artifact paths.
- Acceptance: Known trajectories/contact fixtures yield correct values; one contact is not counted per scan; missing/NaN/stale data invalidate metrics instead of becoming zero; schema rejects invalid values and distinguishes measured versus derived metrics.

### R4.3 — Publish first reproducible planner comparison

Dependencies: `R4.2`.

- Files: `src/robot_lab/robot_lab_benchmark/`, `src/robot_lab/robot_lab_registry/config/experiments.yaml`, `docs/`.
- Implement: Use Bumperbot + deterministic arena + Gazebo. Compare two wired planners with all other settings fixed. Predeclare at least five seeds, timeout, tolerance and resource budget; keep tuning and evaluation seeds separate.
- Acceptance: Manifest reruns reproduce scenario; raw traces support metrics; all failures retained; report distributions/failure rate rather than only best run; thresholds are measured and justified; bag artifacts contain real data.

## R5 — Robot-class qualification

### R5.1 — Qualify Bumperbot and Labbot mobile tasks

Dependencies: `R4.3`.

- Files: `src/robot_lab_robots/bumperbot/`, `src/robot_lab_robots/labbot/`, `src/robot_lab_controller/`, `src/robot_lab_localization/`, `src/robot_lab_navigation/`.
- Implement: Verify robot-specific inertias, wheel geometry, footprint, sensors/control and overlays using golden harness. Preserve Bumperbot mapping/RGB-D/coverage; do not claim Labbot RGB-D without implementation.
- Acceptance: Both bases spawn at known pose, drive/turn/stop and complete clear/obstacle navigation with limits/watchdogs. Truth and odometry are separate; robot-specific results recorded on one named backend.

### R5.2 — Qualify Go2 locomotion

Dependencies: `R5.1`.

- Files: `src/robot_lab_robots/unitree/go2_description/`, `src/robot_lab_adapter/`, `src/robot_lab_robots/config/robots.yaml`.
- Implement: Wire simulation wrapper, sensors and controller into actual launch. Implement closed-loop stance then bounded gait/base-velocity interface with contact/state estimation and effort/joint limits; raw effort publishing is not gait control.
- Acceptance: Measured stable stance, commanded displacement, turn and stop on flat ground; tilt/effort/fall handling works; then complete a named terrain task with tracking/contact/effort evidence.

### R5.3 — Qualify Berkeley Humanoid Lite balance and walking

Dependencies: `R5.1`.

- Files: `src/robot_lab_robots/berkeley_humanoid_lite/`, `src/robot_lab_adapter/robot_lab_adapter/humanoid_standing_controller.py`, `src/robot_lab_robots/config/robots.yaml`.
- Implement: Resume historical P3.4 after ownership check. Verify joints/inertias/limits; wire simulation wrapper; implement closed-loop balance before stepping/walking. A fixed pose is not proof of balance.
- Acceptance: Declared stance-duration and bounded perturbation recovery pass; flat-ground stepping/walking and stop measured; falls/limits enforced. Stair qualification waits for flat-ground success.

### R5.4 — Integrate real multirotor SITL flight

Dependencies: `R5.1`.

- Files: `src/robot_lab_robots/quadrotor_sitl/`, `src/robot_lab_adapter/robot_lab_adapter/mavros_offboard_controller.py`, `src/robot_lab_bringup/`.
- Implement: Select/pin one FCU-SITL integration with license/dependency decision. Add rotor/thrust dynamics, actuator allocation, IMU/pose, ENU/NED conversion, arming/offboard/readiness and failsafe. Display URDF fixed rotors are not propulsion.
- Acceptance: Simulated takeoff, hover, 3D waypoints, landing and command-loss failsafe pass; actual altitude/FCU state measured; no 2D follower or differential-drive controller substitutes for flight; no real FCU connection.

## R6 — Environment qualification

### R6.1 — Qualify geometry map alignment and resets

Dependencies: `R2.3`, `R3.1`.

- Files: `src/robot_lab_maps/tools/`, `src/robot_lab_maps/config/arena_navigation.yaml`, `src/robot_lab/robot_lab_registry/config/environments.yaml`, `src/robot_lab_bringup/`.
- Implement: Preserve worlds/provenance; validate units, mesh/collision geometry, map origin/resolution, spawn/goal clearance and generation seeds. Bind reset to actual backend API rather than universal /gazebo/reset_world metadata.
- Acceptance: Runtime coordinates match maps; resets restore poses/velocities/actors and sensor/estimator histories; converted worlds preserve required geometry or reject unsupported content; artifacts/versioned seeds recorded.

### R6.2 — Add real dynamic and sensor-disturbance cases

Dependencies: `R6.1`, `R4.2`.

- Files: `src/robot_lab_maps/`, `src/robot_lab/robot_lab_registry/config/scenarios.yaml`, `src/robot_lab_algorithms/`, `src/robot_lab/robot_lab_benchmark/`.
- Implement: Verify actor collision/sensor effects; add deterministic noise, bias, drift, delay, dropout, occlusion and outlier injection with separate seeds. Geometry occlusion is not generic sensor degradation.
- Acceptance: Fault traces repeat; truth is not contaminated; baseline/degraded cases share task and budgets; recovery time/failure criteria measured; moving obstacles are observed and interact as specified.

### R6.3 — Qualify 3D terrain and aerial representations

Dependencies: `R6.1`, `R5.2`, `R5.3`, `R5.4`.

- Files: `src/robot_lab_maps/`, `src/robot_lab/robot_lab_registry/config/`, `src/robot_lab/robot_lab_benchmark/`.
- Implement: Provide height/elevation/voxel/mesh queries where needed, legged traversable surfaces and aerial free volumes/geofences. Do not infer 3D feasibility solely from a 2D occupancy projection.
- Acceptance: Class-appropriate terrain/flight missions pass; overhang, foothold and altitude collisions detected in validation; maps/trajectories and results carry dimensionality and frame metadata.

## R7 — Algorithm breadth

### R7.1 — Normalize numerical and ROS algorithm adapters

Dependencies: `R4.3`, `R3.2`.

- Files: `src/robot_lab_algorithms/`, `src/robot_lab_adapter/`, `src/robot_lab/robot_lab_registry/config/algorithms.yaml`.
- Implement: Separate numerical classes from ROS wrappers; enforce input/output types, rates, timestamps, frames, covariance, lifecycle, seeds/reset, parameter bounds and failure codes. Wire useful kernels or mark educational; validate mathematical names against implementations.
- Acceptance: One adapter per category does useful input→output work with installed entrypoints, analytic/oracle tests and ROS tests; missing data cannot report success; replay deterministic; common safety layers disclosed separately.

### R7.2 — Five perception pipelines

Dependencies: `R7.1`.

- Files: `src/robot_lab_algorithms/`, `src/robot_lab/robot_lab_registry/config/`, `docs/tutorials/perception.md`.
- Implement: Implement the five candidates or documented equivalents in the breadth specification below; give each a method subrecord, equations/source, ROS adapter, labels/input stratum and benchmark experiment.
- Acceptance: Five substantive pipelines integrated and benchmarked within fair input strata; conversion utilities do not count; precision/recall, IoU where applicable, occlusion robustness and latency measured.

### R7.3 — Five localization methods

Dependencies: `R7.1`.

- Files: `src/robot_lab_localization/`, `src/robot_lab_mapping/`, `src/robot_lab_algorithms/`, `docs/tutorials/localization.md`.
- Implement: Implement the five candidates or justified replacements; group sensor/map assumptions and initialization/relocalization protocol; retain ORB-SLAM3 optional unless ABI/dependency tests pass.
- Acceptance: Five actual pose-output methods with ATE/RPE, convergence and failure measurements; no localization estimate comes from undisclosed perfect simulator truth.

### R7.4 — Five state-estimation methods

Dependencies: `R7.1`.

- Files: `src/robot_lab_algorithms/`, `src/robot_lab_localization/`, `docs/tutorials/state_estimation.md`.
- Implement: Implement five mathematically distinct estimators on declared state/dynamics/measurement models. Preserve toy examples with honest names or replace with full equations and numerical tests.
- Acceptance: Five integrated estimators with analytic/nonlinear/noisy fixtures, RMSE/divergence/CPU and NEES/NIS where defined; duplicate EKF labels and pose averaging called graph optimization do not count.

### R7.5 — Five sensor-fusion methods

Dependencies: `R7.1`.

- Files: `src/robot_lab_algorithms/`, `src/robot_lab_localization/`, `docs/tutorials/sensor_fusion.md`.
- Implement: Implement five fusion methods in matched attitude-only and pose-fusion strata; handle measurement timing, covariance, frame conversion and biased/dropped sensors.
- Acceptance: Five integrated and benchmarked methods; compare like input sets; attitude/pose error, consistency, delay/dropout robustness and cost measured; republishers do not count.

### R7.6 — Five global planners

Dependencies: `R7.1`.

- Files: `src/robot_lab_planning/`, `src/robot_lab_algorithms/`, `src/robot_lab_navigation/`, `docs/tutorials/planning.md`.
- Implement: Implement distinct graph/grid and sampling-based planners using common collision checking, footprint, goal and compute budget. Verify edges, not only sampled endpoints.
- Acceptance: Five integrated/benchmarked planners; valid paths, no-path handling, path cost, solve time and success distributions tested. Same algorithm via two packages does not inflate distinct count.

### R7.7 — Five local planners

Dependencies: `R7.1`.

- Files: `src/robot_lab_motion/`, `src/robot_lab_algorithms/`, `src/robot_lab_navigation/`, `docs/tutorials/`.
- Implement: Implement five path-following/reactive/trajectory methods with common constraints and explicit distinctions. Validate upstream ROS version availability before selecting TEB/MPPI or a replacement.
- Acceptance: Five integrated/benchmarked methods receive same global paths and sensors; collision, tracking, progress, smoothness, latency and obstacle-recovery metrics recorded; safety-layer effects disclosed.

### R7.8 — Five low-level or model-based controllers

Dependencies: `R7.1`.

- Files: `src/robot_lab_controller/`, `src/robot_lab_algorithms/`, `src/robot_lab_adapter/`, `docs/tutorials/`.
- Implement: Implement PID, LQR, constrained linear MPC, nonlinear MPC and feedback-linearization/backstepping control (or justified nonlinear replacement) on a common supported plant. Record equations, actuator model, limits and solver deadlines.
- Acceptance: Five integrated/benchmarked controllers with tracking, disturbance, stability-envelope, saturation recovery, effort and deadline tests; PID anti-windup and watchdogs work; robot-class plant differences explicit.

## R8 — Backend qualification and scaling

### R8.1 — Qualify PyBullet and MuJoCo combinations

Dependencies: `R5.1`, `R6.1`.

- Files: `src/robot_lab_pybullet/`, `src/robot_lab_mujoco/`, `src/robot_lab_bringup/test/`.
- Implement: Verify import fidelity, frames, stepping, contacts, limits, sensors/noise and reset. Do not silently substitute a generic box model. Start with mobile; add class/backend rows only after that class mission passes.
- Acceptance: R2 contracts and R4 mobile experiment pass on each backend with artifacts; differences measured, not assumed identical physics; missing sensor modes gated; non-mobile support separately evidenced.

### R8.2 — Qualify Isaac on a named host configuration

Dependencies: `R5.1`, `R6.1`.

- Files: `src/robot_lab_isaac/`, `scripts/`, `docs/status/support-matrix.md`.
- Implement: Validate Python/ROS subprocess boundary, runtime discovery, import, control, sensors and failures on a chosen compatible host. Implement needed LiDAR/RGB-D or gate modes; parameterize host paths. Historical Jetson startup reports are not qualification.
- Acceptance: Live startup/reset/command/sensor/mission evidence recorded; child failure aborts cleanly; offline is never success. If engine cannot run, record exact blocker and continue other lanes.

### R8.3 — Resource-bounded concurrent experiments

Dependencies: `R3.5`, `R4.3`, `R8.1`.

- Files: `src/robot_lab/robot_lab_benchmark/`, `src/robot_lab_gui/`, `src/robot_lab_adapter/`.
- Implement: Add CPU/memory/GPU/RTF budgets, bounded queues, cancellation and independent artifacts. Distinguish multiple robots in one world from independent simulations.
- Acceptance: Two runs maintain isolated clocks/seeds/results; overload reported; stop/reset scoped; scheduling does not silently change compared algorithms' budgets.

## R9 — Reproducibility, learning and release

### R9.1 — Provenance licenses and clean-host reproduction

Dependencies: `R4.3`, `R1.3`.

- Files: `LICENSES/third-party-notices.md`, `scripts/`, `docs/`, `src/robot_lab/robot_lab_registry/config/`.
- Implement: Pin external revisions/dependencies; verify actual upstream licenses and replace placeholder source URLs. Document storage/download/runtime requirements; top-level MIT does not relicense third-party assets.
- Acceptance: Fresh documented host/container builds and reruns golden comparison without developer directories; provenance and redistribution decisions recorded; optional dependencies explicit.

### R9.2 — Seven real comparison tutorials

Dependencies: `R7.2`, `R7.3`, `R7.4`, `R7.5`, `R7.6`, `R7.7`, `R7.8`, `R3.4`.

- Files: `docs/tutorials/`, `README.md`.
- Implement: Upgrade current numerical demos to saved experiments; add separate global/local planning and control guides plus robot/backend examples, failure interpretation and parameter-study protocol.
- Acceptance: Every command exercised; each category links real results for five methods; tables/plots generated from artifacts; GUI and CLI tutorials share manifests and explain applicability.

### R9.3 — Evidence-generated support matrix and release gate

Dependencies: `R5.1`, `R5.2`, `R5.3`, `R5.4`, `R6.2`, `R6.3`, `R8.1`, `R8.2`, `R8.3`, `R9.1`, `R9.2`.

- Files: `docs/status/`, `README.md`, `ROADMAP.md`, `.github/workflows/`.
- Implement: Generate inventories/support rows from metadata plus runtime evidence, with revision/date/host/backend/robot/scenario/seeds. Reconcile all scope promises before release; do not silently waive an unfinished backend requirement.
- Acceptance: Required robot/world/seven-category tasks pass; every release claim maps to artifacts; failures/skips and limitations published. Any reduced-scope release needs an explicit recorded decision; never call remaining work complete.

### R9.4 — Optional physical Bumperbot HIL

Dependencies: `R5.1`, `R9.1`.

- Files: `src/robot_lab_firmware/`, `src/robot_lab_utils/`, `docs/`.
- Implement: Wait for physical equipment, safe test setup and explicit authorization. Review serial timeouts, units, direction, limits, disconnect and e-stop before powered motion.
- Acceptance: Supervised HIL checklist/logs and recovery procedure; hardware claims kept separate. No real motor/FCU action is authorized by simulation work; simulation release does not depend on this task.

## R7 breadth specification: candidates, not current support

Select and pin actual implementations during each task after checking licenses,
maintenance, ROS-release compatibility and mathematical validity. Preserve useful
existing methods. A replacement needs a recorded rationale; do not silently
upgrade ROS or claim a candidate is installable because upstream documentation
mentions it. "Five" is an implementation/evidence target, not a count of labels.

| Task | Five target methods/pipelines | Fair comparison boundary |
|---|---|---|
| R7.2 Perception | Scan obstacle clustering; Euclidean point-cloud clustering; DBSCAN clustering; RANSAC ground removal plus obstacle segmentation; voxel occupancy obstacle pipeline | Use labeled input strata and consistent obstacle definitions. Compare compatible cloud pipelines on the same observations; conversions alone do not count. Add RGB-D vision as a separate stratum. |
| R7.3 Localization | Wheel dead reckoning; AMCL; ICP scan matching; NDT registration; RGB-D SLAM/localization using the RTAB-Map path | Separate odometry, LiDAR-map and visual-input strata. Report ATE/RPE, initialization and relocalization success. |
| R7.4 State Estimation | Linear KF; EKF; UKF; particle filter; error-state/invariant EKF | Define state/dynamics/measurement models. Use nonlinear and noisy fixtures; report consistency where mathematically applicable. |
| R7.5 Sensor Fusion | Complementary attitude filter; Mahony; Madgwick; wheel/IMU EKF; wheel/IMU/GNSS UKF | Attitude methods share IMU inputs; pose-fusion methods share odometry/IMU/GNSS inputs. Do not rank unlike state outputs as equivalent. |
| R7.6 Global Planning | Dijkstra; A*; PRM; RRT; RRT* | Common collision checker, footprint and compute budget. NavFn/Smac may supply implementations/baselines; avoid duplicate family counts. |
| R7.7 Local Planning | DWB/DWA; Regulated Pure Pursuit; TEB or documented compatible time-parameterized replacement; MPPI; Follow-the-Gap with explicit goal/path policy | Same global path, sensors and limits; disclose reactive versus trajectory-optimizing behavior and common safety layer. |
| R7.8 Control | PID with anti-windup; LQR; constrained linear MPC; nonlinear MPC; feedback-linearization or backstepping control | Common plant, reference, state inputs, disturbances and actuator limits. Define robot-class applicability; wheel and flight actuators are not interchangeable. |

Each R7.2–R7.8 task remains partial until all five methods have numerical tests,
installed adapters, compatible experiments and measured comparison records.
Create a subrecord before implementing each method:

```yaml
methods:
  canonical_algorithm_id:
    state: queued
    owner: null
    implementation_ref: null
    applicable_robots: []
    input_stratum: null
    numerical_tests: []
    integration_experiment: null
    benchmark_artifacts: []
    remaining: []
```

Upstream discovery: [Nav2 plugin catalog](https://docs.nav2.org/rolling/configuration_and_development/navigation_plugins/)
and [robot_localization filter documentation](https://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html).
These describe families, not the installed Humble API. Verify the selected release
and local interfaces before coding; research the primary paper/documentation for
each new method. Prefer maintained upstream implementations where suitable and
label educational approximations honestly.

## Common implementation and evidence contract

Every task requires scoped code/config/assets, dependencies and provenance,
positive/negative tests, runtime evidence where claimed, updated documentation,
honest registry maturity, and an exact handoff. Tests asserting file existence,
imports or counts remain static checks; none substitutes for mission evidence.

For each runtime artifact retain: task/experiment ID, code revision and dirty
state, resolved manifest, robot/world hashes, backend/version, host, seed set,
budgets/tolerances, command, exit code, raw trace/bag locations, failure/skip
reasons and result schema version. Avoid committing large bags or licensed assets
without review; retain a checksum and documented retrieval location.

Next task: consult `handoff.next_task` in the YAML ledger. See
[AGENT_HANDOFF](docs/AGENT_HANDOFF.md) for commands, safety, ownership, session
resumption and the work-pause template. Agents must leave enough evidence to
continue without the previous chat.
