# Robot Lab continuous roadmap

This file is the hand-off ledger for the repository-wide program. It is the
first file an agent should read before changing the platform. Update it in the
same change that completes, blocks, adds, or materially re-scopes a task.

Last updated: 2026-08-31

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
| P2 — unified composition | queued | Independent selectors and compatibility-preserving launch/CLI |
| P3 — robot integrations | queued | Working mobile, legged, humanoid, and aerial representatives |
| P4 — environments | queued | Diverse deterministic 2D/3D benchmark environments |
| P5 — algorithm breadth | queued | At least five runnable alternatives in every required category |
| P6 — benchmarking | queued | Repeatable scenarios, metrics, result capture, and reports |
| P7 — hardening | queued | CI matrices, provenance/licenses, documentation, and end-to-end qualification |

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

- [ ] **P2.1** Introduce `robot_lab_bringup` with selectors for robot,
  environment, simulator, and scenario.
- [ ] **P2.2** Add independent selectors for perception, localization, state
  estimation, global planning, local planning, and control.
- [ ] **P2.3** Resolve selectors into launch fragments and parameter overlays;
  reject invalid combinations before processes start.
- [ ] **P2.4** Wrap `bumperbot_bringup/simulated_robot.launch.py` as the first
  adapter and retain its public arguments until a documented deprecation.
- [ ] **P2.5** Add namespaces and frame-prefix contracts so parallel and
  multi-robot experiments do not collide.
- [ ] **P2.6** Add `list`, `describe`, `validate`, `launch --dry-run`, and
  `doctor` commands.

### P3 — robot integrations

- [ ] **P3.1** Qualify Bumperbot as the reference differential-drive robot.
- [ ] **P3.2** Integrate a second, independently maintained mobile base.
- [ ] **P3.3** Turn one existing Unitree quadruped description into a simulated,
  commandable legged profile with sensors and odometry.
- [ ] **P3.4** Turn one existing humanoid description into a simulated,
  commandable profile with a stable standing/walking controller.
- [ ] **P3.5** Integrate one multirotor SITL profile with pose, IMU, camera, and
  velocity/trajectory command contracts.
- [ ] **P3.6** Add per-class smoke scenarios and documented safety/compute limits.

### P4 — environments

- [ ] **P4.1** Qualify all 14 existing Gazebo worlds and their occupancy-map
  provenance.
- [ ] **P4.2** Add deterministic empty, obstacle, maze, narrow-passage, and
  warehouse navigation arenas.
- [ ] **P4.3** Add rough terrain, stairs/ramps, and stepping-stone arenas for
  legged/humanoid robots.
- [ ] **P4.4** Add indoor and outdoor 3D/aerial courses with ground truth.
- [ ] **P4.5** Add dynamic-obstacle and sensor-degradation variants.
- [ ] **P4.6** Add seeds, reset services, spawn zones, goals, and reference paths.

### P5 — algorithm breadth

Each category must reach five `integrated` implementations and then five
`benchmarked` implementations. Candidates in the registry are a queue, not a
completion claim.

- [ ] **P5.1** Perception: five sensor/environment interpretation pipelines.
- [ ] **P5.2** Localization: five global/relative pose solutions.
- [ ] **P5.3** State estimation and sensor fusion: five filters/estimators.
- [ ] **P5.4** Global planning: five interchangeable global planners.
- [ ] **P5.5** Local planning: five obstacle-aware trajectory/path followers.
- [ ] **P5.6** Control: five low-level/model-based control methods including
  PID, linear control, MPC, and nonlinear control.
- [ ] **P5.7** Normalize parameters, topic/action contracts, lifecycle behavior,
  and failure reporting across adapters.

### P6 — benchmarking

- [ ] **P6.1** Define versioned experiment/result schemas and provenance fields.
- [ ] **P6.2** Record success, collisions, time, path length, clearance, energy
  proxy, CPU, memory, real-time factor, and localization/trajectory error.
- [ ] **P6.3** Add seeded launch/reset/run/stop orchestration and rosbag capture.
- [ ] **P6.4** Add ground-truth adapters and per-robot metric normalization.
- [ ] **P6.5** Generate machine-readable results and comparison plots/tables.
- [ ] **P6.6** Check in small reference results and regression thresholds.

### P7 — hardening

- [ ] **P7.1** Add fast PR tests and scheduled full simulation matrices.
- [ ] **P7.2** Pin external sources and document asset/code licenses.
- [ ] **P7.3** Add install/bootstrap/doctor flows for supported hosts.
- [ ] **P7.4** Add tutorials that reproduce one comparison in each category.
- [ ] **P7.5** Validate real Bumperbot hardware and explicitly separate HIL-only
  claims from simulation claims.
- [ ] **P7.6** Publish a support matrix with measured evidence and known limits.

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
- Gazebo Classic is the current simulator path; simulator abstraction is not
  implemented yet.
- Only Bumperbot currently has a complete simulation/navigation stack. Imported
  Unitree and Berkeley assets are description-only and must not be advertised as
  commandable robots.
- ORB-SLAM3 is optional and currently carries an OpenCV ABI warning when its
  external build does not match ROS `cv_bridge`.
- Physical serial hardware, motor direction, and the controller protocol still
  require hardware-in-the-loop validation.
- Upstream robot/model assets have separate licenses. Provenance and
  redistribution review are required before release.
