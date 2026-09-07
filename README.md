# Robot Lab — Multi-Robot Simulation and Algorithm Laboratory

Robot Lab aims to make robots, simulators, maps and algorithms independently selectable so that you can learn how they work and compare their performance under reproducible conditions.

**Current state: a research prototype and integration foundation, not a fully interchangeable or production-grade platform.** The Bumperbot-oriented ROS 2 stack is the strongest implementation. Additional robot assets, simulator adapters, algorithm kernels, a desktop GUI and benchmark infrastructure exist, but important runtime connections and qualification tests remain incomplete.

Status reconciled on **2026-09-07**, against source revision `dff388f`. This documentation update does not fix or qualify runtime code.

## Start here

- [Audit and evidence](docs/status/audit-2026-09-07.md): what was inspected/tested, confirmed defects and verification limits.
- [Implementation roadmap](ROADMAP.md): ordered work, dependencies and acceptance criteria toward the full platform.
- [Agent handoff](docs/AGENT_HANDOFF.md): how to resume, claim work, avoid conflicts and record evidence.
- [Machine-readable status](docs/status/platform-status.yaml) and [support matrix](docs/status/support-matrix.md): current state, not historical completion claims.
- [Architecture](docs/architecture/overview.md) and [tutorials](docs/tutorials/index.md): current wiring, target contracts and learning material.

## What exists, and what that means

| Inventory at the audited revision | Evidence and limitation |
|---|---|
| 26 discoverable ROS packages | Includes optional `orbslam3`; discovery is not a clean build result |
| 20 registry robots across 5 classes | 5 labeled `integrated`, 15 `cataloged`; labels are not independent task qualification |
| 17 main-launch robot profiles | Profile IDs and registry IDs are not fully aligned |
| 26 environment entries | World/map assets include legacy worlds and deterministic arenas; not every robot/backend combination is qualified |
| 43 algorithm entries | 39 labeled `integrated`, 4 `cataloged`; includes utilities, educational kernels and unfinished ROS adapters |
| 18 scenarios and 15 experiments | Metadata and presets, not 33 successful recorded missions |
| 4 simulator launch routes | Gazebo, PyBullet, MuJoCo and Isaac; this is not feature parity |

### Verification snapshot

The 2026-09-07 audit reported **485 passing, 1 failing, 9 deselected** source tests, plus **5 passing, 1 deselected** selected backend tests. The failure constructs `DeadReckoning` without initializing ROS. Passing backend checks include basic PyBullet/MuJoCo physics, not complete navigation missions.

No clean rebuild, full simulator mission, interactive GUI session or hardware test was performed in that audit. Stored `colcon` results also contain failures. There is therefore no justified blanket “all tests passing,” “CI passing,” or “all simulators qualified” claim. See the audit for exact scope and reproduction commands.

## How it currently works

The main execution route is:

```text
GUI or ros2 launch
  → robot_lab_bringup/simulated_robot.launch.py
  → resolve robot profile + environment + mode + simulator
  → start robot description and simulator
  → start mode-specific mapping/localization/navigation nodes
  → sensor data → estimated pose → plan → command → simulated robot
```

| Mode | Intended behavior |
|---|---|
| `display` | Robot model display in RViz |
| `loc` | Known-map localization with AMCL and local state estimation |
| `slam` | 2D mapping with SLAM Toolbox |
| `3d_slam` | RGB-D mapping with RTAB-Map; requires actual camera streams |
| `nav` | Known-map localization plus Nav2 navigation |

The default navigation configuration uses AMCL, `robot_localization` EKF, **SmacPlanner2D** and **Regulated Pure Pursuit**. These defaults are not automatically replaced by choosing an algorithm in the GUI or registry. The Gazebo-oriented route also includes command multiplexing for navigation and teleoperation.

The target experiment model adds independent perception, localization, state-estimation, sensor-fusion, global-planning, local-planning and control selectors. Today the registry can describe that model, but the composition CLI only previews it and the main launch's `algorithm` argument is not applied. The GUI, registry and launch profiles are not yet a single authoritative execution path.

## Package map

Paths below are relative to `src/`; only the registry and benchmark packages live inside its `robot_lab/` subdirectory.

| Area | Packages |
|---|---|
| Catalogs, validation and result records | `robot_lab/robot_lab_registry`, `robot_lab/robot_lab_benchmark` |
| Composition and launch | `robot_lab_adapter`, `robot_lab_bringup` |
| Robot/world assets | `robot_lab_description`, `robot_lab_robots`, `robot_lab_maps`, `robot_lab_models` |
| Additional simulator adapters | `robot_lab_pybullet`, `robot_lab_mujoco`, `robot_lab_isaac` |
| Mapping and localization | `robot_lab_mapping`, `robot_lab_localization` |
| Planning and motion | `robot_lab_navigation`, `robot_lab_planning`, `robot_lab_motion` |
| Algorithm kernels/adapters | `robot_lab_algorithms` |
| Control, teleoperation and utilities | `robot_lab_controller`, `robot_lab_utils` |
| Desktop and coverage application | `robot_lab_gui`, `robot_lab_vacuum_cleaning` |
| Hardware, interfaces and examples | `robot_lab_firmware`, `robot_lab_msgs`, `robot_lab_cpp_examples`, `robot_lab_py_examples` |
| Optional external adapter | `ORB_SLAM3` (package name `orbslam3`) |

### Robots and environments

- **Bumperbot:** reference differential-drive description, sensors, control, mapping, localization, navigation and cleaning workflows. Strongest integration path, but not freshly mission-qualified in this audit.
- **Labbot:** lightweight differential-drive description and navigation-related configuration; needs independent end-to-end qualification.
- **Go2 and Berkeley Humanoid Lite:** descriptions, joint-control and sensor assets exist. Their main launch profiles allow `display` only; walking and terrain traversal are not established.
- **Quadrotor SITL:** description and MAVROS-related controller code exist; a complete flight/SITL mission is not established.
- **Other cataloged robots:** imported descriptions span legged, humanoid and manipulator models. Asset availability is not locomotion/control support.

The 26 environments comprise 14 legacy/general worlds, 5 deterministic navigation arenas, 3 terrain worlds, 2 aerial courses and 2 dynamic/occlusion variants. Arena generators and occupancy-map consistency checks are useful foundations. The rough-terrain registry ID is `outdoor_terrain`, not `terrain_rough`. `nav_sensor_degraded` supplies occluding geometry, not a general noise/dropout framework.

### Algorithms and benchmarking

| Registry category | Entries | Examples or current scope |
|---|---:|---|
| Perception | 8 | Scan/cloud conversion, obstacle detection, clustering and segmentation |
| Localization | 6 | AMCL, RTAB-Map, dead reckoning and motion-model entries |
| State estimation | 5 | Kalman/EKF and educational estimator kernels |
| Sensor fusion | 5 | IMU processing and wheel/IMU/GPS fusion kernels |
| Global planning | 5 | Dijkstra, A*, NavFn, RRT and Voronoi-style planning |
| Local planning | 5 | TEB, DWB, pure pursuit, PD path following and follow-the-gap |
| Control | 9 | Wheel/joint/standing/offboard control entries and application utilities |

These counts do **not** establish five distinct, mathematically validated, ROS-connected alternatives per category. Several entry points only spin an empty ROS node; perception entry points also have object/node mismatches. Some named methods are simplified approximations. MPC, LQR and nonlinear-control breadth remains future work.

Benchmark code includes schemas, output/report generation, orchestration helpers and regression thresholds. Some measurements are fixed placeholders, and orchestration can report success without a successful mission. **Do not use its current results to rank algorithms.** The benchmark `ros2 run` executable installation also needs repair.

### Simulator and GUI limits

Gazebo is the reference integration route. PyBullet and MuJoCo have physics engines and ROS bridge code; Isaac uses a separate runtime subprocess and can fall back to offline behavior. A simulator process starting or an offline stub running is not qualification.

The three non-Gazebo spawners publish the wrong message type on `/clock`; command/odometry wiring needs reconciliation. Their camera coverage is insufficient for `3d_slam`, and Isaac lacks scan publication. See the support matrix before selecting a backend.

The Tkinter GUI has launch profiles, process logs, registry browsing, drive/map tools, vacuum/benchmark/test/health tabs and telemetry monitoring. These are interface features, not proof that each underlying workflow works. In particular, its algorithm dropdown does not currently switch the running algorithm stack.

## Browse the repository safely

From the repository root, these commands use the source registry without a workspace build (Python 3 with PyYAML/jsonschema is required):

```bash
export PYTHONPATH="$PWD/src/robot_lab/robot_lab_registry${PYTHONPATH:+:$PYTHONPATH}"
python3 -m robot_lab_registry.cli summary -c src/robot_lab/robot_lab_registry/config
python3 -m robot_lab_registry.cli list robots -c src/robot_lab/robot_lab_registry/config
python3 -m robot_lab_registry.cli list algorithms -c src/robot_lab/robot_lab_registry/config
python3 -m robot_lab_registry.cli describe experiment bumperbot_simulation -c src/robot_lab/robot_lab_registry/config
python3 -m robot_lab_registry.cli validate -c src/robot_lab/robot_lab_registry/config --cross-references
```

`describe` takes a **singular entity type plus its ID**; `validate` requires `-c`. Cross-reference validation passes at the audited revision, but capability/category/simulator validation has known gaps: passing validation does not prove a runnable composition.

Preview only—this does not launch a robot, including with `--no-dry-run`:

```bash
python3 -m robot_lab_registry.cli launch \
  -c src/robot_lab/robot_lab_registry/config --dry-run \
  --composition '{"robot_id":"bumperbot","environment_id":"small_office","simulator":"gazebo","algorithm_ids":{"localization":"amcl"}}'
```

For an already built ROS 2 Humble workspace, the intended entry points are below. Their syntax follows the source; they were **not mission-tested in the latest audit**:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=nav map_name:=small_office robot_model:=bumperbot simulator:=gazebo
ros2 run robot_lab_gui robot_lab_gui
```

Ubuntu 22.04 / ROS 2 Humble on arm64 is the historical development baseline, not a newly verified installation guarantee. Review `scripts/bootstrap.sh` before using it: it installs/builds dependencies and currently suppresses some failures. `scripts/test_fast.sh` and `scripts/doctor.sh` are partial checks, not acceptance gates; CI also has branch coverage and ignored-failure defects. Follow the roadmap's baseline-recovery tasks before trusting a fresh installation.

## Path to the intended platform

The [roadmap](ROADMAP.md) and [agent handoff](docs/AGENT_HANDOFF.md) are the implementation authority. The delivery order is: recover trustworthy build/test evidence; repair simulator contracts and fail-fast behavior; make composition selections executable and strictly validated; complete one measured mobile experiment; then qualify robot classes, algorithm alternatives, maps and remaining backends in bounded increments.

Success means selectable mobile, legged, humanoid and aerial robots; 2D/3D environments; at least five genuinely distinct supported alternatives in each requested algorithm category; and reproducible comparisons with measured outcomes. Each supported combination needs explicit dependencies, contracts, launch configuration, numerical tests where applicable, a runtime smoke test, and evidence-backed maturity. A numerical kernel, a catalog row, a GUI selector and a completed mission are different milestones.

Before implementing, read the handoff, claim one task, preserve other agents' work and run its stated acceptance checks. Update the roadmap and machine-readable status with commands, revision, results and remaining limitations. Do not promote a feature to integrated/qualified merely because imports or schema checks pass.

## License

See [LICENSE](LICENSE) for repository licensing and [third-party notices](LICENSES/third-party-notices.md) for imported assets and libraries. Third-party components retain their own terms; a top-level license does not relicense vendored code, meshes or optional integrations.
