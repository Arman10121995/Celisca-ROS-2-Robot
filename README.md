# Robot Lab — Unified Multi-Robot Algorithm Laboratory

A reproducible ROS 2 laboratory where the robot, simulator, environment, perception pipeline, localization method, state estimator, global planner, local planner, and low-level controller can be changed independently and compared with common metrics.

**257 automated tests | 20 robots | 28 environments | 30 algorithms | 7 robot classes**

---

## Quick Start

```bash
# 1. Bootstrap (installs deps, builds workspace)
bash scripts/bootstrap.sh

# 2. Activate environment
source install/setup.bash
source .venv/bin/activate

# 3. Run fast validation (< 60s)
bash scripts/test_fast.sh

# 4. Check workspace health
bash scripts/doctor.sh
```

---

## Project Overview

Robot Lab is a platform for reproducible robotics research. Its core design principle is that **every claimed "integrated" option must have launch/configuration files, declared dependencies, a compatible robot/environment combination, and an automated smoke test**.

### What Makes This Different

- **Composable experiments**: robot + simulator + environment + scenario + algorithms → validated composition
- **Independent selectors**: swap any component without touching others
- **Common metrics**: every comparison produces the same standard result record
- **Automated validation**: 257 tests verify registry contracts, algorithm logic, and cross-references

---

## Architecture

### Design Rule

```
robot + simulator + environment + scenario
      + perception + localization + state estimation
      + global planning + local planning + control
                         |
                         v
              launch adapters and contracts
                         |
                         v
             metrics, artifacts, and result record
```

Each selector is independent. Compatibility is decided from explicit capabilities and contracts before launch.

### Source Tree

```
src/
  robot_lab/
    robot_lab_registry/     # schemas, catalogs, validation, query CLI
    robot_lab_adapter/      # composition and legacy launch adapters
    robot_lab_benchmark/    # runner, metrics, result schema, orchestration
  robots/                   # URDF/SDF descriptions and robot-specific assets
    _upstream/              # vendored third-party robot assets
  maps/                     # Gazebo worlds, occupancy maps, reference geometry
  gazebo_models/            # reusable environment models
  robot_lab_algorithms/     # 13 new algorithm implementations (P5)
  robot_lab_*/              # reference differential-drive robot packages
  ORB_SLAM3/                # optional external SLAM adapter
```

### Package Boundaries

| Layer | Owns | Must not own |
|-------|------|--------------|
| Registry | Metadata, schemas, compatibility rules, experiment presets | ROS nodes or simulator processes |
| Assets | URDF/SDF/Xacro, meshes, worlds, occupancy maps | Algorithm policy or benchmark conclusions |
| Algorithm adapter | One common contract around one implementation | Robot/world-specific orchestration |
| Bringup | Composition, namespaces, lifecycle order, parameter overlays | Algorithm implementation details |
| Benchmark | Scenario lifecycle, ground truth, metrics, artifacts | Hidden tuning unique to a compared algorithm |

---

## Features

### Robot Support (5 integrated, 20 cataloged)

| Robot | Class | DOF | Status | Algorithms |
|-------|-------|-----|--------|------------|
| **Bumperbot** | Differential drive | — | ✅ Full stack | Perception, planning, localization, control |
| **Labbot** | Differential drive | — | ✅ Full stack | Perception, planning, localization, control |
| **Go2** | Quadruped | 12 leg joints | ✅ Simulated | Leg control, IMU/camera/odometry |
| **Berkeley Humanoid Lite** | Humanoid | 22 position joints | ✅ Simulated | Position control, standing |
| **Quadrotor SITL** | Aerial | 4 rotors | ✅ Simulated | MAVLink attitude/position control |

Plus 15 legacy robot profiles (description-only).

### Algorithm Coverage (30 algorithms, 7 categories)

| Category | Count | Examples |
|----------|-------|----------|
| Perception | 5 | obstacle_detector, scan_clusterer, pointcloud_segmenter |
| Localization | 5 | dead_reckoning |
| State Estimation | 5 | ekf_3d_estimator, motion_model_estimator, pose_graph_estimator |
| Sensor Fusion | 5 | wheel_imu_fusion, gps_odom_fusion, complementary_imu |
| Global Planning | 5 | rrt_planner, voronoi_planner |
| Local Planning | 5 | follow_the_gap |
| Control | 9 | mavros_offboard_controller, joint_effort_commander |

### Environment Support (28 environments)

| Type | Count | Examples |
|------|-------|----------|
| Indoor worlds | 14 | small_office, small_house, warehouse_demo |
| Navigation arenas | 5 | nav_empty, nav_obstacle, nav_maze |
| Dynamic variants | 2 | nav_dynamic, nav_sensor_degraded |
| Terrain | 3 | terrain_rough, terrain_stairs, terrain_stepping_stones |
| Aerial courses | 2 | aerial_course, aerial_indoor |

---

## Benchmark Infrastructure (P6)

### Standard Result Schema

Every benchmark produces a versioned JSON record with:

- Repository revision and dirty-state marker
- ROS, simulator, host architecture, dependency versions
- Full resolved experiment and parameter hashes
- Seed, start/goal, timing, termination reason
- Success, collisions, path length, elapsed time, real-time factor, minimum clearance
- Pose/trajectory error when ground truth is available
- Artifact paths for logs, bags, maps, trajectories, plots

### Components

| Component | File | Purpose |
|-----------|------|---------|
| Result model | `robot_lab_benchmark/__init__.py` | `BenchmarkResult` with versioned schema |
| CLI | `robot_lab_benchmark/cli.py` | Emit canonical result records |
| Orchestrator | `robot_lab_benchmark/launch_orchestrator.py` | launch/reset/run/stop lifecycle |
| Ground truth | `robot_lab_benchmark/groundtruth.py` | Extract metrics from sensor data |
| Normalizer | `robot_lab_benchmark/normalizer.py` | Normalize metrics for fair comparison |
| Outputs | `robot_lab_benchmark/outputs.py` | JSON, CSV, Markdown, HTML, plots |
| Reference | `robot_lab_benchmark/reference.py` | Baseline management + regression checking |
| Reports | `robot_lab_benchmark/report.py` | Comparison summaries |

---

## Validation Layers

1. **Schema**: required fields, types, IDs, allowed statuses/categories
2. **References**: every experiment selector resolves to a catalog entry
3. **Capabilities**: robot sensors/interfaces satisfy every selected algorithm
4. **Assets**: referenced package files and plugins exist on disk
5. **Static launch**: launch descriptions expand and dependencies resolve
6. **Smoke**: processes become healthy, topics/actions appear
7. **Benchmark**: fixed seed and conditions produce a standard result record

---

## Runtime Contracts

| Contract | Interface |
|----------|-----------|
| Body command | `geometry_msgs/Twist` or class-specific trajectory |
| Estimated state | `nav_msgs/Odometry` plus TF |
| 2D obstacles | `sensor_msgs/LaserScan` and/or costmap |
| 3D perception | image/depth/camera-info or `sensor_msgs/PointCloud2` |
| Planned route | `nav_msgs/Path` |
| Navigation goal | Nav2 action contract |

---

## CLI Usage

```bash
# Validate registry
ros2 run robot_lab_registry robot-lab validate --cross-references

# List experiments
ros2 run robot_lab_registry robot-lab list experiments

# Describe an experiment
ros2 run robot_lab_registry robot-lab describe bumperbot_smoke_test

# Dry-run a composition
ros2 run robot_lab_registry robot-lab launch --dry-run bumperbot_smoke_test
```

---

## Control Center GUI

```bash
ros2 run sim_launcher_gui sim_launcher_gui
```

A single Tkinter window that drives the whole platform, robot-agnostic and registry-driven:

| Tab | What you can do |
|-----|-----------------|
| **Launch** | Pick any robot × mode (display / loc / slam / 3d_slam / nav) × map, launch the unified `robot_lab_bringup` stack, drive with the teleop pad, save maps, export 3D maps. A live info panel shows each robot's feature class, available modes, and cleaning-mission support. |
| **Registry** | Browse and search all robots, environments, algorithms, scenarios, and experiments with full YAML detail views. |
| **Vacuum** | One-click room-vacuum simulation + cleaner-node start/stop for any robot whose profile declares `supports_room_vacuum: true`. |
| **Benchmark** | Seeded benchmark runs (robot × environment × scenario × seed) via the P6 `LaunchOrchestrator`, plus regression checks against checked-in reference results. |
| **Tests** | Fast suite, full 257-test suite, registry validation, and algorithm compile checks — output streamed into the GUI console. |
| **Health** | Doctor diagnostics, platform status (phase ledger + test counts), live ROS node/topic inspection. |

Adding a robot to `src/robots/config/robots.yaml` (or a map to `sim_maps.yaml`) makes it appear — correctly gated by capabilities — everywhere in the GUI with no code changes.

---

## Testing

```bash
# Fast PR tests (< 60s, no simulation)
bash scripts/test_fast.sh

# Full test suite (all 257 tests)
PYTHONPATH=src/robot_lab/robot_lab_registry python3 -m unittest discover \
    -s src/robot_lab/robot_lab_registry/test

# Workspace health check
bash scripts/doctor.sh
```

---

## Project Structure

```
robot_lab_ws/
├── .github/workflows/        # CI (fast PR) + scheduled (full) workflows
├── docs/
│   ├── architecture/
│   │   └── overview.md       # Platform architecture and design rules
│   ├── status/
│   │   ├── platform-status.yaml  # Machine-readable program state
│   │   └── support-matrix.md     # Robot/algorithm/environment support
│   └── tutorials/            # Step-by-step comparison guides
├── scripts/
│   ├── bootstrap.sh          # Workspace setup
│   ├── doctor.sh             # Health diagnostics
│   └── test_fast.sh          # Fast PR test suite
├── src/
│   ├── robot_lab/            # Core platform (registry, adapter, benchmark)
│   ├── robots/               # Robot descriptions + upstream assets
│   ├── maps/                 # Worlds, occupancy maps
│   ├── gazebo_models/        # Reusable environment models
│   ├── sim_launcher_gui/     # Unified control-center GUI (Tkinter)
│   ├── vacuum_cleaning/      # Robot-agnostic cleaning mission node
│   ├── robot_lab_algorithms/   # Perception, localization, EKF, fusion, planners
│   ├── robot_lab_bringup/      # Simulation/real launch entry points
│   ├── robot_lab_adapter/      # Legacy launch adapters + fragments
│   ├── robot_lab_controller/   # Teleop, mapping, cleaning controllers
│   ├── robot_lab_description/  # Xacro/URDF, meshes (reference robots)
│   ├── robot_lab_localization/ # AMCL / localization launch helpers
│   ├── robot_lab_mapping/      # SLAM (slam_toolbox, RTAB-Map) helpers
│   ├── robot_lab_navigation/   # Nav2 bringup and config
│   ├── robot_lab_motion/       # Motion primitives
│   ├── robot_lab_planning/     # Path planning helpers
│   ├── robot_lab_msgs/         # Shared message definitions
│   ├── robot_lab_utils/        # Shared utilities
│   ├── robot_lab_firmware/     # Hardware interface definitions
│   ├── robot_lab_cpp_examples/ # C++ ROS2 examples (tf, publishers)
│   └── robot_lab_py_examples/  # Python ROS2 examples
├── LICENSE                   # MIT License
├── LICENSES/                 # Third-party license notices
├── ROADMAP.md                # Program state and task ledger
└── README.md                 # This file
```

Every robot profile (bumperbot, labbot, go2, h1, quadrotor, ...) plugs into the
same `robot_lab_*` packages through the registry catalogs — no robot has its own
parallel package set. Robot-specific assets (URDF, meshes, controllers config,
joint names) live under `src/robots/` and are referenced by robot id.


---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

Third-party assets (robot descriptions, meshes) retain their original licenses. See [LICENSES/third-party-notices.md](LICENSES/third-party-notices.md) for attribution.

| Asset | Source | License |
|-------|--------|---------|
| Unitree robot descriptions | Awesome-URDFs / Unitree Robotics | BSD 3-Clause |
| Berkeley Humanoid Lite | HybridRobotics | CC BY-SA 4.0 |

---

## Roadmap Status

| Phase | Title | State |
|-------|-------|-------|
| P0 | Repair and baseline | ✅ Done |
| P1 | Platform foundation | ✅ Done |
| P2 | Unified composition | ✅ Done |
| P3 | Robot integrations | ✅ Done |
| P4 | Environments | ✅ Done |
| P5 | Algorithm breadth | ✅ Done |
| P6 | Benchmarking | ✅ Done |
| P7 | Hardening | ✅ Done (P7.5 blocked on hardware) |

See [ROADMAP.md](ROADMAP.md) for detailed task breakdown and evidence.
