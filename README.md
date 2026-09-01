# Robot Lab — Unified Multi-Robot Algorithm Laboratory

**A reproducible ROS 2 laboratory where every component — robot, simulator, environment, scenario, perception pipeline, localization method, state estimator, global planner, local planner, and low-level controller — can be changed independently and compared with common metrics.**

---

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS 2: Humble](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform: Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![Arch: arm64](https://img.shields.io/badge/Arch-arm64-green)](https://arm.com/)
[![Tests: 257 Passing](https://img.shields.io/badge/Tests-257%20Passing-brightgreen)](https://github.com/features/actions)
[![CI: Passing](https://img.shields.io/badge/CI-Passing-brightgreen)](.github/workflows/)

**257 automated tests | 20 robots | 28 environments | 30 algorithms | 7 robot classes | 18 scenarios | 15 experiments**

Last updated: 2026-09-01

---

## Quick Start

```bash
# 1. Bootstrap (installs dependencies, builds workspace)
bash scripts/bootstrap.sh

# 2. Activate environment
source install/setup.bash
source .venv/bin/activate  # If using Python virtual environment

# 3. Run fast validation (< 60s)
bash scripts/test_fast.sh

# 4. Check workspace health
bash scripts/doctor.sh
```

---

## Project Overview

Robot Lab is a **production-grade robotics research platform** that turns ROS 2 from a collection of packages into a **cohesive, testable, benchmarkable laboratory**.

Its core design principle is: **"An experiment is a validated composition, not a monolithic mode."**

This means every experiment is a validated combination of:
- robot + simulator + environment + scenario
- perception + localization + state estimation
- global planning + local planning + control

All leading to: launch adapters and contracts → metrics, artifacts, and result records

### What Makes This Different

- **Composable experiments**: Every component can be swapped independently
- **Independent selectors**: Compatibility decided from explicit capabilities and contracts before launch
- **Common metrics**: Every comparison produces the same standard result record
- **Registry-driven**: All metadata in YAML catalogs
- **Automated validation**: 257 tests verify everything
- **Benchmark-first**: Every algorithm has standard result schema and regression testing
- **Unified composition**: One bringup handles all robots via robot_model parameter

---

## Architecture

See [Architecture Overview](docs/architecture/overview.md) for complete details.

### Design Philosophy

Each selector is independent. Compatibility is decided from explicit capabilities and contracts before launch.

### Package Boundaries

| Layer | Owns | Must NOT Own |
|-------|------|--------------|
| Registry | Metadata, schemas, compatibility rules, experiment presets | ROS nodes or simulator processes |
| Assets | URDF/SDF/Xacro, meshes, worlds, occupancy maps | Algorithm policy or benchmark conclusions |
| Algorithm Adapter | One common contract around one implementation | Robot/world-specific orchestration |
| Bringup | Composition, namespaces, lifecycle order, parameter overlays | Algorithm implementation details |
| Benchmark | Scenario lifecycle, ground truth, metrics, artifacts | Hidden tuning unique to a compared algorithm |

### Source Tree

```
robot_lab_ws/
├── docs/
│   ├── architecture/overview.md       # Complete architecture documentation
│   ├── status/platform-status.yaml    # Machine-readable program state
│   ├── status/support-matrix.md       # Support matrix
│   └── tutorials/                       # Step-by-step guides
├── scripts/
│   ├── bootstrap.sh                    # Workspace setup
│   ├── doctor.sh                       # Health diagnostics
│   └── test_fast.sh                    # Fast PR test suite
├── src/
│   ├── robot_lab/                      # Core platform
│   │   ├── robot_lab_registry/         # Metadata catalogs
│   │   ├── robot_lab_adapter/          # Composition adapters
│   │   ├── robot_lab_benchmark/        # Benchmarking
│   │   ├── robot_lab_algorithms/       # Algorithm implementations
│   │   └── ...
│   ├── robot_lab_robots/               # 20 robot descriptions
│   ├── robot_lab_maps/                 # 28 environment maps
│   ├── robot_lab_gui/                  # Control Center GUI
│   └── ORB_SLAM3/                      # Optional SLAM
├── LICENSE
├── LICENSES/third-party-notices.md
├── ROADMAP.md
└── README.md
```

---

## Features

### Robot Support (20 robots across 4 classes)

| Robot | Class | DOF | Status | Features |
|-------|-------|-----|--------|----------|
| Bumperbot | Differential drive | 2 | Integrated | Reference robot, full stack |
| Labbot | Differential drive | 2 | Integrated | Mesh-free, lightweight |
| Go2 | Quadruped | 12 | Integrated | 12 effort-controlled joints |
| Berkeley Humanoid Lite | Humanoid | 22 | Integrated | 22 position joints |
| Quadrotor SITL | Aerial | 4 | Integrated | MAVLink control |
| 15 Unitree robots | Quadruped/Humanoid | 12-42 | Cataloged | Vendored descriptions |

### Algorithm Coverage (30 algorithms, 7 categories)

| Category | Count | Examples |
|----------|-------|----------|
| Perception | 5+ | obstacle_detector, scan_clusterer, pointcloud_segmenter |
| Localization | 5+ | amcl, rtabmap, hector_slam, dead_reckoning |
| State Estimation | 5+ | ekf_3d_estimator, motion_model_estimator, pose_graph_estimator |
| Sensor Fusion | 5+ | wheel_imu_fusion, gps_odom_fusion, complementary_imu |
| Global Planning | 5+ | rrt_planner, voronoi_planner, a_star_planner |
| Local Planning | 5+ | follow_the_gap, pure_pursuit, pd_motion_planner |
| Control | 9+ | joint_effort_commander, humanoid_standing_controller, mavros_offboard_controller |

### Environment Support (28 environments)

| Category | Count | Examples |
|----------|-------|----------|
| Indoor worlds | 14 | small_office, small_house, warehouse_demo |
| Navigation arenas | 5 | nav_empty, nav_obstacle, nav_maze |
| Dynamic variants | 2 | nav_dynamic, nav_sensor_degraded |
| Terrain | 3 | terrain_rough, terrain_stairs, terrain_stepping_stones |
| Aerial courses | 2 | aerial_course, aerial_indoor |

---

## CLI Usage

### Registry CLI

```bash
# Validate registry
ros2 run robot_lab_registry robot-lab validate --cross-references

# List all entities
ros2 run robot_lab_registry robot-lab list robots
ros2 run robot_lab_registry robot-lab list algorithms

# Describe an experiment
ros2 run robot_lab_registry robot-lab describe bumperbot_smoke_test

# Dry-run a composition
ros2 run robot_lab_registry robot-lab launch --dry-run bumperbot_smoke_test

# Doctor diagnostics
ros2 run robot_lab_registry robot-lab doctor
```

### Benchmark CLI

```bash
# Run a benchmark
ros2 run robot_lab_benchmark benchmark \
    --experiment bumperbot_smoke_test \
    --robot bumperbot \
    --environment small_office \
    --seed 42 \
    --output-dir ./results

# With rosbag capture
ros2 run robot_lab_benchmark benchmark --bag-capture true
```

---

## GUI

```bash
ros2 run robot_lab_gui robot_lab_gui
```

- **Launch tab**: Robot/Mode/Map selection, drive pad, save maps
- **Registry tab**: Browse all 5 registries
- **Vacuum tab**: Room vacuum mission control
- **Benchmark tab**: Seeded runs and regression checks
- **Tests tab**: Run test suites
- **Health tab**: Doctor diagnostics and live ROS graph

---

## Workflow Examples

### Launch a Simulation

```bash
# Navigation with Bumperbot
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=nav map_name:=small_office robot_model:=bumperbot

# Display only
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=display robot_model:=labbot

# 3D SLAM (requires RTAB-Map)
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=3d_slam map_name:=small_house robot_model:=bumperbot
```

### Vacuum Cleaning

```bash
ros2 launch robot_lab_bringup simulated_room_vacuum.launch.py \
    robot_model:=bumperbot map_name:=small_house
ros2 run robot_lab_vacuum_cleaning vacuum_cleaner
```

---

## Testing

```bash
# Fast PR tests (< 60s)
bash scripts/test_fast.sh

# Full test suite (257 tests)
PYTHONPATH=src/robot_lab/robot_lab_registry python3 -m unittest discover \
    -s src/robot_lab/robot_lab_registry/test

# Health check
bash scripts/doctor.sh
```

**257/257 tests passing, 0 errors, 0 failures**

---

## Platform Status

| Phase | Title | State |
|-------|-------|-------|
| P0 | Repair and baseline | Done |
| P1 | Platform foundation | Done |
| P2 | Unified composition | Done |
| P3 | Robot integrations | Done |
| P4 | Environments | Done |
| P5 | Algorithm breadth | Done |
| P6 | Benchmarking | Done |
| P7 | Hardening | Active (P7.5 blocked) |

**Next:** P7.6 (Support Matrix)
**Blocked:** P7.5 (Hardware HIL)

---

## Documentation

- [Architecture Overview](docs/architecture/overview.md)
- [Support Matrix](docs/status/support-matrix.md)
- [Platform Status](docs/status/platform-status.yaml)
- [ROADMAP](ROADMAP.md)
- [Tutorials](docs/tutorials/)

---

## License

MIT License - See [LICENSE](LICENSE)

Third-party assets retain their original licenses:
- Unitree: BSD 3-Clause
- Berkeley Humanoid Lite: CC BY-SA 4.0
- ORB-SLAM3: GPL-3.0

See [LICENSES/third-party-notices.md](LICENSES/third-party-notices.md)

---

## Contributing

1. Read ROADMAP.md, docs/status/platform-status.yaml, docs/architecture/overview.md
2. Claim next task by setting state to 'active' in ROADMAP.md
3. Implement end-to-end: configuration + adapter + dependency + test + documentation + provenance
4. Update both ledgers with evidence

---

*Built for reproducible robotics research.*
*2026 Robot Lab Team*
