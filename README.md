# Robot Lab — Unified Multi-Robot Algorithm Laboratory

**A reproducible ROS 2 laboratory where every component — robot, simulator, environment, scenario, perception pipeline, localization method, state estimator, global planner, local planner, and low-level controller — can be changed independently and compared with common metrics.**

---

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS 2: Humble](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform: Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![Arch: arm64](https://img.shields.io/badge/Arch-arm64-green)](https://arm.com/)
[![Tests: 257 Passing](https://img.shields.io/badge/Tests-257%20Passing-brightgreen)](https://github.com/features/actions)
[![CI: Passing](https://img.shields.io/badge/CI-Passing-brightgreen)](.github/workflows/)

**257 automated tests | 20 robots | 26 environments | 43 algorithms | 5 robot classes | 18 scenarios | 15 experiments | 4 simulators**

Last updated: 2026-09-03

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
- **Multi-simulator**: One launch dispatch routes to Gazebo, Isaac Sim, PyBullet, or MuJoCo

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
│   ├── robot_lab_isaac/               # Isaac Sim simulator adapter
│   ├── robot_lab_pybullet/            # PyBullet simulator adapter
│   ├── robot_lab_mujoco/              # MuJoCo simulator adapter
│   ├── robot_lab_robots/              # 20 robot descriptions
│   ├── robot_lab_maps/                # 26 environment maps
│   ├── robot_lab_gui/                 # Control Center GUI
│   └── ORB_SLAM3/                      # Optional SLAM
├── LICENSE
├── LICENSES/third-party-notices.md
├── ROADMAP.md
└── README.md
```

---

## Features

### Robot Support (20 robots across 5 classes)

| Robot | Class | DOF | Status | Features |
|-------|-------|-----|--------|----------|
| Bumperbot | Differential drive | 2 | Integrated | Reference robot, full stack |
| Labbot | Differential drive | 2 | Integrated | Mesh-free, lightweight |
| Go2 | Quadruped | 12 | Integrated | 12 effort-controlled joints |
| Berkeley Humanoid Lite | Humanoid | 22 | Integrated | 22 position joints |
| Quadrotor SITL | Aerial | 4 | Integrated | MAVLink control |
| 15 Unitree robots | Quadruped/Humanoid | 12-42 | Cataloged | Vendored descriptions |

### Algorithm Coverage (43 algorithms, 7 categories)

| Category | Count | Examples |
|----------|-------|----------|
| Perception | 8 | obstacle_detector, scan_clusterer, pointcloud_segmenter |
| Localization | 6 | amcl, rtabmap, hector_slam, dead_reckoning |
| State Estimation | 5 | ekf_3d_estimator, motion_model_estimator, pose_graph_estimator |
| Sensor Fusion | 5 | wheel_imu_fusion, gps_odom_fusion, complementary_imu |
| Global Planning | 5 | rrt_planner, voronoi_planner, a_star_planner |
| Local Planning | 5 | follow_the_gap, pure_pursuit, pd_motion_planner |
| Control | 9 | joint_effort_commander, humanoid_standing_controller, mavros_offboard_controller |

### Environment Support (26 environments)

| Category | Count | Examples |
|----------|-------|----------|
| Indoor worlds | 14 | small_office, small_house, warehouse_demo |
| Navigation arenas | 5 | nav_empty, nav_obstacle, nav_maze |
| Dynamic variants | 2 | nav_dynamic, nav_sensor_degraded |
| Terrain | 3 | terrain_rough, terrain_stairs, terrain_stepping_stones |
| Aerial courses | 2 | aerial_course, aerial_indoor |

### Simulator Support (4 backends)

One launch dispatcher (`simulated_robot.launch.py`) selects the physics backend with the `simulator:=` argument. Every mode declares which simulators it supports in `sim_modes.yaml`, and each backend exposes the same spawn interface (world/model/pose arguments).

| Simulator | Package | Status | Notes |
|-----------|---------|--------|-------|
| Gazebo Harmonic | `robot_lab_description` | ✅ Qualified | `gz sim` headless; furniture-mesh worlds load cleanly |
| Isaac Sim | `robot_lab_isaac` | ✅ Qualified | 6.0.1.0 native pip install on the 1 TB SSD; map meshes (celisca_floor_1, 91936 tris) and robot spawn live-verified with GUI |
| PyBullet | `robot_lab_pybullet` | ✅ Qualified | 3.2.7 source-rebuilt vs NumPy 2.x; live launch verified (incl. `use_sim_time`) |
| MuJoCo | `robot_lab_mujoco` | ✅ Qualified | 3.12.0 source-built; live launch verified (incl. `use_sim_time` and `celisca_floor_1`) |

```bash
# Select the physics backend (default: gazebo)
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=nav map_name:=small_office robot_model:=bumperbot simulator:=gazebo

ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=nav map_name:=small_office robot_model:=bumperbot simulator:=isaac
```

The GUI Launch tab exposes the same dropdown, gated per mode by the `simulators:` list in `sim_modes.yaml`. Unknown or unsupported simulator values are rejected before any process starts.

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

The GUI provides a **Launch** tab with dropdowns for robot, simulator, and GUI mode (Auto/GUI/Headless). All four simulators are selectable per the `simulators:` list in `sim_modes.yaml`. The summary panel shows resolved paths, and the output console streams launch logs in real time.

- **Launch tab**: Robot/Mode/Map/Simulator selection, drive pad, save maps
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

# Navigation with a different physics backend
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=nav map_name:=small_office robot_model:=bumperbot simulator:=pybullet

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

# Full test suite (257 registry tests + bringup profile tests)
PYTHONPATH=src/robot_lab/robot_lab_registry python3 -m unittest discover \
    -s src/robot_lab/robot_lab_registry/test
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
    src/robot_lab_bringup/test/test_sim_profiles.py \
    src/robot_lab_bringup/test/test_simulator_backends.py -q

# Health check
bash scripts/doctor.sh
```

**257/257 registry tests passing, 0 errors, 0 failures.** Bringup profile tests
(206 collected, including 7 simulator-dispatch tests and 6 simulator-backend
smoke tests for PyBullet, MuJoCo, and Isaac offline-mode) also pass.

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

**In Progress:** P7.8 (Multi-simulator physics backends — Gazebo Harmonic, PyBullet & MuJoCo live-verified; Isaac Sim 6.0.1 installed natively via pip on the 1 TB SSD with runtime subprocess wired; Jetson TSC caveat documented)
**Blocked:** P7.5 (Hardware HIL)

---

## Simulator Backends

All four backends are qualified on the Jetson AGX Orin (arm64) target:

| Backend | Version | Status |
|---------|---------|--------|
| Gazebo (Harmonic) | gz-sim8 8.15.0 | ✅ Qualified — all worlds incl. celisca_floor_1 load headless |
| PyBullet | 3.2.7 (rebuilt vs NumPy 2.2.6) | ✅ Qualified — live launch, full topic contract, use_sim_time fixed |
| MuJoCo | 3.12.0 (source-built C lib + pip) | ✅ Qualified — live launch, full topic contract, MJCF mesh paths fixed |
| Isaac Sim | 6.0.1.0 (pip, aarch64 wheel, 1 TB SSD) | ✅ Qualified — native pip install, runtime subprocess, SDF mesh loading, full topic contract live-verified |

Launch any backend via the unified dispatcher:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=nav map_name:=small_office robot_model:=bumperbot simulator:=pybullet
# simulator:= gazebo | pybullet | mujoco | isaac
```

### Isaac Sim native install (aarch64, 1 TB SSD)

Isaac Sim ≥ 5.0 ships aarch64 pip wheels but requires **Python 3.12**, while
ROS 2 Humble's `rclpy` is Python-3.10-only.  The two therefore run as separate
processes:

- `robot_lab_isaac/isaac_spawner.py` — the ROS 2 node (Python 3.10).  Owns
  the topic contract (`/clock`, `/joint_states`, `/odom`, `/imu/out`, `/tf`),
  forwards `/cmd_vel`, and manages the child process.
- `robot_lab_isaac/isaac_runtime.py` — the Isaac Sim runtime child, executed
  under a dedicated Python 3.12 virtualenv on the 1 TB SSD.  Instantiates
  `SimulationApp`, builds the stage (pre-built USD `world_stage`, or the map
  SDF's static STL meshes converted to USD at runtime), imports the robot
  URDF, steps physics, and streams state to the parent over stdin/stdout
  (line-delimited JSON).

The virtualenv and all caches live on the SSD and never touch the eMMC:

```bash
# one-time setup (uv bootstraps Python 3.12 on the SSD)
export UV_PYTHON_INSTALL_DIR=/workspace/uv/python
uv venv /workspace/isaac_env --python 3.12 --seed
/workspace/isaac_env/bin/python -m pip install \
    'isaacsim[all,extscache]==6.0.1.0' \
    --extra-index-url https://pypi.nvidia.com --resume-retries 10
```

The spawner parameter `isaac_python` (default `/workspace/isaac_env/bin/python`)
points at the runtime interpreter.  If it is missing — or if Kit aborts on this
platform — the spawner logs a clear message and the rest of the launch graph
continues in offline mode.

> **Platform note (2026-09):** Isaac Sim aarch64 builds are supported on NVIDIA DGX Spark and Jetson AGX Orin. On Jetson, the GUI renders via Vulkan on the integrated GPU. Note that rviz2 cannot share the GPU with Isaac Sim — when using Isaac Sim with `gui:=true`, rviz2 will report `GLXBadDrawable` (expected); use Isaac Sim's native viewport instead.

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
