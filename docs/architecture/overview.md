# Robot Lab platform architecture

## Design rule

An experiment is a validated composition, not a monolithic mode:

```text
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

Each selector is independent in the canonical model. Compatibility is decided
from explicit capabilities and contracts before launch. The existing Bumperbot
`display`, `loc`, `slam`, `3d_slam`, and `nav` modes remain supported as legacy
presets while the new composition layer is built.

## Source-tree target

```text
src/
  robot_lab/
    robot_lab_registry/     # schemas, catalogs, compatibility, query CLI
    robot_lab_bringup/      # composition and legacy launch adapters (P2)
    robot_lab_benchmark/    # runner, metrics, result schema (P6)
  robots/                   # descriptions and robot-specific assets
    _upstream/              # vendored third-party robot assets
  maps/                     # worlds, occupancy maps, reference geometry
  gazebo_models/            # reusable environment models
  bumperbot_algorithms/     # 13 algorithm implementations (P5)
  bumperbot_*/              # reference robot and legacy-compatible adapters
  ORB_SLAM3/                # optional external-library adapter
```

## Package boundaries

| Layer | Owns | Must not own |
|-------|------|--------------|
| Registry | Metadata, schemas, compatibility rules, experiment presets | ROS nodes or simulator processes |
| Assets | URDF/SDF/Xacro, meshes, worlds, occupancy maps | Algorithm policy or benchmark conclusions |
| Algorithm adapter | One common contract around one implementation | Robot/world-specific orchestration |
| Bringup | Composition, namespaces, lifecycle order, parameter overlays | Algorithm implementation details |
| Benchmark | Scenario lifecycle, ground truth, metrics, artifacts | Hidden tuning unique to a compared algorithm |

## Canonical entities

### Robot

Required metadata includes class, maturity, supported simulators, locomotion,
sensors, command/state interfaces, frames, capabilities, source/provenance, and
the smoke experiments that justify its status.

**Integrated robots:**

| Robot | Class | DOF | Simulators | Source |
|-------|-------|-----|------------|--------|
| Bumperbot | Differential drive | — | Gazebo | First-party |
| Labbot | Differential drive | — | Gazebo | First-party |
| Go2 | Quadruped | 12 leg joints | Gazebo | Unitree (vendored) |
| Berkeley Humanoid Lite | Humanoid | 22 position joints | Gazebo | HybridRobotics (vendored) |
| Quadrotor SITL | Aerial | 4 rotors | Gazebo | First-party |

### Environment

Required metadata includes simulator/world reference, dimensionality, tags,
map/ground-truth availability, dynamics, supported robot classes, spawn zones,
source/provenance, and qualification status.

**Environment categories:**

| Category | Count | Examples |
|----------|-------|----------|
| Indoor worlds | 14 | small_office, small_house, warehouse_demo |
| Navigation arenas | 5 | nav_empty, nav_obstacle, nav_maze |
| Dynamic variants | 2 | nav_dynamic, nav_sensor_degraded |
| Terrain | 3 | terrain_rough, terrain_stairs, terrain_stepping_stones |
| Aerial courses | 2 | aerial_course, aerial_indoor |

### Algorithm

Required metadata includes category, family, implementation package/plugin,
status, input/output contract, required capabilities, supported robot classes,
upstream source, and smoke/benchmark evidence. Algorithm entries are adapters;
an upstream project name alone is not an integration.

**Algorithm coverage (30 algorithms, 7 categories):**

| Category | Count | New (P5) | Legacy |
|----------|-------|----------|--------|
| Perception | 5 | obstacle_detector, scan_clusterer, pointcloud_segmenter | 2 |
| Localization | 5 | dead_reckoning | 4 |
| State Estimation | 5 | ekf_3d_estimator, motion_model_estimator, pose_graph_estimator | 2 |
| Sensor Fusion | 5 | wheel_imu_fusion, gps_odom_fusion, complementary_imu | 2 |
| Global Planning | 5 | rrt_planner, voronoi_planner | 3 |
| Local Planning | 5 | follow_the_gap | 4 |
| Control | 9 | mavros_offboard_controller, joint_effort_commander | 7 |

### Scenario

A scenario defines the task and stopping conditions independently of a map: for
example fixed-start waypoint navigation, relocalization after pose loss,
coverage, rough-terrain traversal, or aerial inspection.

### Experiment

An experiment pins one item in every required dimension plus parameters, seed,
time limit, metrics, and artifact policy. It is the unit of smoke testing and
benchmarking.

## Runtime contracts

All adapters support a namespace. The reference single-robot contract is:

| Contract | Interface |
|----------|-----------|
| Body command | `geometry_msgs/Twist` or class-specific trajectory beneath the robot namespace |
| Estimated state | `nav_msgs/Odometry` plus TF |
| Global pose | `map -> odom` TF where applicable |
| Local body pose | `odom -> base_link`/`base_footprint` TF |
| 2D obstacles | `sensor_msgs/LaserScan` and/or costmap |
| 3D perception | image/depth/camera-info or `sensor_msgs/PointCloud2` |
| Planned route | `nav_msgs/Path` |
| Navigation goal | Nav2 action contract for compatible ground robots |
| Ground truth | simulator adapter output, never substituted for estimated state |

Robot-class-specific contracts (joint trajectories, gait commands, aerial
setpoints) must be converted at the control boundary rather than leaking into
global planning or benchmark schemas.

## Validation layers

1. **Schema:** required fields, types, IDs, allowed statuses/categories.
2. **References:** every experiment selector resolves to a catalog entry.
3. **Capabilities:** robot sensors/interfaces and environment dimensionality
   satisfy every selected algorithm and scenario.
4. **Assets:** referenced package files and plugins exist.
5. **Static launch:** launch descriptions expand and dependencies resolve.
6. **Smoke:** processes become healthy, topics/actions appear, and a minimal
   task completes or fails in a classified way.
7. **Benchmark:** fixed seed and conditions produce a standard result record.

Only levels 1–4 belong in the registry package. Runtime validation lives with
bringup and benchmark packages.

## Benchmark architecture (P6)

### Standard result schema

Every benchmark produces a versioned JSON record:

```json
{
  "schema_version": "1.0",
  "experiment_id": "bumperbot_smoke_test",
  "robot_id": "bumperbot",
  "environment_id": "small_office",
  "scenario_id": "bumperbot_smoke_test",
  "seed": 42,
  "success": true,
  "elapsed_seconds": 12.5,
  "path_length_m": 18.4,
  "collision_count": 0,
  "min_clearance_m": 0.75,
  "revision": "a22c378",
  "timestamp_utc": "2026-09-01T12:00:00Z"
}
```

### Benchmark components

| Component | Package | Purpose |
|-----------|---------|---------|
| `BenchmarkResult` | `robot_lab_benchmark` | Versioned result schema |
| `LaunchOrchestrator` | `robot_lab_benchmark` | launch/reset/run/stop lifecycle |
| `GroundTruthAdapter` | `robot_lab_benchmark` | Extract metrics from sensor data |
| `MetricNormalizer` | `robot_lab_benchmark` | Normalize for fair comparison |
| `OutputGenerator` | `robot_lab_benchmark` | JSON/CSV/MD/HTML/plots |
| `ReferenceBenchmark` | `robot_lab_benchmark` | Baseline + regression checking |
| `ReferenceRegistry` | `robot_lab_benchmark` | Multi-baseline management |
| `BenchmarkRunner` | `robot_lab_benchmark` | Seeded run manifest |
| CLI | `robot_lab_benchmark/cli.py` | Emit canonical records |

### Orchestration lifecycle

```
launch  →  reset  →  run (rosbag capture)  →  stop  →  manifest.json
```

## CI/CD architecture

| Workflow | Trigger | Duration | What it runs |
|----------|---------|----------|--------------|
| `ci.yml` | Push/PR | < 60s | `scripts/test_fast.sh` (compile + P5/P6 logic + registry) |
| `scheduled-full.yml` | Daily 06:00 UTC | ~5min | Full colcon test + all 257 unit tests |

## Testing architecture

**257 tests across 4 test files:**

| Test file | Count | Coverage |
|-----------|-------|----------|
| `test_p5_algorithm_breadth.py` | 12 | Category coverage, node assets, algorithm logic, cross-references |
| `test_p6_benchmarking.py` | 65 | Schema, orchestration, ground-truth, normalization, outputs, regression, licenses, bootstrap, doctor, tutorials, support matrix |
| `test_bumperbot_qualification.py` | 28 | Bumperbot metadata, contracts, smoke test |
| `test_*.py` (other) | 152 | Registry, selectors, launch fragments, adapter, environments, robots |

## Provenance and licensing

- **License:** MIT (see `LICENSE`)
- **Third-party assets:** Unitree (BSD 3-Clause), Berkeley Humanoid Lite (CC BY-SA 4.0)
- **License tracking:** `LICENSES/third-party-notices.md` documents all external assets
- **Tests verify:** Every upstream asset has a license file and is documented
