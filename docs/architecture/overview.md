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
  maps/                     # worlds, occupancy maps, reference geometry
  gazebo_models/            # reusable environment models
  bumperbot_*/              # reference robot and legacy-compatible adapters
  ORB_SLAM3/                # optional external-library adapter
```

The initial reorganization is additive. Moving working packages merely to make
the directory tree look tidy would break paths while providing no runtime
isolation. Package ownership is separated first; source moves can occur only
after launch and CI consumers use the canonical interfaces.

## Package boundaries

| Layer | Owns | Must not own |
|---|---|---|
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

### Environment

Required metadata includes simulator/world reference, dimensionality, tags,
map/ground-truth availability, dynamics, supported robot classes, spawn zones,
source/provenance, and qualification status.

### Algorithm

Required metadata includes category, family, implementation package/plugin,
status, input/output contract, required capabilities, supported robot classes,
upstream source, and smoke/benchmark evidence. Algorithm entries are adapters;
an upstream project name alone is not an integration.

### Scenario

A scenario defines the task and stopping conditions independently of a map: for
example fixed-start waypoint navigation, relocalization after pose loss,
coverage, rough-terrain traversal, or aerial inspection.

### Experiment

An experiment pins one item in every required dimension plus parameters, seed,
time limit, metrics, and artifact policy. It is the unit of smoke testing and
benchmarking.

## Runtime contracts

All adapters will support a namespace. The reference single-robot contract is:

| Contract | Interface |
|---|---|
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

## Standard result outline

P6 will version the exact schema. Every result must at minimum include:

- repository revision and dirty-state marker;
- ROS, simulator, host architecture, and dependency versions;
- full resolved experiment and parameter hashes;
- seed, start/goal, timing, and termination reason;
- success, collisions, path length, elapsed simulation time, real-time factor,
  minimum clearance, and resource usage;
- pose/trajectory error when ground truth is available;
- artifact paths for logs, bags, maps, trajectories, and plots.

This provenance is required so a visually successful demo is not mistaken for
a reproducible comparison.
