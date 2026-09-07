# Robot Lab support matrix

Last updated: 2026-09-07. Baseline reviewed: `dff388f`.

This matrix reports implementation and evidence, not registry maturity labels.
No new full missions, GUI sessions or hardware operations were performed by the
audit. See [audit evidence](audit-2026-09-07.md),
[architecture](../architecture/overview.md), [ROADMAP.md](../../ROADMAP.md) and
[agent handoff](../AGENT_HANDOFF.md).

## How to interpret support

| Stage | Establishes | Does not establish |
|---|---|---|
| Cataloged | Metadata or vendored assets exist | Installed launch, physics or algorithm execution |
| Static-checked | Selected schema/asset/profile assertions pass | Real sensor traffic or a completed task |
| Physics-checked | A specific engine case steps physics | Robot control, ROS contracts or navigation |
| Partial runtime implementation | Launch/node/control code exists, with gaps | Qualification of every advertised combination |
| Scenario-qualified | A pinned task completes with recorded contract checks | Other robots/maps/backends/modes |
| Benchmarked | Repeated measured runs with provenance and valid metrics | Fairness of unrelated scenarios or fixture results |

These are documentation/evidence distinctions, not new YAML schema values.
Existing registry `integrated` labels are overbroad and do not substitute for
scenario qualification. The audit did not freshly establish scenario-qualified
or benchmarked status for a complete platform combination.

## Platform and simulator support

Ubuntu 22.04 and ROS 2 Humble are baseline targets. The workspace contains
arm64/Jetson setup history; portability and performance still require reproducible
checks. One machine's cached installation does not prove universal support.

| Backend/component | Implementation/evidence | Current limit |
|---|---|---|
| Gazebo Harmonic | Primary launch/control path, robot/world assets, historical headless world-load notes | Strongest reference route; no fresh end-to-end navigation or all-world recertification |
| PyBullet | Spawner, wheel command bridge, state/scan output; selected tests include basic real physics | Partial runtime implementation, not complete ROS/navigation qualification |
| MuJoCo | Model import, wheel-actuation bridge, state/scan output; selected tests include basic real physics | Partial runtime implementation, not complete ROS/navigation qualification |
| Isaac Sim | ROS spawner plus separate runtime, stage/robot import and state streaming; historical installation/boot notes | Experimental; no fresh native startup/mission test, no scan/RGB-D publisher, offline fallback is not physics evidence |
| ORB-SLAM3 | Optional external-library wrapper | Requires compatible dependencies including OpenCV/cv_bridge ABI; not required for reference simulation |

Historical versions, Docker image sizes and host paths are installation history,
not current qualification. The old claim that all four engines have a complete,
live-verified ROS contract is not supported by this audit.

### Backend interface matrix

“Implemented” below describes code, not a passing complete runtime contract test.

| Interface | Gazebo route | PyBullet | MuJoCo | Isaac |
|---|---|---|---|---|
| Clock | Gazebo/ROS bridge route; verify advancement | Wrong type: `Time` on `/clock` | Wrong type: `Time` on `/clock` | Wrong type: `Time` on `/clock` |
| Commands | `ros2_control` plus legacy controller/multiplexer | `/cmd_vel` to differential-drive wheels | `/cmd_vel` to wheel velocity actuators | `/cmd_vel` to runtime/wheel code; general control unqualified |
| Odometry | Controller odometry and local EKF configuration | Simulator base state on `/odom` | Simulator body state on `/odom` | Runtime state on `/odom` |
| IMU | Description/plugin/configuration assets | Implemented from simulator state | Implemented from simulator state | Implemented from runtime state |
| 2D scan | LiDAR assets for compatible robots | Raycast output implemented | Raycast output implemented | No publisher in current spawner |
| RGB/depth/camera info | Bumperbot sensor assets and RTAB-Map configuration | Not implemented in current bridge | Not implemented in current bridge | Not implemented in current bridge |
| Joint states | Controller/plugin route | Implemented | Implemented | Implemented |
| TF | Description/controller/estimator routes; ownership needs checks | Publishes `odom → base_footprint`; estimator conflict risk | Same | Same |
| Ground truth vs measurements | Separation needs experiment-level verification | Truth reused as odometry; independent measurement/truth contract unqualified | Same limitation | Same limitation |
| Reset/seed/readiness | Foundations; lifecycle not recertified | End-to-end contract unqualified | End-to-end contract unqualified | End-to-end unqualified; fallback must be explicit |

The clock must be `rosgraph_msgs/msg/Clock`, not `builtin_interfaces/msg/Time`.
Topic existence does not prove consumers can use its type or timing. Non-Gazebo
bridges use absolute topics. The legacy EKF consumes
`/robot_lab_controller/odom`, while they publish `/odom`; joystick multiplexer
output also differs from their command subscription. Repair these before claiming
backend equivalence.

## Robot support

Catalogs contain 20 robots: five labeled integrated and 15 cataloged. The main
profile file has 17 entries, with some different IDs. Promote only exact
robot/backend/task combinations with evidence.

| Robot/class | Assets and code | Main profile modes | Assessment / next evidence |
|---|---|---|---|
| Bumperbot / mobile | Description, LiDAR/RGB-D, wheel control, localization, mapping, Nav2, cleaning | `display`, `loc`, `slam`, `3d_slam`, `nav` | Strongest Gazebo reference path; recertify seeded navigation, contracts and measured result |
| Labbot / mobile | Lightweight description, LiDAR, differential-drive config, static checks | `display`, `loc`, `slam`, `nav` | Partial; needs its own control/localization/navigation mission |
| Go2 / legged | Description, 12 effort-joint assets, IMU/RGB/odometry config, commander | `display` under `unitree_go2` | Joint assets are not walking; qualify locomotion, falls and traversal |
| Berkeley Humanoid Lite / humanoid | Description, 22 position-joint/standing assets, IMU/estimated-odometry code | `display`; biped variant also display-only | Standing commands are not verified balance/walking; needs stability and task qualification |
| Quadrotor SITL / aerial | Description, sensor assets, MAVROS offboard-related code | Advertises all five modes | Overbroad; no verified autopilot/SITL takeoff–waypoints–landing route; Nav2 modes do not establish flight |
| Other Unitree/legacy robots, including manipulator assets | Descriptions, meshes and metadata | Generally `display` or no matching profile | Catalog/model availability only; individual spawn/actuation/task qualification needed |

Selecting an engine does not supply a matching gait, whole-body controller or
flight controller. Current non-Gazebo command paths are wheel-oriented. Display
semantics differ: Gazebo selection opens RViz; other selections launch a
simulator viewer with `gui=true`.

## Modes and user-facing workflows

| Workflow | Present | Limit / qualification prerequisite |
|---|---|---|
| Model display | RViz and backend viewer routes | Assets do not prove control; non-Gazebo display not headless-safe by default |
| Known-map localization | AMCL plus EKF | Topic/frame/time agreement and accuracy against separate truth |
| 2D SLAM | SLAM Toolbox configuration | Actual scan/odometry, TF, map consistency and completed task |
| 3D SLAM | RTAB-Map configuration | RGB-D/camera info unavailable through current non-Gazebo bridges |
| Navigation | Nav2, default SmacPlanner2D and Regulated Pure Pursuit | Reference task with explicit outcome and collision evidence |
| Frontier mapping / cleaning | Substantial controller logic plus basic vacuum package | Reconcile duplication; measure coverage, completion and safety |
| GUI | Profiles, browser, drive/map tools, benchmark/tests/health tabs, monitor | Not exercised in audit; divergent configuration sources; `algorithm` does not change launched stack |
| Independent experiment CLI | Queries, validation, configuration output | `robot-lab launch` execution unfinished; component launch construction fails |
| Multi-robot | Namespace-related foundations | No concurrent qualification; absolute topics/TF prevent assuming isolation |

Current allowlists accept combinations beyond runtime capabilities. Unsupported
combinations must fail before startup rather than appear successful without data.

## Algorithm coverage

There are 43 entries: 39 labeled integrated and four cataloged. This is not five
distinct working alternatives per category.

| Category | Entries | Limits |
|---|---:|---|
| Perception | 8 | Includes conversion/calibration utilities and obstacle/cluster/segmentation examples; new ROS entry-point integration has defects |
| Localization | 6 | AMCL, RTAB-Map and EKF-related integrations plus examples; not six qualified interchangeable methods |
| State Estimation | 5 | Numerical approximations and empty-node wrappers; pose-graph-labeled example is not graph optimization |
| Sensor Fusion | 5 | Filters and republishers; real I/O, covariance semantics and comparative measurement incomplete |
| Global Planning | 5 | Dijkstra, A*, NavFn, RRT, Voronoi-style entries; correctness, feasibility and selection-to-execution require verification |
| Local Planning | 5 | TEB, DWB, pure pursuit, PD follower, follow-the-gap entries; not five qualified interchangeable planners |
| Control | 9 | Relays, utilities/applications, standing/joint/aerial code; completed comparable MPC/nonlinear/LQR breadth absent |

Every method needs numerical correctness, installed working I/O, compatibility,
scenario completion and measured comparison. Utilities, aliases and metadata
checks cannot fill the five-per-category learning goal.

## Environment support

There are 26 entries, all labeled integrated. Treat this as inventory until each
relevant backend/robot task has evidence.

| Group | Count | Assets | Evidence limit |
|---|---:|---|---|
| Legacy/general | 14 | Small office/house/warehouse, warehouse demo, bigger warehouse, empty, residential demo, simple box, six Celisca variants | Asset/profile checks and historical load notes; not all combinations mission-tested |
| Navigation arenas | 5 | `nav_empty`, `nav_obstacle`, `nav_maze`, `nav_narrow_passage`, `nav_warehouse` | Generators and world/occupancy tests; require real reset/seed/task validation |
| Terrain | 3 | `outdoor_terrain`, `terrain_stairs`, `terrain_stepping_stones` | Geometry does not prove traversal; old `terrain_rough` label refers to `outdoor_terrain` ID |
| Aerial | 2 | `aerial_course`, `aerial_indoor` | Geometry exists; no verified flight mission |
| Variants | 2 | `nav_dynamic`, `nav_sensor_degraded` | Actors and occlusion; not a general sensor noise/dropout framework |

Spawn zones, goals and reference paths are recorded for deterministic arenas.
Import fidelity, world/map transforms, free-space spawns, actor resets and
class-appropriate tasks still need qualification. A 3D world is not a 3D planner.

## Benchmark infrastructure

| Component | Present | Required correction/evidence |
|---|---|---|
| Schema and outputs | Records, CSV/Markdown/HTML, plots | Provenance, units, missing-data handling, labeled fixtures |
| Launch/reset/run/stop | Lifecycle helper code | Fail on startup/reset error; readiness/outcomes cannot be replaced by waiting |
| Metrics and ground truth | Extraction/helper code | Remove fixed executor metrics; measure contacts, clearance, trajectory and separate truth |
| Seeds/baselines | Manifest/reference/regression utilities | Apply seeds to actual engines/algorithms, repeat and store raw artifacts |
| CLI and GUI | Commands/UI exist | ROS executable discovery broken in audited install; no trusted single-command path |
| Comparison | Reporting/threshold logic | No validated end-to-end comparative benchmark established |

Do not publish placeholder scores. A zero collision count without a contact
measurement source is unknown, not collision-free.

## Test evidence

This is the recorded audit result, not a live CI badge or the result of this
documentation edit. Exact selection/exclusions belong in the linked audit.

| Check | Recorded result | Scope |
|---|---|---|
| Package discovery | 26 | Includes optional ORB-SLAM3 |
| Selected source tests | 485 passed, 1 failed | Many metadata/static/numerical checks; failure constructs `DeadReckoning` without ROS initialization |
| Selected backend tests | 5 passed | Includes basic PyBullet/MuJoCo physics, not navigation |
| Registry cross-references | Passed | Not compatibility correctness |
| Adversarial composition checks | Invalid combinations accepted | Unknown simulator/wrong-category validation gaps |
| Excluded runtime tests | Nine launch/reset/recording and one Isaac startup case | Not run; not passing evidence |
| Stored colcon results | Contain failures | Not fresh clean-build certification |
| Full missions / GUI / hardware | Not exercised | No new end-to-end qualification |

The old “257 passing, zero failures, CI passing” summary was stale. CI excludes
pushes to active `master` and suppresses backend failures with `|| true`; bootstrap
also suppresses dependency failures. Repair gates and publish dated logs before
claiming green CI.

## Known Limits

- Hardware HIL: physical Bumperbot validation remains separate and blocked pending
  the device and safe operator setup; simulation checks do not authorize actuation.
- Cross-backend clock, command, odometry, TF, sensor and namespace contracts differ;
  missing sensors must block incompatible modes.
- Registry maturity and launch profiles disagree. Legged/humanoid locomotion and
  aerial SITL flight are not qualified.
- Algorithm selection, several wrappers and numerical methods need implementation
  or correctness work.
- Benchmark lifecycle, outcomes and metrics are not trustworthy for comparison yet.
- Optional-engine boot/offline fallback is not real physics or task evidence;
  machine-specific installation is not portability.
- ORB-SLAM3 dependency/ABI compatibility needs separate checks.
- Real-time/resource performance, seeded reliability and multi-robot isolation
  have not been established by this audit.

## Promotion and maintenance checklist

Follow [ROADMAP.md](../../ROADMAP.md). For each promotion record the robot,
backend/version, environment, scenario, algorithms/params, revision, seed and
machine; attach commands and raw logs/results. Verify clock/sensor/action traffic,
TF ownership, command timeout, reset/repeatability, classified outcome and measured
metrics. Update this matrix, roadmap and machine status together.

Promote individual tested combinations; keep untested ones explicit. Do not turn
asset counts, node startup, offline fallback, fixtures or passing metadata tests
into platform-wide support claims.
