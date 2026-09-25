# Robot Lab support matrix

Last updated: 2026-09-25. Current source revision: `d06a411`.
The 2026-09-07 audit at `dff388f` remains the historical selected-test
baseline. Later evidence includes PyBullet/MuJoCo live drive, RGB-D and reset
smokes (2026-09-14), Isaac Sim sensor/drive/reset and five-seed R4 mission
(2026-09-16), R5.1 bounded mobile missions (2026-09-24), R5.2 Go2 stance and
opt-in policy trials, the measured R5.2 feed-forward/inverse reverse A/B, the R5.2 five-case flat-ground screening suite, and the R5.3 BHL contact-fidelity audit (2026-09-25).
The current fast check is 465 passed/1 skipped; the latest map suite is 35
passed. These scoped results do not certify all combinations.

This matrix reports implementation and evidence, not registry maturity labels.
It distinguishes the dated 2026-09-07 audit from later scoped runtime evidence.
No new universal mission, GUI-session or hardware qualification is claimed.
See the [workflow](../WORKFLOW.md), [audit evidence](audit-2026-09-07.md),
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
scenario qualification. The 2026-09-07 audit did not freshly establish scenario-qualified
or benchmarked status for a complete platform combination. Later exact-cell
records are listed explicitly below; no universal promotion is implied.

## Platform and simulator support

Ubuntu 22.04 and ROS 2 Humble are baseline targets. The workspace contains
arm64/Jetson setup history; portability and performance still require reproducible
checks. One machine's cached installation does not prove universal support.

| Backend/component | Implementation/evidence | Current limit |
|---|---|---|
| Gazebo (Ignition Fortress 6.18) | Primary launch/control path, robot/world assets, historical headless world-load notes. The launch executes `ign gazebo` 6 through Humble's binary `ros_gz_sim` 0.244 (verified 2026-09-14 from the running process name and the `ign-gazebo-6` plugin path); Gazebo Harmonic (gz-sim 8.15) is also installed but is not what this launch path runs | Strongest reference route; no fresh end-to-end navigation or all-world recertification |
| PyBullet | Spawner, wheel command bridge, state/scan/RGB-D output; selected tests include basic real physics. 2026-09-11: loads all 17 robot descriptions and materialises static geometry for all 26 worlds (boxes/spheres/cylinders/planes/meshes, `model://` includes resolved, Collada staged to STL). 2026-09-14: live headless smoke passed (`test_pybullet_runtime.py` with `ROBOT_LAB_RUN_PYBULLET_SMOKE=1`) - bumperbot drives in the commanded direction (0.2 m/s commanded, avg 0.199 m/s over 4.5 s sim), RGB-D 320x240 rgb8+32FC1 at ~5 Hz sim cadence via a dedicated mirror render client, reset restores pose/velocity/command to 3.8e-5 m; Collada `<unit meter>` scaling fixed so world meshes materialise at metre scale. 2026-09-15: full R4 `point_to_point_navigation` missions run live through `robot-lab-live-mission` - 5 evaluation seeds (1101-1105) all reach the 1.5 m goal in `nav_empty` with truthful `success` outcomes, 0 contact events, 1.3535 +/- 0.0014 m measured trajectory, mean 5.45 s sim, mean RTF 0.349, and real rosbag2 captures of 801-804 msgs/run | Missions are qualified on one open world only (R6.1 spawn clearance still open: warehouse spawn zones can intersect static geometry now meshes are metre-correct); `nav_empty` puts no geometry in scan range, so avoidance, collisions and non-trivial clearance are unexercised; startup under-travel until wheel slip settles; not yet compared against the canonical Gazebo R4.3 reference arm |
| MuJoCo | Model import, wheel-actuation bridge, state/scan/RGB-D output; selected tests include basic real physics. 2026-09-11: all 17 robots import with their assets and a floating base; MJCF worlds generated for all 26 maps. 2026-09-14: live headless smoke passed (`test_mujoco_runtime.py` with `ROBOT_LAB_RUN_MUJOCO_SMOKE=1`, MuJoCo 3.12.0) - bumperbot drives 0.285 m in 1.5 s sim at 0.2 m/s commanded with -0.003 m lateral drift, RGB-D 320x240 rgb8+32FC1 (finite-depth fraction 1.0), reset restores pose to 1.3e-5 m. 2026-09-15: full R4 `point_to_point_navigation` missions run live through `robot-lab-live-mission` - 5 evaluation seeds (1001-1005) all reach the 1.5 m goal in `nav_empty` with truthful `success` outcomes, 0 contact events, 1.3531 +/- 0.0027 m measured trajectory, mean 5.54 s sim, mean RTF 0.378, and real rosbag2 captures of 814-860 msgs/run | Missions are qualified on one open world only (R6.1 spawn clearance still open: warehouse spawn zones can intersect static geometry now meshes are metre-correct); `nav_empty` puts no geometry in scan range, so avoidance, collisions and non-trivial clearance are unexercised; not yet compared against the canonical Gazebo R4.3 reference arm |
| Isaac Sim | ROS spawner plus a separate runtime under Isaac's Python 3.12. Named host recorded 2026-09-16 ([`evidence/r82-isaac-jetson-2026-09-16/host.json`](evidence/r82-isaac-jetson-2026-09-16/host.json)): Jetson AGX Orin, L4T R36.5.2, Isaac Sim 6.0.1, PhysX on the CPU (its CUDA module fails to load with error 222). 2026-09-14: installation auto-detected; boots with Isaac's bundled `libnvJitLink` preloaded; through `ros2 launch` the runtime reaches ready (245 s cold, 30 s warm), publishes /joint_states, /odom/ground_truth and /clock at 50 Hz, builds SDF worlds as USD collision prims, and stops cleanly on SIGINT in 2 s with no Kit process left. 2026-09-16: the committed `/scan` and RGB-D publishers are verified live against the map ray-cast (360/360 rays within 5 cm, centre depth 7.340 m vs 7.338 m), root-body odometry is re-expressed at the URDF root, reset returns to the start pose (0.000 m) with a monotonic clock, the wheel-drive instability is fixed (rotor armature plus the description's own collision geometry: 0.298 m/s at 0.3 commanded, yaw 0.587 rad/s at 0.6), and a live R4 `point_to_point_navigation` mission runs five evaluation seeds in `nav_empty` ([`evidence/r82-isaac-mission-2026-09-16/`](evidence/r82-isaac-mission-2026-09-16/README.md)): 5/5 `success`, 0 contact events, 1.3507 +/- 0.0004 m measured trajectory, mean 5.44 s sim, mean RTF 0.131 | Bumperbot only; `nav_empty` puts no geometry in scan range and the mission controller consumes `/odom/ground_truth`, so no localization stack is qualified on Isaac; `/scan` is published every 12 physics steps (27 msgs/run), too slow for scan-driven autonomy without its own check; RTF 0.131 with PhysX on the CPU; the ~15 % wheel/body speed gap from 2026-09-14 is not isolated |
| ORB-SLAM3 | Optional external-library wrapper | Requires compatible dependencies including OpenCV/cv_bridge ABI; not required for reference simulation |

Historical versions, Docker image sizes and host paths are installation history,
not current qualification. The old claim that all four engines have a complete,
live-verified ROS contract is not supported by this audit.

### Backend interface matrix

“Implemented” below describes code, not a passing complete runtime contract test.

| Interface | Gazebo route | PyBullet | MuJoCo | Isaac |
|---|---|---|---|---|
| Clock | Gazebo/ROS bridge route; verify advancement | `rosgraph_msgs/msg/Clock` on `/clock`; monotonic/reset checked | `rosgraph_msgs/msg/Clock` on `/clock`; monotonic/reset checked | `rosgraph_msgs/msg/Clock` on `/clock`; monotonic/reset checked |
| Commands | `ros2_control` plus legacy controller/multiplexer | `/cmd_vel` to differential-drive wheels | `/cmd_vel` to wheel velocity actuators | `/cmd_vel` to differential-drive wheel velocity drives (damped, with rotor armature); legged/other control unqualified |
| Odometry | Controller odometry and local EKF configuration | Base state on `/odom/ground_truth`, twist in the body frame | Body state on `/odom/ground_truth` (scalar-first quaternion converted, body-frame twist) | Root-body state re-expressed at the URDF root on `/odom/ground_truth`, body-frame twist |
| IMU | Description/plugin/configuration assets | Implemented from simulator state | Implemented from simulator state | Implemented from runtime state |
| 2D scan | LiDAR assets for compatible robots | Batched ray tests from the laser link frame; 360/360 rays within 5 cm of the map (nav_maze, 2026-09-16) | Rays from the laser link pose, skipping the robot's own bodies; 360/360 within 5 cm | PhysX raycasts from the laser link, ignoring robot bodies; 360/360 within 5 cm |
| World geometry | SDF worlds loaded natively | Parsed with the shared `sdf_world.py` reader (same as Isaac): primitives + meshes, `model://` resolved, Collada staged to STL (26/26 maps produce geometry; pre/post-refactor loaders equivalent within 1e-4 m) | MJCF generated from the same SDF by `robot_lab_maps/tools/gen_mjcf_worlds.py` (26/26 maps) | Spawner parses the SDF with `sdf_world.py`; the runtime creates USD collision prims: 26/26 maps build (box bounds within 1e-6 m), nav_maze verified live |
| Robot import | Native URDF/xacro; 17/17 robots spawn live (`OK creation of entity`) | 17/17 robot descriptions load; 17/17 spawn live through `ros2 launch` with world geometry | 17/17 import with assets, defaults and a floating base; 17/17 spawn live through `ros2 launch` | Bumperbot imports and reaches ready live; other robots not yet run through Isaac |

Import is not stable simulation, so stepping was measured separately: all 17
robots survive a 2 s passive drop in MuJoCo with no `QACC` blow-up
(2026-09-14). The earlier `Nan, Inf or huge value in QACC` warnings came from
collision meshes that interpenetrate at the rest pose (Unitree H1-2 thumbs
inside the wrists, Unitree B1 thighs inside the trunk); those pairs are now
excluded automatically. Passive stability is not locomotion: treat non-Gazebo
backends as display/visualization-grade for legged and humanoid robots until a
class mission passes.
| RGB/depth/camera info | Bumperbot sensor assets and RTAB-Map configuration | OAK-D topics (320x240 rgb8, 32FC1 metres, pinhole camera_info) from the software renderer; centre depth 7.342 m vs 7.342 m ray-cast from the map | Same topics from an MJCF camera on the base; centre depth 7.339 m vs 7.339 m | Same topics from a USD camera plus replicator annotators, default lights added to unlit worlds; centre depth 7.340 m vs 7.338 m |
| Joint states | Controller/plugin route | Implemented | Implemented | Implemented |
| TF | Description/controller/estimator routes; ownership needs checks | Simulator does not publish estimator-owned `odom → base_footprint`; controller/estimator ownership is tested separately | Same | Same |
| Ground truth vs measurements | Separation needs experiment-level verification | Dedicated truth stream; controller odometry is separate | Same limitation | Same limitation |
| Reset/seed/readiness | /robot_lab/ready + /robot_lab/health + /robot_lab/reset contracts declared and tested (R2.3) | Reset restores the spawn pose (0.000 m) with a monotonic clock | Same | Reset now reaches the runtime (`world.reset()`; previously acknowledged but ignored): 0.000 m, monotonic clock; offline reports WARN health |

The non-Gazebo bridges now use the typed simulation clock and separate truth from
controller odometry. Topic existence alone still does not prove that a
consumer has compatible QoS, frame, timestamp or estimator semantics. Absolute
topic names and class-specific command paths remain exact-cell checks.

## Robot support

Catalogs contain 20 robots: five labeled integrated and 15 cataloged. The main
profile file has 17 entries, with some different IDs. Promote only exact
robot/backend/task combinations with evidence.

| Robot/class | Assets and code | Main profile modes | Assessment / next evidence |
|---|---|---|---|
| Bumperbot / mobile | Description, LiDAR/RGB-D, wheel control, localization, mapping, Nav2, cleaning | `display`, `loc`, `slam`, `3d_slam`, `nav` | Strongest Gazebo reference path; recertify seeded navigation, contracts and measured result |
| Labbot / mobile | Lightweight description, LiDAR, differential-drive config, static checks | `display`, `loc`, `slam`, `nav` | Partial; needs its own control/localization/navigation mission |
| Go2 / legged | Description, 12 effort-joint assets, IMU/RGB/odometry config, guarded stance controller and opt-in flat-ground ONNX policy | `display`; explicit Go2/MuJoCo/localization policy checkbox | Partial live evidence: short stance, forward/stop, turn, command-loss stop, direct foot contacts and two five-case flat-ground screening suites. The opt-in inverse map still leaves -0.15 m/s in a deadband; `terrain_stairs` fails at the first ledge with 0.115 m displacement and 0.527 rad peak tilt. Fall recovery and navigation remain unqualified |
| Berkeley Humanoid Lite / humanoid | Description, 22-joint effort/standing assets, IMU/odometry, effort policy and safety diagnostics | `display`; opt-in effort-policy route | Partial live stance/startup evidence. Held walking/turning stalls at a policy fixed point; rate/filter/contact tuning was negative. Target-domain retraining or a new measured hypothesis is required before walking/terrain claims |
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
| 3D SLAM | RTAB-Map configuration | RGB-D/camera info now published by the PyBullet, MuJoCo and Isaac bridges (depth checked against the map); no RTAB-Map mapping run qualified on any backend |
| Navigation | Nav2, default SmacPlanner2D and Regulated Pure Pursuit | Reference task with explicit outcome and collision evidence |
| Frontier mapping / cleaning | Substantial controller logic plus basic vacuum package | Reconcile duplication; measure coverage, completion and safety |
| GUI | Profiles, browser, drive/map tools, benchmark/tests/health tabs, monitor | Composition preview/validation and command autofill are tested; a GUI-launched mission still needs exact readiness, cleanup and artifact evidence |
| Independent experiment CLI | Typed resolver, validation, manifest output and live execute path | Default dry-run is not a mission; each executed robot/backend/map/task cell still needs scenario evidence |
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

Navigation arenas are generated from the same SDF sources used by the bridge.
When an SDF world already contains a ground collision, the MuJoCo fallback
plane is visual-only; this prevents duplicate foot contacts. Geometry tests
and generator checks pass, but this plant-parity fix does not qualify humanoid
locomotion or terrain traversal.

## Benchmark infrastructure

| Component | Present | Required correction/evidence |
|---|---|---|
| Schema and outputs | Records, CSV/Markdown/HTML, plots | Provenance, units, missing-data handling, labeled fixtures |
| Launch/reset/run/stop | Lifecycle helper code | Fail on startup/reset error; readiness/outcomes cannot be replaced by waiting |
| Metrics and ground truth | Extraction/helper code | Remove fixed executor metrics; measure contacts, clearance, trajectory and separate truth |
| Seeds/baselines | Manifest/reference/regression utilities | Apply seeds to actual engines/algorithms, repeat and store raw artifacts |
| CLI and GUI | Shared typed resolver, validation, manifest output and command autofill | Each live cell still needs readiness, task outcome, cleanup and artifact checks |
| Comparison | Reporting/threshold logic | No validated end-to-end comparative benchmark established |

Do not publish placeholder scores. A zero collision count without a contact
measurement source is unknown, not collision-free.

## Test evidence

This table combines the dated audit with later scoped checks. Exact historical
selection and exclusions belong to the [audit](audit-2026-09-07.md); current
rows name their revision and scope explicitly.

| Check | Recorded result | Scope |
|---|---|---|
| Package discovery | 26 | Includes optional ORB-SLAM3 |
| Selected source tests | 485 passed, 1 failed | Historical 2026-09-07 audit at `dff388f`; failure constructed `DeadReckoning` without ROS initialization |
| Current fast suite | 466 passed, 1 skipped | `scripts/test_fast.sh` at `d06a411`; includes registry cross-reference validation |
| Current map suite | 35 passed | `robot_lab_maps` after generated-world contact fix; generator `--check` also passes |
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
  missing sensors and class-specific controllers must block incompatible modes.
- Registry maturity and launch profiles disagree. Go2 locomotion and BHL walking
  are partial; aerial SITL flight is not qualified.
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
