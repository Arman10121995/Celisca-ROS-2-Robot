# Robot Lab — Multi-Robot Simulation and Algorithm Laboratory

Robot Lab aims to make robots, simulators, maps and algorithms independently selectable so that you can learn how they work and compare their performance under reproducible conditions.

**Current state: partial research platform with measured mobile, flight and native Panda workflows.** The GUI, simulator adapters, installed assets and algorithm framework support the named experiments below. Full robot/map/mode/backend qualification, controller breadth and reproducible algorithm comparisons remain open.

Status updated on **2026-10-08**. The October 8 extension checkpoint has exact
physical source manifests; its publication and software checks are recorded in
[October 8 evidence](docs/status/evidence/extensions-finish-2026-10-08/README.md). [Current status](docs/status/CURRENT_STATUS.md)
collects verified work, limits and the next tasks. The [completion audit](docs/status/audit-2026-10-02.md)
reopens unsupported R6/R7/R8/R9 completion claims. Generated metadata, simulated
success and invented performance scores do not establish working missions.
Scoped measured robot results remain in the [status ledger](docs/status/platform-status.yaml).

PX4 trial logs were moved to the workspace SSD with verified content and
compatibility links, freeing **15.79 GiB** on internal storage. Use the
[storage guide](docs/STORAGE.md) for new builds/trials.
The 2026-09-30 MuJoCo localization regression is repaired in the current
source: BHL and Go2 effort loops preserve feedback cadence, and GUI
commands preserve the configured map/robot spawn. Sequential Celisca floor-1
repeats walked upright. Go2 requires the GUI's flat-ground policy checkbox;
BHL's walking policy defaults on. See the
[before/after traces and instructions](docs/status/evidence/r52-r53-mujoco-regression-2026-09-30/README.md).


The GUI provides **PX4 X500 Flight**, Takeoff/Hold/Land and **Altitude Up/Down**,
with measured native flight in `nav_empty` and `nav_obstacle`. World selections
persist and commands autofill. Four-wheel work adds measured body-pose checks,
obstacle routes and selected localization/mapping/reset repairs. See the
[done / remaining checklist](docs/status/CHECKLIST.md) and
[current measured trials and remaining gaps](docs/status/continuation-2026-10-08.md)
and [flight guide](docs/tutorials/px4_x500.md). Full platform qualification
remains partial; these results apply to the named recorded cells.

**Launch & control** keeps Drive, Arm, Hand, Drone and their limits in a
separate right-hand column beside setup and command/algorithm review. Robot
category/type filters and structural tags distinguish wheel counts, leg counts,
single/dual arms, mobile manipulators and drones. The sidebar opens the other
workspaces; panels and dividers adapt to available space. See the
[GUI workspace guide](docs/tutorials/gui-workspace.md).

**Drone** now includes all seven Drive buttons beside altitude/flight actions,
with gentler 0.3 rad/s yaw. **Arm** offers **Use current tool pose**, measured
XYZ and small axis target buttons with separate Plan/Execute. Actual native
X500 flight controls and native Panda 1 cm targets, interruption/rejection and
physical grasp repeats pass in `nav_empty`. See
[current controls evidence](docs/status/evidence/extensions-finish-2026-10-08/controls/README.md).

**Registry → Preview 3D** now renders robots and worlds natively inside the
GUI, with orbit, pan, zoom and Fit. Parent families collect source variants and
components; complete models remain in Launch and isolated limbs stay nested
for inspection. See the [embedded viewer guide](docs/tutorials/registry-3d.md)
and [actual rendering/live-plant checks](docs/status/evidence/registry-3d-2026-10-07/README.md).

**Worlds** generates real occupancy PGM/YAML files. **Installed Extensions**
opens downloaded assets in the normal Launch selectors with commands filled
in automatically; it has no download buttons. On this SSD installation there
are **113 installed robot profiles** (59 native Menagerie MJCF models, 48 robot-assets
URDF variants/fragments, and six official TurtleBot3/Husky/TurtleBot4 descriptions), **14 imported dataset worlds**, and three packaged
Gazebo environment examples. Their source, dependency and import checks are
recorded separately from robot missions. See the
[installation evidence](docs/status/evidence/extensions-integrated-2026-10-05/README.md).
Worlds now exposes seed X/Y for selecting another connected room. Heightfields
use separate transformed triangle slices; hospital imports retain originals
and repair the pinned escaped fixtures and mesh frames in derived assets.
Read [the current world checks](docs/status/evidence/extensions-finish-2026-10-08/README.md).
TurtleBot 4 Standard/Lite expose Display, Localization, 2D/3D SLAM and
Navigation in all four simulators, with compatible algorithms and commands
filled in. Forty named mode trials and eight final sensor/Drive trials measure
reset/resume, actual map saves, source lidar/RGB-D frames and clear/obstacle
Nav2 goals. Other maps and vendor hazards/docking remain experiments. See the
[TurtleBot 4 guide](docs/tutorials/turtlebot4.md).
Official **TurtleBot3 Burger, Waffle and Waffle Pi** now have source-backed
physical wheels and LDS sensing, with twelve final normal GUI Drive/Stop/
publisher-loss/joint/TF trials across all four engines. Their original 3.5 m
lidar uses a named `nav_empty` spawn near geometry. Forty-eight real mode
screens enable Localization, 2D SLAM and Navigation on all four engines, with
actual map saves and clear/obstacle goals. RGB-only source cameras leave 3D
SLAM pending. See the [TurtleBot3 guide](docs/tutorials/turtlebot3.md).
**Husky** now drives four independent physical wheels in all four engines.
Four Drive/sensor screens and twenty localization, 2D/3D mapping/export and
clear/obstacle navigation screens enable its normal GUI modes, algorithms and
commands. This uses a declared lab sensor and motor kit; other maps, hardware
and outdoor missions require separate qualification. See [the Husky guide](docs/tutorials/husky.md).
The new **Arm** tab
controls native Panda joints on MuJoCo with measured Home, Stop, cancellation
and heartbeat loss. **Hand** adds original coupled-finger opening/closing,
bounded force, Cancel/Stop and GUI Reset. A physical 3 cm cube lift, hold and
gravity release is measured on MuJoCo/`nav_empty`; see the
[Panda guide](docs/tutorials/panda_arm.md) and
[grasp evidence](docs/status/evidence/panda-gripper-2026-10-06/README.md).
**Cartesian Plan / Execute** adds real MoveIt KDL/OMPL/FCL planning through the
native Panda actuators, with measured TCP targets, collision rejection,
stale-plan invalidation, Stop/watchdog and reset in static `nav_empty`.
The [October 7 control-column regression](docs/status/evidence/panda-cartesian-controls-column-2026-10-07/README.md)
passes after the shared command/spawn and catalog changes and refreshes its strict proof.
Cross-backend qualification, remaining vendor/example imports, Servo,
planned payload/grasp setups and mobile-manipulator control remain in the [roadmap](ROADMAP.md) and
[extension guide](docs/ASSET_EXTENSION_GUIDE.md).

## Start here

- [Current status](docs/status/CURRENT_STATUS.md): published source, available GUI workflows, measured robot scopes and remaining work.
- [Completion audit](docs/status/audit-2026-10-02.md): corrected claims and evidence boundaries; earlier audits remain historical.
- [Implementation roadmap](ROADMAP.md): ordered work, dependencies and acceptance criteria toward the full platform.
- [Agent handoff](docs/AGENT_HANDOFF.md): how to resume, claim work, avoid conflicts and record evidence.
- [Robot readiness execution guide](docs/AI_ROBOT_READINESS_GUIDE.md): finish physical motion, saved mapping and navigation by exact model/backend/map; upstream reuse, probes and guarded promotion.
- [Priority patch guide](docs/PATCH_EXECUTION_GUIDE.md): current fixes and detailed execution/acceptance steps before further expansion; also available in GUI Health.
- [Worlds and robot extension guide](docs/ASSET_EXTENSION_GUIDE.md): source pins, occupancy generation and concrete terrain/manipulation implementation contracts.
- [Done / remaining checklist](docs/status/CHECKLIST.md), [machine-readable status](docs/status/platform-status.yaml) and [latest measured trials](docs/status/continuation-2026-10-08.md): scoped state, evidence and release blockers.
- [Operational workflow](docs/WORKFLOW.md): how to inspect, test, run, record and promote a simulation result.
- [Architecture](docs/architecture/overview.md) and [tutorials](docs/tutorials/index.md): current wiring, target contracts and learning material.

## What exists, and what that means

| Source/installed inventory on this Jetson, October 7 | Meaning |
|---|---|
| 26 discoverable ROS packages | Current CI builds 25; optional `orbslam3` is excluded |
| 24 core + 113 installed extension launch profiles | 137 raw profiles, including components/source variants; availability depends on installed SSD assets |
| 72 complete robot families / 112 selectable variants | Consolidated Launch choices; 25 inspection profiles stay nested/out of standalone Launch |
| 26 core + 17 installed world profiles | 43 raw profiles under 33 map families |
| 139 robot records / 48 algorithms / 23 scenarios / 21 experiments | Merged registry inventory; entries and labels are separate from mission qualification |
| Four simulator routes | Gazebo, PyBullet, MuJoCo and Isaac; support is robot/task/backend-specific |

### Verification snapshot

Published `4fe611a` passes required CI and Scheduled Full Test Suite after the
MuJoCo 3.15 world-contact compatibility fix: 25 core packages, 27 physics and
134 integration checks (13 skips), plus registry validation. Required fast:
1002 pass / four skips; scheduled fast: 1003 pass / three skips; both four
deselections. The predecessor `73fcc47` also passes after the shared-utilities import fix. Its [exact logs](docs/status/evidence/extensions-finish-2026-10-08/README.md)
are separate from the newer runtime trials and final checkpoint checks.
Earlier published `091d388` [remote CI](docs/status/evidence/ci-extensions-2026-10-07/README.md)
passes a 25-package core build, **933 fast checks / four skips / four
deselections**, **20 physics checks**, **134 integration checks / twelve
skips**, and registry validation. The complete log and report are retained.

The October 7 rebuilt GUI has **94 passing local GUI checks** and five additional
real-Tk layout checks. The preceding control-column stage has **936 fast
passes / one skip / four deselections** and **146 integration passes**; those
source stages remain distinct. The final normal Burger/PyBullet Drive and
native Panda physical Plan/Execute repeats pass against the actual published
GUI files. Read [GUI evidence](docs/status/evidence/gui-controls-column-2026-10-07/README.md)
and [the current continuation](docs/status/continuation-2026-10-08.md).
The October 8 local stage records 1005 fast, 104 GUI, 27 physics and 147
dedicated-display integration passes. Occupancy v4 repairs omitted open wall
chains; the current hospital map/navigation repeat is tracked in that checkpoint.

The [evidence index](docs/status/runtime-evidence-index.yaml) retains individual
hashed positive and negative trials; [the support matrix](docs/status/support-matrix.md)
recomputes the latest result for each exact measured cell.
Unlisted cells are untested and full-release gates remain blocked. Historical
CI failures, test totals and robot trials remain in their dated reports.
Software checks, static previews, clean-host reproduction and actual robot
missions are separate milestones. See [Testing](docs/TESTING.md) and
[the support matrix](docs/status/support-matrix.md).

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

A mode declares the ordered pipeline it runs, and each step that has an
`algorithm_category` is a selector. Mode categories and algorithm categories
are one taxonomy (`config/sim_modes.yaml` is the authority):

| Mode | Category | Selectable within the mode |
|---|---|---|
| `display` | Perception & Visualization | perception |
| `loc` | Localization | localization, state estimation, sensor fusion |
| `slam` | 2D Mapping & Localization | localization (SLAM backend), state estimation, sensor fusion, perception |
| `3d_slam` | 3D Mapping & Localization | localization, state estimation, perception |
| `nav` | Navigation | global planning, local planning, control, localization, state estimation, sensor fusion |
| `flight` | Native PX4 3D Flight | PX4 estimation/control, manual XYZ/yaw and 3D position goals; aerial obstacle planning remains separate |

Each of the seven registry categories is a launch argument:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav   robot_model:=bumperbot map_name:=small_office simulator:=pybullet   global_planning:=navfn_planner local_planning:=mppi_controller   localization:=amcl state_estimation:=ekf_localization_node
```

Each argument takes an algorithm ID, `auto` (the mode's declared default) or
`none` (run the mode without that stage). Selections are resolved **before any
process starts**, against
[`config/algorithm_dispatch.yaml`](src/robot_lab_bringup/config/algorithm_dispatch.yaml),
which records for every cataloged algorithm whether it starts its own node,
switches a Nav2 plugin, is provided by a stack the mode already runs, or cannot
run and why. A selection that the mode does not run, or that names an algorithm
this workspace cannot start, fails with that reason instead of being silently
replaced by a default. The CLI resolver emits the same arguments, so a dry-run
command and a GUI launch are the same command.

Display mode also accepts `robot_model:=none` (show a world on its own) and
`map_name:=none` (show a robot with no world) in every backend.

In navigation mode, use RViz's **2D Goal Pose** toolbar tool. The supplied
RViz view sends it to `/robot_lab/goal_pose`; the navigation relay timestamps
it with the simulator clock before submitting it to Nav2. Car profiles use
curvature-aware planning and path following; an explicitly selected planner
that requires turning in place is rejected for those profiles.

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

- **Bumperbot:** reference differential-drive description, sensors, control, mapping, localization, navigation and cleaning workflows. R5.1 recorded bounded forward/turn/reverse/stop/watchdog and clear/obstacle Nav2 missions on MuJoCo, with a PyBullet Bumperbot drive pass; broader backend/map coverage remains open.
- **Labbot:** differential-drive description, sensors/control and named bounded Drive/navigation/Celisca screens; remaining map/backend/mode cells and longer avoidance routes need qualification.
- **Go2:** the standard MuJoCo route has measured bounded stance, an opt-in flat-ground ONNX policy, forward/stop, turning, command-loss and direct foot-contact evidence. The opt-in inverse map passed four five-case flat-ground screening suites. The named stairs task still fails at the first ledge (`0.527 rad` peak tilt, `0.115 m` displacement). A nominal-pose re-stand attempt and a separate opt-in learned get-up actor both failed the measured 60 N collapse; the learned actor stays bounded and fails closed. Terrain traversal and navigation remain unqualified; fall recovery is now measured instead — the opt-in get-up ladder stands the robot from placed pitch collapses (4 of 4) but ends worse than leaving it off on every reachable perturbation, so it stays off by default. See the [Go2 tutorial](docs/tutorials/go2.md).
- **Berkeley Humanoid Lite:** the standard MuJoCo route has measured stance/startup-bend evidence and an effort-policy path. The held-turn/walk stall is diagnosed as a policy fixed point; torque filtering, 2 kHz physics-only and fresh intra-interval PD A/B tests did not revive sustained motion. A duplicate generated-world ground contact was fixed, but contact duplication was not the stall remedy. Target-domain retraining or a new measured hypothesis is next; walking and terrain are not established.
- **PX4 X500:** actual Gazebo Harmonic/PX4 Flight, ROS2/GUI Drive, altitude, goals, takeoff/hold/land and command loss are measured in `nav_empty` and `nav_obstacle`; other worlds and aerial autonomy remain R5.10.
- **Legacy quadrotor_sitl:** retained display fixture; the retired MAVROS example does not provide a working flight mission.
- **Other cataloged robots:** imported descriptions span legged, humanoid and manipulator models. Asset availability is not locomotion/control support.

The 26 core environments include legacy/general worlds, five navigation arenas, terrain, aerial and dynamic/occlusion entries. Seventeen installed extensions add fourteen dataset worlds and three Gazebo examples, consolidated under map families. Arena generators and occupancy-map consistency checks are useful foundations. The rough-terrain registry ID is `outdoor_terrain`, not `terrain_rough`. `nav_sensor_degraded` supplies occluding geometry, not a general noise/dropout framework.

### Algorithms and benchmarking

| Registry category | Entries | Examples or current scope |
|---|---:|---|
| Perception | 8 | Scan/cloud conversion, obstacle detection, clustering and segmentation |
| Localization | 7 | AMCL, RTAB-Map, dead reckoning and motion-model entries |
| State estimation | 5 | Kalman/EKF and educational estimator kernels |
| Sensor fusion | 5 | IMU processing and wheel/IMU/GPS fusion kernels |
| Global planning | 7 | Dijkstra, A*, NavFn, RRT and Voronoi-style planning |
| Local planning | 6 | TEB, DWB, pure pursuit, PD path following and follow-the-gap |
| Control | 10 | Wheel/joint/standing/offboard control entries and application utilities |

The catalog includes utilities, educational approximations and partial adapter contracts. Each comparative method still needs verified mathematics, installed input/output, compatible dispatch and measured outcomes. Five distinct, ROS-connected and fairly compared alternatives in each requested category remain R7/R9.2.

Benchmark code includes schemas, output/report generation, orchestration helpers and regression thresholds. Some measurements are fixed placeholders, and orchestration can report success without a successful mission. **Do not use its current results to rank algorithms.** Use the [operational workflow](docs/WORKFLOW.md) and the current evidence ledger to distinguish static checks, live trials and qualified scenarios.

### Simulator and GUI limits

Gazebo is the reference integration route. PyBullet and MuJoCo have physics engines and ROS bridge code; Isaac uses a separate runtime subprocess and can fall back to offline behavior. A simulator process starting or an offline stub running is not qualification.

Legacy model/world import screens and the newer pinned extension imports are
recorded separately. Source descriptions and generated geometry do not prove
stable rest poses, texture parity, actuation or missions for every imported
asset. Native MJCF profiles remain MuJoCo-specific; controller support is
explicit. Generated fallback planes are visual-only when an SDF ground
collision exists, preventing duplicate MuJoCo contacts. Remaining source-world,
material and collision/spawn defects stay in R3.6/R6.6.

The simulator clock, truth/odometry, command-watchdog, TF and readiness contracts
have been repaired for the non-Gazebo bridges. This does not make the backends
interchangeable: absolute topic names, estimator inputs, class-specific
controllers and task-specific sensors still require explicit qualification.
Isaac Sim has live scan/RGB-D/drive/reset and five-seed R4 mission evidence on a
named Jetson host, but not universal robot/backend coverage.

The Tkinter GUI has launch profiles, process logs, registry browsing, drive/map tools, vacuum/benchmark/test/health tabs and telemetry monitoring. These are interface features, not proof that each underlying workflow works. Its algorithm slots are built from the active mode's pipeline steps, list only algorithms the bringup layer can actually start, and are applied to the launch; modes and backends that the current selection cannot run are disabled with the reason attached rather than silently re-selected.

## Browse the repository safely

From the repository root, these commands use the source registry without a workspace build (Python 3 with PyYAML/jsonschema is required):

```bash
export PYTHONPATH="$PWD/src/robot_lab/robot_lab_registry:$PWD/src/robot_lab_utils:$PWD/src/robot_lab_adapter${PYTHONPATH:+:$PYTHONPATH}"
python3 -m robot_lab_registry.cli summary -c src/robot_lab/robot_lab_registry/config
python3 -m robot_lab_registry.cli list robots -c src/robot_lab/robot_lab_registry/config
python3 -m robot_lab_registry.cli list algorithms -c src/robot_lab/robot_lab_registry/config
python3 -m robot_lab_registry.cli describe experiment bumperbot_simulation -c src/robot_lab/robot_lab_registry/config
python3 -m robot_lab_registry.cli validate -c src/robot_lab/robot_lab_registry/config --cross-references
```

`describe` takes a **singular entity type plus its ID**; `validate` requires `-c`.
Cross-reference validation passes at the current source snapshot, but passing
validation does not prove a runnable robot/backend/task composition. The
`launch` subcommand is safe dry-run by default; add `--execute` only after the
resolved manifest and isolation plan have been reviewed.

Preview only—the `launch` subcommand is dry-run by default and does not start
a robot:

```bash
python3 -m robot_lab_registry.cli launch \
  -c src/robot_lab/robot_lab_registry/config \
  --composition '{"robot_id":"bumperbot","environment_id":"small_office","simulator":"gazebo","algorithm_ids":{"localization":"amcl"}}'
```

For this built ROS 2 Humble workspace, start the updated GUI:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_lab_gui robot_lab_gui
```

Use the [GUI guide](docs/tutorials/gui-workspace.md) and
[operational workflow](docs/WORKFLOW.md) for selection, isolated CLI launches,
readiness, recording and cleanup. Ubuntu 22.04/ROS 2 Humble on this Jetson is
the measured development host. Required CI build/test failures propagate;
clean-host simulator missions and full dependency/license reproduction remain
R9.1. Review host bootstrap commands separately from ordinary GUI operation.

## Path to the intended platform

The [roadmap](ROADMAP.md) and [agent handoff](docs/AGENT_HANDOFF.md) are the implementation authority. The current user priority is the new extensions: R3.6/R6.5–R6.7, R5.10 and R5.7–R5.9, while preserving completed fixes and exact task owners. Then resume the original wheeled/legged mission matrix, algorithm comparisons, concurrency and clean-host reproduction.

Success means selectable mobile, legged, humanoid and aerial robots; 2D/3D environments; at least five genuinely distinct supported alternatives in each requested algorithm category; and reproducible comparisons with measured outcomes. Each supported combination needs explicit dependencies, contracts, launch configuration, numerical tests where applicable, a runtime smoke test, and evidence-backed maturity. A numerical kernel, a catalog row, a GUI selector and a completed mission are different milestones.

Before implementing, read the handoff, claim one task, preserve other agents' work and run its stated acceptance checks. Update the roadmap and machine-readable status with commands, revision, results and remaining limitations. Do not promote a feature to integrated/qualified merely because imports or schema checks pass.

## License

See [LICENSE](LICENSE) for repository licensing and [third-party notices](LICENSES/third-party-notices.md) for imported assets and libraries. Third-party components retain their own terms; a top-level license does not relicense vendored code, meshes or optional integrations.
