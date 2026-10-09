# Worlds, robot assets and manipulation: continuation guide

Updated October 9, 2026. New [initial implementations](status/implementation-2026-10-09.md)
include exact-model Unitree policies, generic native actuator/Servo/teach-replay,
measured Panda object scenes, Stretch sensor/navigation candidates, static
PX4 planning and a real isolated experiment queue. Physical qualification
follows initial implementation. The latest historical physical work is recorded in
[the October 8 checkpoint](status/evidence/extensions-finish-2026-10-08/README.md).
Read [current status](status/CURRENT_STATUS.md) and
[the continuation](status/continuation-2026-10-08.md) for measured scopes and
remaining owners; exact CI and real GUI/robot evidence are separate.

The October 5 user request extends the existing robot/map/algorithm project.
Follow the [ledger](status/platform-status.yaml), [roadmap](../ROADMAP.md) and
[stabilization guide](PATCH_EXECUTION_GUIDE.md). Work on `master`; large assets,
builds and output remain on the SSD. Source `scripts/ssd_env.sh` first.

## Sources inspected

| Source | Pinned snapshot / decision |
|---|---|
| [robotics-upo map plugin](https://github.com/robotics-upo/gazebo_ros2_2Dmap_plugin/tree/317a17d4dc8004e14299d767278f0ecbcb857819) | Use Fortress branch `317a17d4`; default Humble branch is Classic. Source headers are Apache-2.0, root notice MIT; both retained. Build against ABI 6 even with Harmonic installed. |
| [mlherd worlds/models/maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps/tree/9d26e1f41cd7979bbee66ed852e7ef316e22b19f) | `9d26e1f4`; catalogs include model-only and archive assets, not just `.world`. README describes Gazebo 9/11. No root license found; inspect each upstream asset before redistribution. |
| [terrain generator](https://github.com/saiaravind19/gazebo_terrain_generator/tree/4946f4c8150633e4c1fb2ffe9a2ab4f495de9577) | `4946f4c8`, BSD-3-Clause. Generates Gazebo heightmaps/buildings. Requires provider credentials/data terms; output defaults to `/tmp`, so set `GAZEBO_TERRAIN_OUTPUT_PATH` on SSD. |
| [robot-assets](https://github.com/ankurhanda/robot-assets/tree/24b61c71c07e831a55c5bac1e8d3e2ee83ff9b99) | `24b61c71`; catalog every robot URDF variant, but distinguish partial forearms/grippers from complete robots. Upstream model links supplied; no blanket root license found. |
| [URDFHub](https://www.urdfhub.com/#robots) | Eight featured upstream robot sources at inspection. A directory, not a downloadable simulation stack; inspect the linked vendor/ROS repositories. |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/4d038b3feae26ec82b46a4d586379114012a8ac7) / [robot_descriptions](https://github.com/robot-descriptions/robot_descriptions.py) | Broader complementary sources. Prefer verified physics assets and maintained upstream ROS descriptions to redundant mirrors. Audit licenses per model; MJCF availability does not prove URDF/USD control parity. |

The [source snapshot](status/asset-sources-2026-10-05.yaml) records pinned
upstreams. Agents install the extension assets on the SSD; operators use the
normal **Launch** robot/map selectors and autofilled command. The **Installed
Extensions** tab opens those same selections and shows remaining import repairs.
It has no download controls.

### Install and integrate assets (agent/bootstrap workflow)

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/install_geometry_dependencies.sh
source scripts/ssd_env.sh
python3 scripts/provision_extension_assets.py --kind all
# Recheck existing pinned SSD sources without downloading:
python3 scripts/provision_extension_assets.py --kind all --no-download
```

The installer stores source checkouts, dependency archives, derived geometry,
actual occupancy maps and model checks under
`$ROBOT_LAB_RUNTIME_ROOT/external_assets/`. Its `installed/` snapshots feed the
GUI, bringup and common registry. Refresh Installed Assets or reopen the GUI
after installation. Missing source/model files are filtered out; an extension
cannot overwrite a legacy profile with the same ID. Checkouts reject source
edits and wrong revisions rather than resetting them. Do not commit multi-GB
source trees into this repository.

Native Menagerie models use their actual MJCF in MuJoCo. Their ROS description
exports the compiled rigid geometry, while measured native body transforms and
hinge/slide states reach RViz. Display holds the authored pose passively;
`display_hold:=false` runs native dynamics without a locomotion controller.
The native Panda is the measured exception: `arm_control:=auto` selects its
real joint controller and the Arm tab; use `arm_control:=none` for passive
Display. Compatible native fixed models now expose explicitly enabled joint,
coupled-hand, Servo and teach/replay controls; two Stretch profiles add native
base/sensors and experimental higher tasks. Three source-matched Unitree policy
variants are installed under existing families. Read
[native controls](tutorials/native-controls.md). Installed control metadata
records implementation and pending validation separately. Other native imports
retain their prior workflows. Native MJCF
profiles are limited to MuJoCo; enabling another backend requires an actual
plant and control adapter. Textures/skins and complete visual parity remain
separate from rigid geometry checks.

Robot-assets and official TurtleBot3/Husky/TurtleBot4 URDF imports resolve mesh resources
and retain source scales. Vendor Xacro runs against a source-backed SSD ament
index, without installing vendor packages into the system. Husky's Display
derivative disables its obsolete upstream control dependency and resolves its
relative extras include. The October 8 lab derivative now adds separate
physical four-wheel skid control, declared motor/solver settings and a named
lidar/RGB-D/IMU kit; [the Husky guide](tutorials/husky.md) records its scope.
Higher modes require exact backend-specific measured certificates.
Fetch's obsolete Classic XML extensions and byte-identical duplicate nodes
are repaired only in derived files. The malformed R2 gripper snapshot loses
one dangling duplicate-parent joint referencing a missing ankle; the sensor,
foot and their valid parent remain. Its exact removed joint is recorded. Eve uses the manufacturer's pinned BSD-licensed QB Hand
meshes. Two upstream URDF fixtures contain no robot links and cannot launch.
URDF profiles use existing backend import paths in Display; full cross-backend
runtime qualification and control remain pending.

World imports expand real model includes and archive dependencies, resolve
mesh/material files, preserve original sources and record removed Classic
plugins. Missing Office models are supplied from pinned OSRF ServiceSim;
Factory uses the original Gazebo coke-can model. The importer chooses a free
spawn from real geometry, prefers sufficiently large floor slabs, generates
MJCF and invokes the actual Fortress occupancy plugin. The generated grid
uses a complete conservative static slice mask and the actual Fortress plugin;
primitives are projected once to avoid repeated geometry queries per cell.
It represents the selected seed-connected height slice, not every room or floor.
The Worlds tab initializes editable Seed X/Y from the selected world's spawn.
Choose another seed to inspect a disconnected component. The actual custom-seed
Generate and Stop actions are measured in the October 8 checkpoint. Terrain
projection clips each transformed triangle at the chosen height and rasterizes
separate polygons, preserving gaps. Generation caches compare world, seed,
recipe, output and every mesh-dependency hash before reusing an export.
Occupancy v4 preserves every triangle/plane intersection, including open wall
sheets omitted by the old closed-path projection. All 17 installed extension
worlds have actual v4 grids with unchanged source worlds and measured spawns.
The [eight short hospital routes](status/evidence/extensions-finish-2026-10-08/navigation/open-wall-backends/README.md)
pass for Bumperbot/Labbot across all four engines. Other world missions,
upper-floor travel, flight and scripted actor behavior remain unqualified.

### Refresh installed grids without moving robots

Stop any owned simulator first. The agent refresh command retains existing
world/spawn/initial-pose fields, preserves old maps and profile snapshots, and
updates the installed Launch/Registry map paths. It serializes against asset
provisioning and rejects output on internal storage. A fresh output directory
is required. Reopen the GUI after registration and inspect the generated grid.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ROBOT_LAB_REFRESH_RUN_DIR=$(mktemp -d "$ROBOT_LAB_RUNTIME_ROOT/occupancy-refresh-XXXXXX")
python3 scripts/refresh_installed_occupancy_maps.py \
  --output "$ROBOT_LAB_REFRESH_RUN_DIR/run"
# For a later single-world refresh, use a different fresh output path:
# python3 scripts/refresh_installed_occupancy_maps.py --map dataset_hospital --output "$ROBOT_LAB_REFRESH_RUN_DIR/hospital"
```

The real [export and preservation receipts](status/evidence/extensions-finish-2026-10-08/occupancy-v4-refresh/verification.json)
and actual GUI selections establish registration. Qualify new robot routes
separately with fresh body/floor/heading evidence. A cache hit or export count
does not establish navigation.

The installation report records exact entries and repair reasons. Native import
checks and source counts never imply control or mission acceptance. URDFHub
is a directory of upstream sources: overlapping models are covered by the
installed libraries, and official pinned TurtleBot3/Husky descriptions cover the remaining
featured models. Equivalent models retain their actual source provenance. All 100 Fortress 6.18 example SDFs are downloaded from a pinned source;
empty/default/shapes environment examples are installed. The other 97 are
plugin/robot fixtures requiring behavior/resource review. Fuel environments
need their own dependency imports and qualification.

## Generate a 2D grid

In GUI **Worlds**, choose a registered world, resolution, slice height and SSD
output directory, then **Generate 2D Occupancy Grid**. The generated map lives
in a fresh dated directory alongside the derived mapping SDF, Gazebo log and
JSON report. Existing occupancy maps remain intact. **Stop Generation** stops
the owned process and its Gazebo child.

Equivalent CLI:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_lab_maps generate_occupancy_map.py \
  --world "$PWD/src/robot_lab_maps/maps/nav_obstacle/worlds/nav_obstacle.world" \
  --output-dir "$ROBOT_LAB_RUNTIME_ROOT/generated_maps" \
  --resolution 0.1 --height 0.3 --seed-x -7 --seed-y -7
```

The pinned upstream plugin uses conservative AABB intersections. Its mesh
path assumes a unit-sized cube; Robot Lab instead takes actual indexed mesh
vertices, scene units, scale and composed world pose, then slices the surface
into a grid-aligned collision mask for the generator. Decorative visuals are not
collision obstacles. Scripted actors are excluded with a recorded note.
Heightfields are tessellated by the shared converter described below and
sliced by the same plane. A single-height static slice is not a multilevel
floor map or aerial planner; a terrain sample at or above the slice height is
marked solid, so one slice cannot represent a multilevel floor.

### Heightfield layout (measured, not assumed)

`<heightmap>` terrain is converted once in
`robot_lab_utils/heightfield.py` and used by Gazebo, MuJoCo, PyBullet and
Isaac. The SDF spec does not pin three details that each change where the
surface ends up, so they were measured on the installed Gazebo by dropping
spheres on asymmetric rasters and reading the settled poses back:

- **elevation is normalised by the raster's own maximum**, not by 255 — a
  uniform value of 100 with `size` z = 4 produced a surface at 4.0 m;
- **raster row 0 is maximum +y**;
- **the grid is cell-centred and spans `(n-1)/2 · size/n`**, not `size/2` —
  Gazebo's own AABB measured ±9.69697 m for a 20 m / 33-sample raster
  (±9.84615 m and ±9.41177 m for 65 and 17 samples).

A test pins each value. Keep the elevation measured from the collision pose
origin (Gazebo does not centre it), and note that MuJoCo's native `<hfield>`
radius is `size/2` rather than the cell-centred span — interior points match,
the outer half-cell differs.

Before activating an export: inspect occupied/free/unknown regions, compare
floor-plan landmarks and furnished geometry, check origin/resolution and a
free spawn, then record a Nav2 route on the generated map. Register reviewed
outputs in map profiles/registry and regenerate backend worlds from the same
source. R6.5 remains partial until its named geometry and mission checks pass.
The plugin flood-fills only the free region connected to the selected seed;
disconnected rooms stay unknown. The furnished floor-1 and floor-2 exports
generated on October 5 contain 711/630 occupied and 4,781/4,626 free cells at
0.1 m resolution. Neither export labels a mesh-mask obstacle as free. These
are projection checks, not navigation acceptance or whole-building coverage.

## External worlds and terrain

1. Pin the source and record original/model-specific licenses. Download the
   complete include/mesh/texture closure on SSD. Detect unresolved resources,
   frame-relative poses, actors and unsupported plugins before conversion.
2. Inventory installed Fortress/Harmonic examples and licensed upstream/Fuel
   models. Existing Robot Lab worlds are only a subset of those sources.
   Classic plugins are not ABI-compatible with Fortress/Harmonic.
3. Terrain integration must configure `GAZEBO_TERRAIN_OUTPUT_PATH`, caches and
   provider settings before starting its local server. Keep tokens private;
   save public generation inputs and data attribution in a manifest.
4. Add a canonical heightfield/triangle representation. Done for the four
   backends: `robot_lab_utils/heightfield.py` produces one metre layout and
   each backend consumes it (MuJoCo native `<hfield>`, PyBullet and Isaac
   triangle meshes), with the measured conventions documented above. Compare
   sampled elevations, support contacts and scans; Isaac runtime contact and
   the outer half-cell span difference are still to be measured.
5. Execute terrain driving, legged and flight tasks only where the robot has
   working control. Store negative imports and missions. No flattened floor
   or geometry-only display qualifies a terrain mission.

## Arms, hands and mobile manipulators

Start with a licensed Panda or UR5 and upstream ROS 2/MoveIt configuration.
Wire actual joint trajectory actions and feedback in one simulator, then
port the same joint/limit/actuator contract to the other three. GUI needs home,
joint jog, Cartesian jog, plan/execute/cancel and Stop with command ownership.
[MoveIt Servo](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
provides the upstream Cartesian/joint jogging path; it still needs the correct
planning scene, joint controller, transforms and collision geometry.

Add a simple gripper before a dexterous hand. Test mimic/tendon/coupling,
measured joint motion, force limits and real object grasp/release. Then use a
maintained Fetch/PR2/TIAGo source for a mobile manipulator; verify base/arm
frames, payload stability, navigation footprint, cancellation and simultaneous
command arbitration. Acceptance is a measured navigate/reach/grasp/transport/
release mission, with object state and contact—not a moving URDF preview.

R5.7 is partial: native Panda joint/Home/Stop/action controls now have measured
MuJoCo dynamics, rejection, cancellation and heartbeat-loss evidence; see the
[Panda guide](tutorials/panda_arm.md). R5.8 adds the native coupled-finger
GripperCommand, actual GUI Hand controls and a physical cube lift/hold/release
with Cancel/Stop/watchdog and GUI reset; see
[grasp evidence](status/evidence/panda-gripper-2026-10-06/README.md).
Native Panda MoveIt KDL/OMPL/FCL Cartesian Plan/Execute now has measured
static-world/native-geometry collision and physical TCP proof in `nav_empty`;
see [final control-column Cartesian evidence](status/evidence/panda-cartesian-controls-column-2026-10-07/README.md).
Servo, dynamic/attached-object scenes, arbitrary repeated pick/place, other
hands and cross-backend manipulation qualification remain. R5.9 is queued; the source
catalog does not implement mobile manipulation. Preserve the wheeled and legged workflows while adding
these components.

## Resume after this installation checkpoint

Read [the exact installation evidence](status/evidence/extensions-integrated-2026-10-05/README.md)
before changing the importer or enabling modes. Do not rerun source downloads
or old PX4 relocation as a substitute for controller work.

1. Preserve the named Panda GUI display/state screens. Extend runtime checks
   to each model/backend, including valid joint-limit rest poses, mimic/tendon
   coupling, steady hold, textures/skins, and shutdown diagnostics. Isaac Panda
   feedback is measured, but stable authored-pose hold is not qualified.
2. R5.7: preserve the measured native Panda controller and GUI joint/Home/Stop
   implementation, including the position-only action contract and heartbeat
   ownership, upstream MoveIt2 KDL/OMPL/FCL and seeded native Panda Cartesian
   Plan/Execute. Preserve native collision exclusions, 0.005 rad soft margins,
   exact model/world hashes and the checked-edge retimer. Add Servo and a
   dynamic/attached-object scene before planned pick/place; repeat independent
   TCP/contact measurements on each additional map/backend.
3. R5.8: preserve the measured coupled Panda controller and supported cube
   setup; repeat different object dimensions/masses and pick/place. Native
   Robotiq's 0–255 command has the opposite closure direction to Panda and its
   linkage is not a pair of Panda slide joints. Implement its own actuator,
   coupling, opening and force conversion before adding GUI control. Add
   dexterous-hand adapters only against their exact native model.
4. R5.9: choose installed Fetch or another maintained mobile manipulator;
   implement base/arm controllers and sensing, qualify base navigation, then
   measured navigate/reach/grasp/transport/release with command arbitration.
5. R6.6/R5.10: compare each imported world's visuals, collisions, floor
   support, spawn, ceiling and map alignment in each backend. Record actual
   wheeled routes and PX4 flights separately. A seed-connected height slice
   can leave disconnected rooms unknown and does not encode all drop-offs.
6. Coordinate R6.7 with its recorded owner; preserve its measured heightfield
   conventions. Finish provider attribution, deterministic GUI generation,
   Isaac terrain contacts and class-specific traversal.
7. Resume the older R5.6 obstacle/mapping/reset matrix and algorithm/release
   acceptance after those extension tasks. Update the ledger, checklist and
   GUI Health with exact measured cells and retained negatives.

## TurtleBot 4 continuation

The user additionally requests TurtleBot 4. Standard/Lite and the official
Create 3 dependencies are installed at pinned Humble revisions. The normal
Launch selectors/autofill and eight backend Display/state checks are recorded
in [the latest evidence](status/evidence/panda-turtlebot4-2026-10-05/README.md).
Both variants now have actual GUI physical Drive/WASD/Stop and publisher-loss
checks on all four engines in `nav_empty`; see
[Drive evidence](status/evidence/turtlebot4-drive-2026-10-05/README.md).
Display uses the lab wheel controller and original passive wheel-drop springs,
not vendor firmware. Follow [the TurtleBot 4 guide](tutorials/turtlebot4.md) for
joint dimensions, source licenses, sensor adapters and mission acceptance.
October 6 adds forty source-matched localization/reset/resume, real 2D/3D
Save Map and clear/obstacle Nav2 trials across all four backends, plus eight
final source-frame/lidar/depth/Drive screens. Normal GUI enables all five
modes and auto-selects the measured algorithms; named maps are `nav_empty`
and navigation's `nav_obstacle`. Other maps are experiments. See
[mode evidence](status/evidence/turtlebot4-modes-2026-10-06/README.md) and
[sensor evidence](status/evidence/turtlebot4-sensors-2026-10-06/README.md).
Preserve the Gazebo selected-world 2 ms cap, Isaac pose-derived ideal twist,
headless render cadence and map-export/GUI cleanup. Continue longer routes,
other maps, vendor hazards/docking and visual parity; do not substitute
ideal-sensor simulation for the vendor firmware stack.

## Recording and enabling a new controller

1. Claim the exact ledger lane and record backend, robot, source pin and map.
   Preserve an existing owner's terrain/controller changes. Read prior negative
   trials before changing gains, axes, frames or support claims.
2. Build the changed ROS packages after editing. Compare source and installed
   Python SHA-256, including byte-compilation diagnostics. The native MuJoCo
   package copies Python modules with setuptools despite symlink-install.
   Record the executed model, derived URDF, control/sensor configuration,
   upstream licenses, dependency revisions and actual producer bytes.
3. Use a separate SSD qualification installation when trying a previously
   disabled mode. Copy installed catalog metadata and point its profiles at the
   same pinned assets. Provisional test modes stay in that qualification root;
   the normal operator catalog keeps its measured backend restrictions. Set a
   separate `ROS_DOMAIN_ID`, finite trial budget and SSD log directory. Run one
   physics trial at a time on this Jetson.
4. Exercise actual GUI selection, algorithm defaults, command preview, Run and
   Stop. Observe engine body/joint/object state independently of command or
   controller status. For mapping, record accepted poses/occupied cells or
   changing finite 3D geometry, then use the GUI Save Map and validate its
   PGM/YAML or SQLite/PCD contents. For navigation, test a clear goal and a
   goal whose straight path intersects an obstacle; use body error after a
   simulation settling window and swept geometry clearance. Keep estimate
   error and ground truth separate.
5. Capture reset during activity, monotonic clock, resumed estimation/mapping
   and post-reset motion. Measure lost publisher input, command expiry,
   Stop/cancel and fresh-state ownership. Source/metadata PASS strings cannot
   replace physical observations. Keep failures and later corrections in
   distinct directories; record upstream cleanup errors even when the owned
   launch returns zero.
6. Add immutable actual reports and their SHA-256 to
   `docs/status/asset-runtime-support.yaml`. `robot_lab_utils.asset_support`
   verifies source pins/control configuration and recomputes acceptance from
   measured body, map or object values. TurtleBot4 Nav2 settings are also
   fingerprinted. Panda Hand requires contact, bounded effort, physical lift
   and release plus interruption/reset measurements. Panda Cartesian requires
   physical TCP errors, collision rejection, real joint/finger invalidation,
   bounded Stop/watchdog, monotonic reset and clean planner exit; native
   resources and source/world files are fingerprinted. Do not copy these
   model-specific contracts onto a different tendon, linkage or arm.
7. Run `provision_extension_assets.py --no-download` for the changed source to
   refresh normal Launch profiles, registry sensor/controller metadata and
   Installed Extensions descriptions. Verify actual GUI backend mode gating,
   algorithm auto-selection, command autofill and new controls again. Update
   the roadmap, ledger, checklist, tutorial and evidence index in the same
   checkpoint. Software tests and remote CI are separate from robot missions.
   Keep a task partial until its full stated acceptance passes.

## TurtleBot3 integration and repeat protocol

The October 7 lane connects the already installed official Burger/Waffle/
Waffle Pi models to lab physical controllers. Read the
[operator guide](tutorials/turtlebot3.md), [actual Drive evidence](status/evidence/turtlebot3-sensors-2026-10-07/README.md)
and latest R3.6 ledger state. Display/Drive proof and mode proof remain
separate. The old TurtleBot4/Panda contracts must survive shared changes.

1. Claim R3.6 before changing the model or command paths. Preserve another
   agent's R6.5 occupancy review and R6.7 terrain work. Start from the current
   containing `master` commit, source `scripts/ssd_env.sh`, and allocate a new
   persistent directory under `$ROBOT_LAB_RUNTIME_ROOT`. Do not overwrite a
   failed or successful trial directory when repeating it.
2. Keep the official description pin `90a68bd2e3c61c12966779da89d8eeaec82730e9`
   and simulation pin `a35a56c8b04877dc89772b598084d8ce648a9023`, with their
   Apache-2.0 notices. `scripts/extension_mobile_control.py` generates a
   separate `drive.urdf` and `drive-controllers.yaml`; source checkouts remain
   unchanged. Inspect the actual wheel collision radius, source joint axes,
   centers and mounted scan/IMU frames before adapting another model.
3. Preserve 0.033 m wheel radius, 0.160/0.288 m executed track and the source
   local wheel-axis transforms. The Waffle SDF rounds its track to 0.287 m;
   the executed URDF geometry governs control. Undefined virtual-frame masses
   are explicitly 1e-6 kg with 1e-9 kg·m² diagonal inertia; authored inertias
   remain unchanged. PyBullet's implicit 1 kg defaults failed the turn-stop
   screen. This regularizer is a lab assumption, not vendor payload data.
4. Preserve the measured 0.18 m/s / 0.3 m/s² and 1.2 rad/s / 2 rad/s² caps,
   source-named wheels, 5 Nm/15 rad/s simulation limits, MuJoCo wheel armature
   0.0002, velocity gain 0.1 and acceleration-limited watchdog deceleration.
   Those MuJoCo settings are profile-local; default wheel profiles retain the
   existing values. Reducing speed or rotor inertia alone failed the tilt
   limit, so retain those negatives when evaluating a different remedy.
5. Keep the LDS 360-ray / 0.12–3.5 m / `base_scan` contract at 5 Hz. The
   Fortress derivative attaches source scan/IMU geometry at the URDF frame's
   local origin, not at the already applied model-relative SDF pose. The
   source cameras are RGB-only. Camera rendering and RGB/depth algorithms
   require their own implementation and physical measurement before 3D SLAM
   can be enabled. Other bridges currently retain ideal sensor behavior.
6. Keep `nav_empty`'s named (-4,-4,0 yaw) override in the robot profile.
   `robot_lab_utils.robot_spawn.map_spawn_override` validates finite values;
   bringup merges it after map/global-robot defaults, and explicit launch
   arguments override that result. GUI command preview includes the same
   named pose. Changing to Celisca must remove these extra spawn arguments.
   Never extend the LDS range to hide a geometry-free AMCL/SLAM spawn.
7. Build changed packages, then compare executed modules with source:

   ```bash
   source scripts/ssd_env.sh
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   colcon build --packages-select robot_lab_utils robot_lab_mujoco \
     robot_lab_bringup robot_lab_gui robot_lab_navigation --symlink-install
   source install/setup.bash
   ```

   Python files can be copied despite symlink-install. Compare actual source
   and installed SHA-256, not only build return codes. Keep logs on SSD.
8. Repeat the archived real GUI producer from
   `status/evidence/turtlebot3-sensors-2026-10-07/asset_turtlebot3_burger-mujoco/producer.py`
   under Xvfb with a dedicated `ROS_DOMAIN_ID` below 233, `PROBE_ROBOT`,
   `PROBE_BACKEND` and a new `PROBE_ROOT`. It creates an owned normal Display
   launch, observes independent engine body and joint state, exercises neutral
   enable/W/S/A/D/Space and publisher loss, verifies mounted TF/scan geometry
   and closes the GUI normally. Require physical tilt <0.3 rad and both
   wheels' finite changing state. Check producer, launch and all owned plants
   for clean termination before the next case.
9. Preserve the original producer report and raw trace. `collect_sensors.py`
   derives maximum body tilt and joint-position ranges from real samples;
   it cannot replace the trial. Trace counts include Stop/Close callbacks
   after the producer's earlier report counts. Every derived report links the
   exact original report and trace checksums. `drive_lidar_screen` recomputes
   physical motion, neutral, stop, mount, range and joint/tilt limits.
10. Qualify disabled modes through a separate SSD catalog pointing to the same
    pinned executed URDF. Use the archived actual mode producer for three
    robots × four backends × Localization/SLAM/Navigation-clear/Navigation-
    obstacle: 48 named producers. Workflow checks require both translations,
    both turns, publisher loss, monotonic reset, estimator/graph restart and
    post-reset movement. SLAM needs accepted poses and occupied cells plus
    an actual GUI YAML/PGM save and clean exporter exit. Navigation uses
    RViz-style topic goals, fresh independent truth, 0.15 m / 5°, a simulation
    settling second, source world obstacles and ≥0.02 m swept clearance.
    Burger radius is 0.185 m; Waffle/Pi radius is 0.26 m. An obstacle route
    must actually detour around a blocked direct path.
11. Archive the real producer return codes, reports and pre-trial model/drive/
    sensor/spawn/runtime fingerprints. Record capture timing honestly: the
    initial mode producer swept runtime packages but omitted `robot_lab_maps`.
    Its named map assets are verified against recorded Git HEAD and installed
    bytes during collection. Generation/controller-file fingerprints recorded
    at collection are distinguished from the pre-trial executed-URDF hashes.
    Add map/world, generated controller and critical runtime fingerprints to
    `asset-runtime-support.yaml`; source changes require new evidence.
12. Recompute `apply_recorded_modes` against the actual numeric reports. Require
    both clear and obstacle navigation reports for each enabled backend. Only
    then provision `turtlebot3_vendor --no-download` into the normal catalog.
    The installed merger preserves prior profiles on failed replacements;
    a failed replacement never qualifies new behavior. Check all 48 normal
    GUI robot/backend/mode selections, actual algorithm defaults, Run state,
    command/spawn autofill and unsupported 3D SLAM. Exercise a normal owned
    navigation launch too. Keep software checks, UI checks and physical
    mission evidence in separate records.
13. A shared GUI/bringup change also needs a native Panda normal-profile
    Cartesian repeat before refreshing its strict certificate. The October 7
    repeat is in [Panda control-column regression](status/evidence/panda-cartesian-controls-column-2026-10-07/README.md).
    Preserve its original three actuator/control files and physical cube
    grasp certificate. Changing unrelated documentation links does not qualify
    a robot or justify replacing source hashes without actual runtime proof.
14. Update roadmap, ledger, checklist, tutorial, support index and GUI Health
    together. Run appropriate source/physics/integration and Xvfb checks,
    commit on `master`, push to `origin/master` as authorized, then observe
    exact remote CI. Keep R3.6 partial until all model/controller/material/
    mission acceptance passes. Next extend map and repeated-route coverage;
    preserve other extension owners before returning to the old roadmap.

## Maintain complete catalogs and the embedded 3D inspector

The native [Registry viewer guide](tutorials/registry-3d.md) and
[actual rendering/live-plant evidence](status/evidence/registry-3d-2026-10-07/README.md)
describe the current implementation. Continue it as follows:

1. Claim the relevant R3.6/R6.6 scope and preserve other task owners. Inspect
   actual source descriptions before grouping. `asset_groups.yaml` records
   reviewed families, exact profile/registry IDs, preferred complete variants
   and component/reference roles; it is presentation metadata, not a controller
   certificate. Do not merge robots merely because filenames resemble one another.
2. Keep executable profile IDs and saved manifests stable. `AssetGroups`
   supplies parent choices and compatible variant selection; unsupported source
   alternatives retain their own mode/backend restrictions. Newly installed
   uncurated complete assets stay independent until reviewed. Test old exact
   IDs, normal command autofill and missing preferred-source fallbacks.
3. Prefer a complete authored assembly. Use `audit_contained_components` to
   measure source link/internal joint containment; exclude only documented
   empty standalone mount frames. This does not prove mesh/inertia equivalence.
   If a compatible assembly is absent, define and test real mount frames,
   collision/inertia/transmission contracts before adding a composed robot.
   Unattached components remain inspection-only, not standalone Launch choices.
4. Parent maps by the actual complete room/floor. Shell/furniture/actor source
   variants remain explicit. Do not invent multi-floor placements or flatten
   distinct environments into one source asset. Retain source include chains,
   original meshes/units, actors and provenance.
5. `asset_preview.py` resolves URDF joint/visual FK, compiled native nominal
   keyframes and SDF visuals/includes. The existing SDF reader's default physics
   collision path must remain unchanged. Missing/invalid geometry reports an
   error instead of showing a fake placeholder. X500 inspection reads its
   source SDF without importing the FCU/offboard runtime.
6. Prepare full geometry in an owned, bounded background loader with all meshes,
   buffers and logs on SSD. Render into Tk's own matching GLX child window.
   OpenGL calls remain on Tk's thread; cancel stale loads by generation and
   release buffers/context/drawable before destroying the widget. Do not re-add
   external viewer windows or ROS preview command publishers.
7. Test actual full-GUI rendering/input, not just nonempty metadata. Read the
   rendered GL_BACK buffer before presentation, save a whole GUI screenshot,
   verify orbit changes pixels, pan/zoom/Fit change the measured camera, and
   preserve selected robot/map/command/process. Test Close, load errors and
   source scale. Keep the original failed Tk/GLX/readback/source-load stages.
8. Preview during a normal owned simulation. Record independent neutral body
   truth and zero movement commands, then actual Drive/Stop/watchdog control.
   The archived Burger/PyBullet producer supplies this protocol. Static visual
   proof must never enable a controller, localization or navigation mode.
9. Source pose defects remain source tasks. The October 8 hospital derivative
   repairs only the exact reviewed fixture placements from a checksum-matched
   source and records each change. Collada node transforms, metres, textures
   and native Gazebo loading have separate checks; camera Fit is not a repair.
   Furniture is explicitly fixed as a static snapshot in every engine. Retain
   original sources, failed floor/route trials and exact successful screens;
   wider collision/spawn/actor/multi-floor missions still keep R6.6 partial.
10. Update the ledger, roadmap, checklist, tutorial and GUI status together.
    Run meaningful grouping/FK/missing-geometry tests and actual Tk/OpenGL
    tests. Shared launcher/planning changes need the source-matched physical
    Panda regression before refreshing its strict certificate. Keep rendering,
    software tests and robot missions separate, then commit/push on `master`
    and check exact CI. Texture/actor/GPU and remaining mission work stays open.

## Reproduce the native Panda Cartesian qualification

The [GUI workspace guide](tutorials/gui-workspace.md) describes the control
column and category/type filters. For further GUI work, claim R3.4 alongside
the runtime task, preserve exact saved profile IDs and shared resolver output,
and classify reviewed families in `robot_taxonomy.yaml`. Keep source components
under their parent and label unknown imports Unclassified. Do not infer a
controller from a structural tag. Run taxonomy and actual Tk command/layout/
neutral-input checks, build copied packages, then repeat the real normal
Drive and Panda screens below. Extend their source contracts from measured
pre-trial hashes, archive any failure, update Health/ledger/roadmap together,
and inspect exact CI after the authorized master push.

1. Source the SSD environment and ROS/installed workspace. Build
   `robot_lab_utils robot_lab_bringup robot_lab_gui robot_lab_mujoco`. Verify
   installed Python/launch hashes against source; do not rely on symlinks alone.
2. Preserve the pinned native MJCF. `native_arm_description.py` generates the
   planning URDF/SRDF from compiled native axes, collision hulls, limits and
   exclusions. Review this against actual FK and native contacts if the model
   changes. A visual-only URDF does not supply a collision scene.
3. Start one isolated domain/owned GUI launch with MuJoCo/Display/`nav_empty`.
   The static scene must acknowledge its world checksum and geometry. Test
   actual floor/self-collision and unreachable requests; they must produce
   no physical movement. Do not bypass the scene status.
4. Use Arm Plan/Execute for small z/y offsets. Record independent physical
   hand-body TF plus the source TCP offset, joint feedback and real controller
   results. Require 0.02 m / 5°, fresh truth and actual displacement above 0.02 m.
   `GetPositionIK` uses the measured seed and collision avoidance, then OMPL
   plans joint constraints. Preserve every checked edge when retiming.
5. While a plan is cached, jog an actual joint and move actual fingers; both
   must disable Execute. Test invalid input, Cancel, Stop, missing heartbeats,
   reset with monotonic clock/Home, monitor connected, Stop/Close and a new Run.
   The 15 s action duration and speed limits remain unchanged.
6. Inspect **each child exit**, not only the launch root. MoveIt Humble on this
   host needed process-local plugin residency for clean callback destruction;
   preserve the SDK hashes and [upstream lifetime report](https://github.com/moveit/moveit2/issues/1597).
   A plugin/version change needs a new shutdown trial. Retain failed raw logs.
7. The actual archived producer in
   `status/evidence/panda-cartesian-controls-column-2026-10-07/final-responsive/producer.py` takes
   `PANDA_PROBE_OUT` (new SSD directory) and `PANDA_PROBE_NORMAL=1` for the
   installed profile. Run it under `xvfb-run -a` with a dedicated ROS domain.
   Source/SDK/native hashes precede launch. Its optional flag override is
   explicitly experimental and must not be used as a normal-profile claim.
8. Archive compact reports/producer/hash manifests; leave large traces and
   source snapshots on SSD. Add the exact report to the evidence index and
   `arm_planning` support certificate; recompute numeric acceptance, provision
   that source, then check normal GUI autofill and repeat actual execution.
   Keep fixture planning disabled until its moving cube/pedestal and attached
   object transitions exist in the planning scene.
