# Worlds, robot assets and manipulation: continuation guide

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
Display. Other native imports do not gain walking/trajectory/grasp control. Native MJCF
profiles are limited to MuJoCo; enabling another backend requires an actual
plant and control adapter. Textures/skins and complete visual parity remain
separate from rigid geometry checks.

Robot-assets and official TurtleBot3/Husky/TurtleBot4 URDF imports resolve mesh resources
and retain source scales. Vendor Xacro runs against a source-backed SSD ament
index, without installing vendor packages into the system. Husky's Display
derivative disables its obsolete upstream control dependency and resolves its
relative extras include; this does not provide a Husky driving controller.
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
Robot navigation/flight missions and scripted actor behavior remain unqualified.

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
see [Cartesian evidence](status/evidence/panda-cartesian-2026-10-06/README.md).
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

## Reproduce the native Panda Cartesian qualification

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
   `status/evidence/panda-cartesian-2026-10-06/producer.py` takes
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
