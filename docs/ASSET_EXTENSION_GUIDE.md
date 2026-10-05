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
Neither option provides a walking, trajectory or grasp controller. Native MJCF
profiles are limited to MuJoCo; enabling another backend requires an actual
plant and control adapter. Textures/skins and complete visual parity remain
separate from rigid geometry checks.

Robot-assets and official TurtleBot3/Husky URDF imports resolve mesh resources
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

R5.7/R5.8/R5.9 are queued implementation tasks. Controllers, MoveIt scenes,
grasp missions and cross-backend manipulation qualification are not delivered
by the source catalog. Preserve the wheeled and legged workflows while adding
these components.

## Resume after this installation checkpoint

Read [the exact installation evidence](status/evidence/extensions-integrated-2026-10-05/README.md)
before changing the importer or enabling modes. Do not rerun source downloads
or old PX4 relocation as a substitute for controller work.

1. Preserve the named Panda GUI display/state screens. Extend runtime checks
   to each model/backend, including valid joint-limit rest poses, mimic/tendon
   coupling, steady hold, textures/skins, and shutdown diagnostics. Isaac Panda
   feedback is measured, but stable authored-pose hold is not qualified.
2. R5.7: claim the task, use native Panda's actual seven arm actuators and
   joint limits, then expose a real ROS trajectory action with measured
   feedback/cancel/Stop. Add GUI joint jog/home/plan/execute with single command
   ownership. Use pinned upstream MoveIt2 configuration for Cartesian planning;
   validate collisions and reachable/failed targets before adding a capability.
3. R5.8: control the actual coupled Panda/Robotiq gripper, qualify limits and
   contact force, then measure closure, object grasp/hold/release. A slider or
   rendered hand is insufficient. Add dexterous-hand actuator/tendon adapters
   only against their exact native model.
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
