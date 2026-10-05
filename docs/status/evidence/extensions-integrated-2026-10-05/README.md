# Installed extension checkpoint, October 5

Downloaded models and worlds now use the existing Launch selectors and
autofilled Run command. Installed Extensions opens those same selections;
operators do not download assets through the GUI.

Source baseline `cae43fd`, Jetson AGX Orin / ROS Humble. The containing commit
captures the source stage; [manifest.json](manifest.json) hashes the changed
implementation and measured artifacts. Large source trees, meshes, occupancy
PGMs/YAMLs and retained trial logs remain on the SSD at
`/workspace/molar/robot_lab_runtime/external_assets/` and
`/workspace/molar/robot_lab_runtime/extensions-integrated-2026-10-05/`.

## Installed and checked

- 59 pinned Menagerie native MJCF models compile through the actual MuJoCo
  SDK; rigid geometry is exported for ROS and native body transforms reach RViz.
- 48 robot-assets URDF variants/fragments pass actual PyBullet imports, plus
  three official TurtleBot3 descriptions and the official Husky description.
  These are 111 Display profiles, not 111 distinct qualified robot missions.
  Two upstream fixtures contain no robot links and are excluded.
- All fourteen dataset worlds resolve their actual model/mesh dependencies;
  three Fortress environment examples also have installed profiles. Real
  occupancy exports and derived MJCF are recorded in
  [world-import-checks.json](world-import-checks.json).
- All 100 Fortress example SDFs are downloaded. The other 97 plugin/robot
  demonstrations require fixture-specific resource and behavior review.
- [The actual Tk command probe](installed-command-report.json) selects all
  111 installed robot profiles and seventeen worlds, verifies autofilled
  commands, and checks eight URDFHub links to installed equivalents.

[Model checks](model-import-checks.json) record native engine imports,
source/dependency hashes and explicit derivation repairs. Original sources
remain unchanged. Eve uses manufacturer QB Hand meshes; Office uses original
OSRF ServiceSim dependencies. R2 loses a dangling duplicate-parent joint;
its sensor/foot and valid parent remain. Vendor Xacro uses an SSD source-backed
ament prefix. Husky's Display derivative omits obsolete controller lookup.

## Actual GUI display/state screens

Each producer selects `dataset_room2`, uses GUI Run, observes ROS state and
then uses GUI Stop. Original failed probes remain in the SSD run directory.
Reports are copied unchanged:

| Robot / backend | Measured report |
|---|---|
| Native Menagerie Panda / MuJoCo | [Native report](gui-native-report.json) |
| robot-assets Panda / PyBullet | [PyBullet report](pybullet_panda_valid-gui-native-report.json) |
| robot-assets Panda / MuJoCo | [URDF MuJoCo report](mujoco_urdf_panda-gui-native-report.json) |
| robot-assets Panda / Gazebo Fortress | [Gazebo report](gazebo_panda_valid-gui-native-report.json) |
| robot-assets Panda / native Isaac Sim | [Isaac report](isaac_panda_clock_valid-gui-native-report.json) |

All observe finite simulator joint states, body transforms, description and
clock progression. Gazebo includes the fixed world joint. Isaac repeats clock
stamps between physical ticks; the corrected screen requires nondecreasing
time and at least two seconds of advancement. Clock repeats are not a reset.
The reports cover display/state and owned cleanup. Gazebo/Isaac shutdown
diagnostics and Isaac Panda's stable authored-pose hold remain gaps. These
screens do not measure arm trajectory tracking, grasp, locomotion or missions.

Native Display passively holds its authored/home pose with `mj_forward`.
`display_hold:=false` runs native dynamics without a motion controller.
The rigid ROS export is not a replacement physics plant or a texture/skin
equivalence claim. Native MJCF profiles are MuJoCo-only.

## Repairs and validation

Live native rendering initially failed because the source framebuffer was
smaller than the viewer; the node now sizes it before rendering. A later
observer found empty joint arrays: NumPy joint types did not compare correctly
with this SDK's enum values. Explicit integer comparisons now publish actual
hinge/slide states and correctly place keyed free bases. The new native physics
test pins both behaviors. Repeated SIGINT during EGL cleanup is handled once.

Occupancy generation originally mistook `ign service` exit code zero for an
acknowledgement even on timeout. It now retries timed-out requests. Large-world
primitive bounds are projected once into a complete conservative slice mask,
then the actual upstream plugin flood-fills that mask. This avoids per-cell
Gazebo geometry queries. Floor slabs constrain spawn selection; chair feet
cannot become the world floor. Rotated bounds, slice height and cell overlap
have meaningful numerical counterexamples.

Validation: seven-package build; **772 fast tests**, two skips and one
integration deselection; **123 integration tests**; **7 physics tests**;
**43 Tk command/Drive tests**; registry cross-reference validation. These
totals are software checks, separate from the measured display screens.

Remaining R3.6/R6.6 work: per-model license/texture/skin and stable pose review,
full model/backend runtime coverage, actor/plugin preservation, floor/ceiling
and map alignment, robot routes, and broader Fuel/fixture imports. R5.7–R5.9
still need actual arm/hand/mobile-manipulator controllers and measured missions.
R6.7 remains with its recorded terrain owner. The full project remains partial.
