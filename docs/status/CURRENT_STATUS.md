# Current Robot Lab status

Updated October 8, 2026. The current extension checkpoint stays on **`master`**;
[its evidence](evidence/extensions-finish-2026-10-08/README.md) records exact runtime
source stages, publication and checks. The project is
**partial**. The [ledger](platform-status.yaml) records owners and task states;
the [roadmap](../../ROADMAP.md) defines acceptance. Historical audits and trial
reports retain their original dates, revisions, failures and measurements.

## Available GUI workflows

Launch now places **Drive, Arm, Hand, Drone and their limits in a right-hand
column** beside experiment setup and command/algorithm review. Robot category
and type filters distinguish two/four wheels, two/four legs, single/dual arms,
mobile manipulators, hands and drones. Smaller windows switch between Set up
and Command & algorithms while keeping controls visible. Reopen the rebuilt
GUI to load the new layout. Read the [workspace guide](../tutorials/gui-workspace.md).

Robot and map selectors use complete parent families with exact source
variants. Components stay under their parent in Registry. **Registry →
Preview 3D** opens a native embedded viewport with orbit, pan, zoom and Fit.
Source scenes, including corrected hospital variants, are inspected across recorded stages;
textures, actors and large-mesh GPU performance remain separate work. Read the
[Registry guide](../tutorials/registry-3d.md).

Compatible algorithms and the Run/Copy command fill from the same selection.
Structural tags and a selectable asset do not grant controller or mission
support. Unsupported modes keep their actual explanations.

**Drone** now has all seven Drive buttons and gentler 0.3 rad/s yaw; actual
X500 flight/release/Stop/loss/landing pass in `nav_empty`. **Arm** adds measured
tool XYZ and 1 cm target buttons with separate Plan/Execute. The normal native
Panda/MuJoCo repeat passes submillimetre TCP errors, collision/reach rejection,
interruptions/reset and physical cube lift/release. Read the updated
[Panda](../tutorials/panda_arm.md) and [Drone](../tutorials/px4_x500.md) guides.
Bumperbot/MuJoCo hospital navigation retains failed body endpoints and the reverted 0.03 m goal experiment. The stationary lidar diagnostic identifies omitted open wall chains; corrected occupancy v4 improves measured grid alignment, and the unchanged normal navigation repeat is active.

## Measured work and its limits

| Robot or feature | Recorded working scope | Remaining qualification |
|---|---|---|
| Bumperbot and Labbot | Bounded Drive/Stop/watchdog, named mapping/reset and clear/furnished navigation screens; Celisca 20% scale/spawn repairs | Remaining Celisca/backend/mode cells, longer avoidance routes and repeated goals |
| TurtleBot3 Burger/Waffle/Waffle Pi | Twelve physical Drive/source-LDS screens and 48 localization/reset, 2D SLAM/export and clear/obstacle Nav2 screens across Gazebo, MuJoCo, PyBullet and Isaac | Other maps, longer routes, RGB rendering/vendor firmware; RGB-only cameras leave 3D SLAM unavailable |
| TurtleBot4 Standard/Lite | Eight final physical Drive/lidar/RGB-D screens and forty localization/reset, 2D/3D SLAM/export and clear/obstacle Nav2 screens across the four engines | Other maps/routes, materials, docking and vendor hazard/firmware behavior |
| Husky | Four physical Drive/sensor screens and twenty localization/reset, 2D/3D mapping/export and clear/obstacle navigation screens across all four engines; normal GUI defaults/autofill and production navigation | Additional maps/routes, payload/outdoor/vendor hardware, contacts and broader estimator qualification |
| Ackermann/rear/anti-Ackermann, 4WS and mecanum | Physical steering/roller drives, named body-truth goals and selected mapping/reset/watchdog screens | Current-setting repeats across patterns/backends/maps/modes, contacts, cancellation and long missions |
| Go2 and Berkeley Humanoid Lite | Repaired MuJoCo startup and bounded upright walks on the recorded Celisca path | BHL sustained-turn/low-speed stalls; Go2 terrain/recovery; remaining humanoid/quadruped policies and mission cells |
| Native Menagerie Panda | MuJoCo joint/Home/Stop, Hand, physical cube lift/release and MoveIt KDL/OMPL/FCL Plan/Execute in named static worlds | Servo, attached/dynamic objects, repeated pick/place, other arms/hands/backends and mobile manipulation |
| PX4 X500 | Real Gazebo Harmonic takeoff, hover, goals, Drive, altitude controls, landing/disarming and bounded command loss in `nav_empty`/`nav_obstacle` | Other installed worlds' spawn/ceiling/flight cells, obstacle-aware aerial planning/mapping and other backends |
| Worlds and occupancy generation | Fourteen dataset worlds/three examples; editable GUI seeds, actual refreshed dependency-guarded exports, separate terrain triangle slices and derived hospital pose/frame/static-snapshot repairs | Remaining reviewed routes/registration, disconnected regions, collision/contact/actor parity, upper hospital floors and 97 example fixtures |
| Terrain | Shared heightfield conversion and recorded Gazebo/MuJoCo/PyBullet geometry/contact screens | Separately owned GUI/provider workflow, Isaac terrain contacts and traversal missions |

Exact commands, source fingerprints, numeric outcomes and retained negatives
are linked from the [October 8 continuation](continuation-2026-10-08.md),
[checklist](CHECKLIST.md) and [support matrix](support-matrix.md). The final
control-column trials additionally measure neutral Burger/PyBullet Drive and
native Panda TCP errors of 0.00649/0.00639 m and 0.780/0.765°. These do not
rerun every earlier robot mission.

## Inventory and verification

The installed Jetson snapshot contains **137 raw launch profiles**: 24 core
and 113 installed extensions. Consolidation presents **72 complete robot
families / 112 selectable variants**, with 25 inspection components/reference
profiles kept out of standalone Launch. **43 world profiles** appear under
33 map families. The merged registry contains 139 robot records, 43 environments,
48 algorithms, 23 scenarios and 21 experiments. These are inventory counts.
Installed asset availability is host-specific.

The [evidence index](runtime-evidence-index.yaml) keeps individual positive/negative
trials and the support matrix recomputes each latest exact cell.
Unlisted cells are untested; all full-release gates remain blocked. Static
previews, catalog counts and unit tests are separate from mission evidence.

| Check | Recorded result | Scope |
|---|---|---|
| October 8 local checks | 25 core packages built; 1005 fast/one skip/four deselections, 104 GUI, 27 physics and 147 dedicated-display integration tests pass | Current source stage; recorded warnings and desktop-layout failure retained |
| October 7 local GUI | 94 passed; five additional final layout checks passed | Actual Tk layout, commands, selections, controls and resizing |
| Preceding local fast/integration stage | 936 fast passed, one skip/four deselections; 146 integration passed | Named control-column source stage; later presentation changes have focused GUI checks |
| Remote CI at `091d388` | 25-package core build; 933 fast passed/four skips/four deselections; 20 physics passed; 134 integration passed/12 skips; registry validation passed | Exact published source on the CI host, optional `orbslam3` excluded; no robot mission qualification |
| Installed/published parity | Twelve GUI/runtime files match the published revision | [Actual hashes](evidence/gui-controls-column-2026-10-07/published-source.json) |

Published `4fe611a` passes required/scheduled CI (25 packages; required fast 1002/four skips, scheduled fast 1003/three skips; 27 physics; 134 integration/13 skips; registry) after the MuJoCo 3.15 repair. The predecessor `73fcc47` also passes required CI and Scheduled Full Test Suite
after the shared-utilities import fix. [October 8 evidence](evidence/extensions-finish-2026-10-08/README.md)
retains those exact complete logs separately from the final runtime checks. Local tests, remote CI and simulator trials keep
separate source stages and skip/failure records. Clean-host mission reproduction,
physical joystick/hardware checks and concurrent-simulator qualification remain open.

## Work order for continuing agents

1. Follow **motion → saved/reloaded SLAM → navigation** for each complete
   compatible variant/backend/map. Preserve the control column and measured
   Husky/TurtleBot3/TurtleBot4/Panda/PX4 scopes; claim work in the ledger.
   Read [the AI readiness guide](../AI_ROBOT_READINESS_GUIDE.md).
2. Continue R3.6 asset/controller/license/material work and R6.5/R6.6 world,
   occupancy, collision/spawn/actor qualification. Coordinate with the owners
   of the occupancy and R6.7 terrain lanes.
3. Continue R5.10 flight-world checks, R5.7/R5.8 Servo/object/hand/backend work
   and R5.9 mobile manipulation with actual outcome measurements.
4. Resume the original wheel/legged map matrix, physical joystick, MuJoCo
   performance, sustained locomotion/recovery, fair algorithm comparisons,
   concurrency and clean-host reproduction.

Use the [agent handoff](../AGENT_HANDOFF.md),
[extension guide](../ASSET_EXTENSION_GUIDE.md),
[patch guide](../PATCH_EXECUTION_GUIDE.md) and
[workflow](../WORKFLOW.md). Work directly on `master`, preserve other agents'
edits and keep large artifacts on the mounted workspace SSD.
