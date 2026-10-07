# Current Robot Lab status

Updated October 7, 2026. Runtime checkpoint: **`091d388` on `master`**, followed
by documentation updates with unchanged runtime files. The project is
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
Nine distinct source scenes are inspected across the recorded stages;
textures, actors and large-mesh GPU performance remain separate work. Read the
[Registry guide](../tutorials/registry-3d.md).

Compatible algorithms and the Run/Copy command fill from the same selection.
Structural tags and a selectable asset do not grant controller or mission
support. Unsupported modes keep their actual explanations.

## Measured work and its limits

| Robot or feature | Recorded working scope | Remaining qualification |
|---|---|---|
| Bumperbot and Labbot | Bounded Drive/Stop/watchdog, named mapping/reset and clear/furnished navigation screens; Celisca 20% scale/spawn repairs | Remaining Celisca/backend/mode cells, longer avoidance routes and repeated goals |
| TurtleBot3 Burger/Waffle/Waffle Pi | Twelve physical Drive/source-LDS screens and 48 localization/reset, 2D SLAM/export and clear/obstacle Nav2 screens across Gazebo, MuJoCo, PyBullet and Isaac | Other maps, longer routes, RGB rendering/vendor firmware; RGB-only cameras leave 3D SLAM unavailable |
| TurtleBot4 Standard/Lite | Eight final physical Drive/lidar/RGB-D screens and forty localization/reset, 2D/3D SLAM/export and clear/obstacle Nav2 screens across the four engines | Other maps/routes, materials, docking and vendor hazard/firmware behavior |
| Ackermann/rear/anti-Ackermann, 4WS and mecanum | Physical steering/roller drives, named body-truth goals and selected mapping/reset/watchdog screens | Current-setting repeats across patterns/backends/maps/modes, contacts, cancellation and long missions |
| Go2 and Berkeley Humanoid Lite | Repaired MuJoCo startup and bounded upright walks on the recorded Celisca path | BHL sustained-turn/low-speed stalls; Go2 terrain/recovery; remaining humanoid/quadruped policies and mission cells |
| Native Menagerie Panda | MuJoCo joint/Home/Stop, Hand, physical cube lift/release and MoveIt KDL/OMPL/FCL Plan/Execute in named static worlds | Servo, attached/dynamic objects, repeated pick/place, other arms/hands/backends and mobile manipulation |
| PX4 X500 | Real Gazebo Harmonic takeoff, hover, goals, Drive, altitude controls, landing/disarming and bounded command loss in `nav_empty`/`nav_obstacle` | Other installed worlds' spawn/ceiling/flight cells, obstacle-aware aerial planning/mapping and other backends |
| Worlds and occupancy generation | Fourteen installed dataset worlds and three Gazebo examples; actual UPO/static-collision grid exports including furnished Celisca | Reviewed navigation registration, disconnected regions, collision/spawn/actor parity, hospital source-pose defect and 97 remaining example fixtures |
| Terrain | Shared heightfield conversion and recorded Gazebo/MuJoCo/PyBullet geometry/contact screens | Separately owned GUI/provider workflow, Isaac terrain contacts and traversal missions |

Exact commands, source fingerprints, numeric outcomes and retained negatives
are linked from the [October 7 continuation](continuation-2026-10-07.md),
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

The evidence index retains **122 records / 114 exact measured cells**.
Unlisted cells are untested; all full-release gates remain blocked. Static
previews, catalog counts and unit tests are separate from mission evidence.

| Check | Recorded result | Scope |
|---|---|---|
| Final local GUI | 94 passed; five additional final layout checks passed | Actual Tk layout, commands, selections, controls and resizing |
| Preceding local fast/integration stage | 936 fast passed, one skip/four deselections; 146 integration passed | Named control-column source stage; later presentation changes have focused GUI checks |
| Remote CI at `091d388` | 25-package core build; 933 fast passed/four skips/four deselections; 20 physics passed; 134 integration passed/12 skips; registry validation passed | Exact published source on the CI host, optional `orbslam3` excluded; no robot mission qualification |
| Installed/published parity | Twelve GUI/runtime files match the published revision | [Actual hashes](evidence/gui-controls-column-2026-10-07/published-source.json) |

The [CI report and hashed complete log](evidence/ci-extensions-2026-10-07/README.md)
retain the exact result. Local tests, remote CI and simulator trials keep
separate source stages and skip/failure records. Clean-host mission reproduction,
physical joystick/hardware checks and concurrent-simulator qualification remain open.

## Work order for continuing agents

1. Preserve the control column, exact command/controller gates and completed
   TurtleBot3/TurtleBot4/Panda/PX4 scopes. Claim ownership in the ledger.
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
