# Robot Lab support matrix

Updated October 8, 2026 against the extension checkpoint and its exact
pre-trial source manifests; see the publication receipt for the pushed source. The platform is **partial**. This page
summarizes implementation and named evidence; the [ledger](platform-status.yaml)
owns task states and the [roadmap](../../ROADMAP.md) defines acceptance.

## Read support by exact composition

The [generated measured matrix](evidence/extensions-finish-2026-10-08/support-matrix-current.md)
and [JSON report](evidence/extensions-finish-2026-10-08/support-matrix-current.json)
validate **205 indexed reports / 147 distinct measured cells**. The latest
measurement for an exact robot/map/backend/task/steering cell controls its
record. Unlisted combinations are **not tested**. All full-release gates
remain blocked by unfinished robot, world, algorithm, backend and reproduction
work. The [runtime index](runtime-evidence-index.yaml) retains report/source
hashes and repeats; static previews do not become mission records.

| Evidence level | What it establishes |
|---|---|
| Catalog/import | Source, profile and assets exist; controller support remains separate |
| Source/physics checks | The named software assertion or real engine case passes |
| GUI/preview checks | Widgets, commands or source geometry render correctly; static inspection |
| Measured runtime screen | The exact named plant, sensors and bounded outcome pass |
| Full qualification | All task acceptance, repeats/failures and provenance are satisfied; still incomplete for the full platform |

Historical registry maturity labels and generated demonstration scores do not
replace measured runtime evidence.

## Current robot workflows

| Robot/plant | Backend and measured worlds | Available scope | Remaining work |
|---|---|---|---|
| Bumperbot / Labbot | Named four-backend and Celisca/arena stages | Physical Drive, selected localization/mapping/reset and Nav2 goals; spawn/scale/furniture repairs | Complete furnished/actor/map/mode matrix, longer avoidance and repeated goals |
| TurtleBot3 Burger/Waffle/Waffle Pi | All four engines; `nav_empty` and navigation `nav_obstacle` | Twelve source LDS/Drive screens; 48 localization/reset, 2D SLAM/export and clear/obstacle navigation screens; normal algorithms/autofill | Other maps/routes, vendor behavior and RGB rendering. RGB-only cameras do not enable 3D SLAM. |
| TurtleBot4 Standard/Lite | All four engines; named `dataset_room2`, `nav_empty` and `nav_obstacle` stages | Display/state, eight final Drive/lidar/RGB-D screens and forty mode/reset/export/navigation screens; normal five-mode selection | Other maps/routes, materials, hazards/docking and vendor firmware |
| Husky | All four engines; `nav_empty` / `nav_obstacle` | Four-wheel Drive/source sensors; twenty localization/reset, 2D/3D mapping/export and clear/obstacle navigation screens; normal defaults/autofill | Other maps/routes, vendor hardware/firmware and broader estimator qualification |
| Ackermann/rear/anti-Ackermann | Named wheeled trials | Physical steering, watchdog and selected navigation evidence | Slalom/parking, full current-setting backend/map/mode and reset/contact matrix |
| Four-wheel steering | Four engines; named clear/obstacle and mapping stages | Opposite-phase, in-phase/crab and pivot; all sixteen clear pattern/backend screens and tighter named crab/obstacle repeats | Repeat all final patterns/settings; final heading, contacts, reset, command loss and long mapping missions |
| Mecanum | Four engines; named clear/obstacle and PyBullet mapping stages | Physical passive rollers/lateral travel, clear-map goals, actual 2D SLAM/reset and selected navigation | Remaining current-setting obstacle/map/mode/backends and repeat qualification |
| Go2 / Berkeley Humanoid Lite | MuJoCo; bounded flat/Celisca work | Repaired startup/feedback and bounded walks, explicit policy gates | BHL held-turn stalls; Go2 stairs/get-up; robust terrain/goal work and other model policies |
| Native Menagerie Panda | MuJoCo; named `dataset_room2` and static `nav_empty` stages | Arm joint/Home/Stop; Hand and physical cube lift/release; MoveIt Cartesian Plan/Execute with collisions, invalidation and interruption/reset | Servo, attached/dynamic scenes, repeated object tasks, other hands/arms/backends and mobile manipulation |
| PX4 X500 | Gazebo Harmonic; `nav_empty` / `nav_obstacle` | Real FCU flight, GUI Takeoff/Hold/Land, altitude/Drive and goal/command-loss screens | Other worlds' flight/spawn/ceiling cells, obstacle-aware aerial planning/mapping and other engines |
| Quadrotor SITL legacy fixture | Display fixture | Legacy description only; retired offboard example | Use the measured PX4 X500 Flight profile; no legacy flight mission claimed |
| Remaining imported assets | Profile-declared Display backends; native MJCF restricted to MuJoCo | Installed source descriptions and inspection; exact controller gates | Per-model rest/material/license, actuation/sensor/reset and class mission qualification |

[October 8 continuation](continuation-2026-10-08.md),
[TurtleBot3](../tutorials/turtlebot3.md), [TurtleBot4](../tutorials/turtlebot4.md),
[Panda](../tutorials/panda_arm.md), [PX4](../tutorials/px4_x500.md) and
[Go2](../tutorials/go2.md) link exact commands and measurements. Localization
without a locomotion policy is not a walking/navigation mission.

## GUI, worlds and inventory

Launch contains a right-hand Drive/Arm/Hand/Drone control column with limits,
compatible algorithm defaults, exact command autofill and robot category/type
filters. Registry shows complete families, exact variants and nested components
in a native embedded 3D viewer. Static preview does not command a running robot.
Final normal Burger/PyBullet neutral/Drive and Panda Plan/Execute regressions
pass against the published layout; the earlier sample-count failure remains.
See [GUI evidence](evidence/gui-controls-column-2026-10-07/README.md).

The installed Jetson snapshot has 24 core plus 113 extension launch profiles,
consolidated into 72 complete robot families/112 selectable variants, with
25 inspection profiles kept out of standalone Launch. There are 26 core plus
17 installed world profiles under 33 map families. The merged registry has
139 robot records, 43 environments, 48 algorithms, 23 scenarios and 21
experiments. Inventory is host-specific and grants no universal support.

Fourteen dataset worlds and three Gazebo examples are installed. UPO collision
height-slice exports include `nav_obstacle` and both furnished Celisca floors.
They describe a seed-connected static region; reviewed occupancy registration,
disconnected rooms/floors, actor behavior and world/backend spawn/collision
missions remain. Hospital has six extreme source fixtures requiring actual
placement review; preview Fit does not repair that world. Ninety-seven other
Gazebo example SDFs remain plugin/robot fixtures to review.

Shared heightfields have named Gazebo/MuJoCo/PyBullet geometry/contact screens.
Terrain GUI/provider work, Isaac contacts and traversal missions remain in the
separately owned R6.7 lane. Physical joystick and large-map MuJoCo performance
remain P2/P4.

## Software and release boundary

Published `091d388` CI passes the full 25-package core build, 933 fast checks
with four skips/four deselections, 20 physics checks, 134 integration checks
with twelve skips and registry validation. The final local GUI has 94 passing
checks; preceding local fast/integration counts belong to their own source
stage. [CI evidence](evidence/ci-extensions-2026-10-07/README.md) and
[Testing](../TESTING.md) retain the exact scopes.

| Registry category | Entries | Qualification boundary |
|---|---|---|
| Perception | 8 | Includes utilities/kernels; actual method I/O and fair comparisons remain |
| Localization | 7 | Named stack missions are separate from seven interchangeable methods |
| State Estimation | 5 | Numerical/filter contracts and measurement provenance remain per method |
| Sensor Fusion | 5 | Input/frame/covariance and fair comparison work remains |
| Global Planning | 7 | Dispatch/plugin identity, feasibility and fair comparisons remain |
| Local Planning | 6 | Robot/steering compatibility and measured avoidance matter |
| Control | 10 | Wheel/joint/policy/flight and utility entries are separate contracts |

The 48 algorithm catalog entries include utilities, kernels and adapters.
Five substantive, correctly named, ROS-connected and fairly measured
alternatives in every requested category remain R7/R9.2. Comparison drafts,
benchmark scaffolding, clean-host installation inventories and generated
metrics cannot close that work. Multi-simulator concurrency, full provenance/
license review and clean-host mission reproduction remain open.

## Known Limits

Hardware HIL, physical joystick trials and clean-host simulator missions remain
unqualified. Imported model/world display and the measured static previews do
not supply missing controllers, dynamic actors or universal backend parity.
Retain each task's failure/skip records and exact map/plant/revision boundary.

Preserve the [October 2 audit](audit-2026-10-02.md),
[historical October 1 report](support-matrix-2026-10-01.md) and dated runtime
artifacts. The old September support table is retained in Git history; its
claims do not override these newer exact cells. Continue extensions before
older milestones using the [checklist](CHECKLIST.md),
[workflow](../WORKFLOW.md) and [agent handoff](../AGENT_HANDOFF.md).
