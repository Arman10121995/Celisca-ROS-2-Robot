# Current Robot Lab status

October 13: implemented on master, experimental.

- **Control (R7.8).** Six path-tracking controllers now drive through
  Nav2's FollowPath: PID with anti-windup, LQR, constrained linear MPC,
  nonlinear MPC, feedback linearisation and backstepping. They are
  selected with `control:=` or the GUI's Motion controller slot.
  - The same control ids used to start side-topic demonstration nodes.
  - A shared supervisor rotates the robot in place, stops it before
    collisions and stops it when the pose goes stale (watchdog).
  - A model-plant benchmark covers tracking, disturbance, stability
    envelope, saturation and compute time. Nonlinear MPC tracks 90°
    corners eight times closer than the others. LQR has a false
    equilibrium 1 m off the path at 90° or more.
  - [Evidence](evidence/r78-control-benchmark-2026-10-13/README.md),
    [tutorial](../tutorials/control_comparison.md).
- **Control in Gazebo.** All six controllers reached all 15 nav_maze goals
  for Bumperbot. Backstepping and LQR held the plan closest, at 5.3 and
  5.6 cm cross-track RMS.
- **Sensor fusion (R7.5).** The fusion ids now run real filters:
  complementary, Mahony, Madgwick and a new attitude ESKF, plus a
  wheel+gyro EKF, an EKF with GNSS and a UKF. Before, the complementary
  filter's axes were swapped and the "UKF" was dead reckoning.
  - On modelled sensors, the ESKF holds tilt to 0.46° RMS. The gyro-heading
    EKF keeps position to 0.42 m over 300 s, where wheel dead reckoning
    drifts 18 m.
  - [Evidence](evidence/r75-fusion-benchmark-2026-10-13/README.md),
    [tutorial](../tutorials/sensor_fusion_comparison.md).
- **Planner reruns.** With a clearance margin, RRT/RRT\* reach 14/15 goals
  (were 8 and 9) and Voronoi 15/15. MPPI capped at 0.3 m/s still reaches
  7/15: it gets within 0.05-0.19 m but does not settle into Bumperbot's
  tight arrival window.
- **Missions.**
  - Workflow/reset screens pass 8/8 for Labbot and Bumperbot in all four
    backends ([evidence](evidence/r61-workflow-reset-2026-10-13/README.md)).
  - 13 of 20 4WS/mecanum obstacle missions pass
    ([evidence](evidence/r56-4ws-missions-2026-10-13/README.md)). The
    pivot-mode arrivals were aborted by a 10 s progress checker; the
    overlay is fixed and a rerun is queued.
- **Gazebo cars.** The car description had no ground-truth publisher, so
  Gazebo car missions timed out; it now has one.
- **Stale transforms.** In 9 of about 90 recent Nav2 trials (R4.3, the
  planner comparisons and the car missions), a Nav2 server's whole TF view
  froze while AMCL kept publishing. The robot then spun in place, overshot
  its goal, or the planner started from an old pose.
  - The probe has since caught it twice. A fresh listener kept receiving
    `map -> odom` while controller_server's newest transform stayed fixed.
  - So the server's subscription stops, not the publisher.
  - **Cause:** an ABBA deadlock in the installed tf2/tf2_ros 0.25.23
    ([ros2/geometry2#992](https://github.com/ros2/geometry2/issues/992)),
    fixed upstream in 0.25.24. Apt still has 0.25.23.
  - `scripts/build_tf2_deadlock_fix.sh` overlays 0.25.24 (ABI-identical
    headers). Validation runs follow.

October 12 (later): measured on master.

- **Celisca navigation.** The Celisca mission passes in all 32 cells (4
  simulators x Labbot/Bumperbot x plain/furnished floors 1-2). Each
  mission is a detour goal, cancel/stop, resume and near-obstacle goal.
  Final error vs truth is at most 0.139 m.
  - The furnished maps now contain their furniture.
  - Labbot and Bumperbot now default to Regulated Pure Pursuit with 0.55 m
    inflation, because DWB stops at a furnished wall end (recorded); DWB
    stays selectable.
  - [Evidence](evidence/p3-celisca-missions-2026-10-12/README.md).
- **MuJoCo speed.** Large-world MuJoCo steps are about twice as fast, with
  identical contacts. Labbot on celisca_floor_1 runs at RTF 0.675 (was
  0.31). [Evidence](evidence/p4-mujoco-step-2026-10-12/README.md).
- **Legged policies.** Model-matched rl_sar policies walk on `/cmd_vel`
  through each backend's in-step PD path.
  [Evidence](evidence/p7-rl-sar-mujoco-walk-2026-10-12/README.md),
  [tutorial](../tutorials/legged-policies.md).
  - MuJoCo: Go2, A1, B2, Go2W and the Unitree G1 29-DoF humanoid.
  - Isaac (PhysX drives at 200 Hz): Go2, A1, Go2W and G1. B2 overshoots
    and drifts there.
  - Gazebo (effort trajectory controller with the policy gains): Go2.
  - G1 is the first humanoid walking here from a downloaded policy: a
    recurrent LSTM actor, exported with its state as explicit inputs.
    It does 0.38-0.39 m/s straight, spins and strafes in MuJoCo and Isaac.
  - Isaac and Gazebo offer localization for these robots; mapping and
    navigation stay gated.
- **Planner comparisons.** These run on Bumperbot in nav_maze under
  Gazebo, with identical seeded goals for every planner.
  - The R4.3 predeclared A* vs NavFn comparison is done: 14/15 against
    15/15, time ratio 0.98
    ([evidence](evidence/r43-planner-comparison-2026-10-12/README.md)).
  - The five-controller local planning comparison is the first measured
    R9.2 tutorial ([tutorial](../tutorials/local_planning_comparison.md)).
  - The robot_lab RRT/RRT*/PRM/Voronoi planners and the pure-pursuit and
    follow-the-gap controllers now serve Nav2's actions. Selecting them
    therefore really plans and drives.
- **Drive patterns.** All 4WS steering patterns and mecanum drive in all
  four backends
  ([evidence](evidence/p56-drive-patterns-2026-10-12/README.md)).
- **Licenses.** 118 of 125 declared dependencies are resolved from
  installed metadata
  ([evidence](evidence/r91-license-inventory-2026-10-12/README.md)).
- **Support matrix.** It is regenerated from hashed reports: 211
  measured cells ([matrix](support-matrix.md)).
- **PX4 cleanup.** PX4 flights get a run id, a reaper and their own
  Harmonic discovery ports. A leftover flight had flooded every Gazebo run
  with ~6,500 log lines/s. Measurement scripts isolate Gazebo discovery.

October 12: the PX4 X500 gains an experimental RGB-D variant (upstream
`x500_depth`) and RTAB-Map 3D mapping with a saved, reloadable database and
exported cloud, selectable in the GUI; one mapping and one localization
flight on nav_obstacle are recorded as implementation diagnostics
([evidence](evidence/r510-aerial-mapping-2026-10-12/README.md),
[guide](../tutorials/px4_x500.md)). The October 9 continuation below was
completed (built, installed, takeoff support-band fix, control-column layout).

The [latest October 9 continuation](implementation-continuation-2026-10-09.md)
adds persistent Drone/Drive click highlights, native Stretch coordinated
navigate/articulate tasks, selected-world takeoff corridor/ceiling checks and
actual measured Nav2 queue workloads. These implementations await physical
qualification; historical robot/CI certificates retain their source scopes.

Updated October 9, 2026. New [implementation-first work](implementation-2026-10-09.md)
adds experimental Unitree policies, native joint/hand/Cartesian teach-replay
controls, measured Panda object scenes, Stretch sensor-gated mapping/Nav2,
static PX4 3D Plan/Execute, coplanar collision reduction and real concurrent
experiments. Nine changed packages built; these implementations have pending
physical/test qualification. Testing/validation is deferred until the initial
implementation batches. The previous measured extension checkpoint stays on **`master`**;
[its evidence](evidence/extensions-finish-2026-10-08/README.md) records exact runtime
source stages, publication and checks. The project is
**partial**. The [ledger](platform-status.yaml) records owners and task states;
the [roadmap](../../ROADMAP.md) defines acceptance. Historical audits and trial reports retain their original dates, revisions, failures and measurements.

## Go2 + go2/himloco measured verification — 2026-10-12

Fast-tier acceptance on the current source stage: official `scripts/test_tiers.sh fast` -> **PASS** (1017 passed, 4 deselected, ~3.5 min, archived `fast_tier_final.log`). Go2 + `go2/himloco` bounded live smoke (ROS_DOMAIN_ID 97, `nav_empty`, headless): policy `go2/himloco` RUNNING **1783 steps, zero command sent**, base height **0.286 m**, neutral orientation, SIGINT-clean exit (archived `status.json`, `smoke-ok-final.log`, `legged-smoke-20261012c.log`). Tutorial documentation: `## Run` sections present in every tutorial; `test_p6_benchmarking.py::TutorialDocumentationTests` -> **66 passed**; per-file manifest 50/51 exit 0; isolated suite 12 passed. Measured artifacts archived on the SSD at `/workspace/molar/ledger/status/evidence/` (SUMMARY.md, logs, status.json); ledger entries in `platform-status.yaml` (October 12, 2026 entries).

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
Occupancy v4 restores omitted open mesh walls. All 17 installed extension
worlds have real refreshed grids and unchanged spawns; 34 GUI selections
autofill their new map paths. Both bases now pass the short hospital route on
all four engines with unchanged physical endpoint/floor gates. Earlier failed
endpoints and the reverted 0.03 m experiment remain. MuJoCo performance stays
open: one matched Labbot hospital window measures 0.1305× real time.

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
| Worlds and occupancy generation | Fourteen dataset worlds/three examples; all 17 actual occupancy-v4 exports with unchanged world/spawn fields and GUI paths; eight short hospital routes; editable seeds, terrain slices and derived hospital pose/frame/static-snapshot repairs | Remaining reviewed routes/registration, disconnected regions, collision/contact/actor parity, upper hospital floors and 97 example fixtures |
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

Published runtime `f74b53f` passes required/scheduled CI (25 packages;
required fast 1002/five skips, scheduled fast 1004/three skips; 27 physics;
138 integration/13 skips; registry). Preceding MuJoCo 3.15 repair `4fe611a`
retains its separate successful CI stage. The predecessor `73fcc47` also passes required CI and Scheduled Full Test Suite
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

The later P4 camera-packing change preserves identical ROS wire messages and
passes four serialization cases plus a new physical Labbot hospital route
(0.046 m / +1.97°). Actual before/after clock windows remain about 0.13× real
time; large-map physics optimization remains open. See [the measured timing stages](evidence/extensions-finish-2026-10-08/mujoco-hospital-performance/README.md).
