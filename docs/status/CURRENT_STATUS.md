# Current Robot Lab status

November 14 Claude: [external source licenses (R9.1)](evidence/r91-external-sources-2026-11-14/README.md). The licenses of the source trees the lab runs are now read from their own license files. PX4 v1.16.2 is BSD-3-Clause, and 23 of its 25 submodules are identified. 36 vendor trees resolve. 7 ship no license file (BlueROV2, Husky and the model datasets); keep them local until each model's terms are recorded.

November 14 Claude: occupancy grids and sensor faults (R6.5/R6.2):
- **Free regions:** generated grids now list every free region, and `all` mode also frees closed rooms the seed cannot reach. On `nav_maze` the labelling matched the map plugin cell for cell.
- **Register reviewed grid…:** a new Worlds step and the only way a grid becomes the one Nav2 uses. It refuses grids whose files or world changed, or whose spawn isn't free.
- **Sensor-fault relay:** Gazebo launches take `sensor_faults:=<scenario>` to put seeded, repeatable noise, bias, drift, delay, dropout, occlusion and outliers on live sensor topics, never on truth. Unit-tested; no degraded mission run yet.

November 14 Claude: legged, car and asset implementation, none measured yet:
- **Legged training (R5.2/R12):** new `get_up` task (starts from collapsed poses settled in physics) and a stairs curriculum (2–18 cm rises). The rough-terrain curriculum now really reshapes the terrain; before, it changed only a number. A first Go2 get-up policy is training.
- **Cars (R5.5):** `car_parking` turns the measured heading shuffle and depth trim into a reusable park-at-heading runtime for car navigation. It has unit tests only.
- **Display-only robots (R3.6):** all 77 now show a reason derived from their own source model. The generic servo accepts unnamed vendor actuators, which adds UFactory Lite6 and Google Robot arm jogging (not yet validated).

November 14 Claude: [flipper-aware terrain policy and gait slip recovery (R10.7/R10.6)](evidence/flipper-stairs-2026-11-14/README.md) are implemented and experimental. **Automatic terrain flippers** reads the terrain ahead from the robot's own depth camera and its pitch/roll from the IMU, and sequences a step climb. Measured at a 0.18 m step, neither the policy nor fixed flippers climbs it: the 4 Nm flippers are pushed back by the hull, and the vehicle rears past the 0.6 rad tilt guard. A stronger flipper actuator is needed. Terrain gaits now pause and recover from slip or support loss instead of halting (unit-tested only).

November 14 Claude: [MuJoCo marine plant, current feedforward and volumetric AUV routes (R10.4/R10.5)](evidence/marine-mujoco-currents-2026-11-14/README.md) are implemented and experimental. `simulator:=mujoco` runs the unchanged ROV controller on ported Fortress water forces and matches Gazebo: 1.03 vs 1.07 m surge and 1.44 vs 1.50 rad yaw. A damping-based current feedforward cuts the 15 s drift in a 0.1 m/s cross current from 0.37 m to 1 cm on both simulators. AUV routes now dive or climb around obstacles and rank by time through the current; these are unit-tested only, with no AUV mission flown yet.

November 14 Claude: [distributed swarm behaviours, sensed coverage and member-loss replanning (R10.3)](evidence/swarm-distributed-2026-11-14/README.md) are implemented and experimental. A ROS2swarm port runs dispersion, attraction, aggregation and flocking per robot from its own scan. Coverage now counts cells the scans actually saw, and ground and air groups can continue without a lost member. Measured with three robots: dispersion 0.94 → 3.15 m, and coverage reached 90 % of a 6 × 6 m area after one robot's localization was stopped mid-mission. Aggregation does not pass yet, and RGB-D-off sessions blind a 90° sector of the Bumperbot lidar (keep RGB-D on). Scan bearings now include the Bumperbot's 3.14 rad lidar yaw.

November 14 Claude: [fleet shared zones, crossing conflicts and calibrated RMF energy (R10.2)](evidence/fleet-zones-energy-2026-11-14/README.md) are implemented and experimental. Single-lane passages found in every map, door sweeps and docks are reserved one robot at a time: a robot is held 1 m before a zone a peer holds, with its task suspended, not canceled. Two robots crossing the facility door at once passed natively (one held 12.4 s, closest 2.40 m) and under Open-RMF (closest 0.92 m). RMF now plans with battery drain from a drivetrain calibration (Bumperbot/Labbot 6 %/15 % held-out energy error). Recover held lift can drive the passenger out (unit tests only). Chargers and doors in other worlds remain.

November 14 Codex: native Stretch task recipes now offer position-only, keep-measured-pad and explicit world-pad RPY targets, with private source-model orientation IK and live/final angular checks. [Guide and limits](reach-orientation-2026-11-14.md). Unreachable model-specific orientations remain refused. This is implementation, not a completed loaded-object mission. Claude retains fleet/drone/import work.

November 14 CI: the preceding placement revision failed one ROS launch importer in the unsourced fast tier (1,282 passed, 42 skipped). That unchanged test now runs in the required sourced integration tier and passes locally; the exact-revision published rerun is pending. Historical green CI receipts apply to their named revisions.

November 14 Codex: Panda's Arm page now offers measured support-surface placement targets. It computes tool poses from the acknowledged grasp offset, validates object footprint fit and separately plans above/lower stages; editing and release remain explicit. [Scope and checks](support-placement-2026-11-14.md). Full physical pick/place qualification and unattended sequencing remain open. Claude retains drone/import work.

November 13 Claude: [drone moving obstacles and sensor loss (R5.11)](evidence/drone-moving-2026-11-13/README.md) are implemented and experimental. Depth-cluster Kalman tracks, predicted sweeps, removal of people's ghost trails from the map, walkway look-both-ways and overflight, evasion, and hold-then-land on camera loss. Takeoff now waits for a steady resting EKF heading and height, and restarts PX4's estimator if needed. In `aerial_moving`, runs 32 and 34 passed: goal reached, walkers no closer than 1.08 m, camera loss → hold → land. Run 33 failed before the mission on an estimator fault the gate now refuses. A second backend, PX4 with a MuJoCo plant (`px4_iris`), flew a plain takeoff, route and landing. Gazebo only for moving obstacles; repeats and reacting people remain validation.

November 13 Claude: [real-world area import (R6.8)](evidence/real-world-import-2026-11-13/README.md) is implemented. Globe → elevation, buildings, roads, trees, water, plus SUMO people and traffic, for four simulators. The people/traffic world (70 actors) now loads in Gazebo with matching support heights after two repairs: actor skin paths in the temporary world copy, and a rigged vehicle skin.

November 13 Codex: native Stretch reaches now search bounded configuration detours when the direct arm path is blocked, using the existing source controls and private collision snapshot. Fifteen focused checks include a dense native-contact detour audit; [scope and limits](stretch-search-2026-11-13.md). This adds planning implementation, not a completed loaded object mission. Claude retains drone/import work.

November 13 Codex: [Panda measured object selection and grasp-stage planning](arm-objects-2026-11-13.md) is implemented for native free boxes. The GUI generates source-width/padding candidates and sends explicit collision-aware approach/grasp/lift plans; execution stays separate. Fourteen focused checks, four package builds and installed widget smoke pass. Full new pick/place missions and other geometry/arms/backends remain open. Claude retains drone/import work.

November 13 Codex: [native hand force feedback](hand-force-2026-11-13.md) is implemented with unchanged source gains/limits and explicit GUI activation. A pinned Robotiq component fixture unloaded roughly 12 N to 5.25 N per side for a 5 N request. Forty focused/regression/tutorial checks and GUI protocol smoke pass; three packages build. This is separate from normal GUI/hand-family/lift qualification.

November 13 Codex: [Stretch local pick/place implementation](../tutorials/native-object-tasks.md) adds source-control IK, private collision planning, contact-gated grip/stow steps, GUI recipe generation, a normal free-object bench map and actual Nav2 footprint receipts. 42 focused checks, both installed-source FK/scene attachment and GUI builder smoke pass; three packages build. No completed loaded-manipulation mission is claimed. Claude retains the drone/import lane.

November 13 Codex: native Stretch mobile-task object checkpoints and carry-loss monitoring are implemented as a candidate; [scope and usage](mobile-custody-2026-11-13.md). Physical grasp/transport/release acceptance remains pending. Claude retains the aerial/import lane.
November 13 Codex continuation: payload geometry is now included in both native Stretch costmap footprint publications, with a matching/freshness transport interlock. 26 focused tests, an installed GUI smoke and three package builds pass. Publication is not a Nav2-consumption receipt; physical loaded navigation remains pending.

November 12–13 top priority myAGV Pro: complete official CAD base installed in the normal GUI. Gazebo Display control/sensing/command loss and one normal-GUI Nav2 goal are measured (arrival error 0.125 m / 2.23°). The portable roller variant drives in MuJoCo (0.238 m/s), PyBullet (0.248) and Isaac (0.247), and a PyBullet Nav2 mission passed (0.034 / 0.118 m). R14.3 myAGV Pro + myCobot Pro630: drive plus bounded six-joint arm, Stop and watchdog screens pass in all three; the GUI *Mobile arm* page passed live on PyBullet. Arm mass, speeds, efforts and mount are declared estimates; gripper, planning, pick/place, other maps and hardware remain. R14.5 (2026-11-13): robot lists mark each robot ● measured / ◐ experimental / ○ display only / ✕ unavailable with its modes, from recorded evidence ([guide](../tutorials/gui-quickstart.md#read-a-robots-status-in-the-lists)). R6.7 terrain (2026-11-13): **Worlds & mapping → Terrain** generates procedural or imported-raster terrain worlds with a regenerating manifest; support height measured within 2.1 mm on Gazebo, MuJoCo, PyBullet and Isaac ([evidence](evidence/terrain-generation-2026-11-13/README.md)). R14.4: contributor chapters 9–12 ([extending the lab](../learn/09-extending-the-lab.md)) cover adding robots, worlds and GUI controls. [Guide](../tutorials/myagv-pro.md) · [receipt](evidence/myagv-pro-2026-11-12/README.md).


November 12 source-counterpart continuation: complete MIT Light Robotics octopod installed in the normal GUI, eight-leg classification, native locomotion toggle and command autofill. One actual MuJoCo no-map screen recorded forward/reverse/turn, real joints/TF and command-loss neutral stance within source torque limits. Speed tracking remains uncalibrated; no terrain/SLAM/navigation promotion. Tracked profiles were already complete Apache-2.0 Gazebo source derivatives; their original geometry is verified, with no new runtime claim. [Guide](../tutorials/source-octopod.md) · [receipt](evidence/counterparts-2026-11-12/README.md).


November 12 latest R14.1: source RB-SUMMIT-STEEL is installed with original mechanics, contact model and limits, a Fortress friction/ROS2-control port, declared kit, actual Display holonomic control and GUI defaults. Measured forward/reverse/strafe/yaw and sensor data; publisher-loss travel 0.112 m, final speed zero. Preserve native XML prefixes and original friction frame through fixed-joint merging; expired leases reset wheel motion and retain 4WS angles. Higher-mode GUI/footprint implementation is separate from mission qualification. Continue remaining source-search decisions and license review, then R6.7. [Receipt](evidence/counterparts-2026-11-12/README.md).

November 12 latest R14.1: BlueROV2 Heavy is now a separate experimental source candidate with exact-frame conversion, native forces/watchdog, declared ENU/RGB-D kit and normal GUI Marine command autofill. One actual run measured 1.1373 m travel and 1.4250 rad yaw; source-license/third-party mesh review and broader missions remain pending. Continue Summit-XL and remaining R14.1 search rows before R6.7. [Receipt](evidence/counterparts-2026-11-12/README.md).

November 12 R14.1 continuation: complete VRX WAM-V H is installed separately with a private Fortress ABI port of upstream water equations, source-bounded thrust, normal GUI Marine controls/classification and RGB-D autofill. One clear-water run measured 1.8785 m travel and 0.7413 rad turn; no broad mission/wave/contact qualification. Continue BlueROV2, Summit-XL and remaining source-search rows before R6.7. [Receipt](evidence/counterparts-2026-11-12/README.md).

This page records what was measured. **The goals and remaining work are in one list: the
[roadmap](../../ROADMAP.md)** (implementation queue first, then validation). Remaining-work
notes in the dated entries below are history.

November 12: **initial R13.3 camera implementation complete**, including a trained experimental
checkpoint on SSD and GUI/CLI autofill. Actual Gazebo fixture rendering produced 96 views in 74.239 s;
eight CPU epochs measured held-out mAP@0.5 0.564. Deployed recognition remains weak for cables
(1/13 matches at score 0.4) and has shoe false positives; no learned-camera robot mission is qualified.
[Model receipt](evidence/vacuum-camera-model-2026-11-12/README.md). Continue the next implementation
item, R14.1. The CI repair passes all required gates at `ecaaa133`, run 43155923238.

November 11 CI review: the recent CI failures occurred in ROS setup while the hosted runner
upgraded Firefox through an unreachable Snap store. Both workflows now hold its preinstalled
version during setup; exact published-run verification remains pending. All required project
checks remain enabled. [Receipt](evidence/ci-firefox-2026-11-11/README.md).

November 11 camera continuation (R13.3): the learned-detector executable and GUI Setup selection,
RGB/depth pairing, Objects snapshot gallery, disjoint training/validation split and independent
swept-body clearance gate are implemented. 105 focused vacuum/support checks and 13 GUI checks pass;
three packages build. An actual installed ROS transport diagnostic received a timestamped crop from
explicitly synthetic frames. **No trained checkpoint or new learned-detector robot mission is claimed.**
See [receipt](evidence/vacuum-camera-2026-11-11/README.md) and [guide](../tutorials/vacuum-camera.md).
The user resumed work; the current implementation queue supersedes the historical November 3 stop.

November 10:
- **Robot vacuum** (R13, OOMWOO-class `oomwoo`, small_house): all
  recorded cleaning jobs pass `cleaning_screen`:
  - office 95.4 %;
  - keep-out zone 99.6 % with no body entry or wall crossing;
  - spot beside furniture 90.7 %;
  - hidden low box 97.5 %, found by the LiDAR stall detector;
  - bedroom with 10 recharge stops 98.4 %;
  - MuJoCo keep-out zone 99.6 %;
  - whole home 96.3 % (141.6 m², 77 min, one auto-empty stop).

  - mapping run on a live SLAM map: 6 rooms, after two fixed failures.

  Every run docked within 0.051 m by truth
  ([evidence](evidence/vacuum-2026-11/README.md)).
- **Robot vacuum, later on November 10** (same evidence):
  - **Gazebo bumper** works (world-level Contact system): the hidden-box run bumps 4 times (97.0 %).
  - **Live GUI session** through the real Vacuum tab: an office picked on the room map, paused (truth still
    for 5 s) and resumed, 90.2 %; a zone with a no-go box and a wall drawn with the map tools, 99.5 %.
  - **PyBullet** 97.9 % and **Isaac** 99.3 % on the keep-out zone.
  - **Repeats:** office 95.4 / 94.4 / 95.0 %, keep-out zone 99.6 / 99.4 / 99.4 %, spot 90.7 / 91.2 / 90.2 %.
  - **New features:**
    - vacuum + mop with a rug as a carpet: 99.4 %, the mop never on the rug;
    - carpet boost: 99.7 %;
    - camera floor objects (cable, sock, shoe, pet mess; ideal segmentation detector): all recognised,
      0 contacts after a fix, 86.2 %.
  - **Second building** (`celisca_floor_1_furniture`): **fails** in five runs. Each failure led to a fix:
    another session's `/clock` on the shared domain; self-walling bump marks; wedging under furniture;
    no headless error report; a drifted estimate in a closed map pocket. A rerun with all fixes is queued.
- **Roadmap:** restructured into one goal list with an implementation queue and per-task how-to
  ([ROADMAP](../../ROADMAP.md)); open-source counterparts for self-made robots are task R14.1.
- **Docs:** [Learn Robot Lab from zero](../learn/README.md), an illustrated
  beginner course:
  - 8 chapters, 46 diagrams and 21 generated figures/animations;
  - visual sections in the algorithm tutorials;
  - the GUI's **Docs & tutorials** library shows it structured, with
    figures, animations and rendered diagrams.
- **Fix:** `sensor_fusion.ComplementaryImu` integrated the wrong gyro axes
  (found while drawing its figure); fixed and tested.

November 9 (recorded from the November 5 queues):
- **Car course** (R5.5, round 62): the heading shuffle plus a straight
  depth trim passes the full course in 10 of 12 cells, parking within
  0.003-0.079 m. Gazebo rear-steer and Isaac anti-Ackermann abort in the
  slalom (RPP collision-ahead). Round 59 without the trim: 5 of 15.
- **Go2 training** (R12):
  - `go2_walk_r3` (20 ms actuation delay) passes the drive screen in
    MuJoCo, Gazebo and Isaac; in Isaac it reaches 0.319 of 0.4 m/s.
  - First runs: `go2w_walk_r1` passes in MuJoCo; `g1_walk_r1` fails (no
    command tracking).
  - `go2_rough_r1` walks on flat floors but not on the heightfield.
  - Isaac navigation with `go2_walk_r2` reaches every goal, but only 1 of
    3 seeds lands within the matrix's 0.25 m.
- **Support matrix:** 687 reports, 310 cells, 298 passed and 11 failed
  ([matrix](evidence/support-matrix-2026-11-09/README.md)). On November 10:
  697 reports, 319 cells, 307 passed
  ([matrix](evidence/support-matrix-2026-11-10/README.md)).
- **PX4 manual-flight GUI runs in 8 worlds:** none ran a flight.
  - aerial_course and aerial_indoor never armed: the qualification script
    skipped the takeoff clearance check the gate requires (fixed in
    1cb48d45).
  - The other six never started: the queue gave them ROS domains 233–238,
    beyond CycloneDDS's port range (a queue error, not a product
    failure).
  - The re-run of aerial_course (domain 121) passes `flight_manual_screen`:
    clearance clear with a 3.0 m rise, climb 2.95 m. The other seven worlds
    still need re-running.
- **Robot-vacuum cleaning system** (OOMWOO-class, new R13): implemented
  and experimental ([guide](../tutorials/robot-vacuum.md),
  [continuation](vacuum-continuation-2026-11-09.md)); Gazebo runs in
  progress.

November 5 GUI redesign:
- **Header:** a live session indicator (idle, or running with robot,
  world, simulator and elapsed time) replaces the repeated page title.
- **Sidebar:** icons and Ctrl+1-9 shortcuts.
- **Launch page:** labelled Category/Type filters, title-case sections and
  a plain-words review card with a primary Launch button.
- **Theme:** flat tabs, hairline frames and slim scrollbars.
- **Pages:** Tests and Cleaning missions rebuilt as numbered cards.
- **World registry:** `car_course` is now registered, so the GUI can
  select it.
- [Guide](../tutorials/gui-quickstart.md). This is UI work; robot
  qualification is unchanged.

November 5 (recorded from the October 29 rounds):
- **Car course** (R5.5, round 58): the parking heading shuffle passes the
  full course in 8 of 12 cells, up from 3. All three Gazebo cars now abort
  in the slalom.
- **Go2 training** (R12): `go2_walk_r2` passes the drive screen in MuJoCo,
  Gazebo and Isaac. Navigation with a trained policy passes 1 of 3 seeds
  in MuJoCo and 0 of 3 in Isaac.
- **Support matrix:** 647 reports, 299 cells, 289 passed and 9 failed
  ([matrix](evidence/support-matrix-2026-11-05/README.md)).

**Session stopped at the user's request on November 3.** Work is committed; the owned fleet trial was stopped and cleaned up. Trial 12's interruption is preserved separately from mission failures and successful cases. Resume with a fresh full floor/parking run after the user asks to continue; full platform acceptance remains partial.

November 3: [curved AUV routes, low-relief source gait/backend adapters and concurrent fleet missions](curves-terrain-concurrency-2026-11-03.md) are implemented. Actual scoped tank-curve, octopod terrain and overlapping settled Nav2 results are in the [receipt](evidence/curves-terrain-concurrency-2026-11-03/README.md). Broader per-model/backend and multi-member floor/parking acceptance remains partial. The native GUI controls, resolver profiles, command autofill and interactive architecture/tutorial links reflect these workflows.

Latest workflow continuation from `8b958e97`: bounded multi-member lift queues/round trips, durable SDK lift ownership, explicit interlocked recovery, generation-safe marine replanning and native aerial formation itineraries. Actual empty-car Stop/recovery passed in 35.949 s and a one-command two-stop Bumperbot floor round-trip in 324.259 s. Occupied recovery, concurrent members and new route-flight qualification remain pending. [Delivery and limits](workflow-continuation-2026-11-02.md). Full platform acceptance remains partial.

The [GUI usability refresh](../tutorials/gui-quickstart.md) adds Home, modern shared styling, purpose-based/searchable navigation, responsive sidebar, contextual help, clearer launch/Stop feedback and a selector for every control page. This is UI work; full robot/extension acceptance remains partial.

[Native interactive architecture workspace](evidence/architecture-explorer-2026-11-02/README.md) is delivered inside the existing Health tab, labeled **Learn & diagnose**: twelve diagram views, six tours, linked tutorials/source/package browsing, glossary, networking playground and opt-in actual ROS discovery/QoS. Installed UI and read-only diagnostics are checked; robot mission support remains unchanged.

Latest November 2: [automatic native fleet lift transport](evidence/automatic-floor-transport-2026-11-02/README.md) completed one Bumperbot L0-L1-L0 with autonomous boarding, map/costmap transitions and settled exits. The GUI Facility request is implemented. A real native SDK L0-L1-L0 mission also passed in 359.063 wall seconds; actual SDK process-loss/resumption held the car and required explicit re-enablement. Broader multi-robot cases remain separate. Full platform acceptance remains partial.

[Latest fleet energy/RMF continuation](fleet-continuation-2026-11-02.md): actual source-bound Bumperbot/Labbot effort/rate readback, Bumperbot authored charge/departure/feedback-loss case, exclusive floor/AMCL frame ownership and an actual native RMF physical-door/settled-dock route. The later real SDK multi-floor round-trip is measured; broader qualification remains.

[Earlier native-form delivery](native-forms-2026-10-30.md): measured snake 2D/3D acquisition and saved-map navigation, surface-vessel mapping/two-goal route, AUV acquisition and real articulated flipper/Drive controls. That checkpoint had 151 profiles / 81 selectable families, with 52 worlds / 40 families. Older dated entries below retain their historical scope.

[October 30 amphibious/facility delivery](amphibious-facility-2026-10-30.md): actual sprung shore round-trip/RGB-D acquisition and physical door/lift/dock controls. Current installed GUI: 152 profiles / 82 robot families and 54 worlds / 42 families. That checkpoint left energy unknown; the later named November 2 source-readback/model cases supersede that energy scope.

[November 2 fleet continuation](evidence/fleet-floor-map-transition-2026-11-02/README.md): one Bumperbot physically rode L0-L1-L0 in `fleet_facility`; fresh body observations drove per-member floor classification and the `map.yaml` → `map_L1.yaml` → `map.yaml` Nav2 transitions. That earlier floor case left energy unknown; broader RMF/fleet acceptance is still open.

**Current checkpoint: November 3, 2026; master, curves/terrain/concurrency continuation from `43473387`.
Platform and full extension partial.**

**Isaac ownership correction:** the existing `source_gait_controller` exclusively
publishes `/gait/state` and owns operator leases, Arm, terrain enable and Stop.
The spawner publishes actual diagnostics on `/gait/plant_state`; runtime debug
flags do not establish walking or mission success. The final native trial 13
passed the full 10 simulation-second travel + 2 second hold screen: 0.1926 m
actual body progress and 0.0008 m held drift in 276.177 wall seconds, within
the explicit 600 second budget. Trial 12's 180 second timeout remains a failure.
The same octopod low-relief screen now has actual receipts on all four backends.
Earlier schema-only/runtime-flag claims were not physical mission qualification.

**Fleet follow-up:** the latest four-stop trial completed alpha's physical L1
parking and gamma's independent route, then correctly rejected beta's false
Nav2 arrival while its body remained at the lift sill. Subsequent mesh/native
bevel trials failed too and were withdrawn. The original flat deck is restored;
the authored force PID now settles the leveling residual with a 0.1 mm initial
door-admission threshold and the original 3 mm continuous load/floor tracking
bound, retaining mass, force and speed limits. The full
wave is being repeated; source/build checks do not qualify it. Failed results and the actual first parked endpoint
are preserved in the November 3 receipt.
The level-car repeat exposed a car panel rubbing the neighboring fixed floor;
10 mm lower panel clearance retains its upper edge and actuator limits.
Trial 10 completed three parked endpoints but interrupted Labbot's return
boarding when the initial precision threshold was also applied during loading.
The final two-stage gate preserves the original continuous tracking bound;
all four parked endpoints are required from its new repeat before accepting
the full wave.
That repeat reached three completed transports, then its old 600 s queue wait
canceled the final return. The facility now declares 1800 s queue waiting
with an immutable per-stop expiry, preserving the 600 s default elsewhere
and all live feedback/ownership guards. Trial 12 was stopped at the user's
request after 319.046 wall seconds, with one parked L1 endpoint recorded;
the full repeat remains pending after resumption.

**Occupied recovery protocol:** `fleet_controller.py` now tracks
`occupied`/`occupied_at`/`recovery_initiator`/`recovery_target` per member
and wires `_initiate_recovery` into `_traffic_recovery_tick` so a member
blocked by an occupied peer in the lift car initiates the peer's recovery
instead of yielding. `_occupy_member`, `_unoccupy_member` and
`_complete_recovery` are implemented. All fleet unit tests pass (129). The
physical two-member L0→L1→L0 parking wave and occupied recovery campaign
remain pending a fresh Gazebo run.

[Current workflow delivery and limits](workflows-2026-10-27.md),
[actual receipts](evidence/workflows-2026-10-27/README.md),
[remaining-work guide](../REMAINING_WORK_GUIDE.md).

| Area | Delivered / measured | Remaining |
|---|---|---|
| Catalog | Seven registrations restored; grouped complete robot/world families and the new `gait_low_relief` example | Changed-source qualification and additional reviewed imports; catalog counts are not mission acceptance |
| Fleet | Native sessions/queues/traffic/RMF/coverage; physical facility door/lift/dock, source-bound actuator energy/authored charging and native RMF door/dock route; one measured Bumperbot L0-L1-L0 ride with observed per-member floor/map switching; actual conflict and settled batch cases | Broader SDK conflict/loss, calibrated energy/RMF scheduling, broader conflict/loss, other robot/floor/map/backend transport, contact and larger-fleet missions |
| Aerial swarm | Native multi-FCU controls/workloads; actual nav_empty two-drone range-reference formation and operator-loss landing | Larger groups, other maps/behaviors and member-loss/contact cases |
| Marine | Actual sensor-based map acquisition/export/protected reload/localization and completed ROV route; AUV automatic acquisition, a source-checked 20° tank curve/settled arrival and surface-vessel mapping/two-goal arrival | Further AUV/localization/obstacle/current/loss missions |
| Multi-leg | Source FK/contact odometry, map export/reload/Nav2; hexapod navigation and octopod 3D database; four backend adapters and actual octopod low-relief travel/hold on all four engines | Per-model terrain/recovery/higher-mode/backend qualification |
| Additional forms | Complete snake, surface, sprung amphibious and fixed/actuated tracked sources; actual shore round-trip/mapping, snake saved-map Nav2 and flipper/Drive cases | Further terrain/slip/recovery, articulated-track navigation, other maps/backends |
| Experiments | Actual Nav2/fleet/marine/gait/aerial producers, artifacts/endpoints, fleet batch reset and cleanup | More cases, applied seeds and contacts/clearance |
| Original roadmap | Existing wheeled/Panda/flight/MOLAR/training certificates retained | Detailed remaining car, perception, manipulation, policy transfer/comparison/release work |

Earlier October 26 completion claims from tests/metadata/world loading remain
superseded. Historical failed artifacts and successful exact-cell results are
preserved; the new receipt never relabels them.

Simulator estimation, SLAM, volumetric routes, fleet traffic and Isaac contact
are software/simulator work. They do not wait for physical robot hardware or
terrain-provider setup for the five already installed water worlds. Real sensor
calibration, physical joystick and optional Bumperbot HIL remain separate.

Go2's first trained policy, `go2_walk_r1` (2,000 iterations, 24.58 M
steps, 73 min on the CPU), was measured through the normal launch on
`nav_empty` ([evidence](evidence/training-2026-10-19/README.md)). It walks,
arcs, reverses, strafes, stops and turns on the spot in MuJoCo, Gazebo and
Isaac (spin 0.98 / 0.83 / 0.64 of 1.0 rad/s; the rl_sar policy manages
0.01 / 0.02 / 0.43). Isaac forward speed is only 0.22 of 0.4 m/s. These
are one-run drive screens, not navigation or terrain qualification. Round
57 measured the October 19 fixes: the MuJoCo small_house slam drive went
from 1.49 m to 1.3 cm RMSE, and the rear-steer car now passes the slalom
in four backends and the full course in MuJoCo. Matrix: 626 reports, 294
cells, 282 passed and 11 failed. Earlier marine map-save failures and
RMF stalls are retained; corrected code needs its own successful mission receipt.

The earlier documentation-only checkpoint performed no new runtime qualification; the latest native receipt records its own continuation measurements. The dated
entries below preserve earlier source and measurement scopes; their old
"ongoing/queued" summaries do not override the current guide and ledger.

October 19 (later): **a Training page trains legged policies**
([guide](../tutorials/legged-policy-training.md)).
- EXPERIMENTS → Policy training, or `ros2 run robot_lab_adapter
  legged-train`. PPO trains Go2, A1 or B2 to walk, turn in place, stand
  or recover from pushes, in batched MuJoCo built from the bridge's own
  model and PD actuators, in the contract of each robot's rl_sar policy.
- The page follows a run live, stops it with a checkpoint and export,
  evaluates the exported ONNX through the deployment code and hands it to
  Launch as `legged_policy:=<directory>`, for its own robot only.
- Implemented and experimental. A smoke run trains, exports and evaluates
  end to end. The first real run is in progress (above); no trained policy
  is qualified yet.

Rounds 51-56 ([evidence](evidence/rounds-2026-10-19/README.md)):
- Go2 passes nav_obstacle seeds 1-3 in Isaac with forward arcs (no turn in
  place, which the policy does not hold there). It now passes in all three
  of MuJoCo, Gazebo and Isaac.
- Car publisher loss passes in all six cells. The car course passes in 2
  of 12 cells: the slalom mostly passes, but parking misses the 10°
  heading because the car goal checker ignores heading.
- Camera calibration passes in MuJoCo and PyBullet. Isaac fails on RMS
  (its camera noise) and Gazebo on a 5 px principal point.
- Clean host at `e3821b8`: every step passes, Isaac included (its build's
  external links are now mounted).
- Matrix: 619 reports, 291 cells, 278 passed and 12 failed screens
  ([what changed](evidence/support-matrix-2026-10-19/README.md)).

October 19: **the MOLAR family runs in all four simulators on the detailed
CELISCA floors** ([evidence](evidence/molar-hri-2026-10-16/README.md),
[guide](../tutorials/molar-hri-simulation.md)).
- The normal-GUI check covers drive, W+A turn, lift to 0.68 m, both
  coupled telescope stages to 0.14 m (0.28 m tip), roller, Stop and
  heartbeat loss. It passes for `molar_hri` in Gazebo, MuJoCo, PyBullet and
  Isaac on `nav_empty` and both detailed floors, and for `probot_l` on
  `nav_empty`. An earlier agent's MuJoCo run never moved; Gazebo and Isaac
  had no runs.
- Nine backend fixes got there. Among them: implicit integration and a
  gravity feedforward for the slides; activity and base motion from
  position change (Isaac's velocities jitter); Gazebo caster friction and
  hard stops below the controlled range (DART); and Isaac leaving the
  floors' coincident sliver triangles to its ground plane.
- The other agent's hardening of October 19 is committed (`b5ae10d`): a
  shared effort budget, excursion-aware activity, Gazebo feedback
  validation and STL floor collision.

October 16: **the open items were worked through**, with each cause found
before the fix ([evidence](evidence/open-items-2026-10-16/README.md)).
- MPPI with an RPP arrival assist reaches 15/15 goals (7/15 before).
- The MuJoCo/PyBullet floor lift was the renderers' optical centre
  (MuJoCo's multisampled depth, TinyRenderer's axis); it now reads 0 mm.
- MuJoCo's small_house was 100x too large through a stale mesh cache; it
  now matches its map and draws Gazebo's textures, as PyBullet and Isaac
  do.
- B2 walks in Isaac with a DC-motor actuator on the training floor (0.68
  m/s overshoot before) and passes its three Isaac obstacle missions.
- G1 passes 3/3 in Isaac at a 0.12 m goal (0.256 m before).
- Go2 with a wider planner inflation passes 3/3 in MuJoCo and Gazebo. In
  Isaac, seed 1 ends 0.283 m out (over the 0.25 m screen) and seed 3 still
  aborts. Rotating in place first failed all three Isaac seeds and was
  reverted: the Go2 stops turning on the spot after 1-2 s there.
- Bumperbot's Gazebo slam drive went from 10.2-10.5 cm to 6.4 cm RMSE
  with a one-sample twist window; the cars reach 1-2.5 cm.
- Isaac's camera stays noisy. No anti-aliasing mode or RaytracedLighting
  changed it, PathTracing crashed Kit, and MinimalRendering is noise-free
  but loses the untextured scene.
- The cars retry their approach. The car course, calibration room,
  publisher loss and clean-host Gazebo/Isaac steps are implemented and
  queued for measurement.

October 15 AUV continuation: **native Tethys propeller/fins, explicit trim
and experimental Marine cruise guidance** are installed. One actual GUI reef
run reaches 1.68886 m for a 1.70 m target and 1.70029 m during the following
turn. Both vehicles now use the installed Fortress added-mass sign convention;
the corrected ROV has its own tank surge/depth/Stop diagnostic. AUV Stop
zeros propulsion/fins and allows physical coasting. Independent underwater

October 29: **Aerial R10.3 continuation is measured** on `nav_empty`: two X500s'
takeoff corridor checks pass (check_position air1 [-0.5,-0.5,0.22726] m,
air2 [-3.5,-0.5,0.22649] m); native PX4 EKF range reference applied
(`EKF2_HGT_REF=2`); the adapter's formation-altitude trim is implemented and
unit-tested; bounded flight trials recorded. Establishment of
formation-error/altitude and landing/disarm metrics blocked on a
driver-protocol fix. [Receipt](docs/status/evidence/aerial-r103-2026-10-29/README.md).
estimation, saved SLAM/navigation, other backends and broader qualification
remain open. [Guide](../tutorials/tethys-auv.md),
[source/measurement scope](auv-2026-10-15.md).

October 15 multi-leg continuation: complete **source hexapod, Laboratory
octopod and articulated centipede** now have native MuJoCo Gait/Drive,
source-stance previews and automatic compatible mode/command selection.
Short actual GUI inputs measure about 0.0974/0.1760/0.1252 m forward motion
and held Stop. A second centipede run after timestamp/reset guards measures
0.1235 m. Eight source/contact/IK checks and 19 typed composition checks pass.
Terrain/recovery, independent estimation, SLAM/navigation, other backends and
broader missions remain pending. [Guide](../tutorials/multi-leg-gaits.md),
[exact source/diagnostic scope](gaits-2026-10-15.md).

October 15 marine continuation: **Laboratory ROV**, five actual water/collision
worlds and the native Launch **Marine** controls are implemented experimentally
on Gazebo Fortress. The normal GUI tank run measures physical surge, 1.00 →
1.19 → 1.01 m depth changes and zero-thrust Stop. The observer remains
simulator-pose based; independent underwater estimation, complete AUV, saved
SLAM, volumetric navigation and other backend force adapters remain open.
Seven focused source/allocation checks and seven affected packages build.
[Operator/agent guide](../tutorials/marine-rov.md),
[actual receipt, failures and gaps](marine-2026-10-15.md).

October 15: **R10 is first**. The native Fleet & Swarm tab starts a single
Gazebo world with named Bumperbot/Labbot instances, separate TF/sensors and
source wheel controllers, latched group/instance Drive, actual Nav2 dispatch/
allocation and four ground group kernels. One GUI nav_empty diagnostic records
0.76880/0.71445 m independent movement, a Nav2 result at 0.06218 m physical error,
formation motion, heartbeat-loss Stop and isolated reset without clock rewind.
Settled arrival, contacts and broader mission/backend acceptance remain open;
13 focused checks pass. Marine/gait and the remaining fleet/swarm integrations
precede the older roadmap. [Guide](../tutorials/fleet-and-swarm.md),
[scope, failures and gaps](fleet-2026-10-15.md).

The October 14 next extension is now scoped as R10: shared-world fleets,
fleet management, ground/aerial swarms, underwater robots/maps and six/eight/
many-legged robot families. Source research and agent instructions are
recorded. The latest instruction puts R10 first: shared-world Gazebo base/GUI
first ROV and first multi-leg gait implementations are now partial, followed by remaining marine/fleet integrations and legged missions,
then the previous roadmap. GUI Health shows the same status.
[Detailed guide](../NEXT_EXTENSION_GUIDE.md).

October 14 hand integration: four source hand profiles now have experimental
coordinated poses, source PID/tendon actuation, individual finger groups,
measured external contacts/effort and guarded advancement. Normal GUI
DEX-EE/Robotiq movement diagnostics, actual GUI free-cube contact/release
and 34 focused checks are recorded. Fixture/adapter forwarding, actual support/
contact-loss state and collision-based placement are repaired;
stable grasp/release, all templates/objects and other backends remain pending.
[Guide](../tutorials/native-hand-controls.md),
[scope and retained failure](native-hands-2026-10-14.md).

October 14 aerial integration: an opt-in mapped/depth route layer now combines
actual RTAB-Map clouds and registered depth with static-world planning, rechecks
before execution and requests Hold on blocked/stale observations. GUI selects
sensing/mapping dependencies and shows ages. Four packages build and 23 SDK
checks pass; ROS fixture inputs are explicitly synthetic, with no new flight
qualification. [Guide](../tutorials/mapped-drone-planning.md),
[scope/remaining work](aerial-obstacles-2026-10-14.md).

October 14: thirteen native fixed arms now have experimental MoveIt KDL/OMPL
planning, original-actuator trajectory execution and normal GUI command
autofill. Actual GUI UR5e and SO101 reaches on MuJoCo/nav_empty have 0.489 mm
and 0.147 mm endpoint errors. The other eleven arms, objects, interruptions,
additional worlds and backends remain unqualified.
[Guide](../tutorials/native-arm-planning.md),
[implementation and retained failures](native-arm-planning-2026-10-14.md).



- **October 14: R5.8.native_hands (Codex).** Module, controller guard and resized geometry are validated in CI; the adapter-level displayed grasp→held→released integration test now passes end to end and the whole-plant GUI launch mission is retained as pending. Every native adapter (robotiq_2f85, robotiq_2f85_v4, umi_gripper, shadow_dexee) now derives a documented free object from its measured distal pads (`object_geometry`): closing-axis half-size `min(.32*span, .032)`, transverse `min(.03, .32*span)`, seated between the pads at the open pose. `NativeHandControl` reports measured two-sided force via `object_contacts()` and `check_guard` gates on it; `grasp_object`/`release_object` drive the documented free object without asserting stability; the MuJoCo display `grasp_fixture` generalized through an `adapter` parameter (`adapter=''` preserves the legacy Panda bundle). `test_native_asset_display_grasp_fixture.py` (robotiq_2f85_v4 adapter) exercises the same display+controller+simulation loop the launch uses: the display authors the free cube between the measured distal pads at pad height, `hand_grasp` reaches `holding` on measured two-sided contact, and `hand_release` frees it under contact physics. Verified by 78 fast/physics/display integration tests and a clean 26-package `colcon build` (exit 0). A real installed-adapter full GUI-launch grasp/held/release mission remains pending full-launch validation.

October 13: implemented on master, experimental.

- **Car controls (R7.8.bicycle, Codex).** Three additional source-profile
  controllers—bicycle pure pursuit, Stanley and steering-rate MPC—serve a
  separate Nav2 FollowPath action with actual odometry, full-footprint stopping
  checks and car-safe cancellation/heading handling. GUI Motion controller
  selects them on Ackermann/rear-steer cars, with forward DUBIN planning.
  Numerical/installation checks are separate from pending physical missions.
  [Guide](../tutorials/bicycle-control.md), [scope](continuation-2026-10-13-codex.md).

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
  - On recorded Gazebo IMU data (2026-10-15), the wheel+gyro EKF holds
    Bumperbot to 1.3 cm over 25 m (wheels alone 9.8 cm). On a walking Go2
    trunk the complementary filter tracks tilt best (0.25° RMS) and the
    ESKF worst (0.78°), the reverse of the modelled ranking.
  - [Evidence](evidence/r75-fusion-benchmark-2026-10-13/README.md),
    [tutorial](../tutorials/sensor_fusion_comparison.md).
- **Perception, localization, state estimation (R7.2-R7.4).** The ids now
  run real methods, each measured offline with known truth; localization,
  state estimation and perception are also measured on recorded simulator
  scans and AMCL fixes, and perception on live depth cameras.
  - The three cataloged perception ids now run (2026-10-15, implemented,
    experimental). `depth_image_to_pointcloud` turns depth images into
    clouds in all four simulators, 3.3-6.6 ms each, and the ground and
    cluster scores on its clouds match Gazebo's own. These are the first
    depth-cloud scores for MuJoCo, PyBullet and Isaac
    ([evidence](evidence/r72-perception-benchmark-2026-10-13/depth-node/README.md)).
  - The simulators' camera info put the principal point half a pixel off,
    at (160, 120) for a 320 x 240 image. Depth geometry and a chessboard
    calibration in Gazebo both place it at the pixel centres,
    (159.5, 119.5). The lab's MuJoCo, PyBullet and Isaac cameras now
    publish that.
  - `feature_detection` is feature-based RGB-D visual odometry. In Gazebo's
    textured small_house it drifts 2.9% with ORB at 30 Hz. It fails in
    flat-shaded worlds, on MuJoCo's untextured renders and on Isaac's
    noisy RGB
    ([evidence](evidence/r72-perception-benchmark-2026-10-13/feature-odometry/README.md)).
    Since 2026-10-16 MuJoCo, PyBullet and Isaac draw the worlds' textured
    Collada visuals. MuJoCo's small_house scores like Gazebo's (edge
    correlation 0.47 against 0.46, grey contrast 60 against 54, 18
    before). The rerun drives and the Isaac noise fix are being measured
    ([evidence](evidence/open-items-2026-10-16/README.md)).
  - The remaining MuJoCo/PyBullet floor lift (1.5-2 mm per metre) was
    the renderers' own optical centre. MuJoCo read depth from one sample
    of a 4x multisampled buffer, 3/8 px off; PyBullet's TinyRenderer has
    its axis at (w/2, h/2 - 1). Depth now renders single-sample and
    PyBullet's camera info says (160, 119); a controlled floor reads 0 mm.
  - `camera_calibration` calibrates from chessboard views; a Gazebo board
    run gives the principal point within 0.13 px and fx 0.7% short
    ([evidence](evidence/r72-perception-benchmark-2026-10-13/calibration/README.md)).
    A calibration room with 14 fixed boards
    (`scripts/r7_2_calibration_room_check.sh`) extends this to MuJoCo,
    PyBullet and Isaac; its runs are pending.
  - Perception: KD-tree clustering, adaptive scan breakpoints and a refined
    RANSAC. RANSAC finds all of a 2° floor where a height threshold finds
    43%. On a live Gazebo RGB-D stream the cloud nodes turned out to
    segment in the camera's optical frame, where neither method found the
    floor; in the robot frame both now find all of it (precision 0.97-0.99)
    ([evidence](evidence/r72-perception-benchmark-2026-10-13/README.md),
    [tutorial](../tutorials/perception_comparison.md)). On recorded Gazebo
    and MuJoCo `/scan` the same kernels split maze wall corners: single
    linkage and the fixed scan clusterer lead (F1 0.66-0.67; precision 0.85,
    recall 0.54) where adaptive breakpoints and DBSCAN fragment the walls
    ([live evidence](evidence/r72-perception-benchmark-2026-10-13/live/README.md)).
  - Localization: point-to-point and point-to-line ICP, NDT and correlative
    matching against the map. On ray-cast nav_maze scans they hold 7-24 mm
    where biased odometry drifts 0.55 m; on recorded Gazebo and MuJoCo
    scans 3.6-5.3 cm, against 7.9-8.8 cm for the AMCL that ran with them
    ([evidence](evidence/r73-localization-benchmark-2026-10-13/README.md),
    [tutorial](../tutorials/localization_comparison.md)).
  - State estimation: CV Kalman, CTRV EKF/UKF, particle filter, IMM and a
    window smoother on noisy position fixes: 0.16-0.19 m from 0.43 m raw
    fixes ([evidence](evidence/r74-state-estimation-benchmark-2026-10-13/README.md),
    [tutorial](../tutorials/state_estimation_comparison.md)). On the real
    ~1.6 Hz AMCL fixes the R7.3 runs recorded they do not beat the raw
    fixes (0.08-0.12 m vs 0.08-0.09 m); a good, correlated localizer leaves
    a white-noise filter nothing to remove
    ([live evidence](evidence/r74-state-estimation-benchmark-2026-10-13/live/README.md)).
- **Walking robots navigate (P7).** Leg kinematic odometry feeds the EKF,
  and slam/nav modes select each robot's policy. Over three goal seeds
  (2026-10-15):
  - A1 passes 3/3 in MuJoCo and Isaac; G1 3/3 in both, Isaac after its
    inflation was widened (2026-10-16; 2/3 before). With a 0.12 m goal
    the G1 ends 0.015-0.171 m from truth in both (0.256 m before), inside
    the matrix's 0.25 m screen.
  - Go2 passes 3/3 in MuJoCo and **3/3 in Gazebo**, and 2/3 in Isaac:
    seed 3 stopped on "collision ahead" beside the block. Go2's planner
    inflation is now 0.7 m (`26b961e`); its rerun in all three is pending.
  - B2 passes 2/3 in MuJoCo; the miss is 4 mm over the 0.30 m test. In
    Isaac B2 now walks (below); its first Isaac missions are running.

  Robots that cannot turn on the spot there (Go2 in MuJoCo/Gazebo) or are
  too long to (B2) use the new forward_arc navigation: forward-only
  Hybrid-A* curves with their footprint, and no spin or back-up recovery.
  It cured their aborts beside the block at the nav_obstacle start. B2 and
  G1 first stood still at the 0.2 m/s approach speed; their minimum speeds
  are now 0.4 and 0.3 m/s.
  - The G1's Isaac "start in lethal space" is explained. The G1 walked
    0.44 m from the west wall while still turning, and its leg-odometry
    estimate sat 0.1-0.2 m nearer. That put its centre at the 0.3 m
    inscribed radius, which the grid planner counts as a collision.
  - Walking robots also map: in slam mode on leg odometry, slam_toolbox
    held every walking robot within 4.5-7.1 cm RMSE of truth over a 26 m
    loop (MuJoCo, Isaac, Gazebo), and the MuJoCo/Isaac maps place every
    wall cell on a real wall. The mecanum and 4WS cars map the same loop on
    their wheel odometry to 1.1-2.5 cm in every backend
    ([evidence](evidence/p7-legged-slam-mapping-2026-10-13/README.md)).
    Driven sideways at a fixed heading (2026-10-15), the 4WS crab and
    in_phase patterns hold 0.9-1.7 cm in MuJoCo and Gazebo. The strafing
    mecanum car holds 2.4 cm in MuJoCo and 11 cm in Gazebo, where roller
    slip leaves its odometry 1.02 m off.
  - Bumperbot is the exception in Gazebo: slam_toolbox ends 9.6 cm from
    truth, while the EKF odometry it is given holds 2.7 cm. Its heading
    slips about 0.9° at the first 90° turn.
    - The EKF is not the cause: wheel odometry gives 10.2 cm, the EKF
      10.5 cm (2026-10-16).
    - Bumperbot's odometry TF trails truth by 20 ms (the cars: 0-5 ms).
      Its diff-drive controller averaged the twist over 0.1 s; it now
      reports the latest sample (`5a995e3`).
    - The rerun is pending.
  - Gazebo Go2 now starts standing, and with its physics step capped at
    2 ms (the worlds' 10 ms had run its 500 Hz PD loop at 100 Hz) it walks
    the whole mapping loop and passes the mission
    ([evidence](evidence/p7-legged-navigation-2026-10-13/README.md),
    [tutorial](../tutorials/legged-policies.md)).
- **Planner reruns.** With a clearance margin, RRT/RRT\* reach 14/15 goals
  (were 8 and 9) and Voronoi 15/15. MPPI capped at 0.3 m/s still reaches
  7/15: it gets within 0.05-0.19 m but does not settle into Bumperbot's
  tight arrival window. Stronger goal critics made it worse (6/15) and were
  reverted. With an RPP `Arrival` controller that finishes a failed MPPI
  approach on the same path (`99e2e78`), MPPI reaches **15/15**,
  0.028-0.148 m from truth (2026-10-16; MPPI plus the assist).
- **Missions.**
  - Workflow/reset screens pass 8/8 for Labbot and Bumperbot in all four
    backends ([evidence](evidence/r61-workflow-reset-2026-10-13/README.md)).
  - 4WS/mecanum obstacle missions: 13 of 20 passed first, 19 of 20 on
    the rerun with a wider arrival window, and the mecanum rerun with the
    same window passes in all four backends. Every 4WS pattern and the
    mecanum car passed seed 1 in every backend
    ([evidence](evidence/r56-4ws-missions-2026-10-13/README.md)).
  - Over goal seeds 2 and 3 (2026-10-15), 4WS ackermann, pivot and the
    mecanum car pass 24/24. The cars passed 13/24, missing the near-obstacle
    goal by 0.26-0.36 m against the 0.30 m limit
    ([evidence](evidence/wheeled-repeats-2026-10-15/README.md)).
  - Crab and in_phase first aborted that goal in all 16 runs. A recording
    of DWB's scores showed why: it sampled only forward translation, the
    goal lay behind the base, and standing still scored best. **With
    backward sampling (`ae2989c`, 2026-10-16) they pass 24/24** over seeds
    1-3 in all four backends, 0.006-0.159 m from truth.
  - Car obstacle missions (Ackermann, rear-steer, anti-Ackermann): **10 of
    12** pass. MuJoCo rear-steer hit the tf2 freeze; Gazebo Ackermann
    aborted its detour after repeated collision-ahead stops
    ([evidence](evidence/r55-car-missions-2026-10-13/README.md)).
  - **Cars, 2026-10-16: 35 of 36 pass** over seeds 1-3 in all four
    backends, both goals 0.002-0.224 m from truth
    ([evidence](evidence/car-missions-2026-10-16/README.md)). Two
    configuration fixes got there:
    - The cars had kept Nav2's 0.25 m goal tolerance and stopped at its
      edge; they now use 0.1 m (`ac71687`, 32 of 36).
    - Their planner inflation was below their swept radius, so in four
      runs the body touched the block beside the route and the replan
      aborted. A 0.6 m inflation fixed those (`32c9cf1`).
    - One MuJoCo anti-Ackermann goal stalled 0.15 m out, unable to settle
      inside 0.1 m without turning on the spot. The car tree now backs up
      and retries three times (`51c7543`): that cell and the two other
      weakest pass (0.062-0.139 m, 2026-10-16).
  - A car course (cone slalom and nose-in parking bay, with footprint
    clearance, curvature and final heading checks; `scripts/sim_car_course.sh`)
    is implemented; its runs are pending.
- **Two simulations at once (R8.3, 2026-10-15).** The concurrent
  coordinator's Nav2 workload had never run; three bugs stopped it and are
  fixed.
  - MuJoCo and PyBullet Bumperbot jobs then reach both Nav2 goals within
    2-10 cm of truth, one at a time and side by side, with unchanged
    real-time factors.
  - A memory budget stops only its own job, a coordinator cancel aborts
    the running goal within 3 s, and no process is left behind
    ([evidence](evidence/r83-concurrency-2026-10-15/README.md)).
- **Gazebo cars.** The car description had no ground-truth publisher, so
  Gazebo car missions timed out; it now has one, in the spawned
  description too.
- **Isaac mecanum strafe fixed.** PhysX rolled each roller's 12-sided
  convex collision facet by facet, so the rollers turned at half speed.
  With capsule collisions the car strafes at 0.290-0.293 of 0.3 m/s with
  no drift, and the obstacle mission passes
  ([evidence](evidence/p56-drive-patterns-2026-10-12/README.md)).
- **Stale transforms.** In 9 of about 90 recent Nav2 trials (R4.3, the
  planner comparisons and the car missions), a Nav2 server's whole TF view
  froze while AMCL kept publishing. The robot then spun in place, overshot
  its goal, or the planner started from an old pose.
  - The probe has since caught it twice. A fresh listener kept receiving
    `map -> odom` while controller_server's newest transform stayed fixed.
  - So the server's subscription stops, not the publisher.
  - **Cause:** an ABBA deadlock in the installed tf2/tf2_ros 0.25.23
    ([ros2/geometry2#992](https://github.com/ros2/geometry2/issues/992)),
    fixed upstream in 0.25.24.
  - 0.25.24 ran first as a workspace overlay
    (`scripts/build_tf2_deadlock_fix.sh`, 06:52). Apt then upgraded the
    system packages to 0.25.24 (10:32), and the overlay was removed.
  - With 0.25.24, the R4.3 protocol passes 30/30 goals, and 46 Nav2
    launches show no freeze, where about 4-5 were expected
    ([evidence](evidence/r43-planner-comparison-2026-10-12/tf2-0.25.24-2026-10-13/README.md)).

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
  - Isaac (PhysX drives at 200 Hz): Go2, A1, Go2W and G1. B2 overshot
    there (0.68 m/s for 0.4) and drifted after the stop. With Isaac Lab's
    DC-motor actuator, which its gains and limits match, and the
    friction-1 training floor, it walks 0.29 m/s and stops dead, as in
    MuJoCo (2026-10-16, now its Isaac default).
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
separate source stages and skip/failure records. A clean container now
builds the workspace, passes the fast tier and passes a MuJoCo Bumperbot
mission from rosdep and pinned pip dependencies alone
([R9.1 evidence](evidence/r91-clean-host-2026-10-15/README.md)); a Gazebo
and an Isaac mission through the NVIDIA runtime are added and pending.
Two simulations have run side by side with budgets, cancel and cleanup
([R8.3 evidence](evidence/r83-concurrency-2026-10-15/README.md)). Other
hardware, physical joystick/hardware checks and broader concurrent
qualification remain open.

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
