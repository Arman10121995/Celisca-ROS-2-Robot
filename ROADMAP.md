# Robot Lab roadmap

**This is the project's single goal list.** Every task, goal and piece of
remaining work is here, in one format, in priority order. Updated 2026-11-10.

- **Status and numbers** (what was measured, when, with which revision) are in
  [current status](docs/status/CURRENT_STATUS.md) and the evidence folders it links.
- **The ledger** [platform-status.yaml](docs/status/platform-status.yaml) mirrors
  each task's state and owner for the GUI. Change it together with this file.
- **History:** the roadmap and checklist before this restructuring are kept unchanged in
  [archive/ROADMAP-2026-11-10.md](docs/status/archive/ROADMAP-2026-11-10.md) and
  [archive/CHECKLIST-2026-11-10.md](docs/status/archive/CHECKLIST-2026-11-10.md).
  The old checklist page now points here.

Task IDs (R0.1 … R14.1, P1 … P8) are unchanged. Code, tests and evidence cite
them, so never renumber a task. Add new work as a new ID.

---

## 1. How to work on this roadmap

### Priority: implementation first

The user's standing instruction is **implement everything that is missing
first, then test and validate**. So the roadmap has two queues:

1. **[Implementation queue](#3-implementation-queue)**: missing code,
   controllers, GUI workflows, models and integrations. Work this queue first,
   top to bottom.
2. **[Validation queue](#4-validation-queue-after-implementation)**: repeats,
   matrices, comparisons, clean-host and release checks. These start after the
   implementation queue, or when a validation owner is assigned. No owner is
   assigned yet.

What still counts as implementation work, not validation:
- build fixes;
- the focused checks that show a new feature runs;
- Stop, limits, watchdogs and command-loss handling.

What counts as validation: anything that only repeats or widens measurements
of code that already exists.

### Rules every agent follows

- Read [AGENTS.md](AGENTS.md), [the handoff](docs/AGENT_HANDOFF.md) and the
  task's section below. Claim the task in the ledger (`owner`) before changing
  shared files. Preserve work owned by others.
- Work on `master`; no new branches or worktrees. Commit when the user allows it.
  Other agents commit at the same time, so stage only your own files.
- Large files (checkouts, builds, logs, bags, assets) go on `/workspace`. First
  `source scripts/ssd_env.sh`, and read [Storage](docs/STORAGE.md).
- **One simulation at a time.** Don't run builds or tests while a recorded
  simulation is running. Keep `ROS_DOMAIN_ID` at or below 232 (CycloneDDS
  fails above that on this host).
- Python packages are installed as copies: rebuild with `colcon build
  --packages-select <pkg>` after every edit, before any run.
- Reuse existing controllers, generators, source pins and GUI patterns.
  **Prefer an open-source robot or world over a self-made one** (see R14.1).
  Pin its revision and record its license.
- Evidence: record robot, map, simulator, task, revision, source hashes,
  commands, raw traces and **every failure**. A unit test, a process exit, a
  catalog count or an action's success flag does not prove a robot mission.
- When you finish a step:
  1. update this file (the task's *Have* and *Remaining*);
  2. update the ledger;
  3. update [current status](docs/status/CURRENT_STATUS.md) with the measurements;
  4. update the GUI-visible status, if it changed.

  A task becomes `done` only when its *Done when* is met. Never weaken a
  threshold to close a task.

### Task format

Every task below has the same fields:

| Field | Meaning |
|---|---|
| **State** | `done`, `partial` (useful work exists, acceptance not met), `not started`, `blocked` |
| **Priority** | `I<n>`: place in the implementation queue. `V<n>`: place in the validation queue |
| **Depends on** | Tasks whose results it needs |
| **Goal** | What the finished task gives the user |
| **Have** | What exists now, with links |
| **Remaining implementation** | Missing code or integration (do these first) |
| **How to do it** | Steps, files and entry points for the next agent |
| **Done when** | Acceptance |
| **Validation later** | Measurements that wait for the validation queue |

---

## 2. Project goal

A reproducible ROS 2 Humble lab for learning and comparing robotics
algorithms on many robot types, in 2D and 3D worlds, on four simulators
(Gazebo Fortress/Harmonic, MuJoCo, PyBullet, Isaac Sim), driven from one GUI
and one CLI. Robot, simulator, world, scenario and the seven algorithm
categories are chosen independently **where their physical and data contracts
are compatible**.

The goal is complete when all of these have evidence:
- Every robot class has working launch, control, sensing, Stop/reset and
  recorded missions in its suitable modes: two differential bases, a quadruped,
  a humanoid, a multirotor, four-wheel cars (Ackermann, rear-steer,
  anti-Ackermann, 4WS, mecanum), arms, hands, mobile manipulators, fleets and
  swarms, marine, multi-leg and other forms, the MOLAR family, the robot vacuum.
- Each algorithm category (perception, localization, state estimation, sensor
  fusion, global planning, local planning, control) has five distinct,
  correct, runnable methods and fair, reproducible comparisons.
- Worlds have verified geometry, resets and seeds. Terrain, stairs, aerial,
  moving-obstacle and degraded-sensor tasks exist. A 2D projection is not a 3D map.
- One resolved experiment manifest drives both CLI and GUI. Results measure
  success, contacts, time, path quality, clearance, estimation error, compute,
  real-time factor and effort, with truth kept separate from estimates.
- Every advertised robot/simulator/mode combination has its own evidence, and
  a clean machine can reproduce the comparisons.

Physical hardware (R9.4) is optional and separate from a simulation release.

---

## 3. Implementation queue

Work top to bottom. Each line links to its task.

| Order | Task | Remaining implementation (short) | State |
|---|---|---|---|
| I1 | [R13.2](#r132--vacuum-carpets-and-mopping) Vacuum carpets and mopping | Done 2026-11-10 (Gazebo mop + rug run passes) | implemented |
| I2 | [R13.3](#r133--vacuum-camera-obstacle-recognition) Vacuum camera obstacles | Measured with an ideal detector (0 contacts); learned detector, snapshots | partial |
| I3 | [R14.1](#r141--open-source-counterparts-for-self-made-robots) Open-source robots | First search done; integrate TurtleBot 4 Lite as a vacuum, then WAM-V, BlueROV2, Summit-XL | partial |
| I4 | [R6.7](#r67--terrain-generation-gui-and-four-backend-conversion) Terrain GUI | GUI page, provider manifest, Isaac contact | partial |
| I5 | [R5.11](#r511--drone-moving-obstacles-and-other-drone-backends) Drone moving obstacles | Moving-obstacle prediction, sensor-loss handling, other backends | not started |
| I6 | [R5.9](#r59--mobile-manipulators) Mobile manipulation | Navigate → reach → grasp → transport → release pipeline in the GUI | partial |
| I7 | [R5.7](#r57--manipulators-and-motion-planning) Arms | Pick/place with arbitrary objects, coupled mechanisms, other backends | partial |
| I8 | [R5.8](#r58--robot-hands-and-grippers) Hands | Torque/tendon hand adapters, object workflows | partial |
| I9 | [R10.2](#r102--fleet-management-and-open-rmf) Fleet / Open-RMF | Conflicts, infrastructure, larger fleets, calibrated energy | partial |
| I10 | [R10.3](#r103--swarm-robotics) Swarms | Remaining behaviours, distributed protocols, cooperative coverage, larger aerial groups | partial |
| I11 | [R10.5](#r105--underwater-worlds-sensing-slam-and-navigation) / [R10.4](#r104--underwater-robots-and-controllers) Marine | Estimation, 3D mapping, volumetric routes; other backends | partial |
| I12 | [R10.6](#r106--six--and-eight-legged-robots) / [R10.7](#r107--segmented-and-additional-robot-forms) Multi-leg and forms | Terrain/slip/recovery controllers, estimation, flipper-aware navigation | partial |
| I13 | [R5.2](#r52--go2-quadruped-locomotion) / [R5.3](#r53--berkeley-humanoid-lite-balance-and-walking) / [R12](#r12--legged-policy-training) Legged | Get-up strategy, terrain curriculum, BHL retraining, G1/Go2W policies, GPU training | partial |
| I14 | [R5.5](#r55--ackermann-rear-steer-and-anti-ackermann-cars) Cars | Final-heading (parking) control | partial |
| I15 | [R3.6](#r36--external-robot-assets-in-normal-launch) Assets | Vendor controller interfaces for display-only profiles | partial |
| I16 | [R6.5](#r65--world-to-occupancy-generation) / [R6.6](#r66--external-world-library) / [R6.2](#r62--dynamic-and-sensor-disturbance-cases) Worlds | Occupancy registration, remaining licensed imports and actors, disturbance cases | partial |
| I17 | [R11](#r11--molar--probot-l-family) MOLAR | Sensors, SLAM/Nav2, payload transfer, multi-floor | partial, needs the sensor layout |
| I18 | [P4](#p4--mujoco-large-world-speed) MuJoCo speed | Faster flex/scan/camera updates on large worlds | partial |
| I19 | [R7](#r7--algorithm-breadth) Algorithms | Simulator-applied seeds, remaining method contracts | partial |
| I20 | [R9.1](#r91--provenance-licenses-and-clean-host-reproduction) Provenance | Remaining license checks (PX4 checkout) | partial |

Blocked: [R9.4](#r94--optional-physical-bumperbot-hil) physical hardware
(needs the robot and explicit authorization).

## 4. Validation queue (after implementation)

| Order | Tasks | What to measure |
|---|---|---|
| V1 | R13.1 | Vacuum repeats, more houses, the new carpet/mop/camera features |
| V2 | R5.1, R6.1 | Bumperbot/Labbot repeats over seeds, actor maps, SLAM/3D modes |
| V3 | R5.5, R5.6, P5, P6 | Car course and 4WS/mecanum patterns on all backends and maps |
| V4 | R5.2, R5.3, R12, P7 | Legged walking, turning, navigation and terrain, per backend |
| V5 | R5.4, R5.10, R5.11, P8 | PX4 flight in every installed world; mapped-obstacle flight |
| V6 | R5.7, R5.8, R5.9 | Arm, hand and mobile-manipulation missions per backend |
| V7 | R10.1–R10.8 | Fleet, swarm, marine, multi-leg and form missions, loss and reset |
| V8 | R11 | MOLAR repeats and identified dynamics |
| V9 | R6.2–R6.7 | World geometry, actors, disturbances, terrain contact on all backends |
| V10 | R7.1–R7.8 | Fair five-method comparisons per category |
| V11 | R8.1, R8.3 | Backend mission matrix; concurrent experiments with budgets |
| V12 | P2 | Physical joystick trial |
| V13 | R9.1–R9.3 | Clean-host reproduction, seven comparison tutorials, release matrix |

---

## 5. Task specifications

### R0 — R4: foundation (done)

These are complete; keep them working. Details and evidence are in the
[archived roadmap](docs/status/archive/ROADMAP-2026-11-10.md#r0--documentation-recovery).

| Task | Result | Keep in mind |
|---|---|---|
| R0.1, R0.2 Documentation recovery | Audited baseline and truthful docs | — |
| R1.1 Entry points and paths | Installed executables, package-share asset paths | No developer home paths |
| R1.2 Test tiers | Fast / physics / integration / mission tiers | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` locally |
| R1.3 CI | Required CI fails on real failures | — |
| R2.1–R2.3 Simulator contracts | `/clock` in every adapter; one TF owner per edge; `/robot_lab/ready`, `/health`, `/reset` | Offline fallback is never success |
| R3.1–R3.3, R3.5 Composition | Canonical IDs, typed validation, one resolver/executor, scenario and experiment catalogs | The CLI and GUI must produce the same manifest |
| R3.4 GUI resolver | GUI uses the same resolver; seven algorithm slots; right-hand control column | State `active` in the ledger: keep the GUI tests green |
| R4.1–R4.3 Reference experiment | Lifecycle with truthful outcomes, measured metrics, first planner comparison | — |
| R5.1 Bumperbot/Labbot | Both bases drive, map and navigate | Repeats are V2 |
| R5.4 PX4 X500 flight | Native PX4 v1.16.2 takeoff/hover/waypoints/landing/failsafe | — |
| R8.2 Isaac host | Isaac qualified on this Jetson for named cells | — |

### R3.6 — External robot assets in normal Launch

- **State:** partial · **Priority:** I15 · **Depends on:** R3.1, R3.4
- **Goal:** every robot found in the requested open-source catalogs
  (robot-assets, URDFHub, MuJoCo Menagerie, robot_descriptions) is installed
  with its license and shown in the GUI. Each one is either controllable, or
  clearly marked display-only with the reason.
- **Have:** 152 robot profiles in 82 families. TurtleBot 3/4, Husky and the
  Menagerie Panda are fully controllable on four backends. Native Registry 3D
  preview. Source variants are grouped under parent families.
  [Asset guide](docs/ASSET_EXTENSION_GUIDE.md).
- **Remaining implementation:**
  - vendor controller interfaces for the display-only profiles (arms, legged
    robots, mobile bases), one source-matched controller each;
  - per-model license, texture and skin checks.
- **How to do it:**
  1. List display-only profiles: `src/robot_lab_robots/config/robots.yaml`
     and the provisioned registry (`scripts/provision_extension_assets.py`,
     `robot_lab_utils/installed_assets.py`).
  2. For each family, find the vendor's own control interface (ros2_control
     YAML, MJCF actuators, published policy). Reuse it through the existing
     native adapters (`robot_lab_mujoco/native_*`, `robot_lab_adapter/`).
  3. Provision one model at a time: `scripts/provision_extension_assets.py
     --kind robots --source <src> --entry <id> --no-download`. Never run upstream install scripts.
  4. Enable the GUI mode only after a real joint-state and motion check on the backend.
- **Done when:** each listed robot shows provenance, license and either
  working control or a visible reason why not.
- **Validation later:** per-model motion and mission checks (V6, V11).

### R5.2 — Go2 quadruped locomotion

- **State:** partial · **Priority:** I13 · **Depends on:** R5.1
- **Goal:** Go2 walks, turns, stops, recovers from falls and crosses terrain
  on every backend, then maps and navigates.
- **Have:** rl_sar policies walk on flat floors in MuJoCo, Gazebo and Isaac.
  Navigation passes `nav_obstacle` (3/3 MuJoCo and Gazebo, 2/3 Isaac).
  Leg-odometry SLAM holds 4.5–7.1 cm. Fall detection is implemented.
  Retained negative results: stairs fail at the first ledge, and every get-up
  attempt fails (nominal pose, ported learned actor, support-transfer roll).
  [Go2 tutorial](docs/tutorials/go2.md); full history in the archive.
- **Remaining implementation:**
  - a whole-body get-up strategy that acts correctly at a collapsed pose, or
    a get-up policy trained in the target simulator (R12);
  - terrain traversal: a terrain-trained policy (R12 curriculum) or
    a perceptive gait. Flat-ground policies fail the stairs.
- **How to do it:**
  1. Train `push_recovery` and a get-up task with `legged-train`
     (`robot_lab_adapter/legged_training.py`, `ppo.py`) in MuJoCo, starting
     from the measured collapsed poses (the support-transfer evidence has them).
  2. Train with the rough heightfield and a stair curriculum: extend
     `legged_training.py` terrain options to steps, raising height over training.
  3. Hand each exported policy to Launch with `legged_policy:=<dir>` and run
     the existing screens: drive screen, `terrain_stairs` runner and the 60 N perturbation case.
  4. Keep GUI velocity/SLAM/navigation modes for terrain disabled until those screens pass.
- **Done when:** a measured get-up from the 60 N collapse, and a stairs
  traversal with tracked contact/effort on a named backend.
- **Validation later:** per-backend repeats; navigation on other maps (V4).

### R5.3 — Berkeley Humanoid Lite balance and walking

- **State:** partial · **Priority:** I13 · **Depends on:** R5.1
- **Goal:** BHL stands, walks, turns on the spot and stops reliably.
- **Have:** MuJoCo effort control with watchdog/reset. The ONNX policy walks
  forward and stops. A held turn stalls after about 2 s. This is a policy
  fixed point; actuator rate, filters and contact fixes were all tried and
  ruled out. GUI checkbox `bhl_enable_policy`.
- **Remaining implementation:** retrain the BHL policy in the target domain.
  Add BHL to `legged_training.py` with its observation/action/PD contract,
  then train `walk` and `turn_in_place`.
- **How to do it:**
  1. Read the source policy contract in `src/robot_lab_robots/berkeley_humanoid_lite/` and the
     humanoid controller in `robot_lab_adapter/`.
  2. Add a BHL robot spec to `legged_training.py`, built from the bridge's own MJCF import.
  3. Train and export, then check with the existing GUI drive path
     (`/key_vel` → twist_mux → policy) and `analyze_turn_gait.py`.
- **Done when:** a held ±0.3 rad/s turn and a sustained walk track their
  commands in MuJoCo, with no stall.
- **Validation later:** Gazebo/Isaac transfer; terrain (V4).

### R5.5 — Ackermann, rear-steer and anti-Ackermann cars

- **State:** partial · **Priority:** I14 · **Depends on:** R5.1, R3.3
- **Goal:** three steering geometries drive, park, slalom and navigate with
  correct turning limits on every backend.
- **Have:** distinct steering geometries verified by joint traces. The obstacle
  mission passes 35/36 over seeds 1–3 on four backends. Watchdogs pass.
  The car course with footprint, curvature and heading checks exists
  (`scripts/sim_car_course.py`, `src/robot_lab_maps/tools/gen_car_course.py`).
  It passes 3 of 12 cells; the others end 8–23° off the bay heading.
- **Remaining implementation:** final-heading control. The Nav2 goal checker
  ignores yaw, and pure pursuit aims at the goal point.
- **How to do it:**
  1. Add a yaw-aware goal checker (or enable `stateful` yaw tolerance) in
     the car Nav2 configs (`src/robot_lab_navigation/config/`).
  2. Add a final parking manoeuvre: a short reverse/forward Reeds–Shepp or
     three-point segment to the bay pose, respecting the minimum turning radius.
     Put it in the car controller (`src/robot_lab_controller/`) or as a BT action.
  3. Fix the rear-steer/anti-Ackermann wording in the GUI and tutorials.
- **Done when:** the car course passes with heading inside its limit on a
  named backend for all three geometries.
- **Validation later:** all 12 course cells, repeats, mapping/localization (V3).

### R5.6 — Four-wheel steering and mecanum

- **State:** partial · **Priority:** validation V3 (no missing code known)
- **Goal:** 4WS (Ackermann, crab, in-phase, pivot) and mecanum drive and
  navigate precisely on every backend.
- **Have:** obstacle missions pass in all 20 pattern/backend cells. Crab and
  in-phase pass 24/24 after backward sampling. SLAM holds 1.1–2.5 cm.
  Gazebo mecanum rollers slip (1.02 m odometry RMSE when strafing).
- **Remaining implementation:** none known. The roller slip is a candidate fix
  (contact parameters) if validation confirms it.
- **Done when:** strict final pose, contacts, cancellation, second goals, command
  loss, reset and longer mapping pass for every pattern and backend.

### R5.7 — Manipulators and motion planning

- **State:** partial · **Priority:** I7 · **Depends on:** R3.6, R2.2, R3.4
- **Goal:** arms jog, plan (MoveIt) and pick/place real objects on every backend.
- **Have:**
  - Native Panda: joints, Home, Stop, trajectories, MoveIt Cartesian targets
    and a physical cube grasp.
  - Thirteen native fixed arms with KDL/OMPL planning; UR5e and SO101
    reaches measured.
  - Cartesian Servo controls (`robot_lab_gui/servo_controls.py`).
  - Measured Panda object attachments.

  [Native arm guide](docs/tutorials/native-arm-planning.md).
- **Remaining implementation:**
  - a pick/place pipeline for arbitrary objects: object pose source, grasp
    pose generation, approach/retreat and attach/detach in the planning scene;
  - coupled mechanisms (mimic and four-bar joints);
  - arm controllers on Gazebo, PyBullet and Isaac.
- **How to do it:**
  1. Start from the Panda MoveIt workflow (`docs/tutorials/panda_arm.md`) and
     the attached-object code from the October 9 checkpoint
     (`docs/status/implementation-2026-10-09.md`).
  2. Add a "Pick / Place" panel to the Arm page (`robot_lab_gui/arm_tab.py`)
     that takes an object ID from the world. Object poses come from simulator
     truth, labelled as such, until perception exists.
  3. For other backends, reuse the backend's joint-trajectory interface
     (Gazebo ros2_control, PyBullet/Isaac spawner joint commands) behind the same action.
- **Done when:** a GUI pick → place of a named object, with measured object
  pose, on MuJoCo plus one other backend.
- **Validation later:** the remaining eleven arms, repeats, interruption/reset (V6).

### R5.8 — Robot hands and grippers

- **State:** partial · **Priority:** I8 · **Depends on:** R3.6, R5.7
- **Goal:** grippers and dexterous hands grasp and release objects with
  measured contact forces.
- **Have:** the Panda gripper cube grasp; DEX-EE and three coupled-tendon
  hands with closure/spread, finger selection and contact feedback.
  [Hand guide](docs/tutorials/native-hand-controls.md).
- **Remaining implementation:**
  - raw torque and tendon hand adapters for the remaining source hands;
  - object workflows (stable grasp detection, lift, hold, release) in the Hand page;
  - hands on other backends.
- **How to do it:** extend `robot_lab_mujoco/native_hand_control.py` and
  `robot_lab_gui/native_hand_controls.py`. Measure grasp stability from
  contact forces (`test_native_hand_object.py` shows the pattern).
- **Done when:** a measured stable grasp, lift, hold and release of a documented
  object for each hand family on at least one backend.
- **Validation later:** all finger templates, repeats, other backends (V6).

### R5.9 — Mobile manipulators

- **State:** partial · **Priority:** I6 · **Depends on:** R5.1, R5.7, R5.8
- **Goal:** a mobile base with an arm drives to an object, picks it up,
  carries it and puts it down, from the GUI.
- **Have:** native Stretch / Stretch 3 navigation plus taught articulation
  (`native_mobile_control.py`, `native_mobile_workflow.launch.py`,
  [native controls](docs/tutorials/native-controls.md)), marked "runtime
  validation pending".
- **Remaining implementation:**
  - a mission sequencer: navigate → stop → reach → grasp → transport → release;
  - base/arm interlock: no base motion while the arm is out, as R11 does;
  - footprint changes while carrying;
  - Fetch, PR2 or TIAGo as a second source.
- **How to do it:**
  1. Write the sequencer as a small state machine node in
     `robot_lab_adapter/`. Nav2 goal for the base, then R5.7 pick/place for the arm.
  2. Reuse R11's articulation contract (`robot_lab_utils/urdf_articulation`)
     for the base inhibit.
  3. Add a "Fetch object" panel: choose object and destination, Start, Stop.
  4. For a second robot, check maintained ROS 2 descriptions (Fetch,
     TIAGo by PAL Robotics), with licenses (see R14.1).
- **Done when:** one measured mission with object pose and stability
  traces; base and arm commands cannot interfere.
- **Validation later:** reset, cancel, watchdog and obstacles per backend and map (V6).

### R5.10 — PX4 flight in all installed worlds

- **State:** partial · **Priority:** validation V5 (code exists)
- **Goal:** the X500 flies in every installed world, with correct spawn,
  takeoff clearance and landing.
- **Have:**
  - All worlds selectable; takeoff-clearance check gate; seven-button Drone controls.
  - `nav_empty`, `nav_obstacle` and `aerial_course` manual flights pass.
  - Mapped and live-depth obstacle route planning (`aerial_obstacles.py`,
    [mapped drone planning](docs/tutorials/mapped-drone-planning.md)).
- **Remaining implementation:** world-specific fixes found during validation.
- **Done when:** each world: display, takeoff, hold, manual XYZ, land/disarm and
  reset artifacts; actor/plugin failures reported.

### R5.11 — Drone moving obstacles and other drone backends

- **State:** not started · **Priority:** I5 · **Depends on:** R5.10
- **Goal:** the drone avoids moving obstacles and handles sensor loss in flight.
  Drones also fly in a second simulator.
- **Have:** static plus observed voxel obstacles with a 2 m lookahead
  interruption and no automatic resume
  ([October 14](docs/status/aerial-obstacles-2026-10-14.md)).
- **Remaining implementation:**
  - moving-obstacle tracking and prediction (velocity from consecutive depth
    clusters; constant-velocity or Kalman prediction) and time-aware
    replanning;
  - sensor-loss behaviour: hold, then land after a timeout;
  - a second drone backend, such as PX4 SITL with a MuJoCo or Isaac plant.
    This needs a real FCU-plant bridge, not a metadata change.
- **How to do it:**
  1. Extend `robot_lab_utils/observed_aerial_scene.py` with tracked clusters
     and predicted swept volumes. Feed them to the existing 3D A* planner in
     `robot_lab_adapter/` as time-indexed obstacles.
  2. Add moving actors to an aerial world (`src/robot_lab_maps/`) for testing.
  3. For another backend, read the PX4 simulator-interface docs and write
     a lockstep sensor/actuator bridge.
- **Done when:** a flight in which a moving obstacle crosses the route, and
  the drone replans or holds without contact.
- **Validation later:** repeats, other worlds (V5).

### R6.1 — Map geometry, alignment and resets

- **State:** partial · **Priority:** validation V2
- **Goal:** runtime coordinates match the maps; resets restore the full state.
- **Have:** real occupancy exports for 17 grids; named reset/contact screens.
- **Done when:** geometry, spawn/goal clearance and resets are verified per
  world and backend, with seeds recorded.

### R6.2 — Dynamic and sensor-disturbance cases

- **State:** partial · **Priority:** I16 · **Depends on:** R6.1, R4.2
- **Goal:** seeded moving obstacles and sensor faults: noise, bias, drift,
  delay, dropout, occlusion and outliers.
- **Have:** `src/robot_lab_maps/tools/r6_dynamic_sensor_disturbance.py`
  (framework and tests).
- **Remaining implementation:** inject the faults into the live sensor topics
  (a relay node with seeded fault models), and actor worlds with real collision.
- **How to do it:** a `sensor_fault_relay` node in `robot_lab_utils/`,
  configured per scenario in `scenarios.yaml`, with its own seeds. Truth topics
  are never touched.
- **Done when:** fault traces repeat from seeds; baseline and degraded runs
  share the same task and budgets.
- **Validation later:** recovery-time measurements (V9).

### R6.3 / R6.4 — 3D terrain, aerial volumes and diverse maps

- **State:** partial · **Priority:** validation V9 (code exists)
- **Goal:** height/voxel/mesh queries for legged and aerial tasks; at least six
  distinct seeded layouts across ground, legged and aerial use.
- **Have:** terrain arenas (`gen_terrain_arenas.py`, `gen_heightfield_arena.py`), 54 world profiles.
- **Done when:** class-appropriate terrain/flight missions pass, with
  dimension and frame metadata on maps and results.

### R6.5 — World-to-occupancy generation

- **State:** partial · **Priority:** I16 · **Depends on:** R6.1, R3.4
- **Goal:** generate a correct 2D grid from any selected 3D world in the GUI.
- **Have:**
  - Worlds tab Generate/Stop with editable seeds.
  - `src/robot_lab_maps/tools/generate_occupancy_map.py`,
    `scripts/review_occupancy_maps.py` and `scripts/refresh_installed_occupancy_maps.py`.
  - Rotated triangle exports.
- **Remaining implementation:**
  - handle disconnected free regions (choose by seed or keep all, labelled);
  - a "register map" step after review, so wrong grids are never auto-registered.
- **Done when:** primitive, rotated and furnished mesh worlds produce grids
  that agree with the geometry, and Nav2 uses them.

### R6.6 — External world library

- **State:** partial · **Priority:** I16 · **Depends on:** R3.6, R6.5
- **Goal:** the requested external worlds (mlherd, Gazebo examples, licensed
  Fuel worlds) are installed with license and status.
- **Have:**
  - 14 dataset worlds and three Gazebo examples installed; all 100 Fortress
    example SDFs downloaded.
  - Hospital fixtures repaired in derived variants; `scripts/integrate_external_worlds.py`.
- **Remaining implementation:**
  - the remaining licensed Fuel environments;
  - behaviour and resource review of the 97 plugin/robot fixtures;
  - actors and upper floors.
- **Done when:** every requested source shows its status, and each new world
  class has one robot mission and a generated occupancy file.

### R6.7 — Terrain generation GUI and four-backend conversion

- **State:** partial · **Priority:** I4 · **Depends on:** R6.3, R6.6
- **Goal:** a GUI page that generates terrain from a provider or
  procedurally, saves it on the SSD with attribution, and converts it for all
  four simulators.
- **Have:**
  - Shared heightfield conversion (`robot_lab_utils/heightfield.py`, `sdf_world.py`).
  - Measured Gazebo conventions; MuJoCo/PyBullet contact screens.
  - `terrain_heightfield` world.
- **Remaining implementation:**
  - the GUI page (procedural or provider source, area, resolution, seed,
    spawn, height range → Generate → preview → register as a world);
  - a provider and attribution manifest (provider, polygon, datum,
    metres/pixel, elevation range). Keep tokens out of the repo and logs;
  - Isaac terrain contact.
- **How to do it:**
  1. Add a "Terrain" page to the Worlds tab (`robot_lab_gui/`), reusing the
     occupancy page layout. Write to `$ROBOT_LAB_RUNTIME_ROOT/terrain/<id>/`.
  2. Procedural first: Perlin/diamond-square with a seed (no provider needed).
     Provider tiles are optional and token-gated.
  3. Use `heightfield.py` to write Gazebo SDF, MuJoCo hfield, a PyBullet
     heightfield and an Isaac mesh. Register through the map profile config
     (`src/robot_lab_bringup/config/sim_maps.yaml`).
  4. Check that the same support height appears on all four backends with a
     dropped-box probe (the October 5 probe scripts under
     `docs/status/evidence/r6-7-heightfield-probe-2026-10-05/` show the method).
- **Done when:** a generated terrain regenerates identically from its saved
  inputs and gives matching support heights on four backends.
- **Validation later:** class-specific driving, walking and flight on it (V9).

### R7 — Algorithm breadth

- **State:** partial · **Priority:** I19, then V10 · **Depends on:** R7.1
- **Goal:** five distinct, correct methods per category, compared fairly.
- **Have:** real methods run in every category:
  - **Global and local planning:** eight global planners and five local
    planners measured on identical seeded goals.
  - **Control:** six path-tracking controllers, measured on a model plant and in Gazebo.
  - **Fusion, perception, localization, state estimation:** measured offline
    with known truth and on recorded or live simulator data.

  Tutorials: [global](docs/tutorials/global_planning_comparison.md),
  [local](docs/tutorials/local_planning_comparison.md),
  [control](docs/tutorials/control_comparison.md),
  [fusion](docs/tutorials/sensor_fusion_comparison.md),
  [perception](docs/tutorials/perception_comparison.md),
  [localization](docs/tutorials/localization_comparison.md),
  [state estimation](docs/tutorials/state_estimation_comparison.md).
- **Target methods** (choose a justified replacement where needed):

  | Task | Five methods |
  |---|---|
  | R7.2 Perception | scan clustering, Euclidean cloud clustering, DBSCAN, RANSAC ground + segmentation, voxel occupancy (+ RGB-D vision stratum) |
  | R7.3 Localization | wheel dead reckoning, AMCL, ICP, NDT, RTAB-Map RGB-D |
  | R7.4 State estimation | KF, EKF, UKF, particle filter, error-state/invariant EKF |
  | R7.5 Sensor fusion | complementary, Mahony, Madgwick, wheel/IMU EKF, wheel/IMU/GNSS UKF |
  | R7.6 Global planning | Dijkstra, A*, PRM, RRT, RRT* |
  | R7.7 Local planning | DWB/DWA, Regulated Pure Pursuit, TEB or replacement, MPPI, Follow-the-Gap |
  | R7.8 Control | PID with anti-windup, LQR, linear MPC, nonlinear MPC, feedback linearization/backstepping |

- **Remaining implementation:**
  - simulator-applied seeds (each backend actually reseeds noise and actors);
  - a method subrecord in the ledger for each method, including any method
    whose name does not match its mathematics.
- **How to do it:** the algorithm code is in `src/robot_lab_algorithms/`,
  with ROS wrappers in `src/robot_lab_adapter/` and the catalog in
  `robot_lab_registry/config/algorithms.yaml`. Dispatch is in
  `src/robot_lab_bringup/config/algorithm_dispatch.yaml`. For each method,
  add a numerical test, an installed adapter and an experiment.
- **Done when:** each category has five integrated methods with numerical
  tests and measured comparisons in fair input strata.
- **Validation later:** the comparisons themselves (V10).

### R8.1 / R8.3 — Backend matrix and concurrent experiments

- **State:** partial · **Priority:** validation V11 (code exists)
- **Have:** MuJoCo and PyBullet Bumperbot Nav2 jobs pass serially and at
  the same time. Budgets stop only their own job.
  [Concurrency evidence](docs/status/evidence/r83-concurrency-2026-10-15/README.md).
- **Done when:** R2 contracts and the mobile experiment pass on each backend,
  and concurrent runs keep isolated clocks, seeds and results under budgets.

### R9.1 — Provenance, licenses and clean-host reproduction

- **State:** partial · **Priority:** I20, then V13
- **Have:** 120/125 dependency licenses resolved; asset attribution checked
  (`scripts/asset_attribution.py`); a clean container builds and runs a MuJoCo mission.
- **Remaining implementation:** check the PX4 license against its checkout;
  list licenses for the new open-source robots (R14.1).
- **Done when:** a fresh host reproduces the golden comparison, and provenance is recorded.

### R9.2 / R9.3 — Comparison tutorials and release gate

- **State:** partial · **Priority:** validation V13
- **Have:** seven comparison tutorials (three measured in Gazebo). Support
  matrix: 717 reports, 327 cells, 314 passed (`scripts/r9_3_support_matrix.py`,
  [matrix](docs/status/evidence/support-matrix-2026-11-10b/README.md)).
- **Done when:** every release claim maps to artifacts; failures and limits
  are published.

### R9.4 — Optional physical Bumperbot HIL

- **State:** blocked: needs the physical robot, a safe setup and explicit
  authorization. Simulation work never authorizes motor or FCU actions.

### R10.1 — Shared-world multi-robot sessions

- **State:** partial · **Priority:** validation V7 (code exists)
- **Have:** namespaced controllers, sensors and TF in one Gazebo world;
  per-instance and group Drive; scoped reset. Two overlapping Nav2 missions
  are measured. [Fleet guide](docs/tutorials/fleet-and-swarm.md).
- **Done when:** contacts, member loss/reset and shutdown are measured for
  3–8 members on each advertised backend.

### R10.2 — Fleet management and Open-RMF

- **State:** partial · **Priority:** I9 · **Depends on:** R10.1
- **Goal:** missions are dispatched to a fleet and coordinated at shared
  places (corridors, doors, lifts, chargers).
- **Have:**
  - Humble Open-RMF adapter (`robot_lab_utils/fleet_rmf_adapter.py`) driving real Nav2 missions.
  - Queues, nearest-idle allocation, pause/resume/cancel.
  - Physical door and lift events; one Bumperbot lift ride L0→L1→L0;
    post-exit parking. [Remaining-work guide](docs/REMAINING_WORK_GUIDE.md).
- **Remaining implementation:**
  - conflict resolution with two or more robots on the same route;
  - occupied-lift recovery;
  - calibrated energy (motor-power observations) for RMF scheduling;
  - more infrastructure: chargers and doors in other worlds.
- **How to do it:** extend `fleet_controller.py`, `fleet_transport_queue.py`,
  `fleet_lift_recovery.py` and `fleet_rmf_adapter.py`. Keep reservations and
  observation gates; never move a robot by bypassing them. Batch runs:
  `robot-lab-fleet-workload --spec ...`.
- **Done when:** two-robot conflicts at a shared resource resolve, with
  physical outcomes matching task status.
- **Validation later:** larger fleets, contacts, loss (V7).

### R10.3 — Swarm robotics

- **State:** partial · **Priority:** I10 · **Depends on:** R10.1
- **Goal:** formation, rendezvous, dispersion, flocking and cooperative
  coverage for ground robots and drones, with group Stop and member loss.
- **Have:**
  - Four centralized ground kernels with freshness/separation guards.
  - Two-X500 range-reference formation and operator-loss landing (measured).
  - Formation route Plan/Execute (`aerial_swarm_route.py`, `aerial_swarm_controller.py`).
- **Remaining implementation:**
  - distributed (per-member) behaviour versions;
  - cooperative coverage with sensed coverage;
  - 3–4 drone groups;
  - member-loss replanning.
- **How to do it:** compare with ROS2swarm behaviours (see the
  [extension guide](docs/NEXT_EXTENSION_GUIDE.md)). Keep per-FCU IDs, ports,
  leases and frames. Settle on FCU time; watchdogs use wall time.
- **Done when:** measured relative motion and member-loss outcomes for each
  behaviour on ground and air.

### R10.4 — Underwater robots and controllers

- **State:** partial · **Priority:** I11
- **Have:** the self-made eight-thruster `lab_rov`; the pinned CC-BY MBARI
  Tethys AUV with fins and trim; Fortress buoyancy, drag and added mass.
  Tank surge/depth, reef depth/turn and a gentle tank curve are measured.
  [ROV](docs/tutorials/marine-rov.md), [AUV](docs/tutorials/tethys-auv.md).
- **Remaining implementation:**
  - an open-source ROV (BlueROV2-class, see R14.1);
  - marine force adapters for other backends.
- **Done when:** station keeping, all axes, disturbance and loss measured with body and force evidence.

### R10.5 — Underwater worlds, sensing, SLAM and navigation

- **State:** partial · **Priority:** I11 · **Depends on:** R10.4
- **Have:**
  - Five water worlds (tank, reef, harbour, cave, wreck).
  - IMU/RGB-D bridges, labelled ideal pressure, DVL emulation (no native plugin), directional sonar.
  - ROV visual/ICP and AUV geometry registration.
  - RTAB-Map save/reload; curved AUV routes (`auv_routes.py`).
- **Remaining implementation:**
  - sensor-only localization on saved maps for the AUV;
  - 3D (volumetric) routes;
  - current-aware planning, clearly labelled.
- **How to do it:** `marine_sensors.py`, `marine_navigation.py`, `marine_pipeline.py` and
  `map_archive.py` in `robot_lab_utils/`. Never feed simulator pose into the estimator.
- **Done when:** an obstacle or inspection mission runs from real sensors and estimates.

### R10.6 — Six- and eight-legged robots

- **State:** partial · **Priority:** I12
- **Have:**
  - Pinned MIT `hexapod_ros2` (CAD) and the self-made `lab_octopod`.
  - Tripod/tetrapod/ripple/wave gaits; four backend adapters.
  - Low-relief travel and hold pass on all four simulators.
  - [Gait guide](docs/tutorials/multi-leg-gaits.md).
- **Remaining implementation:**
  - per-model terrain/slip/recovery;
  - independent estimation;
  - saved-map goals for every model;
  - an open-source octopod, if one exists (R14.1).
- **How to do it:** `gait_model.py`, `gait_controller.py`, `gait_odometry.py` and
  `gait_terrain.py` in `robot_lab_utils/`. Keep one owner of `/gait/state`.
- **Done when:** walking on terrain, contacts and saved-map navigation per model and backend.

### R10.7 — Segmented and additional robot forms

- **State:** partial · **Priority:** I12
- **Have:** self-made centipede, snake, catamaran USV, tracked and
  articulated-flipper vehicles, and a sprung amphibious rover. Each has native
  controls; mapping, navigation and shore cases are measured.
  [Native forms](docs/status/native-forms-2026-10-30.md).
- **Remaining implementation:**
  - flipper-aware terrain navigation;
  - terrain/slip/recovery controllers;
  - other backends;
  - open-source counterparts (R14.1).
- **Done when:** locomotion, clearance and sensor/map missions per form.

### R10.8 — Fleet, swarm and marine GUI and experiments

- **State:** partial · **Priority:** validation V7 (code exists)
- **Have:** native Fleet/Swarm/Marine/Gait/Tracks panels with autofill; real workload producers.
- **Done when:** GUI commands exercise each claimed runtime, with measured artifacts.

### R11 — MOLAR / ProBOT-L family

- **State:** partial · **Priority:** I17 (needs the real robot's sensor layout from the user)
- **Goal:** the user's MOLAR robots drive, carry payloads and navigate the
  detailed CELISCA floors.
- **Have:** `molar_hri` passes the GUI check (drive, lift, telescope, roller,
  Stop, watchdog) on four backends and three maps; `probot_l` passes on `nav_empty`.
  [Guide](docs/tutorials/molar-hri-simulation.md).
- **Remaining implementation:**
  - mounted sensors (needs the real layout);
  - SLAM/Nav2 on the floors;
  - payload transfer (lift + roller to a station);
  - multi-floor operation through the fleet lift (R10.2).
- **How to do it:** add sensors to `molar_hri.urdf.xacro`, then reuse the
  Bumperbot SLAM/Nav2 configs with the MOLAR footprint. Payload transfer
  uses the existing articulation contract (`robot_lab_utils/urdf_articulation`).
- **Done when:** a measured route plus payload transfer on a CELISCA floor.

### R12 — Legged policy training

- **State:** partial · **Priority:** I13
- **Have:**
  - The GUI Training page and `legged-train` (PPO, ONNX export) for Go2, A1 and B2.
  - Tasks: walk, turn_in_place, stand, push_recovery; randomisation,
    rough terrain, noise and actuation delay.
  - `go2_walk_r1`–`r3` pass drive screens on three backends.
  - [Guide](docs/tutorials/legged-policy-training.md).
- **Remaining implementation:**
  - a terrain curriculum (the rough policy does not walk the heightfield);
  - G1 command tracking (fails now) and BHL (R5.3);
  - Go2W strafe;
  - GPU training.
- **How to do it:** `robot_lab_adapter/legged_training.py`, `ppo.py` and `legged_train.py`.
  Add robots by their provisioned policy contract; train in MuJoCo; check
  through `legged_policy:=<dir>` on all backends.
- **Done when:** a trained policy performs its task in measured runs on the backends.
- **Validation later:** Isaac speed gap, navigation precision, repeats (V4).

### R13 — Robot-vacuum cleaning system (OOMWOO-class)

#### R13.1 — Core system (implemented and measured)

- **State:** partial (validation) · **Priority:** V1
- **Have:**
  - `robot_lab_vacuum_cleaning`: rooms, lanes, edges, zones, spot,
    mapping runs, no-go areas and walls, stall detector, dock, battery,
    dustbin, consumables, schedules and history.
  - The Vacuum workspace in the GUI.
  - All `cleaning_screen` runs pass in Gazebo small_house. MuJoCo, PyBullet
    and Isaac keep-out runs pass. The Gazebo bumper works (89b71ebb).
  - A live GUI session (office with Pause/Resume, keep-out zone drawn with
    the map tools) passes.
  - [Evidence](docs/status/evidence/vacuum-2026-11/README.md),
    [guide](docs/tutorials/robot-vacuum.md),
    [how it works](docs/learn/07-robot-vacuum-inside.md).
- **Repeats:** office, keep-out zone and spot have three passing runs each.
- **Second building (`celisca_floor_1_furniture`): fails so far.** The
  failures, in order, each with its fix:
  1. interference from another session's `/clock` on the same ROS domain
     (now domain 117);
  2. its own bump marks walled it in under furniture (escape added);
  3. wedged with slipping wheels, the estimate drifting 1.3 m (wedge
     detection added; cliff floor calibration fixed);
  4. stopped with "Stuck" but wrote no report (headless error report added);
  5. the estimate ended in a closed pocket of the map (rejoin added).

  The rerun at 7162a09a is queued. Room 2's furniture area, where the robot
  gets under furniture, is the open problem: consider a furniture-height
  check or keep-outs there.
- **Validation later:** more repeats; other houses.

#### R13.2 — Vacuum carpets and mopping

- **State:** implemented and measured once (2026-11-10) · **Priority:** done in
  the implementation queue; repeats are V1
- **Have:**
  - Carpet areas (a map tool) with Max-suction carpet boost; mopping jobs
    keep off carpets.
  - Clean modes: vacuum, vacuum + mop, mop only.
  - A water-tank model with refill-and-resume at the dock; a mop-pad consumable.
  - `cleaning_screen` treats carpets as no-go areas for jobs that mopped;
    reports and figures show the mop and carpet fields.
  - Core tests (29) and GUI tests pass.
  - Gazebo, small_house, vacuum + mop zone with a rug: pass. Coverage 99.4 %,
    8.13 m² mopped, 14.6 ml of water; the centre stayed at least 0.249 m from
    the rug edge (body radius 0.165 m); docked within 5.9 cm.
    [Evidence](docs/status/evidence/vacuum-2026-11/README.md).
- **Remaining implementation:** none.
- **Validation later:** the carpet-boost run (its first attempt failed to
  launch from an unrelated edit, not the vacuum code), mop-only with a refill,
  repeats.

#### R13.3 — Vacuum camera obstacle recognition

- **State:** implemented and measured once (2026-11-10) · **Priority:** I2 (learned detector remains) · **Depends on:** R13.1
- **Goal:** like commercial vacuums, a front camera recognises cables,
  socks, shoes and pet mess. The robot keeps a class-specific distance and
  shows them on the map.
- **Measured:** Gazebo, small_house copy with a cable, sock, shoe and pet
  mess: all four recognised, **0 bumper contacts**, body ≥ 0.156 m from every
  footprint by truth, `cleaning_screen` pass (86.2 %). The first run touched
  the shoe (a 2 m sighting, 16 cm off); fixed by refined positions and an
  8 cm margin. [Evidence](docs/status/evidence/vacuum-2026-11/README.md).
- **Have:**
  - a front camera on `oomwoo`: RGB-D plus a pixel-aligned semantic
    segmentation camera (`oomwoo.urdf.xacro`, `oomwoo_gazebo.xacro`), bridged
    to `/vacuum/camera/{image,depth,camera_info,labels}` in `gazebo.launch.py`;
  - `vacuum_objects.py` (`vacuum_object_detector`): projects labelled
    pixels through depth and TF to floor objects on `/vacuum/objects`. The
    segmentation detector is an *ideal* detector (simulator class labels;
    positions from depth), and its messages say so;
  - `mission.py`: remembered objects with class berths (cable 15 cm, sock
    10 cm, shoe 6 cm, pet mess 35 cm), re-routing of the active lane,
    `ignore_object` / `forget_objects`, record fields `objects_avoided` and `object_detector`;
  - GUI: object glyphs on the map; the Erase tool ignores an object;
  - `scripts/vacuum_object_world.py`: a world copy with labelled cable, sock,
    shoe and pet mess (Gazebo Label system), checked against free floor in the map;
  - tests: projection, avoidance and ignore (core), drawing and erase (GUI).
- **Remaining implementation:**
  1. A learned detector with a permissive license (for example a torchvision
     detector fine-tuned on images rendered from these worlds; no AGPL code)
     publishing the same labels image or objects message.
  2. GUI snapshots of each object from `/vacuum/camera/image`.
  3. An object-clearance check in `cleaning_screen` (the gaps above come from
     a separate truth-trail computation).
- **Done when:** a recorded job in a house with these objects ends with zero
  contacts with them by truth, and passes `cleaning_screen`.

### R14.1 — Open-source counterparts for self-made robots

- **State:** partial (first search pass done 2026-11-10) · **Priority:** I3 · requested by the user 2026-11-10
- **Search record:** [r14-open-source-candidates-2026-11-10](docs/status/r14-open-source-candidates-2026-11-10.md).
  Finding: the iRobot Create 3 (BSD) is already installed, and the installed,
  measured TurtleBot 4 Lite (Create 3 plus LiDAR) is the quickest open-source
  platform for the cleaning system.
- **Goal:** the lab uses open-source robots wherever a suitable one exists.
  Self-made robots stay only where none exists, with the reason recorded.
- **Why this task exists:** these profiles were authored by agents in this
  project rather than imported. The recorded reasons:
  - no suitable licensed model was found: octopod and centipede, per the
    [October 14 research](docs/NEXT_EXTENSION_GUIDE.md#upstream-candidates-researched-on-october-14);
  - the robot has no published model: OOMWOO's hardware is not frozen;
  - imported models lacked working actuation for the lab's simulators.

  For several of them, no thorough search was recorded.

  | Self-made profile | Open-source candidates to check (verify license and model quality) |
  |---|---|
  | `oomwoo` (robot vacuum) | iRobot Create 3 description (used by TurtleBot 4; real bumper/cliff sensors) |
  | `lab_rov` | BlueROV2 Gazebo models |
  | `lab_usv` | VRX WAM-V (Open Robotics Virtual RobotX) |
  | `ackermann_car`, `rear_steer_car`, `anti_ackermann_car` | F1TENTH, other maintained Ackermann ROS 2 models |
  | `mecanum_car`, `four_wheel_steer_car` | Robotnik Summit-XL (mecanum), other ROS 2 omni/4WS bases |
  | `lab_tracked`, `lab_tracked_articulated` | maintained tracked/flipper ROS 2 robots (search ROS index, Gazebo Fuel) |
  | `lab_snake`, `lab_amphibious`, `lab_octopod`, `lab_centipede` | search again; keep self-made if none is complete and licensed |
  | `quadrotor_sitl` (display) | already superseded by the real PX4 `px4_x500` |

- **How to do it:**
  1. For each row, search the ROS index, Gazebo Fuel, `robot_descriptions`,
     MuJoCo Menagerie, URDFHub and GitHub. Record each candidate in
     `docs/status/asset-sources-2026-10-05.yaml`, with revision, license,
     completeness (inertias, collision, actuators, sensors) and ROS 2 / simulator compatibility.
  2. Choose only complete, redistributable models. Provision with
     `scripts/provision_extension_assets.py` as a **new** profile. Keep the self-made
     profile and its evidence; never replace it in place.
  3. Connect the existing controller for that class: cars → car controller,
     mecanum → holonomic controller, ROV → marine allocator, vacuum →
     `robot_lab_vacuum_cleaning`, which runs on any robot with `supports_room_vacuum`.
  4. Record the outcome per robot in this table: adopted, rejected (why) or
     none found.
- **Done when:** every self-made robot has a recorded search result, and
  each adopted counterpart drives under its class controller on one backend.

### Stabilization patches (P1–P8)

| Patch | State | Remaining | Priority |
|---|---|---|---|
| P1 BHL/Go2 localization regression | done | keep the feedback cadence | — |
| P2 GUI neutral input | partial | physical joystick trial (enable, axes, neutral, unplug, Space, close) | V12 |
| P3 Celisca geometry/navigation | partial | both bases in the remaining cells, obstacle routes | V2 |
| P4 MuJoCo speed | partial | see below | I18 |
| P5 four-wheel steering | partial | repeats (R5.6) | V3 |
| P6 mecanum | partial | repeats (R5.6) | V3 |
| P7 legged/humanoid policies | partial | model-specific contracts (R5.2, R5.3, R12) | I13 |
| P8 PX4 drone | partial | other maps, moving obstacles (R5.10, R5.11) | I5 / V5 |

#### P4 — MuJoCo large-world speed

- **State:** partial · **Priority:** I18
- **Have:** steps about 2× faster with identical contacts (Labbot
  celisca_floor_1 RTF 0.31 → 0.675; hospital 0.13 → 0.344).
  [Evidence](docs/status/evidence/p4-mujoco-step-2026-10-12/README.md).
- **Remaining implementation:** reduce the flex-vertex update, LiDAR scan and
  camera costs in `robot_lab_mujoco` (profile them first, then batch the
  ray casts and render the camera only at its declared rate).
- **Done when:** RTF ≥ 1 on the large worlds, with the same contacts.

---

## 6. Where to find details

| Topic | Document |
|---|---|
| Current measured state | [CURRENT_STATUS](docs/status/CURRENT_STATUS.md) |
| Ledger (machine-readable state and owners) | [platform-status.yaml](docs/status/platform-status.yaml) |
| Session start and handoff | [AGENT_HANDOFF](docs/AGENT_HANDOFF.md) |
| Fleet, aerial, marine, multi-leg working notes | [REMAINING_WORK_GUIDE](docs/REMAINING_WORK_GUIDE.md) |
| R10 sources and design | [NEXT_EXTENSION_GUIDE](docs/NEXT_EXTENSION_GUIDE.md) |
| Assets, worlds, extensions | [ASSET_EXTENSION_GUIDE](docs/ASSET_EXTENSION_GUIDE.md) |
| Stabilization patches | [PATCH_EXECUTION_GUIDE](docs/PATCH_EXECUTION_GUIDE.md) |
| Robot readiness order (motion → SLAM → navigation) | [AI_ROBOT_READINESS_GUIDE](docs/AI_ROBOT_READINESS_GUIDE.md) |
| Large files | [STORAGE](docs/STORAGE.md) |
| Beginner course | [docs/learn](docs/learn/README.md) |
| Old roadmap text, dated narratives, negative results | [archive/ROADMAP-2026-11-10](docs/status/archive/ROADMAP-2026-11-10.md) |
