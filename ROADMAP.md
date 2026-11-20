# Robot Lab roadmap

**This is the project's single goal list.** Every task, goal and piece of
remaining work is here, in one format, in priority order. Updated 2026-11-14.

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
| I0 | [R14.2](#r142--elephant-robotics-myagv-pro) myAGV Pro | Done: Gazebo source variant plus portable roller variant driving in MuJoCo, PyBullet and Isaac; Nav2 measured in Gazebo and PyBullet | implemented |
| I0a | [R14.3](#r143--elephant-myagv-pro-mobile-manipulator) Elephant mobile manipulator | Bare Pro630 drive and joints measured on MuJoCo/PyBullet/Isaac. 11-20: vendor F100 variant and guarded flange planning in MuJoCo. Pick/place and mobile missions with the gripper remain | partial |
| I0b | [R14.4](#r144--contributor-integration-course) Integration guide chapters | Done 2026-11-13: beginner course part 2 (chapters 9–12) in docs/learn and the GUI docs library; clean-host walkthrough remains | implemented |
| I1 | [R13.2](#r132--vacuum-carpets-and-mopping) Vacuum carpets and mopping | Done 2026-11-10 (Gazebo mop + rug run passes) | implemented |
| I2 | [R13.3](#r133--vacuum-camera-obstacle-recognition) Vacuum camera obstacles | Done-when met (ideal and learned). 11-17 retrain 3 installed (fewer pet/cable false positives, pet recall 54 → 60 %); learned zone job passes again (0 contacts, 0.286 m). Sock false positives remain; repeats/held-out room are V1 | implemented |
| I3 | [R14.1](#r141--open-source-counterparts-for-self-made-robots) Open-source robots | Initial source-search/integration pass complete: TurtleBot4, WAM-V, Summit-STEEL and source octopod measured; BlueROV2 remains a restricted local candidate. Remaining licensing/mission qualification recorded separately | implemented; acceptance partial |
| I4 | [R6.7](#r67--terrain-generation-gui-and-four-backend-conversion) Terrain GUI | Done 2026-11-13: Worlds → Terrain page, procedural/raster sources with manifest, support height measured on all four (Isaac included) | implemented |
| I4a | [R14.5](#r145--robot-status-markers-in-the-gui-selectors) Robot status markers | Done 2026-11-13: ● measured / ◐ experimental / ○ display only / ✕ unavailable plus mode letters in both robot lists, from recorded evidence | implemented |
| I4b | [R6.8](#r68--real-world-area-import-from-a-globe) Real-world area import | Done 2026-11-13: globe picker (CesiumJS + OSM), Terrain Tiles + OSM buildings/roads/trees/water, SUMO people/traffic; support and roof heights measured on four simulators | implemented |
| I5 | [R5.11](#r511--drone-moving-obstacles-and-other-drone-backends) Drone moving obstacles | Done 2026-11-13: Kalman tracks, predicted sweeps, walkway look-both-ways and overflight, evasion, sensor-loss hold/land, EKF heading/height gate; 2 passing flights past walkers (Gazebo); PX4 + MuJoCo plant: takeoff, 5 m route and landing | implemented |
| I6 | [R5.9](#r59--mobile-manipulators) Mobile manipulation | TIAGo fetch, Nav2 transit and RGB-D grasp workflows measured on MuJoCo and PyBullet (11-17/18). Manual tool-pose planning measured in the MuJoCo GUI (11-20). Remaining: general perception, Isaac grasp, Stretch pipeline missions, additional backend planners | partial |
| I7 | [R5.7](#r57--manipulators-and-motion-planning) Arms | Done-when met 11-17: URDF Panda GUI pick → place on PyBullet and MuJoCo (one run each). Native Panda box tasks on MuJoCo. 11-17: URDF Panda pick and place also passes on Isaac and Gazebo (all four backends); UR5/UR10/YuMi joint control on Gazebo. 11-18: upright-cylinder pick and place on MuJoCo, PyBullet and Gazebo. Remaining: compound/mesh geometry, fixed-arm perception, other coupled mechanisms | partial |
| I8 | [R5.8](#r58--robot-hands-and-grippers) Hands | Done-when met 11-19: normal GUI grasp → lift 5 cm → hold → return → supported release passes for Robotiq 2F-85 (both variants, support stage), UMI (source axis) and DEX-EE (synthesized tripod, 2/2); parallel-jaw measured in R5.7/R5.9. 11-20: new Wonik Allegro and Shadow Hand (coupled tendons) pass the same cycle (2/2 each). Arm transport and other backends remain | implemented |
| I9 | [R10.2](#r102--fleet-management-and-open-rmf) Fleet / Open-RMF | Done 2026-11-14: shared-zone reservations (map passages, doors, docks), cycle breaker, recovery drive-out, RMF energy from a drivetrain calibration; two crossing runs pass (native and RMF). 11-18: doors, docks and energy without a lift (`fleet_ward`: door crossing, docking and charging pass). Larger fleets are V7 | implemented |
| I10 | [R10.3](#r103--swarm-robotics) Swarms | Done 2026-11-14: ROS2swarm port of per-member scan-only behaviours, scan-sensed cooperative coverage, ground/air member-loss replanning; dispersion and coverage-with-loss measured; aggregation gathers three Bumperbots in `swarm_arena` (11-18); 3–4 drone flight is validation | implemented |
| I11 | [R10.5](#r105--underwater-worlds-sensing-slam-and-navigation) / [R10.4](#r104--underwater-robots-and-controllers) Marine | Done 2026-11-14: MuJoCo ROV plant (ported Fortress water forces; matches Gazebo), damping-based current feedforward (1 cm drift in 0.1 m/s), volumetric and current-aware AUV routes (unit-tested). 11-18: PyBullet ROV plant matches MuJoCo (surge 1.041 vs 1.030 m). 11-19: Tethys AUV on MuJoCo (cruise matches Gazebo, turn 0.86 vs 1.19 rad). AUV mission flown from the sensor estimate (11-19, 0.20 m from the goal). Tethys and the ROV run on all four backends. Saved-map relocalization passes 2/2 in a textured tank (11-19). 11-20: long AUV routes (20 m, curved 23 m, 43 m two-leg) in a new open-water bay; geometry-only relocalization (2/2, structure in view) | implemented |
| I12 | [R10.6](#r106--six--and-eight-legged-robots) / [R10.7](#r107--segmented-and-additional-robot-forms) Multi-leg and forms | Done 2026-11-14: depth/IMU flipper policy, bounded gait slip/support recovery (unit-tested). 2026-11-19: a 40 Nm flipper variant with a full-press climb crosses the 0.18 m step (2/2). Saved-map Nav2 goals for hexapod, octopod and centipede (11-19, MuJoCo). 11-20: push-recovery mission (octopod recovers from 60–100 N pushes, 4/4); tracked vehicle climbs a five-step flight (3/3) with a ledge guard and goes up and over a flight (2/2); angled step climbs at 15–45°. Other backends, per-model terrain missions remain | implemented |
| I13 | [R5.2](#r52--go2-quadruped-locomotion) / [R5.3](#r53--berkeley-humanoid-lite-balance-and-walking) / [R12](#r12--legged-policy-training) Legged | 2026-11-15: Go2W `wheeled_walk` policy strafes (MuJoCo 0.116, Isaac 0.271 m/s); BHL contract/export/warm start implemented, four runs not passed; 11-17: Go2 `get_up_roll` r2 (6000 iterations) stands 29/30 in training eval and gets up in a normal MuJoCo launch from side/back starts and 200–400 N knock-downs (60 N no longer topples it). Walk → get-up → walk switching measured. Go2 stairs not passed after 5 runs (r5 with a height scan steps at the riser; r6 training), G1, GPU training remain | partial |
| I14 | [R5.5](#r55--ackermann-rear-steer-and-anti-ackermann-cars) Cars | 2026-11-15: car course 13/13 on four backends (lookahead 1.25 × turning radius, forward escape when back-up is blocked); park-at-heading runtime and Drive-panel controls | implemented |
| I15 | [R3.6](#r36--external-robot-assets-in-normal-launch) Assets | 2026-11-18: Kinova Jaco 2 gets lab servos (continuous joints with a declared source range; out-of-range zeros spawn at home); MuJoCo screen passes, PyBullet joint 4 overshoots (display-only 65 → 64); 2026-11-18: KUKA iiwa and both Valkyrie forearms also get lab servos (display-only 69 → 66; R2 refused: placeholder hand limits); 2026-11-18: robot_assets ANYmal walks with a policy trained in a legged_gym template (MuJoCo 2/2 drive screens; display-only 66 → 65); 2026-11-17: UR10, UR5, YuMi controllable with declared lab servos (display-only 73 → 69); 2026-11-15: vendor policies for Booster T1 and Unitree G1/H1/H1_2, controllable siblings named, license/mesh audit (114/163 ok). 47 legged need training, 12 URDFs ship no control interface | partial |
| I16 | [R6.5](#r65--world-to-occupancy-generation) / [R6.6](#r66--external-world-library) / [R6.2](#r62--dynamic-and-sensor-disturbance-cases) Worlds | 2026-11-17: `<polyline>` geometry supported, polylines world installed (20/20 example scenes); upper floors covered by `--height` slicing. 2026-11-15: fixture review of 100 Gazebo examples, Fuel licences, 16 worlds with grids, actor collisions. Missions in new world classes are V9 | implemented |
| I17 | [R11](#r11--molar--probot-l-family) MOLAR | CELISCA route and transfer measured; transfer on all four backends. Unladen native and Open-RMF lift round trips pass. Loose cargo and Fleet GUI option implemented; 0.5 kg loaded boarding measured. Loaded lift ride/return, fleet peer, mounted sensors and repeats remain | implemented |
| I18 | [P4](#p4--mujoco-large-world-speed) MuJoCo speed | 2026-11-14: flex culling, exact tick pacing, oversleep payback, sensor thread (celisca_floor_1 0.81, furnished floor 0.99, hospital 0.43); large worlds remain flex-kinematics bound | partial |
| I19 | [R7](#r7--algorithm-breadth) Algorithms | 2026-11-14: Gazebo `sim_seed` (300/300 repeat), sensor-fault relay on all backends, 78 per-method ledger records (9 corrected, 8 noted mismatches) | implemented |
| I20 | [R9.1](#r91--provenance-licenses-and-clean-host-reproduction) Provenance | 2026-11-14: PX4 checkout + 25 submodules and all vendor sources scanned from their license files; 7 trees without license files listed for review | implemented |

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

Preserve these completed baselines and their scoped contracts. R1.3's setup
baseline passes at ecaaa133 (run 43155923238); November 14's takeover/tier
repair also passes published CI at `31db2d73` (run 43562150247). The following
parking batch needs its own run. Details and evidence are in the
[archived roadmap](docs/status/archive/ROADMAP-2026-11-10.md#r0--documentation-recovery).

| Task | Result | Keep in mind |
|---|---|---|
| R0.1, R0.2 Documentation recovery | Audited baseline and truthful docs | — |
| R1.1 Entry points and paths | Installed executables, package-share asset paths | No developer home paths |
| R1.2 Test tiers | Fast / physics / integration / mission tiers | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` locally |
| R1.3 CI | Published takeover revision `31db2d73` passes all tiers, 25-package build and registry; the newer parking batch needs its own run | [Takeover and published result](docs/status/takeover-2026-11-14.md) |
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
- **Done 2026-11-14:** every display-only profile shows a reason derived from
  its own source model (`display_reasons.py`, GUI model details; 77 profiles:
  51 floating legged without a policy, 19 URDFs without a control interface,
  3 floating without a base controller, 2 free-flyers, 2 without usable
  actuators). The native position servo accepts unnamed vendor actuators,
  enabling Menagerie Lite6 and Google Robot jogging (validation pending).
- **Done 2026-11-18 (implemented, experimental): Kinova Jaco 2.**
  `urdf_lab_servos` and the articulation contract accept a continuous joint
  as a position channel when its source `<limit>` declares a finite range
  (Kinova: ±2π; Fetch and PR2 declare none and stay refused). Joints whose
  range excludes 0 spawn at their clamped home on MuJoCo and PyBullet
  (`URDFArticulation.spawn_positions`); before this the Jaco started out of
  bounds and the contract refused every command. MuJoCo screen passes
  (0.028 rad, Stop and heartbeat hold); on PyBullet joint 4 overshoots to a
  0.082 rad steady error (cause not found).
  [Receipt](docs/status/evidence/r36-kinova-2026-11-18/README.md).
- **Done 2026-11-15 (implemented, experimental):**
  - **Booster T1:** walks with the vendor's own policy on the vendor's MuJoCo
    model (booster_gym `T1.pt`, `robot_lab_mujoco.booster_policy`).
  - **Unitree G1/H1/H1_2:** their vendor policies were already wired but were
    still listed as display-only; they are now counted as controllable.
  - **Walking screen:** in a screen through the operator input path, all four
    stayed up with no fault. T1 and G1 track 0.12 m/s within 0.02 m over
    6 s, H1 tracks forward poorly, and all four turn 1.2–1.4 rad of the
    1.5 rad requested.
  - **Display-only profiles:** 77 → 73. Display reasons now name a
    controllable sibling model in the same family (12 profiles).
  - **Asset audit:** `scripts/robot_asset_audit.py` checks license, meshes,
    textures and skins for all 163 profiles. 114 are ok; 48 robot_assets
    models ship no license, and one vendor copy lacks its OBJ materials
    ([evidence](docs/status/evidence/r36-vendor-controllers-2026-11-15/README.md)).
- **Remaining implementation:**
  - **47 legged profiles without a published vendor policy:** Menagerie
    Spot, ANYmal, Cassie, Apollo and others, plus NASA R2/Valkyrie. They
    need trained policies (R12).
  - **12 URDFs that ship no control interface** (19 → 15 on 2026-11-17: UR10, UR5 and YuMi got
    declared lab servos from their own limits via `robot_lab_utils/urdf_lab_servos.py`, screened on
    PyBullet and MuJoCo; the URDF Panda's entry was stale since R5.7.
    [Receipt](docs/status/evidence/r36-lab-servo-arms-2026-11-17/README.md).
    15 → 12 on 2026-11-18: the KUKA iiwa (mounted with a new articulation `fixed_base`, which makes
    the launch fix an unanchored URDF to the world) and both Valkyrie forearm stands pass the arm
    screen on MuJoCo and PyBullet.
    [Receipt](docs/status/evidence/r36-mounted-arms-2026-11-18/README.md)). Remaining: the eight
    R2 variants (placeholder hand limits, effort 1e7), Kinova (unlimited joints), Fetch and PR2
    (mobile bases with unlimited wheels/casters) and Ginger; each reason is shown in the GUI.
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
- **Done 2026-11-14 (implemented):** a `get_up` training task (starts from
  physics-settled collapsed poses, staged upright/height/pose rewards) and a
  stairs terrain with a legged_gym-style rise curriculum (2–18 cm) in
  `legged_training.py` / `legged_train.py` and the GUI Training page; the
  rough-terrain curriculum now really rewrites the heightfield.
- **Remaining implementation:**
  - train and screen the get-up and stairs policies (screens: 60 N collapse,
    `terrain_stairs`). The first Go2 `get_up` policy (1500 iterations, 18 M
    steps) stood in 1 of 10 trials from collapsed poses, as did the
    iteration-1050 checkpoint. It never recovers the recurring pose tilted
    past 90° ([evidence](docs/status/evidence/go2-get-up-2026-11-14/README.md)).
    Roll-over shaping (`get_up_roll`, 2026-11-17) stands 2 of 10; the same side-lying pose remains
    ([receipt](docs/status/evidence/go2-get-up-roll-2026-11-17/README.md)).
    **Done 2026-11-17 (get-up half of the done-when, MuJoCo):** training continued to 6000
    iterations (r2) stands 29 of 30 collapsed starts over three seeds (r1: 7 of 30). In a normal
    MuJoCo launch (`legged_policy:=<dir>`, `scripts/screen_get_up.py`) it gets up from its side
    (0.5 s) and back (1.4 s), and after 200 N and 400 N pushes that flip it (1.5–3.3 s). The 60 N push
    no longer topples it. The policy controller no longer safe-stops get-up policies on tilt.
    [Receipt](docs/status/evidence/go2-get-up-roll-r2-2026-11-17/README.md);
  - Done 2026-11-17: walk → get-up → walk switching (`legged_recovery_policy:=<dir>`): after
    200 N and 400 N pushes the vendor walking policy hands over to the get-up policy, is upright in
    2.7 s, resumes and walks 0.8–1.4 m on a 0.3 m/s command (MuJoCo);
  - get-ups on Isaac and Gazebo;
  - stairs (the other half of the done-when). **2026-11-18: not passed.** r2 stairs training
    (curriculum against the commanded distance) reached 10 cm rises. In a normal MuJoCo launch
    (`scripts/screen_stairs.py`) neither it nor the vendor policy climbs the 18 cm
    `terrain_stairs` or the new 10 cm `terrain_stairs_low`. Every policy stops at the first
    step, and the trained gait keeps one front foot raised. r3 (resumed from the evenly trotting
    `go2_walk_r3`; curriculum level 0.7, 13 cm) walks straight to the first 10 cm step with
    `screen_stairs.py --hold-line`, puts its front feet on it and freezes. r4 (30 % of
    episodes start 0.3 m before a riser) reached only level 0.4 and stops 0.45 m short of
    the first step. All four policies are blind (no terrain observation). r5 (11-19) adds a
    height-scan observation (MuJoCo ray-cast in deployment): it walks 5.97 m on flat ground
    and keeps stepping at the 10 cm riser (base rises 7 cm) but does not climb. r6 (r5 plus
    30 % riser starts, 3000 iterations) also fails: its curriculum stalls at level 0.5 (10 cm)
    and it rises 8.5 cm at the 10 cm riser with 0 steps; flat 5.72 m in 15 s. Stairs is
    time-boxed again; next would be course-like 1.1 m treads or GPU-scale training.
    [Receipt](docs/status/evidence/go2-stairs-r2-2026-11-18/README.md).
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
  then train `walk` and `turn_in_place`. Not done on 2026-11-14: BHL has no
  rl_sar policy contract and deploys through `humanoid_policy_controller`
  (commands-first observation order), so the trainer needs a BHL contract
  template plus an export path to that controller first.
- **How to do it:**
  1. Read the source policy contract in `src/robot_lab_robots/berkeley_humanoid_lite/` and the
     humanoid controller in `robot_lab_adapter/`.
  2. Add a BHL robot spec to `legged_training.py`, built from the bridge's own MJCF import.
  3. Train and export, then check with the existing GUI drive path
     (`/key_vel` → twist_mux → policy) and `analyze_turn_gait.py`.
- **2026-11-15 (implemented, experimental; not passed):** the BHL trainer contract,
  BHL-order export with `bhl_policy_dir:=`, the upstream `humanoid_walk` rewards, a warm start
  from the vendored actor and `scripts/bhl_policy_screen.py`. Four CPU runs: from scratch, the
  policy stands but never steps (r1 and r2). Warm-started from the vendored actor, it degrades at
  the first actor updates (r3 and r4); the vendor policy lasts only 4–6 s per episode under
  random commands in the training environment.
  [Receipt](docs/status/evidence/r53-bhl-training-2026-11-15/README.md).
- **2026-11-20 (not passed):** a fixed-rate fine-tune (`--desired-kl 0`, lr 3e-5, std 0.05; r5) still
  collapses at the first actor update (KL ~1000). Corrected reading: the vendor policy turns the right
  way in the training environment (+1.0 rad in 12 s at +0.3) and stalls at −0.3 as in deployment, so the
  environment reproduces the problem ([receipt](docs/status/evidence/r53-bhl-finetune-2026-11-20/README.md)).
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
  With the forward/reverse heading shuffle plus straight depth trim
  (`car_alignment.py`) it passes 10 of 12 cells (round 62; Gazebo rear-steer
  and Isaac anti-Ackermann abort in the slalom).
- **Done 2026-11-14 (implemented, unit-tested):** the shuffle and trim as a
  reusable runtime, `car_parking.py`, started with car navigation
  (`/car_parking/command` park/cancel, estimate-based report).
- **GUI continuation (2026-11-14, Codex):** normal Drive-panel target fields,
  estimated-pose copy, explicit Park/Cancel, feedback errors and manual/Stop
  cancellation. Runtime now binds requests to the current instance, waits for
  active Nav2, enforces a whole-operation budget and operator lease, requires
  fresh transforms/costmap, checks the swept source footprint and reports
  failure outside final tolerances. Late accepted goals are canceled too.
  [Guide](docs/tutorials/car-parking.md) ·
  [Source/check scope](docs/status/car-parking-gui-2026-11-14.md).
- **Done 2026-11-15 (implemented, experimental):** the two slalom aborts.
  RPP's 0.8 m lookahead cut the S-curve round cone 0 and left 5–6 cm of
  clearance. The car lookahead is now 0.6 m (1.25 × the turning radius), and
  the recovery drives forward on its heading when the back-up is blocked.
  All 12 cells pass, plus a Gazebo anti-Ackermann repeat (13/13), with
  0.08–0.14 m slalom clearance and the bay within 0.10 m and 4.5°
  ([evidence](docs/status/evidence/r55-car-course-2026-11-15/README.md)).
- **Remaining implementation:** none known. The forward escape is
  implemented, but it has not yet recovered a car in a run.
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
  - November 13: explicit measured free-box selection, source-width/padding
    grasp candidates and GUI approach/grasp/lift planning through existing
    collision-aware Panda IK/OMPL; acknowledged canonical-world scene updates.
    Fourteen focused checks, four package builds and an installed widget
    protocol smoke pass; the new full physical object mission is pending.
    [Scope and guide](docs/status/arm-objects-2026-11-13.md).
  - November 14: measured fixed-box support selection and explicit placement
    above/lower plans, computed from the acknowledged grasp transform and
    projected object footprint. Source/plant and actual scene ACK gates remain.
    [Implementation and checks](docs/status/support-placement-2026-11-14.md).

  - November 17 (Claude): **URDF-arm pick and place on PyBullet and MuJoCo.**
    The robot_assets Franka Panda is controlled through the shared URDF
    articulation contract on PyBullet, MuJoCo and Isaac (screens pass on all
    three). A backend-neutral planner (`robot_lab_mujoco/arm_pick_place.py`,
    private MuJoCo kinematics/collision model) and executor
    (`urdf_pick_place.py`) pick a loose cube and place it on a table, judged
    by simulator truth. The GUI arm page has a **Pick / place** section. One
    GUI run passes on PyBullet (22.2 s) and one on MuJoCo (20.5 s): **the
    done-when is met for this arm.** Fixed-base URDFs are welded on
    MuJoCo/Isaac, the finger coupling is mechanical, and loose world objects
    publish on `/simulation/free_bodies`.
    [Guide](docs/tutorials/urdf-arm-pick-place.md) ·
    [receipt](docs/status/evidence/r57-urdf-arm-pick-place-2026-11-17/README.md).

  [Native arm guide](docs/tutorials/native-arm-planning.md).
- **Side strategies (2026-11-14, Codex):** Top/Side/Top + side geometry candidates,
  physical-width projection, explicit open/detached retreat plans and strategy
  invalidation in the existing GUI. All paths still require existing IK/OMPL
  and separate execution. [Takeover receipt](docs/status/takeover-2026-11-14.md).
- **Remaining implementation:**
  - Done 2026-11-18 (implemented, experimental): upright cylinders. The
    planner grips a cylinder across its diameter (`Box.shape`); URDF Panda
    pick and place of a 50 × 90 mm can passes on MuJoCo (28.8 s), PyBullet
    (24.0 s) and Gazebo (25.2 s) in the new `arm_pick_place_can`.
    [Receipt](docs/status/evidence/r57-cylinder-pick-place-2026-11-18/README.md);
  - 2026-11-20 (implemented, experimental): lying cylinders use the measured
    axis to choose a diameter grasp and check the whole oriented footprint
    on the destination table. The fixed-arm GUI fixture has an upright and
    side-lying variant. Isaac has a centred free-cylinder adapter. Private
    Panda planning passes for both orientations. Live side-lying missions
    remain pending.
  - compound/mesh geometry, perception pose sources for
    fixed arms and additional gripper-specific pipelines;
  - coupled mechanisms beyond gear-coupled fingers (mimic chains and
    four-bar joints);
  - Done 2026-11-17: Isaac and Gazebo pick and place for the URDF Panda.
    On Isaac, loose world objects are dynamic bodies that publish on
    `/simulation/free_bodies` (20.2 s, place error 0.0 mm;
    [receipt](docs/status/evidence/r57-isaac-pick-place-2026-11-17/README.md)).
    On Gazebo, `urdf_gazebo_articulation.py` drives the display-hold
    controllers and `gazebo_free_bodies.py` publishes truth (27.8 s and
    30.6 s, place error 0.0 mm). UR5, YuMi and a raised UR10 pass the joint
    screen there; the UR10 lying on the floor does not
    ([receipt](docs/status/evidence/r57-gazebo-urdf-arms-2026-11-17/README.md)).
- **Automatic box tasks (2026-11-14, Codex):** normal Arm page explicitly
  starts 1–5 pick/place cycles through the existing IK/OMPL/actions, measured
  opposing contacts, acknowledged attachments and sustained lift/release
  observations. Manual commands, selection/source changes and Stop interrupt.
  Source-finger bounds add upper pinches while preserving normal link padding.
  One normal GUI Panda/MuJoCo/`mobile_manipulation_lab` run completed two
  cycles and all 38 stages, with sustained unsupported grasp and supported
  release in each cycle. This does not qualify other objects or backends.
  [Guide](docs/tutorials/panda-object-tasks.md) ·
  [Measured receipts and retained failures](docs/status/evidence/manipulation-2026-11-14/README.md).
- **How to do it:**
  1. Start from the Panda MoveIt workflow (`docs/tutorials/panda_arm.md`) and
     the attached-object code from the October 9 checkpoint
     (`docs/status/implementation-2026-10-09.md`).
  2. Extend `grasp_target_controls.py` and `native_object_catalog.py`; preserve
     explicit selection, source binding, measured contacts and scene ACK gates.
     Object poses come from labelled simulator truth until perception exists.
  3. For other backends, reuse the backend's joint-trajectory interface
     (Gazebo ros2_control, PyBullet/Isaac spawner joint commands) behind the same action.
- **Done when:** a GUI pick → place of a named object, with measured object
  pose, on MuJoCo plus one other backend.
- **Validation later:** the remaining eleven arms, repeats, interruption/reset (V6).

### R5.8 — Robot hands and grippers

- **State:** implemented, experimental (done-when met 2026-11-19) · **Priority:** I8 · **Depends on:** R3.6, R5.7
- **Goal:** grippers and dexterous hands grasp and release objects with
  measured contact forces.
- **Have:** the Panda gripper cube grasp; DEX-EE and three coupled-tendon
  hands with closure/spread, finger selection and contact feedback.
  [Hand guide](docs/tutorials/native-hand-controls.md).
- **Force feedback continuation (2026-11-13, Codex):** GUI-requested existing-grasp force regulation adjusts source position targets, retains original PID/transmission/force limits and requested effort fraction, unloads initial over-force and interrupts on contact loss, new/worsening external touch, overload, Stop/loss/deadline. One actual pinned Robotiq component fixture reduced about 12 N to 5.25 N per side for a 5 N request; this is not a GUI/hand-family mission. [Guide](docs/tutorials/native-hand-controls.md#regulate-an-established-grasp) · [receipt](docs/status/hand-force-2026-11-13.md).
- **Remaining implementation:**
  - raw torque and tendon hand adapters: **done 2026-11-20** for the installed source hands
    (Wonik Allegro position servos; Shadow Hand coupled J1 + J2 tendons);
  - arm transport for fixed-mount hand families and additional object workflows;
  - hands on other backends.
- **Source-axis object workflow (2026-11-14, Codex):** Hand page now requests
  bounded UMI vertical lift, measured hold and return before release. Custody
  uses actual finger-relative translation/rotation; supported release requires
  actual support and low velocity for one simulated second. Original source
  motors/rates/preload remain; Stop/watchdog/contact/clock/obstacle guards
  interrupt. Source direction, distal-pad fixture placement, passive-load effort
  guards and bounded mount gravity support are corrected. One pinned UMI
  component lifts 49.0 mm, returns to measured support with 1.52 mm seating
  and releases; 38 hand checks pass. Normal GUI and other family/backend
  missions remain. [Procedure](docs/tutorials/native-hand-controls.md#source-axis-object-transport).
- **GUI hand missions (2026-11-17, Claude):** on MuJoCo, the Robotiq
  2F-85, the Robotiq 2F-85 v4 and UMI each pass grasp → lift 5 cm → 3 s
  hold → return → supported release, judged by measured states.
  - Grasp forces: 11.9, 13.0 and 0.7 N per side.
  - Height held within 0.1 mm.
  - Fixed-mount hands lift relative to a new lab support stage.
  - A pedestal/finger collision that kept the Robotiq from ever closing is
    fixed, and the grasp state is debounced.
  - **DEX-EE does not hold** (the cube tilts out of the fingertip tripod);
    it needs grasp synthesis.
  [Receipt](docs/status/evidence/r58-hands-2026-11-17/README.md).
- **DEX-EE grasp synthesis (2026-11-19, Claude):**
  - `hand_grasp_synthesis.py` chooses a three-finger tripod for the box:
    the thumb against F1+F2 on opposite faces at its mid-plane.
  - Per-finger joint targets come from the executed kinematics, and the
    box is placed for the best force closure.
  - The GUI grasps with the new **Synthesized tripod** pose. Normal GUI
    grasp → lift 5 cm → 3 s hold → return → supported release passes, 2/2
    (about 4–5 N per side).
  - Fixes on the way:
    - the support-stage seating jumped instead of ramping;
    - a returned box held firmly by the fingers lost its stage contact;
    - the pose selector stayed on Curl;
    - the Training tab's 1 s rescan stall tripped the hand heartbeat.
  [Receipt](docs/status/evidence/r58-dexee-synthesis-2026-11-19/README.md).
- **Wonik Allegro hand (2026-11-20, Claude):**
  - New native hand `menagerie_wonik_allegro` (Menagerie right hand, BSD-2): its sixteen
    authored joint position servos, Thumb/Index/Middle/Ring groups, Open, Curl and
    **Synthesized tripod**.
  - The tripod synthesis now serves reviewed hands. Allegro adds capsule tips, a link
    clearance check and press targets on one joint line (d = K⁻¹Jᵀn), so its weak kp 1 servos
    press 2 N along the face normal.
  - The fixture keeps the stage vertical for a hand frame that is not z-up. The Allegro scene
    adds MuJoCo's no-slip pass (soft-friction creep sank the box 7 mm).
  - Normal GUI grasp → lift 5 cm → 3 s hold → return → supported release passes 2/2
    (2.1–2.6 N, 48.5–48.7 mm). The DEX-EE GUI regression passes. Five in-process failures
    are kept.
  [Receipt](docs/status/evidence/r58-allegro-2026-11-20/README.md).
- **Shadow Hand (2026-11-20, Claude):**
  - New native hand `menagerie_shadow_hand` (Menagerie E3M5 right, Apache-2.0): its twenty
    authored servos, with each finger's J0 tendon driving J1 + J2.
  - The synthesis handles the underactuated finger. The tip force must lie along the middle
    phalanx for the tendon joints to balance, so the IK keeps the needed tangential share within
    0.6 of the normal force (friction 1.0). Each press also carries a third of the box weight.
  - Normal GUI grasp → lift → hold → return → supported release passes 2/2 (45.5–46.3 mm). The
    Allegro regression passes.
  [Receipt](docs/status/evidence/r58-shadow-hand-2026-11-20/README.md).
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
- **Object custody extension (2026-11-13, Codex):** existing native Stretch sequencer now accepts explicit grasp/release checkpoints from the GUI. Uses actual free-object contacts, sustained opposing-finger forces, relative translational slip and environmental support; navigation after a grasp checkpoint monitors custody and stops on loss. [Implementation and limits](docs/status/mobile-custody-2026-11-13.md). This is an implemented candidate, not a measured pick/place mission.
- **Payload footprint extension (2026-11-13, Codex):** measured collision-bound union includes the last confirmed payload and its fixed descendants in both costmap footprint topics. Transport waits for matching, fresh footprint publication; stale/invalid payload bounds stop native Drive and cancel active task navigation. This is producer-side implementation; costmap-consumption and physical transport acceptance remain pending.
- **Local object pipeline (2026-11-13, Codex):** normal-GUI pick/place recipe buttons, object-relative/world reaches, coupled source-control IK, bounded private planning/collision worker, contact-gated gripper operations and retraction, retained provisional payload bounds and fresh shape receipts from both Nav2 costmaps. Installed `mobile_manipulation_lab` has two benches, a free 35 g cube and matching occupancy. [Guide](docs/tutorials/native-object-tasks.md). Forty-two focused checks and source FK/scene-attachment/widget checks pass; physical mission acceptance remains partial.
- **Configuration detours (2026-11-13, Codex):** bounded seeded bidirectional search replaces straight-edge-only rejection for reviewed Stretch lift/telescope/wrist channels. Actual source motor-slew sampling, held gripper/base/head, payload collision transform and existing worker/executor limits remain. Fifteen focused checks include a real MuJoCo geometry detour with independent dense contact inspection; no loaded source-robot mission is claimed. [Receipt](docs/status/stretch-search-2026-11-13.md).
- **Orientation reaches (2026-11-14, Codex):** GUI task steps can keep the
  measured first-pad orientation or request a world RPY endpoint. Private
  source FK solves point/rotation residuals; keep mode fixes wrist controls
  and checks the planned/measured orientation envelope. Explicit-pose goals
  also check measured final orientation. [Scope](docs/status/reach-orientation-2026-11-14.md).
- **Alternate IK configurations (2026-11-14, Codex):** measured/midpoint/seeded
  source-control IK candidates share one bounded multi-goal search; duplicate
  and colliding endpoints are excluded. A real synthetic MuJoCo wrist fixture
  selects another configuration when its measured-seed goal collides, with
  independent dense contact inspection. Optional shortening no longer discards
  a found route on inspection-budget expiry. Forty-three focused checks and
  three package builds pass; full loaded source missions remain unqualified.
  [Receipt](docs/status/ik-branches-2026-11-14.md).
- **Directional approaches (2026-11-14, Codex):** GUI pick/place recipes can
  approach from above or either horizontal world axis, with a 3–25 cm distance.
  Place retreat uses the same direction; source-pad orientation stays explicit
  and independent. Editing only builds steps. [Receipt](docs/status/takeover-2026-11-14.md).
- **Have (2026-11-20):** TIAGo manual **Plan pose → Execute plan** in MuJoCo.
  The GUI copies its measured source grasp frame, edits base-frame XYZ and
  optionally sets final orientation. The private planner finds bounded torso/arm
  IK endpoints and checks their joint routes. One 20 mm GUI reach settles
  within 0.15 mm on `nav_empty`.
  [Receipt](docs/status/evidence/tiago-cartesian-2026-11-20/README.md).
- **Remaining implementation:**
  - Done 2026-11-17: fetch with Nav2 transit legs and AMCL in Nav mode;
    the 0.6 m docking moves stay truth-steered (AMCL error 0.07–0.13 m).
    MuJoCo 2/2 (265 s, 262 s), PyBullet 1/1 (117.5 s).
    [Receipt](docs/status/evidence/r59-tiago-nav2-fetch-2026-11-17/README.md);
  - 2026-11-20: the fetch waits for the owned Nav2 action's terminal result
    and a measured base stop before docking. Stop during action admission
    drains a late acceptance. Runtime requalification remains pending.
  - fetch on Isaac: drive and interlock work, but the TIAGo grasp fails
    (2026-11-17: the block tips at the pre-grasp, or the descent stalls).
    [Receipt](docs/status/evidence/r57-isaac-pick-place-2026-11-17/README.md).
    2026-11-18: true link poses showed a sideways approach at block height
    (the high vias were out of reach). The planner now keeps 15 mm from the
    object on transit legs and always comes in over the top. The block no
    longer tips, but while squeezing, the hand drifts 1.8 cm along the
    closing axis.
    [Receipt](docs/status/evidence/r59-isaac-tiago-approach-2026-11-18/README.md).
    2026-11-20: opt-in compliant PhysX jaw coupling and paired commands are
    implemented. Isaac 6.0.1 schema and source-limit checks pass. An actual
    close, lift, carry and release run remains pending.
  - Done 2026-11-18 (implemented, experimental): **the fetch's grasp
    planned from TIAGo's head camera** (`"pose_source": "rgbd"`; GUI
    **Object pose → RGB-D camera**). A look phase tilts the head; three
    agreeing `rgbd_boxes.estimate_box` fits of the world's declared box
    give the object pose; the place uses the grasp-time offset carried by
    the measured arm. Truth still picks the table and judges. Passes on
    MuJoCo (estimate error 1.5 mm, 167.7 s) and PyBullet (5.1 mm, 142.5 s,
    after its camera mirror learned to draw loose objects and SDF colours).
    [Receipt](docs/status/evidence/r59-tiago-rgbd-fetch-2026-11-18/README.md);
  - general recognition/pose for unconfigured, non-box or occluded objects,
    source-specific grasp strategies and further
    source-specific object pipelines;
  - generalize the existing native interlock and measured footprint-consumption gate to other mobile-manipulator adapters.
    2026-11-20: TIAGo fetch requires both costmaps to echo the loaded shape
    before Nav2 dispatch and retains the enlarged footprint through docking
    or Stop until supported release. The outline projects source collision
    bounds at measured joint positions and the measured payload pose.
    Live end-to-end loaded receipt repetition remains open.
  - Broaden TIAGo task planning, grasp/custody, dynamic footprint and map
    workflows beyond the measured fetch variants. Add other-backend pose/joint
    planners; its separate Fortress port remains unavailable. Further
    Fetch/PR2 sources are optional.
- **TIAGo fetch (2026-11-17, Claude): done-when met.** One GUI request
  drives TIAGo to `pickup_table`, picks a 150 mm block, carries it about
  2.3 m and places it on `drop_table` (`mobile_fetch_lab`).
  - Each arm phase is planned on arrival from the measured base pose;
    stages are judged by simulator truth.
  - The receipt has a 2 Hz base/object/finger stability trace.
  - Script and GUI runs pass on MuJoCo (108.9 s GUI, 5.5 mm carry drift)
    and PyBullet (104.6 s GUI, 17.1 mm).
  - Base and arm never move together: the node and the articulation
    contract both enforce it, and an outside probe confirms it (base moved
    0.2 mm under drive commands during arm motion; an arm target during the
    carry was rejected).
  - New articulation `grip_joints` keep a bounded squeeze across Stop and
    while the base drives.
  [Guide](docs/tutorials/urdf-arm-pick-place.md#fetch-with-tiago-mobile-manipulation) ·
  [receipt](docs/status/evidence/r59-tiago-fetch-2026-11-17/README.md).
- **TIAGo joint routes (2026-11-14, Codex):** normal GUI **Plan targets /
  Plan Home → Execute plan**, private compiled collision snapshots, existing
  bounded RRT-Connect, source-rate servo execution, scene/start/plant/lease
  binding and live contact/tracking/Stop guards implemented for MuJoCo.
  A normal GUI head/torso/arm/finger route completes; another run interrupts
  its second route with actual GUI Stop. A separate fixture demonstrates a
  detour with independent dense contact inspection. One hundred final checks
  pass and three packages build. This is joint-target planning, not Cartesian
  IK, object transport, all-map or other-backend qualification.
  [Guide](docs/tutorials/tiago.md#plan-joint-targets-in-mujoco) ·
  [receipt](docs/status/evidence/tiago-joint-planning-2026-11-14/README.md).
- **Second source (2026-11-14, Codex):** complete official Apache-2.0 PAL
  TIAGo PMB2/arm/pal-gripper source installed with five immutable dependencies,
  original URDF mechanics and twelve independent bounded position channels.
  Existing normal GUI Drive/Mobile arm selection, grouping and command autofill
  work. Head/torso/arm/finger targets, forward and Stop are measured in
  MuJoCo (0.297 m), PyBullet (0.449 m) and Isaac (0.377 m). Isaac ports native
  gravity/root-force layout, source caster contact and wheel-loss units;
  head cameras follow moving carriers, including fixed-link fusion handling.
  Source degenerate inertias/import regularization and uncalibrated velocity
  tracking are declared. Display adapter is separate from completed object or
  SLAM/navigation support. [Guide](docs/tutorials/tiago.md) and
  [receipt](docs/status/evidence/tiago-2026-11-14/README.md).
- **Camera-marker targets (2026-11-14, Codex):** RGB/CameraInfo/image-time TF
  feed bounded ArUco/IPPE object poses to the existing source/scene/plant-bound
  task builder. GUI recipes explicitly select simulator geometry or camera
  fiducials for unheld reaches; missing/stale estimates never fall back to
  truth. Actual rendered marker fixture error is 2.523 mm; this is separate
  from a full robot grasp mission. [Receipt](docs/status/camera-object-targets-2026-11-14.md).
- **Unmarked RGB-D box targets (2026-11-14, Codex):** known-size upright
  color boxes now have aligned depth/calibration/image-time TF estimates,
  separate method caches, source/scene/plant/geometry gates and explicit GUI
  recipe selection. The installed unmarked fixture shares the existing map
  family and autofills a native Stretch command. Actual observer-camera
  MuJoCo render error is 0.494 mm; ROS/Tk/source checks pass. Native-camera
  transport is also observed: four native Stretch frames detect the
  cube after head-only source commands. The startup base interlock remains.
  Full pick/transport missions and general object recognition remain separate.
  [Guide](docs/tutorials/rgbd-object-targets.md) ·
  [Receipt](docs/status/evidence/unmarked-rgbd-2026-11-14/README.md).
- **How to do it:**
  1. Extend the existing `native_mobile_tasks.py` sequencer and
     `mobile_task_controls.py` task builder. Preserve independent Nav2 outcomes,
     source/scene binding, measured custody and costmap receipts; do not create
     a second conflicting base/arm owner.
  2. Reuse R11's articulation contract (`robot_lab_utils/urdf_articulation`)
     for the base inhibit.
  3. Add a "Fetch object" panel: choose object and destination, Start, Stop.
  4. For a second robot, check maintained ROS 2 descriptions (Fetch,
     TIAGo by PAL Robotics), with licenses (see R14.1). PAL's
     [official Humble dependency manifest](https://github.com/pal-robotics/tiago_tutorials/blob/humble-devel/tiago_public.repos)
     and [Apache-2.0 description](https://github.com/pal-robotics/tiago_robot/tree/humble-devel)
     provide an upstream starting point; pin exact revisions before provisioning.
     The existing robot-assets Fetch remains Display-only: its derivative
     `base_link` inertia has zero X/Y diagonals and wheel collisions are
     spheres. Do not promote it by copying a drive profile or inventing
     inertias; use reviewed complete upstream source as a separate variant.
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

- **State:** implemented (2026-11-13), experimental · **Priority:** I5 · **Owner:** claude · **Depends on:** R5.10
- **Goal:** the drone avoids moving obstacles and handles sensor loss in flight.
  Drones also fly in a second simulator.
- **Have:**
  - `robot_lab_utils/aerial_tracking.py`: depth clusters → constant-velocity
    Kalman tracks; coasting for 4 s out of view; predicted swept columns; time
    conflicts along the route; a swept memory of where people walked; a 2 m
    standing-height prior.
  - `aerial_obstacles.py`: the floor is estimated from the depth itself; static
    world filter; people's ghost trails are dropped from the RTAB-Map layer.
  - `px4_planning.py`:
    - automatic replanning after holds;
    - walkways: hold at the edge, look ±75°, time the crossing, then cross
      above the swept columns, capped where the camera still sees ground;
    - evasive climb;
    - sensor loss: 1 s grace, hold, land after 10 s, replan on recovery.
  - `px4_ros_controller.py`: takeoff waits for a steady resting heading
    (≤ 0.25°/s) and height; otherwise the runtime restarts PX4's `ekf2` (at
    most 3 times).
  - Test world `aerial_moving` with two walking actors.
  - `scripts/qualify_drone_moving.py`.
  - Second backend: `px4_mujoco_plant.py` (PX4 SITL with a MuJoCo quadrotor
    plant over the simulator interface, HIL lockstep, Iris parameters) as
    robot `px4_iris` on MuJoCo.
  - Tutorial: [mapped-drone-planning](docs/tutorials/mapped-drone-planning.md#moving-obstacles-and-sensor-loss).
  - Measured: runs 32 and 34 passed: goal reached; walkers no closer than
    1.08 m; walkway holds and replans; camera loss → hold 1.7 s → land 12 s.
    Run 33 failed before the mission on an estimator fault, which the takeoff
    gate now refuses. PX4 with the MuJoCo plant took off, flew a
    checked 5 m route and landed.
    [Evidence](docs/status/evidence/drone-moving-2026-11-13/README.md).
- **Remaining implementation:** none for this item.
- **Done when:** met: moving obstacles crossed the route, and the drone held,
  replanned or overflew without contact.
- **Validation later (V5):**
  - repeats and statistics;
  - people who react or turn; vehicles; crowds;
  - other worlds;
  - moving obstacles in the MuJoCo plant;
  - the SITL height-estimate noise (0.05–1.7 m at rest on this host).

### R6.1 — Map geometry, alignment and resets

- **State:** partial · **Priority:** validation V2
- **Goal:** runtime coordinates match the maps; resets restore the full state.
- **Have:** real occupancy exports for 17 grids; named reset/contact screens.
- **Done when:** geometry, spawn/goal clearance and resets are verified per
  world and backend, with seeds recorded.

### R6.2 — Dynamic and sensor-disturbance cases

- **State:** implemented, experimental (relay unit-tested) · **Priority:** I16 · **Depends on:** R6.1, R4.2
- **Goal:** seeded moving obstacles and sensor faults: noise, bias, drift,
  delay, dropout, occlusion and outliers.
- **Have:** `src/robot_lab_maps/tools/r6_dynamic_sensor_disturbance.py`
  (framework and tests).
- **Done 2026-11-14 (implemented, unit-tested):** `sensor_fault_relay.py`, a
  seeded relay for LaserScan/Imu/Odometry/Image with noise, bias, drift,
  delay, dropout, occlusion and outliers in time windows, digest traces, and
  truth topics refused; Gazebo launches take `sensor_faults:=<scenario>`
  (the bridge publishes `<topic>_clean`). [Guide](docs/tutorials/sensor-faults.md).
- **Done 2026-11-14 (later):** MuJoCo, PyBullet and Isaac take
  `sensor_faults:=` too; their own sensor publishers are remapped onto the
  relay inputs. The relay script was never installed, so `sensor_faults:=`
  failed at launch until now. Smoke runs on Gazebo and MuJoCo showed the
  +0.5 m scan bias on `/scan`, with `/scan_clean` from the simulator
  ([evidence](docs/status/evidence/r7-simulator-seed-2026-11-14/README.md)).
- **Done 2026-11-15 (implemented, experimental):** actors with real
  collision. `actor_collisions:=true` adds a moving collider per scripted
  actor (`robot_lab::ActorColliders`, pose computed as Gazebo animates the
  skin). A walking person pushed a free box 3.41 m; without colliders it
  passed through
  ([evidence](docs/status/evidence/r62-actor-colliders-2026-11-15/README.md)).
- **Remaining implementation:** none known. A robot-contact mission is validation (V9).
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

- **State:** implemented · **Priority:** I16 · **Depends on:** R6.1, R3.4
- **Goal:** generate a correct 2D grid from any selected 3D world in the GUI.
- **Have:**
  - Worlds tab Generate/Stop with editable seeds.
  - `src/robot_lab_maps/tools/generate_occupancy_map.py`,
    `scripts/review_occupancy_maps.py` and `scripts/refresh_installed_occupancy_maps.py`.
  - Rotated triangle exports.
- **Done 2026-11-14:** free-region labelling from the complete collision mask
  (`--regions seed|all`, `<world>.regions.json`; on `nav_maze` the seed region
  matched the plugin flood fill exactly and `all` added the 235 m² enclosed
  interior), and an explicit **Register reviewed grid…** step
  (`map_registration.py`) that refuses changed files, a changed world or a
  spawn outside free space.
- **Remaining implementation:** none for the two items; worlds with unresolved
  external models: **done 2026-11-20** for `small_house` (AWS furniture resolved from the installed dataset; its grid agrees
  with `dataset_small_house`, occupied IoU 0.846) ([receipt](docs/status/evidence/r65-small-house-occupancy-2026-11-20/README.md)).
- **Done when:** primitive, rotated and furnished mesh worlds produce grids
  that agree with the geometry, and Nav2 uses them.

### R6.6 — External world library

- **State:** implemented (2026-11-17), experimental; missions per world class are V9 · **Priority:** I16 · **Depends on:** R3.6, R6.5
- **Goal:** the requested external worlds (mlherd, Gazebo examples, licensed
  Fuel worlds) are installed with license and status.
- **Have:**
  - 14 dataset worlds and three Gazebo examples installed; all 100 Fortress
    example SDFs downloaded.
  - Hospital fixtures repaired in derived variants; `scripts/integrate_external_worlds.py`.
- **Done 2026-11-15 (implemented, experimental):**
  - **Fixture review:** `scripts/review_gazebo_examples.py` classified all
    100 example SDFs from their content, including the Fuel models they
    include (33 robot, 23 sensor, 16 plugin, 4 actor demos, 23 scenes, one
    empty file). 95 of 99 load headless.
  - **Fuel licences:** `scripts/fuel_models.py` read the licence of every
    Fuel model used (80 models, all CC0 or CC-BY 4.0).
  - **Installed:** the scenes that load and are licence-clear. 19 of 20 are
    now installed with occupancy grids, including the Fuel `heightmap`,
    `import_mesh` and `lightmap` (34 installed worlds).
    `sdf_world.fuel_cache_path` resolves Fuel URIs from Gazebo's cache
    ([evidence](docs/status/evidence/r66-fixture-review-2026-11-15/README.md)).
- **Polylines (2026-11-17, Claude):** `sdf_world` now reads `<polyline>`
  geometry. Each outline is triangulated (ear clipping; simple polygons
  without holes) and extruded into convex triangle prisms, written as
  cached binary STL (`robot_lab_utils/sdf_polyline.py`). Every backend and
  the occupancy generator load them as ordinary meshes, and MuJoCo's convex
  hulls stay exact. Unit-tested. Installing the Gazebo `polylines` example
  world and running a mission on it is the next step.
- **Remaining implementation:**
  - upper floors: the occupancy generator already slices any elevation
    (`--height`), and the CELISCA floors are separate worlds. No installed
    world has a second storey in one file, so nothing is open until one is
    added;
  - the robot/sensor demos stay fixtures by design.
- **Polylines world installed (2026-11-17):** `gazebo_example_polylines` is installed with 16 prism
  shapes, MJCF and an occupancy grid (now 20 of 20 example scenes). Its polyline models are
  authored 5 m up and fall in the Gazebo demo. The static snapshot keeps the authored poses, so the
  0.3 m slice is free floor, as for the other falling-object demos. Robot missions in the new world
  classes are V9.
- **Done when:** every requested source shows its status, and each new world
  class has one robot mission and a generated occupancy file.

### R6.7 — Terrain generation GUI and four-backend conversion

- **State:** implemented (2026-11-13), experimental · **Priority:** I4 · **Owner:** claude (sole active agent, 2026-11-13) · **Depends on:** R6.3, R6.6
- **Goal:** a GUI page that generates terrain from a provider or
  procedurally, saves it on the SSD with attribution, and converts it for all
  four simulators.
- **Have:**
  - Shared heightfield conversion (`robot_lab_utils/heightfield.py`, `sdf_world.py`), measured Gazebo conventions.
  - `robot_lab_utils/terrain_generator.py`: procedural (seeded spectral) or imported elevation raster
    (GeoTIFF/.npy/PNG with required attribution and license) → SDF heightmap world, MuJoCo hfield
    scene, slope/step occupancy grid, preview and a manifest (inputs, provider, datum, metres/pixel,
    elevation range, file hashes) under `$ROBOT_LAB_RUNTIME_ROOT/terrain/<id>/`; byte-identical
    `regenerate()`; registry merged into the installed maps.
  - **Worlds & mapping → Terrain** page (`robot_lab_gui/terrain_controls.py`): generate, preview,
    use in Launch, Preview 3D, reload settings, check regeneration, remove.
  - Measured: support height through the real launch on Gazebo, MuJoCo, PyBullet and **Isaac**,
    24 points on two terrains within 2.1 mm ([evidence](docs/status/evidence/terrain-generation-2026-11-13/README.md)).
  - Gazebo Fortress heightmap visuals crash its renderer on the Jetson when a camera draws them;
    generated worlds, `terrain_heightfield` and `gait_low_relief` now draw a mesh visual of the same grid.
- **Remaining implementation:** none. Optional: a token-gated web tile provider (keep tokens out of the repo and logs).
- **Done when:** met — generated terrain regenerates identically from its saved inputs and gives
  matching support heights on four backends.
- **Validation later:** class-specific driving, walking and flight on generated terrain (V9);
  imported-raster projections and vertical datums.

### R6.8 — Real-world area import from a globe

- **State:** implemented (2026-11-13), experimental · **Priority:** I4b · **Owner:** claude · **Depends on:** R6.7
- **Goal:** open a globe, select an area, and import it as a 3D world for all four simulators with
  elevation, buildings, vegetation, roads and other features; for a city, optionally people and traffic.
- **Survey first (user principle):** gazebo_terrain_generator (BSD-3, needs a Mapbox token, Gazebo only, no
  roads/trees), OSM2World, blosm, gazebo_osm and SUMO were reviewed; the token-free route reuses CesiumJS
  (globe), Terrain Tiles (AWS Open Data), OpenStreetMap via Overpass and Eclipse SUMO, with R6.7's conversion.
- **Have:** `robot_lab_utils/real_world_import.py` (area ≤ 2 km, local aeqd projection, cached downloads saved
  with the world, buildings extruded from footprints with OSM heights/levels, roads/water visual, trees and
  forests, occupancy blocking buildings/trees/water/slopes, free spawn, manifest with WGS84 box, projection,
  attribution and licenses, offline byte-identical regeneration); `real_world_traffic.py` (SUMO netconvert with
  sidewalks/crossings, random vehicle and pedestrian trips, FCD tracks → Gazebo actors); `real_world_server.py`
  + `web/globe.html` (Cesium globe, place search, rectangle selection, import progress); GUI **Worlds & mapping →
  Real place** (open globe, typed box, result, use in Launch, Preview 3D); CLI. Measured: Rostock centre
  (308 m, 116 buildings, 155 roads, 57 trees) on Gazebo, MuJoCo, PyBullet and Isaac: ground and two roofs
  (20.0 and 27.1 m) within 1.7 cm; the people/traffic world (40 people, 30 vehicles) loads in Gazebo with
  the same support heights and no missing actor meshes (rendering not checked visually)
  ([evidence](docs/status/evidence/real-world-import-2026-11-13/README.md)).
- **Remaining implementation:** none. Optional: satellite texture (Mapbox through gazebo_terrain_generator).
- **Done when:** met.
- **Validation later:** robot missions in imported areas; people/traffic behaviour; projection/datum checks
  against surveyed points.

### R7 — Algorithm breadth

- **State:** partial (I19 implementation done 2026-11-14; comparisons remain V10) · **Priority:** V10 · **Depends on:** R7.1
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

- **Done 2026-11-14 (implemented, experimental):**
  - **Simulator seeds:** `sim_seed:=<n>` adds a Gazebo world system that
    seeds the shared noise generator. In a measured IMU world, the same seed
    repeated 300 of 300 noisy samples; another seed or no seed matched 0.
    MuJoCo, PyBullet and Isaac publish exact sensors. Their seeded noise is
    the sensor-fault relay, which now runs on every backend
    ([evidence](docs/status/evidence/r7-simulator-seed-2026-11-14/README.md)).
  - **Method records:** `scripts/algorithm_method_records.py` writes one
    ledger record per method (`R7.method_records`, 78 ids: dispatch target,
    installed, tests, mathematics, name/math state). From reading each
    node's code: 59 consistent, 9 corrected, 8 named mismatches kept with a
    note, 2 relays ([records](docs/status/evidence/r7-method-records-2026-11-14/README.md)).
    - **Corrected:** `laser_scan_to_pointcloud` started the reverse
      conversion; the retired MAVROS stub was still dispatched; the catalog
      named a nonexistent TEB package and costmap layer; NavFn was listed as
      a potential field, and the simple/noisy controllers as PID;
      `a_star_planner` and `pure_pursuit` named code that is not dispatched.
    - **Kept with a note:** `dijkstra_planner` ran the same NavFn as
      `navfn_planner` (its own plugin since 11-20); the two robot_localization ids are one node;
      `imu_complementary_filter` is Madgwick; `ekf_3d_estimator` is a planar
      CTRV EKF; `motion_model_estimator` is an IMM; `pose_graph_estimator` is
      a sliding-window smoother; `pd_motion_planner` is a PD controller.
- **Converter runtime (2026-11-14, Codex):** official Humble scan/cloud binary
  is provisioned on SSD with repository SHA256/version records. A real ROS
  three-ray scan converted to the expected XYZ points within 0.14 µm; this is
  a conversion fixture, not a robot mission. Native MuJoCo fault routing and
  seed-before-sensors world copies have focused launch checks.
  [Receipt](docs/status/takeover-2026-11-14.md).
- **Dijkstra (2026-11-20, implemented, experimental):** `dijkstra_planner`
  now dispatches `robot_lab_planning::DijkstraPlanner` instead of NavFn. It
  is an eight-connected uniform-cost search with cost-weighted steps, and
  it raises `PlannerException` when there is no path. On the R7.6 protocol
  it reached 15/15 goals against NavFn's 15/15, with paired time and path
  ratios of 0.994 and 1.003. Its worst clearance was 0.261 m, against
  NavFn's 0.030 m
  ([evidence](docs/status/evidence/r7-dijkstra-dispatch-2026-11-20/README.md)).
- **Remaining implementation:** none for I19.
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
- **Done 2026-11-14:** `scripts/external_source_licenses.py` identifies the
  licenses of every runtime source checkout from its license files: PX4
  v1.16.2 is BSD-3-Clause with 23/25 submodules identified (MAVLink generator
  LGPL-3.0, CycloneDDS EPL-2.0, NuttX Apache-2.0); the R14.1 robots resolve
  except BlueROV2 (no license file; package.xml MIT).
  [Report](docs/status/evidence/r91-external-sources-2026-11-14/README.md).
- **Remaining implementation:** per-model terms for the unresolved datasets
  (robot_assets, Gazebo model sets), BlueROV2 and Husky before redistribution.
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
- **2026-11-20:** The shared-world Fortress server and optional client run as
  separate launch-owned processes. A Linux subprocess check measures exit on
  launcher loss. Multi-member simulator shutdown remains V7.
- **Done when:** contacts, member loss/reset and shutdown are measured for
  3–8 members on each advertised backend.

### R10.2 — Fleet management and Open-RMF

- **State:** implemented, experimental · **Priority:** I9 · **Depends on:** R10.1
- **Goal:** missions are dispatched to a fleet and coordinated at shared
  places (corridors, doors, lifts, chargers).
- **Have:**
  - Humble Open-RMF adapter (`robot_lab_utils/fleet_rmf_adapter.py`) driving real Nav2 missions.
  - Queues, nearest-idle allocation, pause/resume/cancel.
  - Physical door and lift events; one Bumperbot lift ride L0→L1→L0;
    post-exit parking. [Remaining-work guide](docs/REMAINING_WORK_GUIDE.md).
- **Done 2026-11-14 (implemented, experimental):**
  - shared-zone reservations (`fleet_zones.py`): single-lane passages from
    every map, door sweeps, docks and declared zones; a member is held 1 m
    before a zone a peer holds, with its task suspended, not canceled;
  - circular yield waits resolve to the oldest task without cancellation;
  - **Recover held lift** can drive the passenger out and park it;
  - RMF energy accounting from a drivetrain calibration
    (`fleet_energy_calibration.py`), gated per model on description hash and
    ≤ 25 % held-out energy error; modeled SOC sent to RMF;
  - measured: native and RMF two-robot door crossings, Bumperbot/Labbot
    calibrations (6 % / 15 % held-out).
    [Evidence](docs/status/evidence/fleet-zones-energy-2026-11-14/README.md).
- **Done 2026-11-18 (implemented, experimental): doors, chargers and energy
  without a lift.** The infrastructure node takes an optional `lift` (one
  floor `L0` without it; lift commands refused), with the lift footprint and
  door path zones in configuration. New world `fleet_ward`: two rooms, one
  physical sliding door, two chargers. Bumperbot and Labbot cross the door
  at once (the door reservation holds one, closest approach 1.05 m, no
  contact), then dock and charge (0.7988 → 0.8047 modeled fraction in 10 s).
  The energy certificates were re-bound after a system update changed the
  readback library (new readback case passes).
  [Receipt](docs/status/evidence/r102-fleet-ward-2026-11-18/README.md).
- **2026-11-20 (implemented, experimental):** Goals, coverage, and traffic
  recovery use the member's observed floor and matching acknowledged Nav2
  map. Lift transport and RMF preserve floor identities on queued routes.
  The Fleet GUI shows the confirmed navigation floor. Focused checks use
  both authored `fleet_facility` grids. Live mixed-floor missions remain V7.
- **Remaining implementation:** none known; larger fleets and RMF in other
  worlds are validation (V7).
- **How to do it:** extend `fleet_controller.py`, `fleet_transport_queue.py`,
  `fleet_lift_recovery.py` and `fleet_rmf_adapter.py`. Keep reservations and
  observation gates; never move a robot by bypassing them. Batch runs:
  `robot-lab-fleet-workload --spec ...`.
- **Done when:** two-robot conflicts at a shared resource resolve, with
  physical outcomes matching task status. *Met for one door, twice
  (2026-11-14).*
- **Validation later:** larger fleets, three-robot passages, occupied-lift
  drive-out run, contacts, loss (V7).

### R10.3 — Swarm robotics

- **State:** implemented, experimental (air pending) · **Priority:** I10 · **Depends on:** R10.1
- **Goal:** formation, rendezvous, dispersion, flocking and cooperative
  coverage for ground robots and drones, with group Stop and member loss.
- **Have:**
  - Four centralized ground kernels with freshness/separation guards.
  - Two-X500 range-reference formation and operator-loss landing (measured).
  - Formation route Plan/Execute (`aerial_swarm_route.py`, `aerial_swarm_controller.py`).
- **Done 2026-11-14 (implemented, experimental):**
  - per-member behaviours from each robot's own scan, ported from ROS2swarm
    (`swarm_behaviours.py`: dispersion, attraction, aggregation, minimalist
    flocking, protection layer);
  - cooperative coverage measured from scan rays (`swarm_coverage.py`);
  - `loss_policy: replan` for ground swarms, coverage and aerial groups;
  - lidar mounting yaw in scan bearings (Bumperbot lidar is turned 3.14 rad);
  - measured: dispersion 0.94 → 3.15 m; coverage 90 % after a real member
    loss. [Evidence](docs/status/evidence/swarm-distributed-2026-11-14/README.md).
- **Done 2026-11-18 (implemented, experimental): aggregation gathers.**
  Members had frozen against walls (the scan guard stops a forward command
  inside its stopping corridor, short of the protection layer) and against
  peers their lidar cannot see. Behaviours now turn in place short of the
  guard's corridor, and a member held by the separation guard turns away
  from the nearest peer. In a new 5 × 5 m `swarm_arena`, three Bumperbots
  starting 3.6–4.0 m apart gather (mean 3.9 → 1.0 m; all within 1.5 m from
  21.9 s and 56 % of 180 s). Mixed Bumperbot/Labbot groups only partly
  aggregate (a Labbot cannot see a Bumperbot). Attraction gathers in the
  open (2.5 → 0.39 m in about 7 s); in the arena it goes to the walls.
  Flocking does not flock (alignment about chance) and is time-boxed.
  [Receipt](docs/status/evidence/r103-swarm-aggregation-2026-11-18/README.md).
- **Remaining implementation:**
  - the −inf lidar sector in RGB-D-off sessions (keep RGB-D on).
- **How to do it:** compare with ROS2swarm behaviours (see the
  [extension guide](docs/NEXT_EXTENSION_GUIDE.md)). Keep per-FCU IDs, ports,
  leases and frames. Settle on FCU time; watchdogs use wall time.
- **Done when:** measured relative motion and member-loss outcomes for each
  behaviour on ground and air. *Ground dispersion and coverage-with-loss
  measured; aggregation and attraction measured 2026-11-18; flocking not passed; air pending.*

### R10.4 — Underwater robots and controllers

- **State:** implemented, experimental · **Priority:** I11
- **Have:** the self-made eight-thruster `lab_rov`; the pinned CC-BY MBARI
  Tethys AUV with fins and trim; Fortress buoyancy, drag and added mass.
  Tank surge/depth, reef depth/turn and a gentle tank curve are measured.
  [ROV](docs/tutorials/marine-rov.md), [AUV](docs/tutorials/tethys-auv.md).
- **Done 2026-11-14 (implemented, experimental):** MuJoCo plant for `lab_rov`
  (`marine_mujoco_plant.py`, `simulator:=mujoco`) with ported Fortress buoyancy,
  damping, added mass and thrusters behind the unchanged controller; the same
  probe gives 1.03 vs 1.07 m surge and 1.44 vs 1.50 rad yaw (MuJoCo vs Gazebo).
  Current feedforward from the declared damping: 15 s in 0.1 m/s cross current
  drifts 1 cm (was 0.37 m upstream).
  [Evidence](docs/status/evidence/marine-mujoco-currents-2026-11-14/README.md).
- **Done 2026-11-18 (implemented, experimental): PyBullet plant.** The
  water and thruster wrench is engine-neutral (`WaterForces`);
  `marine_pybullet_plant.py` integrates it in Bullet (`simulator:=pybullet`).
  The same probe gives surge 1.041 m, yaw 1.439 rad, 3.5 cm drift in a
  0.1 m/s current and disarm on operator loss (MuJoCo: 1.030 m, 1.437 rad,
  0.9 cm). The MuJoCo plant's spawn yaw was read in degrees; fixed.
  [Receipt](docs/status/evidence/r104-marine-pybullet-2026-11-18/README.md).
- **Remaining implementation:**
  - BlueROV2 candidate license review (R9.1);
  - Done 2026-11-19 (implemented, experimental): the ROV on Isaac Sim
    (`marine_isaac_plant.py`, PhysX rigid body with the shared water
    forces): surge 1.038 m and yaw 1.452 rad, as on the other three
    backends. [Receipt](docs/status/evidence/r104-marine-isaac-2026-11-19/README.md);
  - Done 2026-11-19 (implemented, experimental): Tethys plants on MuJoCo,
    PyBullet and Isaac (ported buoyancy, hydrodynamics, thruster and fin
    LiftDrag; `auv_simulator.launch.py simulator:=mujoco|pybullet|isaac`).
    Cruise matches Gazebo (2.42–2.43 vs 2.45 m in 10 s); a cruising turn
    reaches 0.86 / 0.51 / 0.49 rad against 1.19 rad. The added-mass Coriolis term is off
    by default (closest to Fortress 6.18 of three forms).
    [Receipt](docs/status/evidence/r104-tethys-mujoco-2026-11-19/README.md);
  - ~~why the ported turn is slower than Gazebo's~~ **identified 2026-11-20:** Fortress 6.18 applies the SDF's positive
    added-mass values with the opposite sign to Fossen, leaving Gazebo's hull with 20 % of its yaw inertia; steady turn
    rates agree and the gap is spin-up. The port stays Fossen-correct
    ([receipt](docs/status/evidence/r104-tethys-turn-identification-2026-11-20/README.md)).
- **Done when:** station keeping, all axes, disturbance and loss measured with body and force evidence.

### R10.5 — Underwater worlds, sensing, SLAM and navigation

- **State:** implemented, experimental (saved-map relocalization pending) · **Priority:** I11 · **Depends on:** R10.4
- **Have:**
  - Five water worlds (tank, reef, harbour, cave, wreck).
  - IMU/RGB-D bridges, labelled ideal pressure, DVL emulation (no native plugin), directional sonar.
  - ROV visual/ICP and AUV geometry registration.
  - RTAB-Map save/reload; curved AUV routes (`auv_routes.py`).
- **Done 2026-11-14 (implemented, experimental, unit-tested):** volumetric AUV
  routes (DP depth profile over/under obstacles within the 0.1 slope) and
  current-aware ranking by travel time through the operator-authored current,
  refusing > 0.5 rad crab.
- **Done 2026-11-19 (implemented, experimental): a mission from the sensor
  estimate.** On Gazebo, Tethys planned and flew a 3.5 m route from the
  IMU/pressure/DVL estimate (RTAB-Map mapping online) and stopped 0.20 m
  from the goal by truth; the estimate stayed within 0.13 m. A 6.5 m route
  diverged after 3 m (vehicle yaw/pitch drift); a start 1.5 m deep was
  refused by the depth clearance. *Done-when met for one mission.*
  [Receipt](docs/status/evidence/r105-auv-estimated-mission-2026-11-19/README.md).
- **Done 2026-11-19 (implemented, experimental):** AUV saved-map
  relocalization.
  - New `marine_tank_textured` world: unique marker panels in the forward
    camera's view, clear of the route.
  - New opt-in `visual_features:=true` (Tethys mapping is otherwise
    geometry-only and stores no visual words).
  - Tethys maps and saves the route, then relocalizes in a fresh launch and
    flies the mission from the saved map: 2/2 runs, 0.18 and 0.31 m from
    the goal.
  [Receipt](docs/status/evidence/r105-auv-relocalization-2026-11-19/README.md).
- **Remaining implementation:**
  - ~~longer AUV routes~~ **done 2026-11-20:** the 6.5 m divergence came from the controller (measured-sideslip
    feedback into the unstable hull, a second integral on depth, a 1 m line-of-sight lookahead), now fixed. In a new
    60 × 40 m `marine_open_water` bay, Tethys flies 20 m (2/2, 0.18 m), a curved 23 m route (0.14 m) and a 43 m
    two-leg mission (0.10 m) from its sensor estimate. The tank routes now end 0.05–0.08 m from the goal
    ([receipt](docs/status/evidence/r105-auv-long-route-2026-11-20/README.md));
  - ~~relocalization in the default geometry-only mode~~ **done 2026-11-20:** geometry-only localization now starts
    at the mapped origin (the estimator shares the spawn) and ICP localizes on 3D structure in view: 2/2 in
    `marine_tank_textured`; the featureless tank still cannot
    ([receipt](docs/status/evidence/r105-auv-geometry-reloc-2026-11-20/README.md)).
- **How to do it:** `marine_sensors.py`, `marine_navigation.py`, `marine_pipeline.py` and
  `map_archive.py` in `robot_lab_utils/`. Never feed simulator pose into the estimator.
- **Done when:** an obstacle or inspection mission runs from real sensors and estimates.

### R10.6 — Six- and eight-legged robots

- **State:** implemented, experimental (recovery unit-tested) · **Priority:** I12
- **Have:**
  - Pinned MIT `hexapod_ros2` (CAD) and the self-made `lab_octopod`.
  - Tripod/tetrapod/ripple/wave gaits; four backend adapters.
  - Low-relief travel and hold pass on all four simulators.
  - [Gait guide](docs/tutorials/multi-leg-gaits.md).
- **Done 2026-11-14 (implemented, experimental, unit-tested):** bounded slip
  and support-loss stance recovery in terrain mode (`gait_recovery.py`).
  Independent estimation is the existing contact-kinematic odometry; the
  open-source octopod is the R14.1 source octopod.
- **Done 2026-11-19 (implemented, experimental):** saved-map Nav2 goals
  for every gait model on MuJoCo `nav_empty`.
  - Each model acquires and saves a map, then reaches a goal on it in a
    fresh launch, judged by independent truth: hexapod 0.09 m, octopod
    0.08 m, centipede 0.12 m.
  - The centipede passed 1 of 2: the other run stopped 0.157 m off,
    outside the 0.15 m contract.
  - `scripts/qualify_gait_saved_map.py`.
  [Receipt](docs/status/evidence/r106-gait-saved-map-2026-11-19/README.md).
- **Slip mission (2026-11-19, recovery not exercised):** new `gait_slip_patch` world (friction-0.05
  plate; MuJoCo contact priority so the plate's friction applies). The octopod crosses it at its
  0.045 m/s limit without a measured slip. Plates rolled 4–6° (enough to beat the friction) exceed the
  gait's 2 cm terrain envelope, and the gait stops before them. The recovery stays unit-tested.
  [Receipt](docs/status/evidence/r106-gait-slip-2026-11-19/README.md).
- **Remaining implementation:**
  - per-model terrain tuning. **Recovery mission done 2026-11-20:** lateral pushes (60–100 N, 0.3 s, MuJoCo
    base force pulse now in `gait_simulator.launch.py`) trip the octopod's support-loss check, and the bounded recovery
    pauses, resumes in 1.3–1.6 s and finishes the walk (4/4 tripped pushes); the slip trigger itself was not reached
    ([receipt](docs/status/evidence/r106-gait-push-2026-11-20/README.md));
  - saved-map goals on other backends, and a passing centipede run in a second world. **2026-11-20, `nav_obstacle`
    (MuJoCo):** hexapod 0.128 m and octopod 0.099/0.105 m pass; the centipede fails on its long footprint and heading
    ([receipt](docs/status/evidence/r106-gait-saved-map-obstacle-2026-11-20/README.md)). **PyBullet:** the octopod
    passes (0.091 m); the hexapod's in-place turn drifts and the centipede maps too slowly
    ([receipt](docs/status/evidence/r106-gait-saved-map-pybullet-2026-11-20/README.md)).
- **How to do it:** `gait_model.py`, `gait_controller.py`, `gait_odometry.py` and
  `gait_terrain.py` in `robot_lab_utils/`. Keep one owner of `/gait/state`.
- **Done when:** walking on terrain, contacts and saved-map navigation per model and backend.

### R10.7 — Segmented and additional robot forms

- **State:** implemented, experimental (0.18 m step crossed 2026-11-19) · **Priority:** I12
- **Have:** self-made centipede, snake, catamaran USV; imported Gazebo tracked model and
  articulated-flipper vehicles, and a sprung amphibious rover. Each has native
  controls; mapping, navigation and shore cases are measured.
  [Native forms](docs/status/native-forms-2026-10-30.md).
- **Done 2026-11-14 (implemented, experimental):** flipper-aware terrain
  policy from the robot's own depth camera and IMU (`tracked_flippers.py`,
  GUI **Automatic terrain flippers**). Measured at a 0.18 m step: it
  sequences approach → climb, but neither it nor fixed flippers climb; the
  4 Nm flipper cap is back-driven and the hull rears past the 0.6 rad tilt
  guard. [Evidence](docs/status/evidence/flipper-stairs-2026-11-14/README.md).
- **Done 2026-11-19 (implemented, experimental):** `lab_tracked_strong_flippers`
  (40 Nm flipper PID) with a full-press climb state crosses the 0.18 m step
  (up, along the tread, down) in 2 of 2 runs. Fixed flippers or the default
  climb still fail on the same vehicle.
  [Evidence](docs/status/evidence/flipper-strong-2026-11-19/README.md).
- **2026-11-20:** standalone tracked launches use the same per-run cleanup
  watcher as the normal GUI launch. Fleet launches also tag their nodes for
  cleanup if the launcher exits abruptly.
- **Remaining implementation:**
  - other backends for the forms: **USV done 2026-11-20** (the catamaran's surface controller on the MuJoCo, PyBullet
    and Isaac marine plants, within a few percent of Gazebo;
    [receipt](docs/status/evidence/r107-usv-backends-2026-11-20/README.md)); the snake, tracked and amphibious forms remain;
  - angled descents beyond 15° for the tracked vehicle (one track drops first and the 0.35 rad roll hold stops it).
    **Angled approaches done 2026-11-20:** the 0.18 m step is climbed at 15°, 30° and 45° off square and descended
    at 15° (2/2) in a new `terrain_step_wide` ([receipt](docs/status/evidence/r107-tracked-angled-2026-11-20/README.md)).
    **Multi-step descent done 2026-11-20:** up a five-step flight and down
    the other side in `terrain_stairs_over`, 2/2 ([receipt](docs/status/evidence/r107-tracked-over-2026-11-20/README.md)).
    **Multi-step ascent done 2026-11-20:** the
    strong-flipper vehicle climbs a five-step flight (0.9 m, 3/3) in a new `terrain_stairs_flight`. The flipper
    policy now leaves a climb on a level tread with a riser ahead, and a new ledge guard stops forward drive before
    an unseen or > 0.3 m drop, after two runs drove off the flight's 0.9 m end
    ([receipt](docs/status/evidence/r107-tracked-flight-2026-11-20/README.md)).
- **Done when:** locomotion, clearance and sensor/map missions per form.

### R10.8 — Fleet, swarm and marine GUI and experiments

- **State:** partial · **Priority:** validation V7 (code exists)
- **Have:** native Fleet/Swarm/Marine/Gait/Tracks panels with autofill; real workload producers.
- **Done when:** GUI commands exercise each claimed runtime, with measured artifacts.

### R11 — MOLAR / ProBOT-L family

- **State:** implemented, experimental (done-when met 2026-11-17); remaining items below · **Priority:** I17 (mounted sensors need the real robot's layout from the user)
- **Goal:** the user's MOLAR robots drive, carry payloads and navigate the
  detailed CELISCA floors.
- **Have:** `molar_hri` passes the GUI check (drive, lift, telescope, roller,
  Stop, watchdog) on four backends and three maps; `probot_l` passes on `nav_empty`.
  [Guide](docs/tutorials/molar-hri-simulation.md).
- **Remaining implementation:**
  - mounted sensors (needs the real layout);
  - SLAM/Nav2 on the floors: **first measured route 2026-11-15 (Gazebo, celisca_floor_1_detailed)**.
    Route A (9.6 m, multi-turn) succeeds within 0.18 m, and cancel/resume pass; the near-obstacle goal
    B fails. **2026-11-18: goal B passes** (2/2 runs, 0.09 m) after an AMCL overlay for MOLAR's
    wheel-only odometry; route A 0.08–0.12 m
    ([receipt](docs/status/evidence/r11-molar-nav-goal-b-2026-11-18/README.md)). This needed MOLAR's own controllers in every Gazebo mode, `lidar_2d`, a ±105° front scan
    (360° hit its own body) and MOLAR-sized goal selection.
    [Receipt](docs/status/evidence/r11-molar-nav-2026-11-15/README.md);
  - **CELISCA floor transfer (2026-11-17): passes** in Gazebo on `celisca_floor_1_detailed`, the
    same world as route A. The box rests on the station, 0.669 m ahead, with no retract drag.
    **R11's done-when is met.** This needed display-mode controller spawners with
    `--switch-timeout 60` and checker waits scaled for slow worlds.
    [Receipt](docs/status/evidence/r11-molar-celisca-transfer-2026-11-17/README.md);
  - payload transfer (lift + roller to a station): **measured 2026-11-15 (implemented, experimental)**.
    The deck belt (Gazebo TrackController on the private DART plugin; MuJoCo `ConveyorBelt`) carries
    a 0.5 kg box onto a station at the extended deck end, through the normal articulation interface.
    Passes 3/3 in Gazebo and 4/4 in MuJoCo on `nav_empty`
    ([receipt](docs/status/evidence/r11-molar-transfer-2026-11-15/README.md)). The belt force law is
    now shared (`robot_lab_utils/belt.py`), and a **PyBullet deck belt** (`pybullet_belt.py`) is
    implemented and unit-tested (2026-11-17). **The MOLAR PyBullet transfer passes (2026-11-17,
    evening).** The morning's "base yaws at roller start" came from the test: its station overlapped
    the extended deck's front frame, so the telescope stalled against it and its servo shoved the base
    whenever it resumed. PyBullet also ignored the declared friction of static surfaces (now applied
    below 1). With the station clear, the transfer passes once each on PyBullet, MuJoCo and Gazebo
    ([receipt](docs/status/evidence/r11-molar-transfer-station-2026-11-17/README.md); morning
    [receipt](docs/status/evidence/r11-molar-pybullet-transfer-2026-11-17/README.md)).
    **Isaac belt (2026-11-18):** PhysX surface velocity on the deck, as in Isaac's conveyor; the
    transfer passes 2/2 there, so on all four backends
    ([receipt](docs/status/evidence/r11-molar-isaac-belt-2026-11-18/README.md));
  - multi-floor operation through the fleet lift (R10.2). **2026-11-19: MOLAR rides L0 → L1 → L0**
    (2/2) as a new shared-world fleet member: its own base controller, a `base_footprint` root and
    its AMCL overlay. Its floor and Nav2 map switch at both landings. The lift drive was raised to
    4000 N with a faster integral (1600 N was too weak for car plus MOLAR); the Bumperbot ride
    still passes ([receipt](docs/status/evidence/r11-molar-lift-2026-11-19/README.md)).
    It then drives off the car to a Nav2 goal on L1 (2/2, 0.04–0.05 m); a goal that needed a turn
    beside the landing rail failed and is retained. **2026-11-20: the full native fleet transport passes** (Facility transport requests only, L0 → L1 → L0, 2/2, 183–189 s) with per-model transport goals that keep MOLAR's arrival heading; a stop short of the closed gate and a turn in the car failed and are retained; Bumperbot re-passes ([receipt](docs/status/evidence/r11-molar-transport-2026-11-20/README.md)).
  - **Open-RMF lift graph for MOLAR (2026-11-20):** implemented, experimental.
    The SDK uses the configured boarding point and directed approach/exit
    lanes, and the fleet controller uses a guarded Nav2 Spin for the
    in-car heading waypoint. The measured Gazebo L0 → L1 leg completes
    in 114.9 s. L1 → L0 is held by the scan guard during the turn before
    the closed gate on the original landing. The widened L1 layout,
    separate forward retreat and guarded waiting-point Spin allow the full
    Gazebo Open-RMF round trip (223.7 s, both maps and measured settling).
    Mixed-fleet and loaded lift rides remain open. See the
    [original runtime scope](docs/status/evidence/r11-molar-rmf-2026-11-20/README.md)
    and [return clearance follow-up](docs/status/evidence/r11-molar-rmf-return-2026-11-20/README.md).
  - **Fleet cargo (2026-11-20):** the Fleet editor can spawn a loose 0.5 kg
    reference box on the stowed MOLAR deck. The controller checks measured
    box, base and deck poses and holds motion if retention is lost.
    Loaded boarding on Gazebo `fleet_facility` retained the box in 184/184
    samples, with at most 5.0 mm relative drift. The ride and return were
    not run. [Receipt](docs/status/evidence/r11-molar-fleet-cargo-2026-11-20/README.md).
- **How to do it:** add sensors to `molar_hri.urdf.xacro`, then reuse the
  Bumperbot SLAM/Nav2 configs with the MOLAR footprint. Payload transfer
  uses the existing articulation contract (`robot_lab_utils/urdf_articulation`).
- **Done when:** a measured route plus payload transfer on a CELISCA floor.

### R11.1 — Photo-based MOLAR/CELISCA asset refresh

- **State:** done · **Priority:** explicit user request 2026-11-10.
- **Owner:** codex-hri-assets-20261110.
- **Goal:** update both MOLAR variants, manual/photo-based lift and telescope pose/limits, declared sensor frames and detailed CELISCA floor-2/3 charger geometry in the native Robot Lab.
- **Done when:** source and installed model/map paths resolve to the refreshed assets; limits/frames and actual scoped engine loads are verified and recorded. Broad missions, calibrated sensors and physical acceptance remain separate from this asset refresh.
- **Scope:** native molar1 development only; preserve unrelated dirty files and historical certificates. No FMS/AIC/PLC or physical motion commands.

- **Evidence:** `docs/status/model-map-refresh-2026-11-10.{md,json}`; 62 native tests, installed/source checks, normal-GUI PyBullet and MuJoCo controls. Broad R11 remains separate.

### R11.2 — New-photo CAD/projector/charger refresh

- **State:** done · **Priority:** explicit user request 2026-11-11.
- **Owner:** codex-photo-assets-20261111.
- **Goal:** refresh the photo-based robot, equal-width conveyor, wall-mounted charger and HRI assets; retain the existing navigation/station-transfer features.
- **Done when:** native source/installed assets and focused geometry/frame checks pass, with existing navigation/station-transfer modes retained. No broad new mission or physical acceptance claim.
- **Scope:** model/map assets and provisioners on molar1; preserve be016bd0 navigation profiles, MOLAR station-transfer GUI and unrelated work.

- **Evidence:** `docs/status/photo-model-refresh-2026-11-11.{md,json}`; 97 native checks and two package builds, nine protected source hashes retained. Scope is model/map assets, not broader R11 missions.

### R11.3 — Charger display setback

- **State:** done · **Priority:** explicit user map correction 2026-11-11 · **Depends on:** R11.2
- **Owner:** codex (whole-tree review November 11).
- **Goal:** move three visual charger assemblies 0.35 m away from cached docking goals,
  preserving registered floors and real robot targets.
- **Have:** reviewed GLB pins, regenerated visuals/collision adapters and occupancy projections.
  Mesh review confirms identical faces and only 0.35 m vertex translations. Both original source
  GLBs retain their geometry/transforms; 126 copied adapters and 136 installed assets match hashes.
  Git preserves copied CRLF bytes so source checksums survive checkout. Updated the stale 96-part
  test to the actual pinned 148/122 detail nodes; 52 focused map/MOLAR/camera checks pass.
- **Remaining implementation:** none within this display-geometry scope.
- **How to do it:** retain the reviewed pins in `scripts/provision_celisca_detailed.py`, source
  byte policy in `.gitattributes` and each map's `PROVENANCE.json`; use the guarded generator
  and SSD backups for future source refreshes. Do not shift FMS/AIC goals as a display adjustment.
- **Done when:** reviewed source geometry, native/installed asset hashes and occupancy registration
  agree, with no physical commands or navigation-profile changes.
- **Validation later:** broader docking and floor missions remain R11 acceptance.
- **Evidence:** `docs/status/charger-setback-2026-11-11.{md,json}` and
  [whole-tree review](docs/status/evidence/vacuum-camera-2026-11-11/whole-tree-review.md).

### R12 — Legged policy training

- **State:** partial · **Priority:** I13
- **Have:**
  - The GUI Training page and `legged-train` (PPO, ONNX export) for Go2, A1 and B2.
  - Tasks: walk, turn_in_place, stand, push_recovery; randomisation,
    rough terrain, noise and actuation delay.
  - `go2_walk_r1`–`r3` pass drive screens on three backends.
  - [Guide](docs/tutorials/legged-policy-training.md).
- **Remaining implementation:**
  - Go2W strafe: **measured 2026-11-15.** The new wheel gains raise the vendor policy's MuJoCo strafe
    from 0.206 to 0.248 m/s. A new `wheeled_walk` task uses robot_lab's Go2W reward weights (no
    air-time reward). `go2w_walk_r2` strafes at 0.116 m/s in MuJoCo and 0.271 m/s in Isaac, and
    reverses in MuJoCo (r1: 0.006 m/s strafe, no reverse). Isaac reverse is weak (−0.077 m/s).
    [Receipt](docs/status/evidence/r12-go2w-strafe-2026-11-15/README.md);
  - Berkeley Humanoid Lite: trainer template, BHL-order export, `humanoid_walk` rewards and warm
    start implemented 2026-11-15; four runs not passed (see R5.3);
  - Go2 get-up: `get_up_roll` (potential-based roll-over shaping) trained 2026-11-17, stands 2 of 10;
    **continued to 6000 iterations (r2): stands 29/30 and gets up in a normal MuJoCo launch,
    including walk → get-up → walk switching** (2026-11-17,
    [receipt](docs/status/evidence/go2-get-up-roll-r2-2026-11-17/README.md));
  - robot_assets ANYmal (2026-11-18): `legged_templates.py` writes a training template from
    legged_gym's ANYmal C configuration, including its reward weights. r1, on the trainer's
    Go2-sized weights, only stood. r2 walks: MuJoCo drive screens 2/2, forward at 60–70 % of the
    command, spin 0.97 rad/s, weak strafe; Isaac drives except a stalled first phase
    ([receipt](docs/status/evidence/anymal-walk-2026-11-18/README.md));
  - Go2 stairs: the stairs curriculum is now judged against the commanded distance (2026-11-18).
    The old fixed 3 m rule held r1 at 4.4 cm rises; r2 reached level 0.5 (10 cm) by iteration 8205,
    but does not climb in deployment (see R5.2);
  - GPU training (CPU-only on Orin; training possible but slower).
- **How to do it:** `robot_lab_adapter/legged_training.py`, `ppo.py` and `legged_train.py`.
  Add robots by their provisioned policy contract; train in MuJoCo; check
  through `legged_policy:=<dir>` on all backends.
- **Done when:** a trained policy performs its task in measured runs on the backends.
  Go2W strafe: simulation test shows improved lateral velocity with updated PD gains.
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

- **State:** implemented, experimental (done-when met; learned camera measured in two Gazebo jobs, 2026-11-15 and 11-17) · **Priority:** I2 (repeats/held-out rooms are V1) · **Depends on:** R13.1
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
- **November 11 implementation:** the torchvision RGB detector is an installed executable,
  selected exclusively against segmentation/none in launch and GUI Setup. Missing files fail before
  simulation; detector exit shuts down its owned launch. RGB channels/padding/endianness, paired
  timestamps and calibration dimensions are checked. Checkpoint class maps are required; runtime
  downloads are disabled. Read-only capture records per-instance boxes, image hashes and scene groups.
  Training uses disjoint held-out images, checks scene/class coverage and matches inference input range.
  The GUI **Objects** page shows bounded timestamped JPEG crops from both detectors. Generated world
  footprints can feed reporting only; `cleaning_screen` now rejects swept-body overlap and stale truth.
  [Implementation receipt](docs/status/evidence/vacuum-camera-2026-11-11/README.md),
  [operator/agent guide](docs/tutorials/vacuum-camera.md).
- **November 12 initial implementation complete:** actual Gazebo fixture rendering generated 96
  source-camera views across three layouts (64 train / 32 held-out). An eight-epoch CPU run produced
  a content-addressed, locally provisioned experimental model; the GUI prefills its path and CLI
  `vacuum_detector_model:=auto` verifies its checksum. Deployment preserves the trained image transform.
  [Receipt](docs/status/evidence/vacuum-camera-model-2026-11-12/README.md).
- **November 15 multi-map retrain (implemented, experimental):** the dataset generator renders random
  layouts on free floor of nav_empty, nav_obstacle and small_house, with empty background views.
  Retrain 2 (1050 views, hflip, step LR) reaches held-out mAP@0.5 0.800 and is the installed `auto`
  checkpoint. `scripts/evaluate_vacuum_detector.py` measures it through the deployed loader. At the
  new `vacuum_detector_score` default of 0.75, d1 cable is 8/13 (was 1/13) and shoe false positives
  are 0 (were 22). Learned-camera Gazebo zone job, small_house objects: `completed`, `cleaning_screen`
  pass, 88.3 %, **0 bumper contacts**, body ≥ 0.232 m from every footprint by truth, all four
  recognised. One run on a training map; one extra cable sighting.
  [Receipt](docs/status/evidence/r133-camera-detector-2026-11-15/README.md).
- **November 17 retrain 3 (implemented, experimental):** fine-tuned from retrain 2
  (`train_vacuum_detector.py --init`) on 750 new renders, weighted to pet waste and socks, with a
  quarter of the views background-only. On the same held-out sets at score 0.75:
  - pet-waste false positives fall on every set (3 → 0, 4 → 2, 4 → 3), and dB recall rises from
    54 % to 60 %;
  - cable false positives fall on d1 and dB (2 → 0, 11 → 7) and rise on dA (2 → 4);
  - sock false positives fall on dA (10 → 6) but not on dB (16 → 17).

  Installed as `auto`. The learned-camera small_house zone job passes again: `completed`,
  `cleaning_screen` pass, 86.9 %, **0 bumper contacts**, minimum body clearance 0.286 m by truth.
  [Receipt](docs/status/evidence/r133-retrain3-2026-11-17/README.md).
- **Remaining:** sock false positives on dB (17) and pet-waste recall (60 %) are training-quality
  work. Repeats, a held-out room and real objects are validation (V1). Ideal segmentation stays the
  default.
- **History, November 12 quality gap:** held-out mAP@0.5 was 0.564. At the deployed 0.4 score threshold,
  matches/GT are cable 1/13, pet mess 4/5, shoe 8/12 and sock 3/4; shoes have 30 predictions.
  Cable misses and shoe false positives need training improvement before support promotion. These
  are source-primitive renders, not general real-object recognition. Keep ideal segmentation as default.
- **Validation later:** learned per-class recall/false positives on held-out simulator images and
  a physical object-world cleaning job with `vacuum_object_truth:=<world.objects.json>`.
- **Done when:** a recorded job in a house with these objects ends with zero
  contacts with them by truth, and passes `cleaning_screen`.

### R14.1 — Open-source counterparts for self-made robots

- **State:** initial implementation complete; broader adoption/mission acceptance partial (2026-11-12) · **Priority:** I3 · requested by the user 2026-11-10
- **Implemented:** separate `asset_turtlebot4_lite_cleaning` source variant, native bumper/four cliff sensors, stale-feedback pause, GUI opt-in and command autofill. One Gazebo `nav_empty` zone-clean/dock completed (96.8% reported coverage); bumper impact and other-map qualification remain pending. [Receipt](docs/status/evidence/counterparts-2026-11-12/README.md). Continue WAM-V, BlueROV2, Summit-XL and the remaining source-search rows.
- **WAM-V implemented:** separate `asset_vrx_wamv_h` complete source vessel, private Fortress ABI port of upstream water equations, bounded source-aware controller, native timeout, normal GUI classification/autofill and declared RGB-D kit. One clear-water source run measured 1.8785 m travel and 0.7413 rad turn; other missions/maps/waves and message-loss qualification remain pending. [Guide](docs/tutorials/wamv.md).
- **BlueROV2 candidate implemented:** separate `asset_bluerov2_heavy`, source-frame-preserving Heavy plant with native forces, bounded allocator/timeout and declared ENU/RGB-D kit. One Gazebo run measured 1.1373 m travel and 1.4250 rad turn; license/mesh redistribution review and broader missions remain pending. [Guide](docs/tutorials/bluerov2.md). Continue Summit-XL and remaining search rows before R6.7.
- **Summit-STEEL implemented:** complete source `asset_robotnik_summit_steel` with retained source mechanics/contact model, Fortress control/friction-frame port, declared sensing kit and normal GUI defaults/autofill. One Gazebo Display run measured forward/reverse/strafe/yaw, actual sensors, source wheel bounds and 0.112 m publisher-loss travel. Higher-mode pipelines/footprint are implemented but missions remain unqualified. [Guide](docs/tutorials/summit-steel.md). Finish the remaining source-search decisions and license review; preserve the authored four-wheel-steering benchmark, since this source base is mecanum.
- **Source octopod implemented:** separate MIT `asset_light_robotics_octopod`, original 7.5 kg native plant/27 position servos and source scripted gait, GUI classification/opt-in/autofill. One MuJoCo Display run measured forward/reverse/turn, 2.45 Nm maximum force and command-loss neutral stance. Requested speed is uncalibrated (0.12 m/s requested, about 0.25 m/s measured); terrain/SLAM/navigation remain unavailable. [Guide](docs/tutorials/source-octopod.md).
- **Search pass complete:** reviewed pins and reasons for retained authored forms are recorded below and in the search guide. Tracked robots were already imported Apache-2.0 source derivatives. BlueROV2 license review found an unresolved redistribution chain; keep it a local candidate, continue R9.1 licensing before distribution. Continue R6.7 initial implementation next; broader source missions belong to the validation queue.
- **Search record:** [r14-open-source-candidates-2026-11-10](docs/status/r14-open-source-candidates-2026-11-10.md).
  Finding: the iRobot Create 3 (BSD) is already installed, and the installed,
  measured TurtleBot 4 Lite (Create 3 plus LiDAR) is the quickest open-source
  platform for the cleaning system.
- **Goal:** the lab uses open-source robots wherever a suitable one exists.
  Self-made robots stay only where none exists, with the reason recorded.
- **Why this task exists:** these profiles were authored by agents in this
  project rather than imported. The recorded reasons:
  - the historical octopod/centipede search found no suitable model (the octopod finding is superseded by the November 12 MIT source integration), per the
    [October 14 research](docs/NEXT_EXTENSION_GUIDE.md#upstream-candidates-researched-on-october-14);
  - the robot has no published model: OOMWOO's hardware is not frozen;
  - imported models lacked working actuation for the lab's simulators.

  For several of them, no thorough search was recorded.

  | Self-made profile | Open-source candidates to check (verify license and model quality) |
  |---|---|
  | `oomwoo` (robot vacuum) | Adopted as separate `asset_turtlebot4_lite_cleaning`; one native Gazebo zone-clean/dock measured, contact/other maps pending |
  | `lab_rov` | `asset_bluerov2_heavy` experimental source candidate implemented and one physical control run measured; package MIT/root text and third-party mesh license review pending before adoption |
  | `lab_usv` | Adopted as separate experimental `asset_vrx_wamv_h`; native Fortress source control measured, broad qualification pending |
  | `ackermann_car`, `rear_steer_car`, `anti_ackermann_car` | Retain benchmarks: reviewed MIT Racecar original model license is TODO; ROS2 mirror shares 36 mesh blobs without resolving original asset licensing. No matching rear/anti-steer source established in this pass |
  | `mecanum_car` | Adopted separate `asset_robotnik_summit_steel`, complete BSD-3-Clause base; Gazebo source control/sensing measured, higher missions pending |
  | `four_wheel_steer_car` | Retain benchmark: Ranger source asset license unresolved; MIT omni4WS sample has coincident steering/rolling wheel colliders and lacks physical effort limits. No source-preserving ready plant adopted |
  | `lab_tracked`, `lab_tracked_articulated` | Already Apache-2.0 Gazebo Sim source model and declared actuated derivative; original seven-link/six-joint geometry verified against source SHA. Preserve IDs and scoped receipts |
  | `lab_octopod` | Adopted separate MIT Light Robotics source octopod; scripted MuJoCo control measured, speed/terrain/higher-mode acceptance pending |
  | `lab_snake` | Retain benchmark: reviewed candidates have missing license text/template authors, fixed-base topology, or CAD without a complete bounded physics plant; exact pins in search record |
  | `lab_amphibious`, `lab_centipede` | Retain authored benchmarks: reviewed amphibious research does not establish a licensed wheeled land/water plant; no complete matching twelve-leg/six-segment source was found in this bounded pass. Source octopod is an eight-leg alternative |
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

### R14.2 — Elephant Robotics myAGV Pro

- **State:** implemented (2026-11-13); validation partial · **Priority:** I0 (user top priority, 2026-11-12) · **Owner:** claude (from codex, on the user's request 2026-11-13)
- **Goal:** integrate the official myAGV Pro from the [vendor guide](https://docs.elephantrobotics.com/docs/myAGV_Pro_en/) as a normal complete robot in Robot Lab.
- **Have:** separate BSD-3-Clause `asset_elephant_myagv_pro`, complete official CAD visual model/inertias/sensor mounts, geometric corner mapping, declared lab contact/compliance adapter, bounded native holonomic control, all measured passive-joint states, LiDAR/IMU/RGB-D, native GUI family/tags, Drive/strafe and automatic algorithm/command selection. Gazebo Display forward/reverse/strafe/turn/neutral/command-loss are measured. One normal-GUI Nav2 topic goal in `nav_empty` settled at 0.125 m / 2.23° actual body error. [Guide](docs/tutorials/myagv-pro.md) · [receipt and retained failures](docs/status/evidence/myagv-pro-2026-11-12/README.md).
- **Portable variant (2026-11-13):** `asset_elephant_myagv_pro_rollers` (vendor wheel mass/inertia split into a core plus 15 free FUJI rollers, MIT) drives in MuJoCo, PyBullet and Isaac: forward/reverse/strafe/turn/neutral/publisher loss measured in each; source mass imported exactly in PyBullet (20.336 kg) after empty frames stopped getting Bullet's default 1 kg; a full Nav2 mission passed in PyBullet.
- **Remaining implementation:** none for the base. Hardware serial/charging and complete vendor arm-equipped composites need their actual contracts (also R5.9); no fictitious SDK or generic mock-hardware grant. The CAD mass (~20.316 kg) differs from the documented hardware 40 kg; physical mass/tire calibration remains separate.
- **How to do it:** `scripts/provision_myagv_pro.py` pins `elephantrobotics/agv_pro_ros2` at `d3851c768d2e887eef7ad79fc4dcb9a928b070ac`; `robot_lab_utils/myagv_pro.py` owns explicit adaptations. Keep source assets/recipes/negative traces on SSD. Portable real rollers or a backend-supported directional contact model are required before enabling another engine; do not reuse Fortress XML there as if it worked. Preserve conservative wheel limits, joint/body feedback, geometric corner mapping and proper source mounts. DWB RotateToGoal and goal-checker XY thresholds must match; source-specific Jetson sampling budget is intentional.
- **Done when:** source-backed robot launches from the GUI with real motion/joints/sensors/Stop on each advertised backend, compatible pipelines are implemented and exact limitations are visible. Current Gazebo Display and one Nav route have receipts; other backend adapters stay gated until implemented.
- **Validation later:** localization/2D/3D acquisition missions, additional map/backend routes, furnished obstacles, loss/reset histories, hardware SDK operation and calibrated dynamics. Keep other implementation tasks behind this new user priority.

### R14.3 — Elephant myAGV Pro mobile manipulator

- **State:** partial, experimental · **Priority:** I0a (after R14.2, user request 2026-11-12) · **Owner:** codex for the F100 and planning follow-up
- **Depends on:** R14.2, existing R5.7/R5.9 arm adapters.
- **Goal:** add the official arm-equipped myAGV Pro as a complete mobile manipulator alongside the base.
- **Have:** `asset_elephant_myagv_pro_pro630` (`provision_myagv_pro.py --variant pro630`): the roller base plus the official myCobot Pro630 from `elephantrobotics/mycobot_ros2` `d42ff61a` (BSD-2-Clause, the URDF the official MoveIt package uses), composed by `robot_lab_utils/myagv_pro630.py`. Source kinematics, axes, limits, collisions and meshes; declared estimates for arm mass (8 kg), speeds (1 rad/s), effort caps and the deck mount. Bounded position control, Stop, heartbeat watchdog and base interlock on the shared URDF articulation contract, with per-profile `base_motion_limits`. Generic GUI *Mobile arm* page (`robot_lab_gui/urdf_arm_controls.py`) for any profile whose articulation declares `panel: arm`; family `elephant_myagv_pro630` under Mobile manipulators / Arm on omnidirectional base. Drive and arm screens pass in PyBullet, MuJoCo and Isaac; the GUI page passed live on PyBullet ([evidence](docs/status/evidence/myagv-pro-2026-11-12/README.md#myagv-pro--mycobot-pro630-r143-2026-11-13)).
- **2026-11-20:** `asset_elephant_myagv_pro_pro630_f100` uses the official
  Pro630 F100 mount and linkage from `mycobot_ros2` `b8176c2`. The MuJoCo
  Display variant has one finite-effort jaw actuator, five passive followers
  and **Open / Close** in Mobile arm. Both Pro630 variants have bounded
  MuJoCo **Plan pose → Execute plan** for the source flange. The F100 is held
  while planning arm motions. [Guide](docs/tutorials/myagv-pro.md#use-the-f100-gripper-in-mujoco) · [GUI gripper receipt](docs/status/evidence/r143-pro630-f100-2026-11-20/README.md).
- **Remaining implementation:** source grasp-frame and object custody for the
  F100, pick/place and the navigate–reach–grasp pipeline. Extend the physical
  linkage to additional backends before exposing those modes.
- **How to do it:** preserve the vendor flange and follower transforms, one
  shaft effort budget and the real measured linkage. Build on the existing
  source planner and R5.9 grasp/release contract.
- **Done when:** met for the composed robot and bounded arm control. Gripper and planning are follow-ups under R5.8/R5.9.
- **Validation later:** navigation/reach/pick/place transport, contact/calibration and other backend/map missions remain separate measured acceptance.

### R14.4 — Contributor integration course

- **State:** implemented (2026-11-13) · **Priority:** I0b (after R14.3, user request 2026-11-12) · **Owner:** claude (from codex, on the user's request)
- **Depends on:** current source-backed integration recipes.
- **Goal:** explain how a new contributor adds robots, maps and related components, including normal GUI integration.
- **Have:** beginner course part 2 in `docs/learn/`: [9 · Extending the lab](docs/learn/09-extending-the-lab.md) (prerequisites, file map, seven-stage integration path, what "supported" means), [10 · Adding a robot](docs/learn/10-adding-a-robot.md) (source pin/license, model checks, declared estimates, profile fields, feature-gated modes, articulation, provisioning, screens, troubleshooting), [11 · Adding a world and map](docs/learn/11-adding-a-world-and-map.md) (SDF, generated MJCF, occupancy generation, spawn/initial pose, per-simulator checks) and [12 · Putting it in the GUI](docs/learn/12-putting-it-in-the-gui.md) (families/variants, taxonomy, experimental modes, control pages and how to write one, algorithms and dispatch, docs library, evidence/roadmap/ledger, checklist). Worked example: myAGV Pro and Pro630. Listed open in the GUI docs library (*Extending the lab*), the course map and the tutorial index; diagrams rendered.
- **Remaining implementation:** none.
- **Done when:** met; chapters are reachable in the GUI and the tutorial index and use only real paths and commands.
- **Validation later:** clean-host contributor walkthrough (a new person adds a robot and a world by following the chapters) remains under R9.

### R14.5 — Robot status markers in the GUI selectors

- **State:** implemented (2026-11-13, user request) · **Priority:** I4a · **Owner:** claude
- **Goal:** the robot drop-down lists show, at a glance, whether a robot works (measured), works only in
  experimental mode, or is display-only, and in which modes.
- **Have:** `robot_lab_utils/robot_status.py` classifies each robot per selected simulator from recorded
  evidence only (newest generated support matrix, `asset-runtime-support.yaml`, profile `measured_screens`)
  and the profile's modes/controls. `robot_lab_gui/status_markers.py` decorates the family and variant
  popdown rows (symbol, colour, mode letters; values stay exact IDs); a status line and legend sit under
  the selectors. `measured_screens` added for measurements after the last matrix (lab amphibious/snake/USV/
  tracked, WAM-V, BlueROV2, Summit-STEEL, octopod, TurtleBot 4 Lite cleaning, myAGV Pro variants and Pro630).
  Tkinter `focus_get` crashed every focus-loss stop check while a list was open; `workspace_ui.focused_widget` fixes it.
  [Guide](docs/tutorials/gui-quickstart.md#read-a-robots-status-in-the-lists).
- **Remaining implementation:** none. Regenerate the support matrix (`scripts/r9_3_support_matrix.py`) to fold
  `measured_screens` into it when the evidence index is next refreshed.
- **Done when:** met.

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
- **Done 2026-11-14 (implemented, experimental):** profiled with
  `profile_period_s:=`. Changes:
  - static flex culling: identical contacts, step 2.55 → 1.66 ms;
  - each tick lasts exactly the simulated time it advances (240 Hz ticks of
    two 2 ms steps had capped every world at RTF 0.96);
  - oversleep is paid back from the next sleep;
  - scan and camera run on their own thread from state snapshots.

  Live nav RTF: Labbot celisca_floor_1 0.675 → 0.806, Bumperbot 0.832,
  celisca_floor_2_furniture 0.989, hospital 0.344 → 0.431, nav_obstacle
  0.983 ([evidence](docs/status/evidence/p4-mujoco-culling-2026-11-14/README.md)).
- **2026-11-20:** a conservative cached shortlist reduces repeated AABB
  work for large static-flex worlds. The final overlap predicate and meshes
  are unchanged. Compiled wall checks match contacts and trajectory. On a
  208-bound, 32-geom culling-only benchmark, median calculation time drops
  from 345 to 85 μs. The live loop profiler now reports `static_flex_culling`
  separately. Celisca and hospital navigation RTF still need remeasurement.
- **Remaining implementation:** the large worlds stay below real time.
  MuJoCo updates every static flex vertex and element each step: 3.7 of 4 ms
  per tick on celisca_floor_1, 7.7 ms on the hospital. The options that
  remain (fewer collision triangles, SDF or convex static collision) change
  contacts and need their own identity and mission checks.
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
