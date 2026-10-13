# Robot Lab done and remaining checklist

Latest [initial implementation continuation](implementation-continuation-2026-10-09.md):

- [x] Persistent highlighted Drone/Drive direction and altitude click latches;
  click again releases/brakes, Stop/Space clears all shared mirrors.
- [x] Source/scene-bound Stretch navigate + taught arm/hand GUI task editor,
  actual Nav2/body settling, ownership/cancel/loss and SSD event journals.
- [x] Selected-world native X500 swept-envelope takeoff check, ceiling-adjusted
  rise and fresh landed/pose/resource gate before explicit Takeoff.
- [x] Built-in real Nav2 multi-goal queue workload, raw body/action traces and
  source-bound measured result; reset invalidation and requested seed UI.
- [ ] Physical qualification of these new tasks/controls/flight/workloads.
- [ ] Remaining model/backend controllers, global arm/dexterous/object/grasp
  adapters, aerial sensing/mapping/dynamic avoidance, world/provider work,
  simulator-applied seeds and fair algorithm comparisons.

Updated October 9, 2026. **The project is partial.** The user prioritizes
initial implementation before testing/validation. [Current status](CURRENT_STATUS.md)
and [the checkpoint](continuation-2026-10-08.md) link exact physical source stages,
publication and software checks. A checked item below
describes an implemented feature or a named measured screen; it does not
qualify every robot, map, mode or simulator. The
[task ledger](platform-status.yaml) owns task state, the
[roadmap](../../ROADMAP.md) owns acceptance, and the
[latest measurements](continuation-2026-10-08.md) give exact scopes and limits.
The latest user instruction prioritizes the new extensions before resuming
the older roadmap. Existing stabilization fixes must remain working.

## New implementation; validation deferred

- [x] Shared exact-model G1/H1/H1_2 native policy controller, provisioning,
  grouped GUI variants, explicit toggle and Drive command integration.
- [x] Generic source-bounded native position-actuator/coupled-tendon controls,
  GUI joint jogging, measured feedback, Stop/watchdog/reset and SSD registration.
- [x] Native Cartesian press-and-hold jogging on Panda and compatible generic
  position controllers, with authored sites, joint limits and contact prediction.
- [x] Teach measured native arm/hand configurations, save/load source-matched
  sequences and replay with actual settling, bounds, deadlines and interruption.
- [x] Measured Panda cube/pedestal planning scene, acknowledged attach/detach,
  fresh FK/contact requirements and scene-change invalidation.
- [x] Stretch/Stretch 3 native wheels, base/articulation ownership, actual
  source-mounted lidar/RGB-D, truth-bootstrap odometry/IMU and changing footprint.
- [x] Explicit experimental Stretch AMCL, 2D/3D SLAM and Smac2D/RPP Nav2
  stack composition with actual sensor readiness and selected-map save paths.
- [x] Static selected-world 3D aerial Plan/Execute, measured FCU targets,
  continuously checked geometry segments and bounded interrupted waypoint tracking.
- [x] Cached equivalent planar collision reduction for heavy MuJoCo scenes;
  P4 speed/contact measurements remain pending.
- [x] Actual isolated resource-bounded subprocess queue and telemetry, real
  cancel/reset and owned cleanup; remove invented legacy benchmark metrics.
- [x] PX4 single-instance lease, SSD log budget and startup-error cleanup.
- [ ] Complete source-specific controllers for remaining robots/backends,
  dexterous hands, arbitrary-object/mobile tasks, aerial sensors/mapping/dynamic
  avoidance, independently owned terrain/provider work and substantive algorithm
  adapters plus seed/mission/result comparison contracts.
- [ ] Run the focused and broad validation handoff after those implementations.

See [implementation scope](implementation-2026-10-09.md); checks here record
code delivery, not a passed mission or universal support.

## Implemented and measured

- [x] Put all seven directional controls in Drone, share latched state with
  Drive, and reduce default yaw to 0.3 rad/s. Actual owned `nav_empty` flight
  covers neutral input, directions, altitude, release/Stop/loss and landing.
- [x] Add measured Panda tool/target XYZ, **Use current tool pose** and small
  axis target steps with separate Plan/Execute. Actual normal 1 cm X/Y/Z
  targets, rejection/interruption/reset and Hand cube lift/release pass.
- [x] Keep this repository on `master`; preserve existing work. Large sources,
  builds, models and trial output use the workspace SSD. The verified PX4
  relocation reclaimed 15.79 GiB with compatibility symlinks (storage audit).
- [x] Enlarge Celisca maps by 20%, align map/spawn metadata and repair furniture
  collision geometry. Selected Bumperbot/Labbot furnished-map navigation
  screens pass (P3/R6.1); longer avoidance missions remain below.
- [x] Autofill GUI run commands and choose compatible algorithm defaults,
  including 4WS steering patterns, mecanum and native PX4 Flight. Preserve the
  selected map. Unsupported legged actuation modes retain an explanation.
- [x] Redesign Launch with a separate right-hand Drive/Arm/Hand/Drone control
  column and limits; add reviewed robot category/type tags and filters,
  sidebar navigation, Logs and compact setup/command switching at 1024 pixels.
  Preserve exact command, ownership and neutral-input behavior. Final actual
  Tk layout and source-matched normal Drive/Panda regressions are recorded in
  [GUI evidence](evidence/gui-controls-column-2026-10-07/README.md).
- [x] Implement latched/ramped GUI Drive, WASD, joystick neutral calibration,
  immediate Space/Stop and mecanum/crab strafe controls. Software and selected
  live Drive checks pass; physical joystick checks remain (P2).
- [x] Restore BHL/Go2 MuJoCo startup and bounded upright walking on the named
  Celisca floor-1 path. Preserve effort-feedback timing (P1); robust held turns,
  terrain, get-up and other robot policies remain (R5.2/R5.3/P7).
- [x] Implement physical 4WS and passive-roller mecanum drives. Clear-map Nav2
  screens exist for all 16 4WS pattern/backend cells and four mecanum backend
  cells. Later crab precision and obstacle screens have tighter independent
  body measurements; each source stage is recorded (R5.6).
- [x] Repair wheeled reset, monotonic simulation clocks, estimator reseeding
  and mapping history. Selected localization, publisher-loss, reset and
  post-reset movement screens pass; the full matrix remains (R5.6).
- [x] Run real PyBullet mecanum 2D SLAM with accepted poses/occupied cells and
  reset/resume. Run real MuJoCo/Gazebo crab RGB-D mapping with changing finite
  3D clouds. The installed MuJoCo GUI Save Map exports an intact database with
  nine keyframes and a 7,255-point PCD (R5.6).
- [x] Fly the native PX4 X500 through ROS2/GUI on Gazebo Harmonic: takeoff,
  hover, waypoints, Drive, land/disarm and bounded command loss. Actual GUI
  Altitude Up/Down and released holds are measured in `nav_empty` and
  `nav_obstacle` (R5.4). All installed worlds remain selectable (R5.10).
- [x] Add GUI Worlds occupancy generation using the pinned UPO Fortress
  plugin and actual collision-mesh height slices. Real exports exist for
  `nav_obstacle` and both furnished Celisca floors (R6.5). They are previews
  of the seed-connected region, pending reviewed navigation registration.
- [x] Download and install 113 robot profiles on SSD: 59 native Menagerie
  models, 48 robot-assets URDF variants/fragments, and six official TurtleBot3/
  Husky/TurtleBot4 descriptions. Normal Launch selection,
  command autofill and Installed Extensions shortcuts work; no operator
  download controls. Two upstream files have no robot links and are excluded.
- [x] Consolidate robot/map selectors into reviewed complete parent families.
  Preserve exact source variants and keep limbs/grippers nested in Registry.
  Six R2/Valkyrie component topologies are measured inside full source assemblies.
- [x] Add native embedded Registry 3D Preview with actual URDF/MJCF/SDF geometry,
  orbit/pan/zoom/Fit and unchanged launch commands. Nine distinct source scenes
  are rendered across the recorded stages; final control-column repeats cover
  Celisca floor-2 furniture and static/dynamic Room2 geometry. A final live
  Burger/PyBullet preview emits no movement commands and physical Drive/Stop/
  watchdog passes. Texture/GPU/actor and universal asset review remain separate.
- [x] Import all 14 dataset worlds and three Gazebo environment examples with
  real dependency resolution, derived MJCF and actual occupancy exports.
  Robot missions, actor behavior and full visual/backend parity remain separate.
- [x] Record live GUI Panda display/state/Run/Stop checks and preserve failures;
  each report names the exact asset, world, backend and source stage.
- [x] Add official TurtleBot 4 Standard/Lite to normal Launch/autofill and
  measure Display/state/Run/Stop on all four backends in `dataset_room2`.
- [x] Connect TurtleBot 4 physical wheel control in Display and measure GUI
  neutral enable, WASD forward/reverse/turn, Stop and command loss on both
  variants/all four engines in `nav_empty`. Restore original suspension springs
  and repair Isaac's differential-drive watchdog and acceleration timing.
  Eight final source-frame/lidar/RGB-D/Drive checks and forty localization,
  reset/resume, actual mapping/export and clear/obstacle Nav2 trials now pass.
  The normal GUI selects algorithms and autofills commands for all 40 mode
  combinations. Other maps/routes, materials and vendor firmware remain open.
- [x] Add native Panda Arm joint/Home/Stop controls and real MuJoCo position
  trajectories; measure rejection, cancellation and heartbeat loss (R5.7).
- [x] Connect official TurtleBot3 Burger/Waffle/Waffle Pi physical wheels and
  original 360-ray/3.5 m LDS. Twelve final normal GUI Drive/neutral enable/
  WASD/Stop/publisher-loss/joint/TF trials pass across all four engines.
  Preserve original source models, named nav_empty spawn and failed physics
  stages. Forty-eight actual localization/reset/resume, 2D SLAM/GUI-save and
  clear/obstacle Nav2 screens pass. Normal algorithms/commands and a current
  normal MuJoCo obstacle repeat pass; source RGB-only cameras leave 3D SLAM pending.
- [x] Add native Panda Hand open/close/opening/Cancel/Stop and GUI Reset controls.
  Measure real bounded-force cube contact, a 7.57 cm lift and gravity release
  in MuJoCo/`nav_empty`, plus lost heartbeats and reset during closing (R5.8).
  Other hands, objects and manipulation backends remain open.
- [x] Add native Panda MoveIt Cartesian Plan/Execute against actual native
  collision geometry and the selected static world. Measure two physical
  TCP targets, floor/self-collision and unreachable rejection, joint/finger
  plan invalidation, Cancel/Stop/heartbeat loss, reset and clean MoveIt exit
  in MuJoCo/`nav_empty` (R5.7). Attached-object planning and Servo remain open.
  The October 7 actual normal-profile regression after shared spawn changes
  also passes; a second grouped-GUI repeat refreshes the current strict
  planning certificate. Physical errors and all four child exits are archived.
  The final control-column repeat measures 0.00649/0.00639 m and 0.780/0.765°;
  it refreshes the strict certificate including the shared workspace layout.
- [x] Repair Trimesh/NumPy CI compatibility; exact `36f38b5` remote CI passes
  build, fast, physics, integration and registry checks.
- [x] Repair the latest extension CI tier failure. Exact `18ecdd2` passes
  build, fast, physics, integration and registry validation. `dc39922`'s three
  ROS-import failures remain recorded; the unchanged checks run in integration.
  Published `091d388` also passes the 25-package core build, 933 fast checks
  (four skips/four deselections), 20 physics checks, 134 integration checks
  (twelve skips) and registry validation. Later runtime changes need their own
  CI. See [October 7 evidence](evidence/ci-extensions-2026-10-07/README.md).
- [x] Add Husky physical four-wheel skid control and a declared lab sensor/motor
  kit. Four Drive/sensor and twenty localization/reset, 2D/3D SLAM/save and
  clear/obstacle Nav2 screens pass across all engines. Normal twenty GUI mode
  defaults/commands and a production navigation repeat pass. Other maps,
  payloads, vendor hardware and complete estimator/contact qualification remain.
- [x] Repeat twelve TurtleBot3 physical Drive/source-LDS, both TurtleBot4 Isaac
  Drive/lidar/RGB-D and native Panda Hand/MoveIt after shared controller changes,
  before guarded source refresh. Historical higher-mode matrices retain their
  own source stages; these regressions do not rerun every mission.
- [x] Make Worlds seed X/Y editable; measure actual Generate/Stop/custom seed
  and unchanged Launch selections. Fix disconnected terrain triangle slicing;
  the rotated two-hill fixture now exports finite free/occupied cells.
- [x] Repair exactly six pinned escaped hospital fixtures in derived files,
  preserve source/texture/frame provenance, verify native Gazebo mesh bounds,
  fix furniture consistently as a static snapshot and move spawn onto reviewed
  lobby support. Current short routes and retained freefall/import failures are
  recorded separately from remaining world/actor/upper-floor qualification.
- [x] Restore open wall sheets in real occupancy generation and reject v3
  caches. Regenerate all 17 installed extension grids, preserving source worlds
  and measured spawns; registry and 34 actual GUI selections/autofilled paths pass.
- [x] Repeat the short single-floor hospital route for Bumperbot/Labbot on all
  four engines after the v4 repair. All eight pass unchanged physical
  position/heading/floor/freshness/settle gates; original failures remain.
- [x] Remove per-byte camera packing overhead with byte-identical ROS wire
  messages. Four serialization cases and a new native Labbot hospital route
  pass; actual clock windows remain about 0.13×, so P4 is still partial.
- [x] Publish map/camera source `f74b53f` on master with both exact CI runs
  passing: 25 packages, 27 physics, 138 integration / 13 skips and registry;
  89 listed installed Python modules match. Historical failures remain.
- [x] Repair Scheduled Full Test Suite registry imports; exact CI-only `73fcc47`
  passes required and scheduled workflows. New runtime source has separate
  final checks/publication in the October 8 archive.
- [x] Replace invented support/release success with exact hashed measurements,
  retain negative results and block full-release claims on unfinished tasks.
  GUI Health, this checklist, the roadmap and agent guides expose those gaps
  (R9.3).

## Stabilization and original robot goals: resume after the extensions

- [ ] **P2:** Record an actual physical joystick trial: untouched enable,
  independent axes, neutral return, unplug, Space, selection change and close.
- [x] **P3:** Bumperbot and Labbot pass the Celisca mission (detour, cancel,
  resume, near-obstacle goal) on plain and furnished floors 1-2 in all four
  backends, 32/32 (2026-10-12, [evidence](evidence/p3-celisca-missions-2026-10-12/README.md)).
- [ ] **R5.1/R6.1:** Repeat those missions over seeds, add the actor maps and
  the slam/3D modes; one seeded mission per cell is not a repeat study.
- [ ] **P4:** MuJoCo large-world steps are about 2x faster with identical
  contacts (Labbot celisca_floor_1 RTF 0.31 -> 0.675, hospital 0.13 -> 0.344;
  [evidence](evidence/p4-mujoco-step-2026-10-12/README.md)). Real time on large
  worlds still needs the flex vertex update, scan and camera costs reduced.
- [ ] **R5.5:** Complete Ackermann, rear-steer and anti-Ackermann obstacle/
  slalom, mapping, localization and navigation qualification. Retain each
  drive model's turning constraints and independent body/contact evidence.
  The obstacle mission passes in 10 of 12 car/backend cells
  ([evidence](evidence/r55-car-missions-2026-10-13/README.md)).
- [ ] **R5.6/P5/P6:** Repeat current 4WS Ackermann, in-phase/crab and pivot,
  plus mecanum, across all four backends and required maps/modes. Include
  strict final position/heading, measured contacts, repeated obstacle routes,
  cancellation/second goals, command loss, reset and longer mapping missions.
  Earlier passing screens are not a rerun of every final configuration.
  The obstacle mission (detour, cancel, resume, second goal) now passes in
  all 20 pattern/backend cells, one seed each
  ([evidence](evidence/r56-4ws-missions-2026-10-13/README.md)).
- [ ] **R5.2:** Resolve Go2 stair/terrain traversal and roll-axis recovery;
  existing failed get-up trials remain negative evidence. Qualify robust
  walking/turning and sensor/goal missions before broader mode enablement.
- [ ] **P7 (rl_sar):** Go2/A1/B2/Go2W policies walk on `/cmd_vel` in MuJoCo
  ([evidence](evidence/p7-rl-sar-mujoco-walk-2026-10-12/README.md)), Isaac and
  (Go2) Gazebo. Walking robots now navigate with leg odometry: the obstacle
  mission passes for Go2 and A1 in MuJoCo and Isaac and for the G1 in
  MuJoCo ([evidence](evidence/p7-legged-navigation-2026-10-13/README.md)).
  Gazebo Go2, B2 and Isaac G1 reruns, slam-mode mapping and repeats remain.
- [ ] **R5.3/P7:** Resolve BHL held-turn/low-speed stalls and qualify robust
  balance/walk/turn. Audit and adapt model-specific policies for the remaining
  humanoids/quadrupeds, including joint/action/observation contracts and live
  backend actuation. Passive localization is not a walking policy.
- [ ] **R5.10:** Qualify PX4 spawn, resources, floor/ceiling clearance and
  actual flight in each selected world. Add obstacle-aware aerial planning
  and aerial mapping; other drone backends need their own integration/proof.

## Implement the newly requested extensions

- [ ] **R3.6:** Complete per-model license/texture/skin and cross-backend
  runtime checks for the installed Display profiles; integrate remaining
  vendor controller interfaces, then add qualified control modes. All eight
  featured URDFHub models have installed equivalents with actual provenance. Import counts do not qualify joint control, walking or missions.
- [ ] **R3.6, TurtleBot 4:** Preserve the measured four-backend modes, final
  sensors/Drive, reset/resume, saves and clear/obstacle routes. Qualify longer
  routes and additional maps, original materials and vendor hazards/docking;
  hardware/vendor firmware control requires separate integration.
- [ ] **R6.5:** Review generated occupancy origin, mesh scale, height and
  seed; cover disconnected free regions, test rotated geometry and navigation
  alignment, then register validated maps. Editable seeds and actual rotated
  triangle exports now pass; complete map/terrain missions remain open.
- [ ] **R6.6:** Qualify visual/collision/spawn and robot routes across all
  imported worlds/backends. All 100 Fortress example SDFs are downloaded;
  three environment examples are installed, while 97 plugin/robot fixtures
  require behavior/resource review. The six pinned hospital source fixtures
  are repaired in derived variants; qualify remaining physical routes, upper
  floors, textures and actors. Preserve original actors/plugins, audit
  licenses and include remaining licensed Fuel environments.
- [ ] **R6.7:** Implement a terrain-generation GUI with SSD output and shared
  provider/attribution manifest. Shared four-backend heightfield conversion
  now exists with measured Gazebo conventions and MuJoCo/PyBullet contact
  screens; verify Isaac runtime contact, deterministic generation and actual
  robot traversal per backend.
- [ ] **R5.7:** Extend measured native Panda joint/trajectory/Home/Stop controls
  beyond the measured MoveIt Cartesian/static-world screens to Servo,
  attached-object scenes, repeated pick/place and the other arm backends.
- [ ] **R5.8:** Preserve the native Panda physical cube grasp and interruption
  proof. Add model-specific Robotiq/dexterous-hand controls and measure other
  objects, repeated pick/place and the other backends.
- [ ] **R5.9:** Combine a mobile base and arm with consistent TF/controllers,
  navigation-to-object and grasp/place missions through the GUI.

## Complete experiments and release qualification

- [ ] **R6.2/R6.3/R6.4:** Qualify dynamic actors, sensor disturbances, diverse
  indoor/outdoor/elevation worlds and 3D ground/aerial representations.
- [x] **R4.3:** Predeclared A* vs NavFn comparison: thresholds from tuning
  seeds, five evaluation seeds, identical goals, failures explained
  ([evidence](evidence/r43-planner-comparison-2026-10-12/README.md)).
- [ ] **R7.1–R7.8:** Global planning (eight planners) and local planning (five
  controllers) are measured on identical seeded goals
  ([global](../tutorials/global_planning_comparison.md),
  [local](../tutorials/local_planning_comparison.md)). Six path-tracking
  controllers (R7.8) drive through FollowPath and are measured on a model
  plant and in Gazebo ([control](../tutorials/control_comparison.md)).
  Sensor fusion (R7.5) has real attitude and
  pose filters, measured on modelled sensors offline and over ROS
  ([fusion](../tutorials/sensor_fusion_comparison.md)). Perception
  (R7.2), localization (R7.3) and state estimation (R7.4) now run real
  methods, measured offline with known truth: synthetic scenes, ray-cast
  scans along Gazebo trajectories, and noisy position fixes
  ([perception](../tutorials/perception_comparison.md),
  [localization](../tutorials/localization_comparison.md),
  [state estimation](../tutorials/state_estimation_comparison.md)).
  Their runs on simulator sensor data remain. Numerical kernels or
  generated scores do not count.
- [ ] **R8.1/R8.2/R8.3:** Complete the backend mission matrix and real isolated
  concurrent experiments with resource limits, cleanup and repeatable results.
- [ ] **R9.1/R9.2:**
  - Declared dependency licenses are resolved from installed metadata
    (120/125 with the SSD environment sourced; the other 5 are unused
    ROS 1 keys;
    [evidence](evidence/r91-license-inventory-2026-10-12/README.md)).
  - Three of the seven comparison tutorials are measured in Gazebo
    (global, local, control). The other four (perception, localization,
    state estimation, sensor fusion) are measured offline with known
    truth; their simulator-data runs remain.
  - Model/asset attribution is checked against the nearest LICENSE or
    package manifest (`scripts/asset_attribution.py`,
    [evidence](evidence/r91-asset-attribution-2026-10-13/README.md));
    all robot entries agree except px4_x500, whose license is checked
    against the PX4 checkout outside the workspace. A real clean-host
    installation remains.
- [ ] **R9.3:** Regenerate the release matrix as those tasks pass. The current
  matrix intentionally remains partial with full-release gates blocked.
- [ ] **R9.4, optional hardware:** Arrange the actual robot, supervised setup
  and explicit hardware-operation authorization. Simulation work is separate.

Follow [the AI readiness guide](../AI_ROBOT_READINESS_GUIDE.md),
[PATCH_EXECUTION_GUIDE](../PATCH_EXECUTION_GUIDE.md) for stabilization
and [ASSET_EXTENSION_GUIDE](../ASSET_EXTENSION_GUIDE.md) for pinned upstreams,
implementation order and acceptance. Claim ownership in the ledger before
changing shared files; record the exact robot/map/backend/task/source and
measured artifacts when checking an item off. The original Bumperbot/MuJoCo
hospital heading/0.175 m endpoint failures and reverted 0.03 m experiment remain
negative. After the open-wall occupancy repair, all eight short single-floor
hospital routes pass. Two-floor-building, obstacle and other-world missions
remain open; Nav2 success alone remains insufficient.
