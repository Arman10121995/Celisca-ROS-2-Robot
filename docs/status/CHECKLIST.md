# Robot Lab done and remaining checklist

Updated October 5, 2026. **The project is partial.** A checked item below
describes an implemented feature or a named measured screen; it does not
qualify every robot, map, mode or simulator. The
[task ledger](platform-status.yaml) owns task state, the
[roadmap](../../ROADMAP.md) owns acceptance, and the
[latest measurements](continuation-2026-10-05.md) give exact scopes and limits.
The latest user instruction prioritizes the new extensions before resuming
the older roadmap. Existing stabilization fixes must remain working.

## Implemented and measured

- [x] Keep this repository on `master`; preserve existing work. Large sources,
  builds, models and trial output use the workspace SSD. The verified PX4
  relocation reclaimed 15.79 GiB with compatibility symlinks (storage audit).
- [x] Enlarge Celisca maps by 20%, align map/spawn metadata and repair furniture
  collision geometry. Selected Bumperbot/Labbot furnished-map navigation
  screens pass (P3/R6.1); longer avoidance missions remain below.
- [x] Autofill GUI run commands and choose compatible algorithm defaults,
  including 4WS steering patterns, mecanum and native PX4 Flight. Preserve the
  selected map. Unsupported legged actuation modes retain an explanation.
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
- [x] Download and install 111 Display profiles on SSD: 59 native Menagerie
  models, 48 robot-assets URDF variants/fragments, and four official TurtleBot3/
  Husky descriptions. Normal Launch selection,
  command autofill and Installed Extensions shortcuts work; no operator
  download controls. Two upstream files have no robot links and are excluded.
- [x] Import all 14 dataset worlds and three Gazebo environment examples with
  real dependency resolution, derived MJCF and actual occupancy exports.
  Robot missions, actor behavior and full visual/backend parity remain separate.
- [x] Record live GUI Panda display/state/Run/Stop checks and preserve failures;
  each report names the exact asset, world, backend and source stage.
- [x] Replace invented support/release success with exact hashed measurements,
  retain negative results and block full-release claims on unfinished tasks.
  GUI Health, this checklist, the roadmap and agent guides expose those gaps
  (R9.3).

## Stabilization and original robot goals: resume after the extensions

- [ ] **P2:** Record an actual physical joystick trial: untouched enable,
  independent axes, neutral return, unplug, Space, selection change and close.
- [ ] **P3/R5.1/R6.1:** Complete Bumperbot and Labbot on every Celisca plain,
  furnished and actor map/backend/mode cell. Include a multi-turn obstacle
  route, furniture-adjacent goal, cancellation and second goal. Review
  furniture-aware occupancy alignment instead of replacing maps automatically.
- [ ] **P4:** Measure and improve MuJoCo real-time factor on large maps without
  compromising effort-control timing, collision fidelity or sensing.
- [ ] **R5.5:** Complete Ackermann, rear-steer and anti-Ackermann obstacle/
  slalom, mapping, localization and navigation qualification. Retain each
  drive model's turning constraints and independent body/contact evidence.
- [ ] **R5.6/P5/P6:** Repeat current 4WS Ackermann, in-phase/crab and pivot,
  plus mecanum, across all four backends and required maps/modes. Include
  strict final position/heading, measured contacts, repeated obstacle routes,
  cancellation/second goals, command loss, reset and longer mapping missions.
  Earlier passing screens are not a rerun of every final configuration.
- [ ] **R5.2:** Resolve Go2 stair/terrain traversal and roll-axis recovery;
  existing failed get-up trials remain negative evidence. Qualify robust
  walking/turning and sensor/goal missions before broader mode enablement.
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
- [ ] **R6.5:** Review generated occupancy origin, mesh scale, height and
  seed; cover disconnected free regions, test rotated geometry and navigation
  alignment, then register validated maps. The new shared heightfield converter
  extends projection; its occupancy/terrain missions still need qualification.
- [ ] **R6.6:** Qualify visual/collision/spawn and robot routes across all
  imported worlds/backends. All 100 Fortress example SDFs are downloaded;
  three environment examples are installed, while 97 plugin/robot fixtures
  require behavior/resource review. Preserve original actors/plugins, audit
  licenses and include remaining licensed Fuel environments.
- [ ] **R6.7:** Implement a terrain-generation GUI with SSD output and shared
  provider/attribution manifest. Shared four-backend heightfield conversion
  now exists with measured Gazebo conventions and MuJoCo/PyBullet contact
  screens; verify Isaac runtime contact, deterministic generation and actual
  robot traversal per backend.
- [ ] **R5.7:** Integrate manipulators, joint/trajectory commands, MoveIt2
  planning and GUI controls; measure reachable motions and obstacle avoidance.
- [ ] **R5.8:** Integrate robot hands/grippers, control limits and GUI commands;
  measure closure, grasp/contact, hold and release of actual simulated objects.
- [ ] **R5.9:** Combine a mobile base and arm with consistent TF/controllers,
  navigation-to-object and grasp/place missions through the GUI.

## Complete experiments and release qualification

- [ ] **R6.2/R6.3/R6.4:** Qualify dynamic actors, sensor disturbances, diverse
  indoor/outdoor/elevation worlds and 3D ground/aerial representations.
- [ ] **R4.3/R7.1–R7.8:** Run reproducible matched-input algorithm comparisons
  with actual adapters, recordings, failures and metrics; complete the required
  five methods per category. Numerical kernels or generated scores do not count.
- [ ] **R8.1/R8.2/R8.3:** Complete the backend mission matrix and real isolated
  concurrent experiments with resource limits, cleanup and repeatable results.
- [ ] **R9.1/R9.2:** Verify dependency/model licenses, clean-host installation
  and seven tutorials whose commands and plots reproduce real comparisons.
- [ ] **R9.3:** Regenerate the release matrix as those tasks pass. The current
  matrix intentionally remains partial with full-release gates blocked.
- [ ] **R9.4, optional hardware:** Arrange the actual robot, supervised setup
  and explicit hardware-operation authorization. Simulation work is separate.

Follow [PATCH_EXECUTION_GUIDE](../PATCH_EXECUTION_GUIDE.md) for stabilization
and [ASSET_EXTENSION_GUIDE](../ASSET_EXTENSION_GUIDE.md) for pinned upstreams,
implementation order and acceptance. Claim ownership in the ledger before
changing shared files; record the exact robot/map/backend/task/source and
measured artifacts when checking an item off.
