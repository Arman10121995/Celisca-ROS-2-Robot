# Remaining work: agent execution guide

Updated October 27, 2026 after [the native continuation](status/native-workflows-2026-10-27.md) on `master`.
Read [the current delivery](status/workflows-2026-10-27.md),
[actual receipt](status/evidence/workflows-2026-10-27/README.md),
[handoff](AGENT_HANDOFF.md) and [ledger](status/platform-status.yaml).
The roadmap owns full acceptance. Earlier certificates retain their exact
source/model/map/backend/mission scope; initial implementation is separate.

The seven missing registrations are restored. Do not repeat the completed
catalog repair or rebuild the native aerial launch, fleet queue, map export,
marine sensor or gait navigation integrations. Preserve their current code
and real scoped measurements. Full R10/platform acceptance remains partial.

## Working rules and first commands

Work on the existing `master`, claim a concrete ledger scope before changes,
and preserve other owners' work. No extra branch/worktree without a user request.
Keep all generated assets, checkouts, caches, logs, builds and bags on SSD.

```bash
cd /workspace/molar/ros_ws/bumperbot_ws
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
git status --short --branch
git rev-parse HEAD
run_dir=$(mktemp -d "$ROBOT_LAB_RUNTIME_ROOT/remaining-work-20261027-XXXXXX")
```

Read STORAGE.md. The PX4 relocation is complete: do not repeat it. Use native
PX4 `-d`, bound trials/logs and clean up every owned descendant even if the
launch root already exited. Ground/marine use **Fortress 6.18**; PX4 uses
**Harmonic 8**. Keep ROS domains and Gazebo partitions separate between live
sessions/diagnostics. Do not mix plugin ABIs or share `/clock` accidentally.

Implement first; broad validation follows. Build changed packages and use
bounded diagnostics to resolve concrete integration faults. Limits, neutral
enablement, Stop, watchdogs, ownership and actual terminal handling belong in
the initial code. Tests, counts and received messages never establish a mission.

## Delivered baseline to preserve

- **148 robot profiles / 79 selectable families**, **51 worlds / 39 families**.
  Seven historical vendor/Panda IDs are back; stale code certificates expose
  explicit experimental candidates, while geometry/control mismatches cannot.
- **Fleet:** actual shared-world queues, traffic yielding/resumption, coverage,
  optional Humble RMF adapter and native controls. A position-swap mission and
  a source-bound isolated two-goal batch have measured physical results.
- **Aerial:** native multi-FCU launch/GUI, two to four separate identities,
  frames/ports/leases, fresh takeoff seals, latched inputs, FCU time integration,
  startup settling and owned cleanup. Actual two-vehicle flight/landing exists;
  the nav_empty two-drone range-reference formation/operator-loss case now passes. Other maps/groups/behaviors remain open.
- **Marine:** native ROV/AUV forces; real motion-derived DVL plus separate
  sonar/pressure, spawn-heading-aligned IMU estimation, RGB-D/depth registration,
  actual flushed map export/protected reload/localization and bounded missions.
  The ROV's saved-map route has an independent body trace. AUV maps created
  before the heading-reference fix cannot be reused as corrected maps.
- **Multi-leg:** retained KDL chains and continuous support, native source PD,
  2D/3D sensing/mapping/export/reload, AMCL/Nav2 and native goal/spine controls.
  Hexapod saved-map Nav2 returns success with body motion; octopod exports a
  real populated 3D database. These are scoped diagnostics, not terrain passes.
- **Experiments:** the fleet producer dispatches real routes and independently
  measures body endpoints, speed and settling. The GUI snapshots a fleet preset
  into an isolated queue. Random patterns/simulated completion are removed;
  results require producer/specification/trace identity and actual cleanup.

The new native producer implementation is in `extension_workload.py`, `extension_contract.py` and `extension_result.py`; fleet reset is `fleet_reset.py`. Use `provision_tracked_physics.py` only to provision/reproduce the private SSD tracked engine, and keep it scoped to that launch. Source truth-clock and native range measurements must stay separate from FCU estimates. Gait independent body headings include the configured CAD-to-drive offset, and goals wait for active Nav2 lifecycle. Do not replace these with synthetic passes.

## Ordered next work

| Order | Task IDs | Concrete next result |
|---|---|---|
| 1 | R10.3 | Preserve the measured nav_empty two-drone range-reference formation/operator-loss case; extend other behaviors/maps, 3–4 members and member loss |
| 2 | R10.4–R10.5 | Complete AUV transects/saved localization and marine obstacle/current/loss missions without accepting degenerate registration |
| 3 | R10.1–R10.3 | Complete RMF conflicting/infrastructure routes, coverage, larger fleets and physical contact/loss screens |
| 4 | R10.6–R10.7 | Per-model saved-map goals, terrain/slip/recovery and source-appropriate backend implementations |
| 5 | R10.7 | Preserve native tracked forward/loss/Nav2; finish flipper actuation, snake/surface/amphibious families and source-appropriate backends |
| 6 | R10.8 / R4 / R8.3 | Preserve delivered real native producers and measured fleet batch reset; extend AUV acquisition, seeds, contacts/clearance and wider matrices |
| 7 | R6 and older queue | Remaining reviewed maps, car/perception/manipulation/policy-transfer/comparison/release work below |

## 1. Preserve catalog restoration

Relevant code: `scripts/provision_extension_assets.py`, utilities
`asset_support.py` and `installed_assets.py`, GUI `launcher.py`,
`test_asset_provision_preservation.py`.

The repaired IDs are `asset_turtlebot3_burger`, `asset_turtlebot3_waffle`,
`asset_turtlebot3_waffle_pi`, `asset_turtlebot4_standard`,
`asset_turtlebot4_lite`, `asset_husky`, and `menagerie_franka_emika_panda`.
Provisioning merges existing profiles/registry entities and report siblings
on full as well as source/entry-scoped runs. Use cached source pins; do not
run a full import just to repair one profile. Single-model repair supports
`--kind robots --source mujoco_menagerie --entry franka_emika_panda --no-download`.

Do not rewrite historical qualification hashes. Code-only mismatches can expose
candidate workflows through `allow_experimental_assets:=true`; changed geometry
or control resources cannot. The GUI's experimental toggle, default algorithms,
mode gates and exact command must agree. New runtime acceptance needs new
source-qualified measurements on each claimed robot/backend/task.

## 2. Fleet and real experiment completion

Relevant utilities: `fleet_session.py`, `fleet_controller.py`, `fleet_motion.py`,
`fleet_missions.py`, `fleet_rmf_adapter.py`; GUI `fleet_tab.py` and
`experiment_queue.py`; benchmark `fleet_workload.py`, `workload_contract.py`
and `concurrent_runner.py`.

Preserve unique real spawn/home vertices, timed RMF departures, scoped task
identities and route progress, lifecycle readiness, fresh physical peers, scan/
separation bounds, checked retreat poses and bounded yield retries. Yielding
suspends/resumes the original task; it does not manufacture success or cancel.
Already-separated bodies may retreat from the soft buffer; overlap/closing
commands remain stopped. Do not remove physical obstacles when a member leaves
swarm participation.

Reproduce remaining RMF conflicts with actual two-body missions before adding
larger fleets. Add doors/lifts/battery infrastructure only with real adapters;
energy stays unknown otherwise. Coverage produces source-clear sweep waypoints,
not measured sensor coverage completeness. Record real member/task outcomes,
contacts, raw source poses, loss and scoped reset without fabricated metrics.

Real batches use `robot-lab-fleet-workload --spec ...` through the isolated
runner. Fleet goals name actual IDs and world coordinates. Results bind the
producer/spec/trace, actual tasks and independent settled endpoints. Batch-wide
reset now stops/invalidate its workload and uses `fleet_reset.py` to observe
actual member pose/localization/settling and clock continuity. Preserve occupied
destination refusal and owned cancellation. Do not replace it with a generic
`/robot_lab/reset` call against a fleet.

## 3. Aerial continuation

Relevant adapter files: `aerial_swarm_model.py`, `aerial_swarm_runtime.py`,
`aerial_swarm_controller.py`, `px4_swarm_worker.py`, `px4_ros_controller.py`;
bringup `aerial_swarm.launch.py`; GUI `aerial_swarm_tab.py`.
Read [the operator guide](tutorials/aerial-swarm.md).

Native multi-FCU launch/GUI is implemented. Preserve per-instance locks,
`-d`, IDs/ports/namespaces/frames, actual settled spawn priors, model envelope,
single-use geometry seals, operator generations and group-takeoff rejection.
Settling/integration use FCU simulation time; command and telemetry watchdogs
use wall time. Startup alignment freezes at first accepted Takeoff and never
corrects flight from truth. Cleanup must survive ROS node destruction errors.

Continue from the actual source/trajectory receipt, including earlier failed
checks and altitude drift. Compare FCU height/targets and independent Gazebo
body height through takeoff, hold and group movement. Keep altitude/formation
accuracy distinct from flight/arming. Preserve the measured range-reference
case and actual collision/noise sensor input; extend other behaviors, member
loss, worlds and larger groups without treating GNSS/barometer cases as qualified. Aerial
and ground sessions remain separate simulator plugin ABIs.

## 4. Marine continuation

Relevant utilities: `marine_sensors.py`, `marine_navigation_model.py`,
`marine_pipeline.py`, `marine_navigation.py`, `map_archive.py`, native ROV/AUV
controllers; bringup `marine_simulator.launch.py`, `auv_simulator.launch.py`;
GUI `marine_controls.py`.

Preserve sensor-only estimation with configured initial position/heading. The
native Fortress IMU starts with a spawn-relative heading: align that heading
once to the configured physical hull yaw, including Tethys' CAD forward -X.
Do not feed simulator pose corrections into estimation. DVL motion samples
must not be replaced by sonar-callback zeros. Sonar remains a directional ray.
No native DVL plugin exists in the installed Fortress build; use the explicit
emulation rather than declaring a missing plugin operational.

Preserve positive native added-mass coefficients, source propulsion/fin bounds,
explicit trim and AUV cruise/coasting limits. Ahead waypoints use estimated
heading and the source forward axis. AUV planning requires a forward transect
and clear coasting corridor; arbitrary stop-and-pivot paths remain unsupported.
The AUV camera kit is massless and keeps source collision/force geometry.
Copied worlds must retain source-relative mesh/texture resources.

ROV visual/ICP and AUV conservative geometry registration consume actual depth.
Reject ambiguous/degenerate correspondences instead of permitting false map
corrections. Save uses actual RTAB-Map backup/SQLite nodes and occupied points;
reload uses a protected copy with source-world/estimator contracts. Uncertain
or stale localization does not authorize navigation. Continue actual AUV and
obstacle/current/loss missions, retaining every failed plan/body trace.

## 5. Multi-leg and additional forms

Relevant utilities: `gait_model.py`, `gait_controller.py`, `gait_odometry.py`,
`gait_sensor_relay.py`, `gait_pipeline.py`, `map_archive.py`; native MuJoCo
`joint_pd.py`/`gait_contacts.py`; GUI `gait_controls.py`.

Preserve the complete source chains and their KDL lifetimes. Newly landing
swing feet cannot become continuous stance samples. Actual IMU/contact/joints
produce odometry; truth is visualization/tilt/independent measurement only.
Keep scan/camera fixed extrinsics, map-to-odom corrections and saved initial
poses separate from source body spawning. SLAM map and navigation costmap
subscriptions must both use `/gait/map`.

Source gait schedules, force/velocity/joint limits, physical leg count, massive
roots, terrain spawn rays and reset/ownership barriers remain unchanged. Phase
and spine controls apply to actual targets and require re-arm. Extend actual
saved-map goals to every model, then terrain/slip/recovery and source-appropriate
backend forces/controllers. The centipede is complete; snake/tracked/surface/
amphibious forms remain additional source/controller work, not aliases for
that segmented model.

## 6. Finish maps, extension workloads and the older queue

For R6, use `scripts/{integrate_external_worlds,provision_extension_assets}.py`,
map conversion/occupancy/terrain tools and the native Worlds tab. Preserve
Celisca's 20% resize, detailed-floor metre conventions, furniture collision,
selected-world spawns and existing exports. Install complete, licensed worlds
with real resources and reviewed occupancy origin/scale/connected regions.
Finish terrain-provider/manual workflows, attribution and height conventions;
Isaac contact tests are simulator work. Add class-specific actor, multi-floor,
terrain and aerial missions. Metadata-only diverse-map tests do not create worlds.

For R10.8/R4/R8.3, preserve the delivered Nav2/fleet/native-extension producers,
queue snapshots and result contracts. Extend further cases and AUV acquisition
through `robot_lab_benchmark/workload_contract.py`, execution and GUI status.
Reuse process isolation/resource accounting, source identities and raw traces.
Cancel/reset must invalidate actual work. No randomized metrics or preset PASS.

After extension implementation, resume these existing obligations:

- R5.5/R5.6: car parking heading/remaining detour aborts; 4WS/mecanum patterns,
  mapping/reset and obstacle routes. Keep true body/heading/footprint checks.
- R5.2/R5.3/R12: reliable turns, terrain/get-up and additional robot-specific
  policies. Go2's first run is already trained/exported; measure deployment
  rather than repeating training because an old summary still says ongoing.
- R5.7–R5.9/R11: remaining arms and stable grasps, combined mobile tasks, MOLAR
  sensors/SLAM/Nav2/payload transfer and simulated multi-floor routes.
- R5.10/R7.2: actual mapped-obstacle drone flight and sensor-loss interruption;
  Isaac image noise and Gazebo calibration discrepancies.
- R7: substantive, correctly named algorithms in every requested category and
  fair shared-input comparisons. Preserve existing real offline/live comparisons.
- R8/R9: exact backend/concurrency cells, licenses/clean-host reproduction,
  seven measured comparison tutorials and an evidence-generated release gate.
  Keep existing qualified host/cell results; broad parents remain partial.

## Completion and handoff contract

For every delivered feature, record robot/variant, map, backend and version,
task/algorithm, source/config/asset hashes, GUI-generated command, requested and
applied seed, raw observations, measured outcomes, thresholds, interruptions,
cleanup and retained failures. Store large artifacts on SSD and a small receipt
plus producer/reference under `docs/status/evidence/`.

Build changed packages and verify installed module/launch/executable parity.
Run focused checks appropriate to the change. After missing implementation is
delivered, run the separate broader mission matrix. Preserve acceptance bounds;
process exit, unit tests, import counts or an action's success flag alone do not
establish a physically settled mission.

Update README, ROADMAP, current status/checklist, ledger implementation versus
qualification fields and GUI availability/Health together. Set a task to `done`
only when its declared acceptance is met; otherwise use `partial` with the exact
next action. Keep hardware-only work explicitly separate. Commit on `master`
and push when requested. A handoff ends with what changed, measured evidence,
what remains, the next task and any exact dependency that prevents progress.
