# Remaining work: agent execution guide

Updated October 27, 2026, from `f75dc60`. This is a documentation/source and
recorded-artifact reconciliation; it introduces no new runtime qualification.
Read this guide after [AGENT_HANDOFF](AGENT_HANDOFF.md) and the
[task ledger](status/platform-status.yaml). The [roadmap](../ROADMAP.md) owns
acceptance. Earlier receipts retain their exact source and mission scope.

The platform and R10 extension are **partial**. October 26 completion labels
based on tests, metadata and world loading are superseded by this reconciliation.
The existing controllers, tests and successful named missions remain useful.
Simulator estimation, saved SLAM, volumetric routes, fleet traffic and Isaac
contact trials can proceed without physical robot hardware. Physical Bumperbot
HIL, real sensor calibration and a physical joystick trial are separate work.

## Working rules and first commands

Work directly on `master` in this repository. Claim one concrete task with
`owner`, `state: active`, scope and next action before editing runtime code.
Existing runtime ownership remains in the ledger; this documentation change
does not take over another agent's runtime assignment. Preserve source assets,
old failed runs, calibration contracts and valid scoped certificates.

```bash
cd /workspace/molar/ros_ws/bumperbot_ws
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
git status --short --branch
git rev-parse HEAD
run_dir=$(mktemp -d "$ROBOT_LAB_RUNTIME_ROOT/remaining-work-20261027-XXXXXX")
```

Keep checkouts, caches, generated assets, logs, bags and build output on the
mounted SSD. Read [STORAGE](STORAGE.md); the PX4 relocation is already complete.
Use PX4 `-d` unattended, bound trials and log sizes, and clean up only owned
processes. Ground/marine Humble plugins use **Fortress 6.18**; native PX4 uses
**Harmonic 8**. A scene loading in one version does not qualify another ABI's
controller or hydrodynamics.

The user's order remains **implementation first**, followed by broad validation.
Finish missing controllers, dependencies, launch and native GUI paths before
running large matrices. Build/syntax checks and a bounded diagnostic that finds
an integration defect are implementation work. Limits, neutral enablement,
Stop, ownership and watchdogs belong in every initial implementation.

Module names below are relative to their Python package directories:

| Area | Source directory |
|---|---|
| Shared fleet, marine, gait and asset utilities | `src/robot_lab_utils/robot_lab_utils/` |
| Native aerial/PX4 adapters | `src/robot_lab_adapter/robot_lab_adapter/` |
| GUI panels and session wiring | `src/robot_lab_gui/robot_lab_gui/` |
| MuJoCo contact/actuation hooks | `src/robot_lab_mujoco/python/robot_lab_mujoco/` |
| Benchmark workload contract | `src/robot_lab/robot_lab_benchmark/robot_lab_benchmark/` |
| Launch entry points and shared configuration | `src/robot_lab_bringup/{launch,config}/` |

## Baseline to preserve

- Shared Gazebo Bumperbot/Labbot sessions, per-member Drive/Nav2, bounded queues,
  sweep allocation, initial ground kernels and an optional Open-RMF adapter exist.
  October 20 traces contain two completed queued goals and an RMF traffic stall.
- Complete ROV and Tethys AUV have native propulsion and five marine worlds.
  Simulated pressure/DVL/sonar, sensor odometry, real RGB-D RTAB-Map integration
  and estimated 3D Plan/Execute are implemented. A bounded ROV route completed;
  the retained marine map-save trials failed and require a successful follow-up.
- Complete hexapod, octopod and centipede have source gait/PD, measured contacts,
  contact-kinematic odometry and explicit experimental higher-task composition.
  Gait-panel task options do not establish saved-map or navigation acceptance.
- Native single-X500 flight, selected wheeled SLAM/navigation, Panda manipulation,
  MOLAR articulation and the first exported Go2 training run have separate
  historical evidence. Keep each claim attached to its named revision/cell.

Current host inventory: **140 robot profiles / 75 selectable families** and
**51 world profiles / 39 families**. These are availability counts, not counts
of working robot missions. The last map-registration commit added only
`terrain_heightfield`; the fourteen registered 3D entries were not fourteen
new worlds. Recompute counts after repairing the installed manifest.

## Ordered implementation queue

| Order | Task IDs | Deliverable | Required completion evidence |
|---|---|---|---|
| 1 | R3.6 | Restore missing official mobile/Panda profiles and prevent provisioning losses | Preserved existing IDs, valid resources, normal GUI families and exact commands |
| 2 | R10.1–R10.3 | Resolve fleet/RMF traffic and complete coverage/lifecycle behavior | Actual conflicting routes, queue outcomes, separation, loss and scoped reset |
| 3 | R10.3 / R10.8 | Connect multi-FCU runtime to Launch and native GUI | Distinct vehicles/FCUs/ports/frames; explicit actual flight and loss outcomes |
| 4 | R10.4–R10.5 | Complete underwater saved mapping, reload and bounded routes | Acquired map, independent estimator inputs, saved/reloaded mission and body/depth/contact traces |
| 5 | R10.6–R10.7 | Complete multi-leg motion-to-mapping-to-navigation workflows | Real gait, sensor/odometry data, map save/reload and obstacle goals |
| 6 | R10.7 | Complete additional snake/tracked/surface/amphibious families | Source-matched physical actuation and class-appropriate missions |
| 7 | R10.8 / R4 / R8.3 | Finish extension GUI defaults and real experiment workloads | Normal GUI commands, real execution/cancel/reset and measured result artifacts |
| 8 | R6 | Complete world imports, occupancy, actors and terrain workflows | Installed geometry/resources plus class-specific simulator routes |
| 9 | R5 / R7–R9 / R11–R12 | Resume remaining original platform work | Exact criteria and measured comparisons below; release remains partial |

## 1. Restore the installed catalog before adding more assets

Inspect `scripts/provision_extension_assets.py`,
`robot_lab_utils/installed_assets.py`, `asset_groups.py`,
`src/robot_lab_bringup/config/asset_groups.yaml` and the SSD
`external_assets/installed/{robots,registry_robots}.yaml` manifests.

The six absent official profiles are `asset_turtlebot3_burger`,
`asset_turtlebot3_waffle`, `asset_turtlebot3_waffle_pi`,
`asset_turtlebot4_standard`, `asset_turtlebot4_lite` and `asset_husky`.
Their asset directories remain on the SSD. The historical Panda profile
`menagerie_franka_emika_panda` is also absent. Compare the October 5 installed
snapshot and later vendor/Panda qualification records before restoring metadata;
do not overwrite newer actuator, sensor, family or support fields.

Back up all installed manifests and the integration report on SSD. Prefer
source-scoped `--kind robots --source <source_id> --no-download` repairs using
the existing cached source and current provisioner. Review the result before
publication. Source IDs are `turtlebot3_vendor`, `turtlebot4_vendor`,
`husky_vendor` and `mujoco_menagerie`. Unscoped provisioning currently rewrites
the final manifest from that run's successful imports; implement a preservation
or explicit-removal policy so a failed/omitted import cannot erase working IDs.
Do not assume a backup predates every later vendor integration.

Compare before/after IDs and resource paths, refresh the normal GUI and verify
family/variant selection, backend/mode filtering, algorithm defaults and command
autofill. Missing controller qualification remains visible. Do not add download
buttons or count component URDFs as complete independent robots.

## 2. Finish fleet traffic and cooperative missions

Continue in `robot_lab_utils/{fleet_controller,fleet_missions,fleet_motion,
fleet_session,fleet_rmf_adapter}.py`,
`src/robot_lab_bringup/launch/shared_world.launch.py` and
`robot_lab_gui/fleet_tab.py`. Read [the fleet tutorial](tutorials/fleet-and-swarm.md).
Reuse the installed official Open-RMF runtime and existing Nav2 actions.

Preserve lifecycle activation checks, task identities, bounded queues,
terminal-result cancellation barriers, physical arrival/settling checks and
operator-generation guards. Stop/reset clears owned work; pause preserves
explicitly resumable work. A rejected or failed task must remain visible.

Resolve the measured RMF separation/traffic stall. Review interaction between
the conservative serial route gate, RMF's coordinated member motion and timed
departures. Implement conflict recovery and starvation/deadlock handling while
preserving scan and peer clearance. Respect the installed Humble API: system
NumPy 1.x (`/usr/bin/python3 -s`) and a one-argument patrol callback returning
`Confirmation`. A logical RMF graph anchor is not physical charging equipment;
battery, charging, doors and lifts remain unknown/unavailable without adapters.

Deliver an actual two-member conflict mission, queue pause/resume/cancel,
member loss/rejoin, coverage completion and isolated reset. Record both
estimated and simulator-body trajectories, contact/separation observations,
real action results and task journals. Extend to 3–8 members and further maps
only after the initial workflow works. Review other backend/model adapters
against R10.1 acceptance; do not infer them from ROS namespaces.

## 3. Wire aerial swarms into normal operation

Runtime candidates exist in `robot_lab_adapter/{aerial_swarm_runtime,
aerial_swarm_controller,aerial_swarm_model,px4_swarm_worker}.py`. Audit their
unfinished integration before running them. Add the missing bringup launch
entry and native GUI session/control page; integrate with `lab_tabs.py`, the
normal map selector, exact command preview and session shutdown.

Each vehicle needs its own entity binding, FCU instance/system ID, MAVLink
port, frame/topic prefix, rootfs/log directory and lease. Share one Harmonic
world and retain the single-X500 instance-0 exclusion. Check source geometry,
actual supported spawn heights and resource budgets; missing occupancy or an
unsafe takeoff corridor gets a specific capability/error explanation.

Provide neutral Connect, selected/group controls, seven familiar latched
directions, altitude up/down, reduced yaw increments, clearance check,
explicit Takeoff, Hold and Land. Reject stale generations and late queued
takeoff requests. Implement loss handling and collision-aware formation/group
commands without body pose writes. Record actual independent takeoff, hover,
relative movement, Hold/landing and loss; multiple armed FCUs do not establish
flight. Reuse [native X500](tutorials/px4_x500.md) contracts and upstream pins.

## 4. Complete marine sensing, saved maps and navigation

Continue in `robot_lab_utils/{marine_sensors,marine_navigation_model,
marine_navigation,marine_pipeline,marine_camera_relay,marine_controller,
auv_controller}.py`, the marine/AUV launches and `marine_controls.py`.
Read [ROV](tutorials/marine-rov.md) and [AUV](tutorials/tethys-auv.md) guides.
The existing five worlds are sufficient to implement this workflow; it does
not wait for terrain-provider credentials, a physical pool or real sonar.

Preserve Fortress 6.18 mass/drag conventions, source propeller/fin/thruster
bounds and the distinction between hovering ROV control and flow-dependent
AUV cruise/coasting. Label authored noisy DVL/pressure/sonar as simulated
sensors. Keep estimator inputs and simulator-truth diagnostics separate;
measure drift and covariance against independent body observations.

Complete the actual RGB-D -> RTAB-Map -> occupied cloud/database path, map
save, read-only source backup, reload/localization and obstacle route. Preserve
source-fixed estimated-camera frames and capture timestamps; using the truth
body TF in the estimator/SLAM path leaks pose information. Retain the ROS-core
library environment helper and padded-cloud `read_points_numpy` repair. Older
failures include a missing RealSense-linked `/usr/local` library and a padded
XYZ/RGB conversion crash. Confirm installed executables have execute permission.

Use existing bounded 3D planning and source/observed collision checks. Make
unsupported AUV turns, depth/clearance envelopes, stale sensing and interrupted
ownership explicit. Demonstrate save/reload/route/Stop/loss first on a named
ROV world, then applicable AUV and reef/harbor/cave/wreck missions. Other
simulators need real water-force/sensor adapters; static scene conversion
alone leaves those modes unavailable.

## 5. Complete multi-leg higher modes and additional forms

Continue in `robot_lab_utils/{gait_model,gait_controller,gait_odometry,
gait_pipeline,gait_sensor_relay}.py`, MuJoCo `gait_contacts.py`, the gait
launch and `gait_controls.py`. Preserve source masses, CAD axes, actual leg
counts, joint/effort limits, massive roots, contact guards and reset barriers.

For each complete hexapod/octopod/centipede: finish forward/reverse/turn/Stop,
then measure contact-kinematic odometry and mounted lidar/RGB-D. Correct slip,
timestamp/reset, frame and velocity issues before saved 2D/3D SLAM and Nav2.
An experimental Gait task selector remains a candidate until its actual mission
works. Verify source-matched schedules and consumption of GUI phase/spine
settings. Apply realistic footprints, speed/progress limits and foothold/terrain
constraints. Preserve simulator-pose visualization as a separate branch.

Add complete source-pinned or authored snake, tracked, surface-vessel and
amphibious families with their actual joint, contact, buoyancy or track laws.
Keep complete variants/components nested. A tracked robot needs a real track
contact implementation; a visual belt around driven wheels does not establish
that contract. Port only compatible controllers to additional backends.

## 6. Finish maps, extension workloads and the older queue

For R6, use `scripts/{integrate_external_worlds,provision_extension_assets}.py`,
map conversion/occupancy/terrain tools and the native Worlds tab. Preserve
Celisca's 20% resize, detailed-floor metre conventions, furniture collision,
selected-world spawns and existing exports. Install complete, licensed worlds
with real resources and reviewed occupancy origin/scale/connected regions.
Finish terrain-provider/manual workflows, attribution and height conventions;
Isaac contact tests are simulator work. Add class-specific actor, multi-floor,
terrain and aerial missions. Metadata-only diverse-map tests do not create worlds.

For R10.8/R4/R8.3, connect real fleet/marine/gait/aerial workload producers to
`robot_lab_benchmark/workload_contract.py`, experiment execution and GUI status.
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
