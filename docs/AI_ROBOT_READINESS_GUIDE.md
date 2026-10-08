# Finish robot readiness with reproducible physical evidence

Updated October 8, 2026. The user's order is **motion → SLAM → navigation**
for every complete, compatible source variant. This guide supplements
[the handoff](AGENT_HANDOFF.md), [patch guide](PATCH_EXECUTION_GUIDE.md),
[extension guide](ASSET_EXTENSION_GUIDE.md) and
[task ledger](status/platform-status.yaml). Work on the existing `master`.
Read the current ledger before editing; preserve other agents' files and
claim the exact scope. R6.7 terrain remains separately owned.

## Start from the actual installed state

Source `scripts/ssd_env.sh`, ROS Humble and `install/setup.bash`. Large
checkouts, policies, generated models, builds, traces, bags and logs belong
under `/workspace`. Check the installed module's bytes against the source;
`--symlink-install` can still leave copied Python files on this host.

List the complete families and their exact selectable variants. Read each
profile's `drive`, native model, sensors, `supported_modes_by_simulator` and
runtime screens. Components stay beneath their parent; a gripper/forearm is
not another mobile robot. Choose one robot/backend/controller contract at a
time. Model availability and passing import tests do not establish motion.

## Stage 1: motion and interruption

Prefer a maintained, pinned upstream description/controller/checkpoint. Keep
the original model and notices; record every lab derivative. Match joint
order, axis signs, units, frame transforms, masses, actuator transmission,
limits, control period and policy observations. Do not transplant a policy
solely because another robot has the same number of legs.

Use the existing physical implementations first: differential wheels,
Ackermann/rear steering, four-wheel steering, passive-roller mecanum, Husky
side-group motors, native Panda actuators/fingers and native PX4 rotors.
For policies, the official [Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym)
provides MuJoCo deployment configurations; [MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground)
provides additional locomotion environments. Pin and inspect the actual
checkpoint/model/config files and terms before adapting either. The downloaded
October 8 source review is an input inventory, not a locomotion result.

Qualify the class's real movement:

| Class | Required motion before higher modes |
|---|---|
| Wheeled base | Forward/reverse; both turns; commanded steering/strafe patterns where mechanically applicable; changing wheel/steer joints |
| Humanoid/quadruped | Upright startup; sustained commanded walking/reverse/turning; finite effort/state; bounded fall handling |
| Fixed arm/hand | Physical joint/TCP/finger motion; limits; collisions; bounded force/contact for grasp claims |
| Mobile manipulator | Base plus arm/hand control; enlarged footprint; stable combined motion and mass distribution |
| Drone | Actual rotor-driven takeoff/hover/motion/altitude/landing/disarm; command-loss behavior |

Record neutral enable, release, Space/Stop, publisher loss, reset/relaunch,
fresh independent body/joint state and owned cleanup. Bound drift and turning
translation as well as angular rate: the old Isaac Husky yaw-only trial had
an apparently good yaw rate while crawling 0.69–0.88 m. Keep that negative.
Pose writes, base attachment, open-loop odometry or PASS strings cannot prove
physical driving. Reset pose writes are allowed only as the explicit measured
reset operation; driving must come from actuators/contact physics.

`scripts/qualify_robot_drive_gui.py` records normal GUI Drive trials for
side-group/differential bases. It observes `/odom/ground_truth`, wheel joints,
real scans/depth and TF mounts, and retains the exact generated command and
pre-trial source hashes. Extend a probe for the model's actual joint topology;
do not borrow another robot's wheel names or sensor proof.

## Next humanoid policy adapters

The pinned [October 8 Unitree source review](status/evidence/extensions-finish-2026-10-08/policy-sources/source-review.json)
records real checkpoint/configuration/native-model hashes at
`276801e46c5d433564f24658bac64f254b7d2d4b`. Start with these exact native
models and the upstream `deploy/deploy_mujoco/deploy_mujoco.py` contract.

| Model | Actions / observations | Physics / decision period | Checkpoint |
|---|---|---|---|
| G1 | 12 / 47 | 2 ms / 20 ms | `deploy/pre_train/g1/motion.pt` |
| H1 | 10 / 41 | 2 ms / 20 ms | `deploy/pre_train/h1/motion.pt` |
| H1_2 | 12 / 47 | 2 ms / 20 ms | `deploy/pre_train/h1_2/motion.pt` |

These are lower-body policy contracts, not arbitrary full-body URDF controllers.
Read XML actuator/joint order and each configuration's gains, damping, nominal
angles, action scale and command/observation normalization. The upstream
example initializes forward velocity to 0.5 m/s; a Robot Lab adapter must
initialize command to zero and use fresh operator input, bounded startup,
watchdog and explicit fall handling. The existing
`humanoid_policy_controller.py` wraps Berkeley Humanoid Lite, not every
humanoid. Preserve that robot's working controller while adding a separate
model-matched adapter. Do not replace its checkpoint or gains with Unitree's.

The reviewed RL Gym snapshot lacks its configured Go2 checkpoint; do not
manufacture one from another robot. Existing Robot Lab Go2/BHL policies and
recorded limitations remain the baseline. The official
[Unitree MuJoCo learning repository](https://github.com/unitreerobotics/unitree_rl_mjlab)
and [MuJoCo Playground locomotion registry](https://github.com/google-deepmind/mujoco_playground/blob/main/mujoco_playground/_src/locomotion/__init__.py)
provide further model-specific training/deployment candidates. They still
need pinned files, checkpoint/license review and actual controller/physics
acceptance before any GUI mode promotion.

## Stage 2: sensors and saved mapping

Verify sensor mount transforms against the executed model, usable obstacle
ranges, finite depth and actual intrinsics, timestamp/frame ownership and
simulation-clock behavior. An RGB-only camera cannot grant RGB-D SLAM.
Keep estimator inputs explicit: Gazebo Husky uses encoder translation and
IMU yaw rate; the other engines' current EKF input is reference body twist.
That reference experiment is not an encoder/noise accuracy benchmark.

Move through Localization and actual SLAM using the same GUI command path.
Require live accepted 2D poses/occupied cells or changing finite 3D geometry,
then Save Map through the GUI. Inspect YAML/PGM, database integrity/keyframes
and PCD points. Reset and resume motion/mapping while preserving monotonic
time. Reload saved products and repeat longer trajectories before claiming
full mission readiness.

`scripts/qualify_robot_modes_gui.py` runs a bounded actual workflow and export.
When production correctly rejects a missing mode, copy only small provider
metadata to a unique SSD directory and explicitly declare
`--experimental-provider`. Keep absolute original asset paths. Record this
candidate status; never alter the normal store merely to make a test launch.

Worlds → Generate uses actual UPO collision export plus complete static mesh
and heightfield slices. Choose an appropriate height and seed-connected region.
Check floor/deck, ceiling, orientation, metres, obstacles and landmark locations.
The repaired hospital remains a declared static snapshot: all engines fix its
furniture consistently, with original sources retained. Actors, moving fixtures
and upper-floor navigation need separate integrations. A generated PGM or a
dependency hash check alone cannot qualify a world mission.

## Stage 3: localization and navigation

Derive footprint and dynamics from the real model; include all attachments.
Give goals through the GUI/RViz path. Require fresh independent body truth,
physical floor support, final position/heading, obstacle clearance/detours,
stopping, cancellation and a second goal. Native mesh environments require
mesh-aware route/contact checks; the current box-world swept circle is scoped
to box arenas. Record sensor visibility and map/3D registration at the spawn.

`scripts/qualify_world_navigation_gui.py` rejects missing floor support and
records exact source/world/map hashes. `sim_nav_check.py` supports independent
box-world clearance and explicit 0.15 m / 5° endpoint limits. A Nav2 success
message is insufficient: the earlier hospital trial reported success while
falling through missing Gazebo geometry. Preserve that failure and its trace.

Ground Nav2 is appropriate only for compatible mobile robots and connected
traversable regions. Fixed arms use reach/grasp missions; drones need aerial
collision-aware planning. Terrain, stairs and disconnected building decks
require suitable locomotion/planning, not broader mode labels.

## Publish a measured scope

1. Freeze controller/model/sensor/configuration files before a serial batch.
   Isolate ROS domains below 233 and simulator partitions; run one physical
   plant or occupancy generator at a time. Bound duration/log growth.
2. Archive original reports/producers unchanged and hash raw SSD traces/logs.
   Derive extrema from traces in a separate file, retaining their input hashes.
   `scripts/collect_extension_checkpoint.py` does this for the October 8 work.
3. Recompute acceptance from numeric measurements. Include failed attempts
   and current repeats; attach the exact variant/map/backend/source stage.
4. Add immutable reports to `asset-runtime-support.yaml` only for passed cells,
   matching the model, controller, sensors and source fingerprints. Higher-mode
   provisioning uses `apply_recorded_modes`; navigation needs both clear and
   blocked-direct-path evidence. Other maps remain explicit experiments.
5. Preserve historical certificates' source stages. Before updating shared
   guards, run the affected physical Drive branches and normal native Panda
   acceptance. Record which higher modes were repeated and which remain older
   scoped results; changing hashes is not a new mission measurement.
6. Provision/rebuild, then verify normal production GUI availability, default
   algorithms, copied command and a real normal-profile repeat. Update the
   ledger, roadmap, README, checklist, tutorials, support matrix and GUI Health.
7. Run appropriate local tiers and exact remote CI after committing/pushing
   `master`. Keep local tests, CI and robot missions separate. Leave broader
   tasks partial until all their declared acceptance is satisfied.

The remaining extension work includes other vendor wheels/arms/hands and
model-matched legged policies, mobile manipulation, terrain/provider/actor
behavior, wider world missions, flight-world qualification and clean-host
reproduction. Continue those stages before older comparisons/performance
work; retain the explicit gaps in [the checklist](status/CHECKLIST.md).

## Diagnose the remaining Bumperbot hospital endpoint

The matched MuJoCo motor gain/inertia repairs neutral/braking and the original
large heading error. The extended normal GUI route still fails physical position
at 0.175 m even though Nav2 returns success. The 0.03 m estimator-goal experiment
times out at 0.233 m / 152.21°; restore the tested 0.07 m profile. Preserve both
originals and keep the physical 0.15 m / 5° gate unchanged.

Use `scripts/diagnose_scan_map_alignment.py` under Xvfb with sourced SSD/ROS
environments, an isolated domain and one owned plant. It selects normal GUI
Localization, sends no velocity/goal, collects synchronized real scan/body/AMCL
poses and measures endpoint distance to the selected grid's occupied cells.
This is a stationary diagnostic, not a navigation qualification. Check the
selected map's slice height, source lidar mount, observed map/odom/body transforms,
pose/twist integration and command/sensor cadence before tuning AMCL or DWB.
Any accepted controller/localization change needs named normal-GUI body endpoint,
heading, floor and cleanup repeats on the affected backend/map cells.
