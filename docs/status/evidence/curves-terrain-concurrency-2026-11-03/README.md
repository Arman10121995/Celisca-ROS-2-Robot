# Curves, terrain and concurrent fleet receipt

Host: this Jetson, ROS 2 Humble, Fortress 6, MuJoCo/PyBullet and the installed
Isaac Sim 6.0.1 aarch64 SDK. Initial baseline `43473387`, implementation in
`97880df5`, reconciled with subsequent commits through `1d8b29a8`. The final
Isaac repair is the commit containing this receipt. Scope is the named source/worktree hashes, configurations
and measured cases, not a universal robot/map/backend certificate.

Raw logs, native traces and failed trials remain on SSD at:

```text
/workspace/molar/robot_lab_runtime/auv-terrain-fleet-20261102
```

Compact actual results and source hashes are checked in beside this file;
`ssd-artifact-sha256.json` identifies retained larger successful-run artifacts.
`trial-index.json` preserves successes and failures without relabeling them.

## Measured cases

| Case directory | Actual task | Measured result |
|---|---|---|
| `auv-curve-4` | Tethys, `marine_tank`, source-checked 1 m flow run-up + 12 m radius / 20° forward curve, explicit Plan/Arm/Execute/braking | Completed in 34.046 s; independent native body/yaw/speed screen passed within the existing 0.6 m envelope |
| `gait-mujoco-7` | Octopod, `gait_low_relief`, explicit terrain enable/Arm, 10 simulation seconds at 0.02 m/s then Stop/hold | 0.1818 m body travel; 0.0024 m hold drift; 26.847 wall s |
| `gait-bullet-8` | Same source octopod/terrain command and independent body/hold screen | 0.1502 m travel; 0.0005 m hold drift; 55.991 wall s |
| `gait-gazebo-11` | Same source octopod/terrain command, native force telemetry and source-static range model | 0.1850 m travel; 0.0003 m hold drift; 111.774 wall s |
| `gait-isaac-13` | Same source octopod/terrain command, actual PhysX contact tensors and collision rays; one `source_gait_controller` state publisher | 0.1926 m travel; 0.0008 m hold drift; 276.177 wall s, within the explicit 600 s wall budget |
| `fleet-planar-2` | One physical three-base `fleet_facility` world; independent alpha and gamma Nav2 routes with serial traffic reservation explicitly disabled | Both actual Nav2 results and independently settled body endpoints passed in 107.375 s; 45.774 s measured overlapping mission activity |
| `fleet-concurrent-5` — partial, full trial failed | Three-base world, two-member four-stop floor queue and independent gamma mission | Alpha physically reached L1 and parked with its correct map and settled body. Gamma completed; 41.893 s mission overlap. Beta's boarding returned Nav2 success while its body remained at the sill; the independent gate rejected it and canceled returns |

The gait screens require >5 cm actual progress, <3 cm hold drift, final tilt
<0.3 rad and the gait actually disabled at Stop. They do not qualify sustained
terrain/recovery, reverse/turn, saved-map navigation, hexapod/centipede or all
backend/task combinations. Body pose supplies the independent observer and
sensor placement; contact/FK odometry receives no simulator XY.

The fleet body checks require <0.15 m XY error, <0.15 rad yaw error, <0.03 m/s
linear and <0.05 rad/s angular rate. The third member remains an observed
physical peer. These are concurrent single-floor missions, not a successful
multi-member lift/parking campaign.

Each gait `source-sha256.json` includes the other backend files for audit
context. Later Isaac-specific edits differ from these broader snapshots;
the successful MuJoCo/Bullet/Fortress runtime/controller/terrain sources match
their recorded hashes. Do not transfer a named case to changed source.

## Retained failures and corrections

- AUV: wrong trial heading correctly failed source bounds; the first tight
  curve exceeded course tracking, and the 6 m curve left its tracking
  envelope. A 1 m flow run-up, conservative 10–30 m radius envelope and
  measured-course sideslip correction replace those initial attempts without
  increasing fin/propeller force limits. Only the gentle 20° case is measured.
- Fleet: first parking-map cold import blocked the control executor and
  tripped feedback loss; parking grids are now prepared before the timer.
  A later scan stall stopped the route. Disabling cameras did not reliably
  help. The 5 Hz shared-camera derivative retains real cameras; a closed-door
  2.7 m goal remained footprint-blocked, while the declared clear landing
  goal 2.3 m passed. Guards were not bypassed.
- The larger four-stop parking wave reached physical L1 but the former
  100 s wall phase budget expired before observed door/map completion.
  The facility now explicitly declares bounded 900/300/180 s total/phase/
  route budgets. A full successful wave under these budgets remains separate.
- Trial 5 passed alpha's actual L1 exit/parking and gamma's route. Beta's
  front caster caught the square moving-platform edge while the levelled
  car was about 2 mm above the fixed floor. Its wheels advanced localization
  without the body: Nav2 reported success at a body X of 2.8765 m against the
  3.65 m boarding goal. The unchanged independent observer rejected it. A
  6 mm / 80 mm approach bevel was tried within the moving car. The original 50 kg platform mass,
  1600 N lift limit, 0.25 m/s speed limit and arrival/feedback/lease gates are
  retained. That candidate passed 35 geometry/facility contracts and a
  valid SDF/build; the failed physical results below supersede it.
- Trial 6 preserves a failed native startup: Fortress's DART `CustomMeshShape`
  rejected the geometry-only OBJ's missing normals, then its ODE collision
  loader crashed. The generated OBJ now supplies explicit unit face normals
  for every expanded vertex and a local material. Closure/convexity is checked
  after welding coincident render vertices at hard normal seams. The corrected
  35-check result and build are archived; the next actual native trial starts
  and reaches physical robot navigation, which does not by itself pass the wave.
- Trials 7 and 8 rejected actual boarding too: the triangle-mesh bevel and
  then the native-box bevel caught Bumperbot near X=3.03 m. Both candidates
  are withdrawn. Trial 8's exact uncommitted world is retained beside its
  result; its generator/test snapshots remain on SSD. The final source
  restores the original flat primitive deck and addresses the loaded-car
  leveling residual directly. Door/floor admission now requires 0.1 mm
  alignment (previously 3 mm); the authored native integral gain is 1000
  (previously 100), retaining P=40000, D=3000, ±800 integral bounds,
  ±1600 N force, 0.25 m/s velocity, mass, geometry and position limits.
  The force-based upstream [JointPositionController](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/systems/joint_position_controller/JointPositionController.cc)
  remains active with `use_velocity_commands=false`. The actual new floor
  wave must still pass all independent body/door/map/ownership checks.
- Terrain: early Bullet rays intersected the robot and failed initial
  support. Robot-excluding ray filters retain actual world collisions.
  Sparse virtual frames had Bullet's implicit 1 kg masses; source mass is
  restored and an actual force/weight test checks floor collisions survive.
- Fortress originally could not provide contact force through the stock
  component. The pinned private Physics serializer exposes actual optional
  engine force/normal/depth; missing data remains invalid. Gait odometry now
  uses measured 50 Hz encoder intervals matched to contact frames rather
  than differentiating 1 ms encoders against older support sets.
- Terrain ray batching avoids exceeding the 80 ms source-static range
  budget. Trunk pitch was initially being interpreted as differential relief,
  creating positive feedback. Ground ranges now use a level source stance;
  mean heave is removed and body tilt remains a separate guard. Invalid
  kinematic samples never integrate odometry. Continuous insufficient support
  or inconsistent velocity beyond 80 ms of simulation time stops the gait.
- Isaac: retained runs expose the duplicate-floor optimization deleting
  low-relief triangles, the legacy query API returning no hits in SDK 6,
  its mandatory `both_sides`/object-hit interface, hidden collision bounds and
  early contact-view initialization. None of these failed trials qualifies
  Isaac walking. Corrections include the root rigid body, finite hidden-guide
  collision bounds, read-only pose views and explicit contact initialization
  against the SDK's active simulation view. The 180 s trial 12 genuinely
  moved but expired before the unchanged 10 simulation-second travel and
  2 second hold screen. Trial 13 passed those same physical criteria with
  an explicit finite 600 wall-second budget; live freshness, support and
  ownership guards are unchanged.
- Reconciliation: later SDK metadata commits introduced a second `/gait/state`
  producer and misplaced process cleanup. The existing source gait controller
  remains the sole control-state owner. Actual SDK diagnostics now publish on
  `/gait/plant_state`; debug Arm flags cannot assert lease ownership or mission
  success, and publishing diagnostics cannot clear the live Kit process.
- Commit `3c388d81` adds a two-member floor-wave producer, not a completed
  physical result. Commit `1d8b29a8` adds unused occupied-recovery helpers;
  those helpers do not qualify occupied recovery. Preserve both contributions
  and complete their actual integration/mission criteria separately.

## Software checks and CI

Seven changed runtime packages build, including the private native Fortress
force serializer. Final reconciled source/physics contracts: **84 passed**,
including single control-state ownership and live-process preservation. Native
GUI/resolver/architecture checks: **29 passed**, including every gait backend
and the new terrain map command. These are software checks, not additional
robot mission receipts.

The unsourced fast tier initially reported 1175 passed, 46 skipped and two
failures. The missing tutorial Run section and a stale assumption that every
deployed training template is history-free/in the student order are corrected.
Fresh-student export order/width/history and unchanged deployed templates are
checked directly. The subsequent complete unsourced fast tier reports
**1177 passed, 46 skipped, 4 deselected**.

The predecessor GitHub run [41530234108](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/41530234108)
failed during collection because ROS-dependent tests had been placed in the
unsourced fast tier. They now remain required in the sourced integration
tier. No deployed policy weights, source certificates or physical limits
were rewritten to pass CI. Commit `57332c2c` subsequently passed GitHub
[run 41656199263](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/41656199263):
core build, fast tier (1166 passed, 40 skipped, 4 deselected), physics
(65 passed), sourced integration (244 passed, 25 skipped) and registry
cross-references. Later lift commits have separate publication results;
CI does not establish a physical lift mission.

## Reproduce on this host

Build the changed packages, including the registry:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
colcon build --symlink-install --packages-select robot_lab_utils robot_lab_maps \
  robot_lab_mujoco robot_lab_pybullet robot_lab_isaac robot_lab_gui \
  robot_lab_bringup robot_lab_registry
```

Use a new SSD output directory and unused ROS domain/transport partition per
owned trial. The producers use the same actual ROS operations as the GUI,
finite durations/log budgets and owned descendant cleanup. They do not kill
unrelated robot sessions. For example, with this receipt directory as cwd:

```bash
export ROS_DOMAIN_ID=198 IGN_PARTITION=robotlab-new-gait-reproduction
export GAIT_BACKEND=gazebo
export GAIT_CASE_DIRECTORY="$ROBOT_LAB_RUNTIME_ROOT/new-gait-reproduction"
/usr/bin/python3 -s producers/gait_backend_probe.py
```

For the slow Isaac case, use `GAIT_BACKEND=isaac` and explicitly set
`GAIT_WALL_BUDGET=600`. The producer caps this finite wall budget at 600 s;
it still requires the same 10 simulation seconds of travel and 2 seconds of
Stop/hold. Inspect actual state publisher identity in its result, not debug
flags or schema availability.

Other producers: `AUV_CASE_DIRECTORY` for `auv_probe.py`;
`FLEET_CASE_DIRECTORY` for `fleet_planar_probe.py` and the still-unqualified
larger `fleet_concurrent_probe.py`. Their `fleet_three.yaml` is beside them.
Never reuse an output directory or treat its old result as a new run.

The ROS system Python needs its installed NumPy/SciPy ABI. Optional engine/
geometry wheels live on SSD; source the project environment. Full initial
source snapshots, logs and failed artifacts remain on the named host. Next
acceptance is in [the checkpoint](../../curves-terrain-concurrency-2026-11-03.md)
and [agent execution guide](../../../REMAINING_WORK_GUIDE.md).
