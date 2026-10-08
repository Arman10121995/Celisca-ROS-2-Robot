# October 8 extension runtime evidence

Actual scoped trials for Husky, imported worlds, occupancy and shared-control
regressions. The project remains partial. No catalog totals, random metrics or
historical demonstration PASS flags establish a robot mission.

Raw source checkouts, full logs, traces, databases and clouds remain on SSD at
`/workspace/molar/robot_lab_runtime/extensions-finish-2026-10-07/`. The directory
was created October 7; report timestamps/source stages identify the later trials.
[The collection manifest](checkpoint-collection.json) retains exact artifact
bytes/hashes and raw paths. Original producer reports and pre-trial manifests
are copied unchanged; `report-measured.json` / `report-qualified.json` explicitly
identify derived trace envelopes. The evidence index retains the stages and
recomputes the latest exact robot/map/backend/task cell.

## Husky

[Current twenty-mode batch](husky/modes-current/batch-report.json) passed every
producer/root launch exit, with no owned physical plant left after each run.
[The certificate producer](husky/certificate-producer.py) verifies the actually
executed URDF, lab drive/sensor profile, controller/config/backend source hashes
and measured outcomes before granting modes. Original trial source revisions
are `73fcc47` plus the exact pre-trial working-tree hashes. The original batch's
map manifests recorded paths/profiles but omitted map-byte hashes; the separate
[verification receipt](husky/modes-current/map-source-verification.json) checks
that those core map bytes match the unchanged published blobs. It does not
retroactively alter the original manifests. Later production probes hash actual
world/YAML/image bytes before starting.

The final Drive folders are [Gazebo](husky/final-gazebo/report-measured.json),
[MuJoCo](husky/final-mujoco/report-measured.json),
[PyBullet](husky/final-pybullet/report-measured.json) and
[Isaac](husky/isaac-body-feedback/report-measured.json). All four independent
wheels, finite body/floor/tilt, actual sensor calibration/mounts, neutral enable,
forward/reverse/both turns, settled Stop/loss and owned relaunch are measured.
Earlier torque/friction and Isaac yaw-only failures remain. The yaw-only
producer's original PASS is preserved but rejected for excessive translation
by the stronger skid gate; it is not indexed as current successful support.

The [actual normal GUI selections](husky/normal-gui-defaults.json) cover twenty
backend/mode defaults and autofilled commands. The production clear-route repeat
is in [its own report](husky/normal-production-navigation/report.json).
Read [the continuation](../../continuation-2026-10-08.md) for numeric obstacle
results and estimator/reference-input limitations. Reset in the Drive screens
means full Stop/Run; the separate higher-mode workflow proves robot-only reset.
Static-box swept circles are not physical collision/contact telemetry or
all-map navigation, and root cleanup does not certify every child exit.

## World and geometry evidence

- [Fourteen export/dependency/source checks](worlds/registration-review.json):
  actual UPO products, corrected world/furniture/spawn metadata and cache
  dependency/output hashes. This is not fourteen robot missions or full
  landmark alignment/actor/visual qualification.
- [Native Gazebo mesh parity](mesh-parity/report-native-derived.json),
  [actual MeshManager producer](mesh-parity/gazebo_mesh_probe.cc) and original/
  derived JSONL: 22 pairs load, counts/bounds agree; shared-reader largest bound
  difference is 7.11e-15 m. No pointwise topology/texture parity is inferred.
- [Actual corrected hospital GUI previews](preview/report.json): normal embedded
  source scenes, camera controls, unchanged Launch selection and loader cleanup.
- [Rotated disconnected terrain before](terrain-slice/before/) and
  [after](terrain-slice/after/) actual export: before had no free/occupied cells;
  after has 24,734 free / 350 occupied / 1,844 unknown cells. Individual triangles
  preserve separated elevation regions; no terrain traversal is inferred.
- [Actual editable-seed GUI producer](worlds-gui/producer.py) and
  [Generate/Stop result](worlds-gui/gui-worlds-check.json): custom (-6.75,-6.75)
  yields 22,948 free / 1,626 occupied / 37,926 unknown, owned cancellation/complete
  cleanup and unchanged Launch command. Original saved maps are preserved.

Hospital route stages under `navigation/` retain missing-truth, position error,
freefall, dynamic-world interruption and thin-mesh import failures as well as
current passing repeats. Vertical trace checks reject XY success while falling.
Only named ground-floor static routes are claimed; upper floors, actors, long
routes, all-mesh footprint/contact clearance and other robots/maps remain open.
MuJoCo's static thin-mesh fix uses documented [surface inertia](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh-inertia)
for direct world meshes; original robot masses/actuators remain untouched.

## Shared controls and source guards

[Twelve current TurtleBot3 regressions](regressions/turtlebot3/batch-report.json)
and [both TurtleBot4 Isaac plus native Panda regressions](regressions/batch-report.json)
pass actual motion/sensing or physical Hand/MoveIt checks. The
[source-guard receipt](source-guard-revalidation.json) links changed shared hashes
to real current witnesses. It preserves the older 48/40 mode reports and their
source stages; those full matrices were not rerun at this checkpoint.

## Software / publication

The CI-only predecessor **73fcc47d40b8bf03f7ad2fe725da72283f97708c** passes:

- [Required CI 37762602706](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/37762602706):
  25-package core build; 933 fast / four skips / four deselections; 20 physics;
  134 integration / twelve skips; registry validation.
- [Scheduled Full 37762625564](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/37762625564):
  25-package core build; 934 fast / three skips / four deselections; 20 physics;
  134 integration / twelve skips; registry validation.

[Exact run JSON, complete compressed logs and byte hashes](ci/manifest.json)
retain those outcomes. This predecessor only fixes the scheduled registry
PYTHONPATH; it does not qualify the new runtime source. Final local tiers,
publication/source parity and exact runtime-commit CI are recorded separately.

## Policy inputs / remaining acceptance

[Official Unitree policy source review](policy-sources/source-review.json)
records the pinned BSD repository, real G1/H1/H1_2 checkpoint/config/native-model
hashes and absence of its Go2 checkpoint. These downloads have no gait/mode
qualification. Follow [the AI readiness guide](../../../AI_ROBOT_READINESS_GUIDE.md)
for exact model/policy reuse and physical acceptance.

Missing controllers, further compatible maps/long routes, saved-map reloads,
robust legged turning/terrain/recovery, Servo/payloads/other hands and mobile
manipulation, wider PX4 flight-worlds, separately owned terrain/provider/Isaac
contacts, real algorithm comparisons/concurrency/licenses/clean-host missions
remain in [the checklist](../../CHECKLIST.md) and [ledger](../../platform-status.yaml).

## Latest control and software checkpoint

[Current normal arm, Hand, seven-button Drone and shared Drive controls](controls/README.md)
include original physical traces, source guards and retained negative attempts.
The source build completes 25 core packages (optional ORB-SLAM3 excluded).
Fast: 1005 passed, one skip/four integration deselections; GUI: 104 passed;
physics: 27 passed; dedicated-display integration: 147 passed; registry passes.
The earlier desktop compact-window failure, isolated pass and dedicated-display
repeat remain distinct stages. CMake reports missing pytest discovery on this
host; the recorded direct tier commands actually execute pytest. These checks
are separate from mission proof and CI of the forthcoming published revision.
