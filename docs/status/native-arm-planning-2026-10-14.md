# Native fixed-arm planning implementation — October 13–14, 2026

Continued clean published `master` at `f3df4fe`, without another branch or
worktree. The independent child `R5.7.native_planning` preserves Claude's
parent task ownership. Initial implementation precedes broad mission
qualification; the full project remains partial.

## Delivered

- Thirteen installed native arm candidates gain an explicit **Native arm
  planning: MoveIt** GUI option, normal selection/command autofill and saved
  profile/manifest restoration. Existing joint/Hand/Servo/teach controls remain.
- Source-derived full-body URDF/SRDF planning, authored tool sites or labeled
  body origins, joint-reference/anchor transforms, exact capsule unions and
  source collision filters. Free/coupled/unexported models remain rejected.
- Actual upstream MoveIt KDL, OMPL RRTConnect and acknowledged selected-world
  collision scenes. Five-joint arms use position-only IK; six/seven-joint arms
  preserve the requested orientation.
- A real FollowJointTrajectory action over original native position actuators:
  rate/acceleration/position bounds, measured tracking/settling, native contact
  probes, Stop/manual/Servo/reset/heartbeat interruption, source identity and
  bounded inspection/progress deadlines. No live pose-write execution.
- NumPy 2/pybind enum membership fixed so direct native joints retain correct
  radian/metre units and rates. Robust rotation export fixes exact pitch
  singularities without relaxing FK checks.

Read the [operator guide](../tutorials/native-arm-planning.md). The GUI Health
ledger records implementation separately from all-arm/all-map support.

## Focused diagnostics

Four affected packages built. Twenty-two source checks pass (six new
kinematic/shape/path checks plus sixteen existing Panda trajectory checks).
Thirteen actual compiled descriptions match independently evaluated URDF FK
at three source configurations per model; maximum matrix error is below
`1.5e-11`. These inspections establish exported kinematics, not robot missions.

Actual installed Tk command selections and exclusions are recorded in the
receipt. Two real normal-GUI reaches ran with independent native measured-body
TF, actual JointState and actuator/action state:

| Robot / backend / world / task | Requested step | Measured travel | Endpoint error |
|---|---:|---:|---:|
| UR5e / MuJoCo / nav_empty / Display Plan–Execute | +20 mm Z | 19.83 mm | 0.489 mm |
| SO101 / MuJoCo / nav_empty / Display position-only Plan–Execute | +10 mm X | 9.90 mm | 0.147 mm |

Both finish through measured actuator settling and clean owned launch exit.
These are one-shot implementation diagnostics, not the full reach, object,
Stop/loss/reset, repeat or multi-backend acceptance. UR5e's source snapshot
precedes a GUI result-message-only correction; the actuator/export/scene
implementation is unchanged for the later SO101 diagnostic. No prior Panda
source certificate was refreshed or broadened.

## Retained negatives and remaining work

The first numerical run caught incorrect enum-based units; the second catalog
FK run caught the pitch-singularity export. Failed logs remain on SSD. The
first GUI producer's cleanup import was corrected, and its owned orphan was
identified/stopped. The next two UR5e targets rejected because the generic
10 mm padding intersected a shoulder with 3 mm physical floor clearance.
The new adapter explicitly uses 1 mm padding and retains floor collisions;
Panda's existing 10 mm contract is unchanged.

Still open: the other eleven arm missions, held cancellation/watchdog/reset
screens, repeated arbitrary-object pick/place, coupled arm/gripper/equality
exports, torque/dexterous hands, mobile-arm integration and the other three
simulation backends. Static scenes are not future obstacle prediction. Larger
worlds can exceed the bounded native collision admission budget. Source/export
checks and two reaches cannot close `R5.7` or full platform acceptance.

Raw logs, exports, traces and failed trials remain under
`/workspace/molar/robot_lab_runtime/native-arm-planning-2026-10-13/`.
The [receipt](native-arm-planning-receipt-2026-10-14.json) binds source/artifacts
and separates diagnostics from pending qualification.
