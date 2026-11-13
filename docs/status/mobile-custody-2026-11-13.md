# Mobile task object custody — November 13, 2026

Codex owns this narrow R5.9 extension; Claude retains the aerial and world-import
work in the same checkout. Existing source-bound Stretch/Stretch 3 navigation,
taught articulation, heartbeat, cancellation and base-retraction checks remain.

The native task panel now offers **Check grasped** and **Check released**.
Choose the free-object body and two opposing finger body names from the running-model lists;
these are checked against the running model before a task starts. The object
must have its own free joint outside the robot; finger subtrees must be distinct
and belong to the robot. No object is attached, moved or welded by this feature.

Build a task using the existing navigation and taught arm/hand steps:

1. Navigate to the object and replay a taught reach/close/lift sequence.
2. Add **Check grasped**. Both opposing finger subtrees need at least 0.1 N
   normal force, no other supporting contact, and at most 1 cm change in
   object translation relative to the first finger, sustained for 1 simulated second.
3. Retract the arm for driving, then navigate to the destination. Navigation
   legs monitor opposing contacts, support and relative translation. Loss for
   0.25 simulated seconds requests base stop, arm hold and Nav2 cancellation.
4. Replay a taught lower/open/retract sequence and add **Check released**.
   The free object must have environmental support, neither finger may exert
   0.1 N, and object speed must be at most 0.02 m/s for 1 simulated second.

Object observations and checkpoint outcomes are written to the existing actual
SSD task journal. Grasp monitoring is suspended during articulation so an
intentional release/regrasp is possible, and resumes on navigation legs.
Robot/world binding and GUI heartbeat remain mandatory. Stop/intervention
continues to cancel tasks.

These are declared lab thresholds, not vendor calibration. Relative angular
slip, automatic grasp planning, arbitrary compound objects, costmap-consumption confirmation for carrying
footprints, further source robots and other backends remain implementation
work in ROADMAP R5.9. Carry monitoring applies within an active task, not after
its completion. A sequence without checkpoints still reports object outcome
as unassessed. Checkpoints do not constitute a full mission certificate.

Focused tests exercise single-sided versus opposing contact, support,
translational slip, stopped release, non-finite data, frozen/backward simulation
time and source-bound document validation. No physical grasp/transport mission
was run in this delivery. Raw checks are on SSD at
`/workspace/molar/robot_lab_runtime/mobile-custody-20261113/`.

Verification: 20 focused tests pass; robot_lab_utils, robot_lab_mujoco and
robot_lab_gui build successfully. The first run had one stale error-message
assertion (19 passed, 1 failed); updated for the additional step type and retained
the raw failure log. These checks are separate from physical mission acceptance.

## Payload-aware footprint continuation

After a successful grasp checkpoint, the physical payload body is retained
independently of task execution. NativeMobileSensors publishes the convex
hull of current robot and payload collision AABB corners in the actual base
frame, with the existing 3 cm padding, to `/local_costmap/footprint` and
`/global_costmap/footprint` every 0.5 simulated seconds. Fixed descendant
geometry is included; unrelated world bodies, floors and visual-only geometry
are excluded. Original body/arm geometry and its transforms stay unchanged.

The native Drive interlock requires the footprint to match the held payload,
have no geometry error, and be at most 0.75 simulated seconds old. A new task
navigation leg waits for that publication; active task navigation cancels if
it becomes invalid/stale. The existing arm retraction, command lease and
acceleration limits still apply. Clock reversal invalidates a future-dated
footprint until a fresh one is produced. Stopping/canceling a task retains the
payload envelope; a successful measured release checkpoint clears it. This
prevents an interruption from silently restoring a smaller unloaded outline.

The GUI task status exposes the payload and waiting reason, while the mobile
state exposes the published vertices, payload ID, generation time validity
and any geometry error. This is a publication/freshness gate, **not an
acknowledgement that Nav2 has consumed the polygon**. Costmap consumption,
controller collision behaviour and an actual loaded transport mission still
need runtime evidence. The existing Nav2 footprint padding remains separate
from the 3 cm geometric hull padding.

The checkpoint selectors are populated from the running plant, rather than
requiring model names from memory: free collision objects for the object field,
and robot collision bodies for the two fingers. Opposing-finger selection is
still explicit; the executor rejects overlapping subtrees. These lists describe
physical bodies, not guaranteed graspable objects or fingers. No checkpoint
button is enabled when the model contains no free collision object.

Nav2 interface reference: the upstream Humble
[Costmap2DROS implementation](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_ros.cpp)
handles footprint input and publishes the oriented footprint; its
[footprint utilities](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/footprint.cpp)
apply separate costmap padding. This delivery preserves the existing Polygon
input interface. It does not infer receipt from the number of subscriptions.

Verification for this continuation at `1e496a9e`: after Claude's recorded
flight finished, all 26 focused tests passed, including six native-engine
geometry/publisher/interlock cases. robot_lab_utils, robot_lab_mujoco and
robot_lab_gui build successfully. An Xvfb widget smoke check loads the actual
installed task panel, checks its three read-only model selectors, and verifies
a disconnected grasp-button click requests no task. This is UI/control-contract
verification, not a physical grasp or Nav2 mission receipt.

[Focused check output](evidence/mobile-custody-2026-11-13/checks.log) ·
[GUI smoke output](evidence/mobile-custody-2026-11-13/gui.log).
The tests are registered in the normal fast and physics tiers. Full raw logs,
including the build, remain on SSD under
`/workspace/molar/robot_lab_runtime/mobile-custody-20261113/payload/`.

## Local object pipeline continuation

The [GUI object-task guide](../tutorials/native-object-tasks.md) describes the
new source-control reach/grip/retraction steps and recipe buttons. This
supersedes the earlier missing local automatic-reach and consumer-receipt
implementation notes above, not their historical measurement scope.

[42 focused tests](evidence/mobile-custody-2026-11-13/reach-checks-final.log),
[both original source FK diagnostics](evidence/mobile-custody-2026-11-13/reach-source-fk.log),
[actual bench-scene attachment/home poses](evidence/mobile-custody-2026-11-13/reach-attached-scene.log),
[widget-only recipe check](evidence/mobile-custody-2026-11-13/reach-gui.log) and
[installed scene receipt](evidence/mobile-custody-2026-11-13/reach-scene-provision.json)
are retained. Three packages build successfully. The first expanded check run
had 41 passes and one stow-fixture failure: overlapping fictitious telescope
sphere bounds correctly triggered collision rejection. The fixture spacing
was corrected; the collision guard/thresholds were unchanged. The first map
schema probe used the plural type name and then exposed an unsupported robot
class label; corrected to the actual environment API and mobile/manipulator/
hybrid schema classes before acceptance. Negative logs remain on SSD.

The fixture is separate, declared MIT lab apparatus. Native objects are
free, source wheel/arm actuators are unchanged, and both Stretch models retain
the authored cube pose after real MjSpec scene attachment/home reset. This is
not yet a measured pick/place or loaded Nav2 mission. Remaining implementation
and later acceptance stay in ROADMAP R5.9/R5.7.

Source equality reference: [MuJoCo joint equality constraints](https://mujoco.readthedocs.io/en/stable/computation.html#equality).
The predictor uses the source reference-pose offsets and linear polynomial;
nonlinear/closed-loop constraints need separate reviewed adapters.
