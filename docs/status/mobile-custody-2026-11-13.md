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

Verification for this continuation: focused geometry/publisher/interlock tests
are registered in the physics tier, and custody/document checks in the fast
tier. Their execution and package build are pending while Claude's recorded
flight occupies the shared simulator host. The earlier 20-test/build result
above belongs to the preceding checkpoint, not these new footprint changes.
