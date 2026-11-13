# Mobile task object custody — November 13, 2026

Codex owns this narrow R5.9 extension; Claude retains the aerial and world-import
work in the same checkout. Existing source-bound Stretch/Stretch 3 navigation,
taught articulation, heartbeat, cancellation and base-retraction checks remain.

The native task panel now offers **Check grasped** and **Check released**.
Enter the exact MuJoCo free-object body and two opposing finger body names;
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
slip, automatic grasp planning, arbitrary compound objects, dynamic carrying
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
