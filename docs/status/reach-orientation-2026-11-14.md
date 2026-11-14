# Native Stretch reach orientation — November 14, 2026

Codex owns R5.9 reach/GUI files; Claude retains fleet/drone/import work.
The source-control planner now accepts optional first-pad orientation in
`native_world`. The existing GUI task builder exposes Position only, Keep
measured pad and World pad RPY. [Usage](../tutorials/native-object-tasks.md#what-the-planner-actually-does).

Position remains the two physical collision-pad midpoint. Orientation belongs
to the first pad body's actual authored frame; no invented tool offset or
source-axis change is introduced. Explicit RPY is converted to a normalized
world quaternion. The task schema validates finite/unit input and refuses
simultaneous explicit and keep requests. Existing task documents retain their
position-only behavior. Editing adds no motion; Run task remains explicit.

Private IK uses original source actuator, tendon/equality and limit mappings,
with a world rotation-vector residual weighted by 0.15 m/rad and numerical
orientation derivatives. It accepts at most 8 mm positional and 0.05 rad
angular residual. Unreachable orientations are refused. Lift/telescope/wrist
configuration search, payload transforms and original time/node/waypoint
budgets remain. Head, gripper and base stay held.

Keep mode captures the measured pad quaternion for each reach, holds wrist
controls and additionally checks every private sampled configuration against
a 0.05 rad angular envelope. During actual execution, measured deviation over
0.08 rad interrupts the task. Explicit world orientations constrain the
endpoint; measured final error over 0.08 rad fails rather than reporting
completion. Neither mode modifies source motors, gains, physical poses or
object constraints.

Focused checks cover normalized/invalid schema requests, unreachable pad
rotation, actual native FK and a reviewed synthetic rotating-wrist fixture,
unchanged source qpos/gains/grip, preserved orientation in returned reaches,
live/final task interruption and GUI recipe construction. Initial failures
were test-fixture setup issues: its new hinge inherited MuJoCo's degree
default rather than declaring radians, and the task test lacked its telemetry
publisher fixture. These setup failures are retained in the raw check log;
source joint limits were not widened to hide them. The final **39 checks pass**
under Xvfb, including the actual task-builder widget constructing pick/place
recipes with no ROS publisher connected. The MuJoCo package builds; source
hashes of all five changed runtime modules match their installed files. Shared
utils/GUI had already been installed by the other agent's build; the local
MuJoCo rebuild and checks waited for an idle simulator window.

Published CI on the preceding placement revision `d4eb4d5a`, run 43530569788,
failed one newly added R6.8 test after 1,282 passed/42 skipped: its fast tier
has no sourced ROS environment, but the test imports ament and launch. The
unchanged world-copy test is now assigned to the required sourced integration
tier and passes locally. The runner also includes the GUI source package for
unsourced builder tests; two such builder/widget checks pass locally. The full
published exact-revision rerun remains separate and pending.

The logs and source manifest are in [the evidence directory](evidence/reach-orientation-2026-11-14/README.md);
raw output is on SSD at
`/workspace/molar/robot_lab_runtime/reach-orientation-20261114/`.
Software/native geometry and widget checks are separate from a real installed
Stretch/Stretch 3 physical pick/transport/place mission. Multiple IK branches,
perception-derived targets and further source/backend adapters remain open.
