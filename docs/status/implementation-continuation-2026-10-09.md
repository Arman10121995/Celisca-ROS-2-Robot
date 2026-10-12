# October 9 continuation: coordinated tasks, guarded takeoff and real workloads

Initial implementation continues from published `bf4b9ba`, directly on `master`.
The latest user correction requests Drone buttons to stay visibly pressed after
a click, like Drive. This batch implements that and connects separate controls
into operator tasks. Physical/broad test campaigns remain a later lane;
historical certificates keep their source revisions.

| Delivered code | Operator path | Remaining acceptance |
|---|---|---|
| Persistent Drive/Drone click latches | Click once to latch/highlight and ramp. Click again to release/brake. Stop/Space clears all directions, including altitude. Drive/Drone mirrors share state. | Physical flight/input regressions and joystick |
| Native Stretch coordinated tasks | Enable Native mobile controls; choose native workflow nav for base goals. Arm/Hand → Mobile task: add map X/Y/heading and taught sequences, reorder, save/load, then Run task. | Combined physical routes/object missions, other mobile models/backends, global arm planning |
| Selected-world X500 takeoff check | Drone → Check takeoff clearance → inspect clear/ceiling-limited rise → Takeoff within 30 s. Checking commands no movement. | World/spawn/contact/ceiling flight trials, dynamic obstacles, aerial sensing/mapping and other backends |
| Actual Nav2 experiment workload | Benchmark → Concurrent experiments → Nav2 point goals → `X Y heading_deg; ...` → Queue → Run. | Multi-goal/concurrent-plant campaigns, contacts/footprint metrics, applied simulator seeds and fair algorithm comparisons |

## Mobile tasks

The executor lives with the native plant so base/articulation ownership changes
are atomic with physics. Navigation uses Humble `NavigateToPose`, simulation
timestamps and its actual terminal result. Before advancing, it checks native
body truth within **0.15 m / 5 degrees** and waits one simulated second for
measured stopped base/articulation. The arm must be retracted for navigation;
articulation requires a stopped base. Taught targets retain source bounds/rates,
coupled tendons, measured settling and contact/watchdog behavior.

Documents match the executed model SHA and compiled collision-scene ID,
including meshes, rigid flexes, terrain and poses. Bounds are 32 steps / 64
configurations per articulation step. Task-specific GUI heartbeat expires at
0.8 wall seconds. Drive/Space, arm/Servo, RViz goal, Stop, fault, reset or loss
interrupts the task. Owned Nav2 cancellation is asynchronous; stale mux input
stays blocked until terminal response and a short drain period. Actual input,
source identities, transitions and endpoint errors are journaled on SSD under
`$ROBOT_LAB_RUNTIME_ROOT/mobile-tasks/`.

Completion is `measured_steps_reached`; grasp/payload/pick-place outcomes remain
unassessed. This does not supply a global collision-free arm planner. Editing
and save/load request no movement. Read [native controls](../tutorials/native-controls.md).

## Takeoff

The checker derives the installed X500/X500-base collision-box envelope,
accounts for rotating propellers, and adds **0.06 m side/top margins**. It
retains the landing-foot underside without adding a bottom margin that would
overlap the support floor. Current source derives yaw-independent horizontal
bounds ±0.503881 m, bottom −0.227 m and top +0.120423 m relative to `base_link`.
These are conservative geometry bounds, not flight tracking measurements.

The lowest **0.05 m** of the starting envelope is not swept (completion on
October 12): FCU height is aligned once at spawn + 0.24 m, leaving the feet
only 13 mm above the floor, so estimator noise or contact sink made the
support floor itself reject every check. The vehicle rises away from that
band; anything taller beneath it still rejects. Results report
`unchecked_support_band_m`.

The entire vertical swept box is checked against canonical oriented boxes,
spheres, conservative cylinder boxes, finite planes, open/closed mesh surfaces
and solid/heightfield interiors. Triangle/AABB separating axes preserve thin
open walls. Default requested rise is 3 m; a ceiling can reduce it with extra
headroom. A blocked minimum 0.8 m corridor rejects. Unknown/actor geometry
rejects explicitly. Checking is asynchronous; native mesh-library construction
cannot be interrupted mid-call.

Takeoff consumes a fresh landed/level/idle/disarmed FCU check with unchanged
world/model resources, start within 5 cm and age ≤30 wall seconds. This static
check does not qualify actual contact clearance or actor prediction. Manual
and direct airborne goals remain unchecked. Read [PX4 controls](../tutorials/px4_x500.md).

## Nav2 workloads and results

The queue snapshots the literal launch command, algorithms and selected world.
The built-in workload separately waits for Nav2/fresh independent body truth,
executes 1–32 real goals, and records UUIDs, terminal statuses, raw physical
pose/quaternion/velocity and clock samples. Endpoint gates are **0.15 m /
5 degrees / 1 simulated second settling**. Deadlines, stale state, time
regression, rejection, cancellation and wrong endpoints produce failure.

Owned SSD files are `workload.json`, `mission-trace.jsonl` and
`mission-result.json`. The parent verifies run/domain/spec/producer/trace
identity and recomputes successful endpoint errors. Reset invalidates the
result. Process cleanup/resource/capture states stay separate. Contacts and
swept clearance are null. Seeds are **requested and exposed**, not claimed
applied to physics. No random scores or exit-zero mission success is used.
Read [concurrent experiments](../tutorials/concurrent-experiments.md). The
action implementation follows the upstream [Humble Nav2 API](https://api.nav2.org/actions/humble/navigatetopose.html)
and [Humble action client](https://api.nav2.org/nav2-humble/html/robot__navigator_8py_source.html).

## Install and continue

Source `scripts/ssd_env.sh`, ROS and the overlay; rebuild
`robot_lab_utils robot_lab_mujoco robot_lab_adapter robot_lab_gui robot_lab_benchmark robot_lab_bringup`.
Reopen the GUI. Actual build/source diagnostics are in the
[continuation receipt](implementation-continuation-receipt-2026-10-09.json).
No new physical mission or CI acceptance is supplied by this batch.

Completed October 12 (Claude): the five packages build (11.2 s) and every
touched module imports from the install. New pure-logic tests cover the task
document, takeoff sweep and workload result contract (15 + 8 pass). The full
GUI suite passes under Xvfb (104). Two failures that predate this batch were
fixed: a stale Panda fixture assertion, and Arm/Drone control pages wider
than the control column (now two-column rows). The compact-window layout test
still fails only on the shared desktop display, as before.

Remaining initial work: other source-specific robot/backend controllers and
sensors; generic global arm planning, dexterous hands, arbitrary objects/grasp
contracts and other mobile platforms; aerial sensing/saved 3D maps, dynamic
avoidance and other flight backends; world/provider parity and independently
owned terrain tooling; substantive algorithm adapters, simulator-applied seeds
and fair measured comparisons. See [the ledger](platform-status.yaml),
[roadmap](../../ROADMAP.md), [earlier batch](implementation-2026-10-09.md) and
[agent guide](../AI_ROBOT_READINESS_GUIDE.md). Preserve independent terrain/review
ownership and historical negative evidence.
