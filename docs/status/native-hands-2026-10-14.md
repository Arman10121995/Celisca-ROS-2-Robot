# Native hands: implementation and exact diagnostic scope

October 14, 2026. Baselines `46f1d55` / `210a5c9` (hand pose work) and
`b41a33a` (object-work continuation). R5.8.native_hands is **partial**:
initial control/GUI/fixture implementation exists, broader grasp/mission
acceptance remains pending. [Operator and agent guide](../tutorials/native-hand-controls.md),
[source/artifact receipt](native-hands-receipt-2026-10-14.json).

Four installed candidates are `menagerie_shadow_dexee`, both
`menagerie_robotiq_2f85` variants and `menagerie_umi_gripper`. Their normal
MuJoCo Display controls use authored PID/tendon motors, source bounds and
actual state; no joint/object pose writes or physical attachment occur during
motion. Panda keeps its separate source/qualification contract.

## Delivered

- Coordinated curl/pinch/spread and individual finger selection for DEX-EE;
  normalized Open/Close for coupled grippers, joint jogging and source-matched
  taught replay. Editing/selecting controls causes no travel command.
- Measured coordinate/rate/effort and external contacts, per-group threshold
  stops, 0.8 wall-second heartbeat hold, Stop/reset and owned PID memory reset.
- Unclipped actual measurement with independent command projection. Derived
  state/body TF is refreshed for the published joint-state timestamp.
- Normal GUI free-cube fixture, source adapter forwarding and actual object
  presence gates. Grasp/release tracks opposing forces, actual support, slip,
  contact loss and interruption. Source-bounded grasp preload remains distinct
  from manual Stop. Fixture commands and saved profiles autofill correctly.
- Physical cube placement uses collision-pad vertices and a clear opening,
  rather than visual-only body origins. The object remains free on a pedestal.

## What actually ran

The four packages build on the Jetson/ROS Humble SSD workspace. **34 focused
checks pass**: native hand/PID/source counterexamples, original Panda gripper,
object contact/support/loss/NaN behavior and an explicitly synthetic coupled-pad
physics fixture. Tests and fixture shapes are not full robot mission evidence.

Actual installed Tk selection checks cover all four hand commands and the
fixture opt-in, with no plant launched by selection. GUI Health renders R10
as queued runtime; researched extension models do not enter the runnable list.

Earlier normal GUI hand-motion diagnostics on MuJoCo/`nav_empty` measured
DEX-EE F0 curl/Open/Stop (about 0.213 rad maximum coordinate change) and
Robotiq close/Open/Stop. Those reports retain their original publication
stages and hashes; clipping/publication corrections came afterwards.

The final **normal GUI Robotiq 2F-85 / MuJoCo / nav_empty / Display** free-object
diagnostic uses the corrected collision-pad scene and actual Grasp/Release/Stop
buttons. It records 523 joint and 523 controller samples, with a clean owned
launch return code 0:

| Phase | Left/right normal contact | Support | Observation |
|---|---|---|---|
| Before Grasp | 0 / 0 N | measured upward support 0.491 N | Cube starts clear of the fingers |
| Grasp snapshot | 11.941 / 11.941 N | actual support present | Two-sided contact; measured slip 0.170 mm |
| After Release | 0 / 0 N | measured upward support 0.491 N | Fingers no longer contact the free cube |

The selected contact threshold was 2 N/group. **The contact overshoot shows
that this guard is not a hard contact-force limiter.** Original actuator force
limits remain unchanged. This one diagnostic establishes fixture/GUI/control
wiring and a measured contact/release observation, not stable grasp/lift,
force-regulated manipulation, repetitions or full R5.8 acceptance.

Raw traces, commands, source snapshots, failed runs and logs are under
`/workspace/molar/robot_lab_runtime/native-hands-2026-10-14/`. Compact reports
are [preserved in evidence](evidence/r58-native-hands-2026-10-14/README.md).

## Retained negatives and repairs

1. An exploratory temporary `MjsActuator.plugin` proxy access segfaulted.
   Runtime uses explicit source XML and compiled plugin attributes instead;
   no plugin proxy is accessed by the adapter.
2. Robotiq's translated linkage anchors differed by about 4e-16 m and an exact
   array check rejected launch. Only connect/weld coordinates tolerate 1e-12;
   joint/tendon coefficients and motor parameters still compare exactly.
3. The current hand-object continuation referenced a missing GUI property,
   omitted adapter/fixture forwarding and unconditionally reset a nonexistent
   keyframe. These GUI/launch paths are now wired to real plant state.
4. Contact support originally skipped the pedestal and used a contact-frame
   tangent component as world Z. It now handles both geom orders and measures
   world upward force. Contact loss, heartbeat and NaN can no longer retain a
   misleading hold/support state.
5. The positive test model had the right slide joint pointing outward, supplied
   no recurring heartbeat and used an Euler step unstable for its light,
   heavily damped pads. Its source fixture now has symmetric inward axes,
   recurring input and the native Robotiq implicit integrator. Original
   contact/release assertions are retained; failed artifacts remain.
6. A measured-position grasp stop removed preload. Grasp now freezes a bounded
   existing command; manual Stop retains measured hold. The first actual
   object scene also used visual pad origins and failed to release contact.
   Actual collision geometry yields an initially clear cube and real release.
7. The first producer sent duplicate cleanup interrupts, yielding launch
   return code 1 and an event-loop error. The final producer stops its owned
   group once and returns 0. No other process group was stopped.

## Remaining implementation and validation

Force-regulated/contact-limited grasping, arbitrary object fixtures and scene
editing, stable repeated hold/lift/place, all DEX-EE fingers/templates and UMI/
Robotiq-v4 object cells remain open. Further raw torque/tendon hand adapters,
coupled arm planning, mobile manipulation and other backends also remain open.
Validate Stop/reset/heartbeat/GUI loss during actual object missions and replays
after initial implementation. Do not refresh old Panda, arm or flight support
certificates from these checks.
