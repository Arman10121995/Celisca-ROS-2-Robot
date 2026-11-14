# Native dexterous hands and coupled grippers

October 14, 2026, R5.8.native_hands. Four installed source models have
experimental controls in the normal Robot Lab GUI. This implements coordinated
motion and contact feedback; it does not certify an object grasp, lift or
all-map/all-backend mission. Panda keeps its separate measured gripper workflow.

| Robot variant | Motors and coordinates | Hand controls |
|---|---|---|
| `menagerie_shadow_dexee` | Original 12 `mujoco.pid` position actuators, radians, asymmetric source torque limits | Three finger groups; Curl, Pinch F0 + F1, Spread, Open; individual joint targets and taught replay |
| `menagerie_robotiq_2f85` | Original coupled tendon, source control 0–255, original ±5 actuator effort bounds | Normalized closure, Open, measured coupled-finger contact/effort interruption |
| `menagerie_robotiq_2f85_v4` | Its own source linkage/tendon/equality model | Same controls with its own source/plant identity |
| `menagerie_umi_gripper` | Original coupled position tendon, 0–0.05 m; six source placement actuators remain separate | Normalized closure and Open; existing placement jogging and taught replay |

All assets are already installed on the SSD. No GUI download is needed. The
source remains pinned to Menagerie `4d038b3feae26ec82b46a4d586379114012a8ac7`.
DEX-EE's source PID implements position-error P/I/D with integral and slew
bounds; Robot Lab uses that implementation instead of substituting a torque
controller. [Upstream PID description](https://github.com/google-deepmind/mujoco/blob/3.3.6/plugin/actuator/README.md),
[pinned DEX-EE model](https://github.com/google-deepmind/mujoco_menagerie/blob/4d038b3feae26ec82b46a4d586379114012a8ac7/shadow_dexee/shadow_dexee.xml).

## Use the GUI

1. In Launch choose the hand variant, **MuJoCo**, **Display**, and a world
   suited to the fixed mount. Start with `nav_empty`.
2. Open **Hand** in the control column. Click **Enable hand controls for next
   Run**, or check **Native joint controls (experimental)** in Launch. The
   generated command gains `enable_native_articulation:=true`. This selection
   starts no motion. Existing saved profiles preserve this opt-in.
3. Click **Run**. Pose controls remain disabled until the selected launch
   supplies fresh measured hand state and its source/motor identity.
4. Choose a finger group and pose. A closure fraction of 0 is open and 1 is
   the reviewed pose template; it is not a requested contact force. Spread
   applies to DEX-EE's lateral J0 coordinates. Begin with a small fraction.
5. Click **Apply hand pose**. Editing selectors or limits alone sends no
   command. **Open selected fingers** releases that selected group;
   **Stop hand / hold** holds every measured articulation coordinate.

DEX-EE curl uses explicit source-bounded targets J1 = −0.4, J2 = 0.9,
J3 = 0.5 rad at closure 1. J0 spread is at most ±0.25 rad. The pinch template
curls F0/F1 and opens F2. These are operator pose templates, not learned or
object-specific grasp policies. Only Apply activates them. Command targets
are slewed at most 0.4 rad/s and retain the original plugin's slew bound. Other channels use their
existing native rate limits.

The standard native joint selector, Use measured, Home, Stop and taught
sequence controls remain available. Teach captures actual settled source
coordinates. Saved sequences are source-matched and replay actual motors;
coordinate completion says nothing about whether an object stayed grasped.

## Run

The equivalent launch, after sourcing ROS and `install/setup.bash`:

```bash
source scripts/ssd_env.sh
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=menagerie_shadow_dexee simulator:=mujoco \
  mode:=display map_name:=nav_empty enable_native_articulation:=true
```

## Understand the guard and measurements

For the experimental free-object workflow, select **Add free grasp object on
next Run** in Hand. It enables native controls and adds `grasp_fixture:=true`
to the command. The object is authored on a pedestal from actual collision-pad
vertices; visual-only pad origins are excluded. **Grasp object** closes the
source motors and reports opposing contact, actual support and slip.
**Release object** opens the hand; the cube remains a free physics body and
can rest on its support. Fixture selection is locked during a run.

Grasp guards freeze the current bounded closing setpoint to retain contact
preload. Regular pose guards and Stop/heartbeat loss use measured-coordinate
hold instead. Grasp status downgrades on contact loss; support is computed
from actual world contact forces, not inferred from a previous hold state.
There is no automatic resume. A displayed contact hold is not proof of a
stable lift or pick/place mission. The normal Robotiq diagnostic records a
12 N contact overshoot despite a 2 N threshold; this is not force regulation.

**Guard closure** stops advancement of each affected finger group when its
summed external normal contact force exceeds the selected threshold, or a
source actuator reaches the selected fraction of its signed effort bound.
Robotiq fingers are mechanically coupled, so they stop as one group. DEX-EE
finger groups stop independently. Other groups can continue their explicit
targets. A fresh Open, Apply or Stop is required; there is no automatic resume.

Contact readings are actual `mj_contactForce` normal forces in N. Hand
self-contact is excluded from the external-object sum. Actuator effort is
reported separately: N·m for DEX-EE and Robotiq, N for the UMI sliding tendon
and placement translations. Original force bounds, gains, PID settings,
transmissions, equality constraints and collision geometry are preserved.
UMI authors no tendon force clamp: its explicit advancement threshold is
2 N × effort fraction. Neither that threshold nor any contact guard imposes
a hard contact-force limit; readings may overshoot between simulation steps.

Stop, rejection, reset and 0.8 wall-second operator-heartbeat loss hold
measured coordinates and clear owned PID integral/previous-setpoint memory.
No live joint pose/velocity, object pose or attachment is written. Holding a
position still uses the source actuator and can exert force. Use Open to
release contact; it differs from Stop.

Actual feedback is `/articulation/state` plus `/joint_states` and native body
TF for RViz. The state includes a unique plant id, source SHA, executed
motor/coupling contract SHA, measured coordinate/rate/effort, original effort
bounds, pose state, external contacts and blocked groups. Foreign/stale GUI
requests cannot initiate a hand pose. No motion occurs merely by selecting a
robot, arming input, choosing a template or opening this panel.

Measurements are unclipped. Source soft joint limits can permit small real
excursions beyond the command bounds; the readout shows those excursions.
Hold uses the nearest allowed command in that case. Jog inside the command
bounds before teaching that configuration. Derived feedback/body transforms
are refreshed at publication without advancing physics, so they agree with
the joint state's simulation timestamp.

## Implementation and validation handoff

### Source-axis object transport

UMI exposes **Object lift height**, **Lift and hold object** and **Lower to
start** in the existing Hand page. Add the free-object fixture before Run,
establish measured opposing contacts with Grasp object, wait for the source
axis to settle, then explicitly request a 2–15 cm lift. This commands the
original vertical position motor at its original bounded rate and keeps the
finger preload. Other fixed-mount hands require an arm transport adapter.

Completion requires measured object height, low speed, no environmental
support, retained opposing contacts and bounded finger-relative translation
and rotation for one simulated second. Moving the whole hand is no longer
mistaken for object slip. **Lower to start** returns the original source-axis
setpoint; a supported return requires actual settled support. Press **Release
object** afterward: `released` only means finger contact has ended;
`supported_release` requires one simulated second of supported, settled motion.

Stop, manual commands, invalid requests, contact loss, changed axis, new or
worsening external finger contact, stale simulation clock and lost operator
heartbeat interrupt transport. There is no automatic resumption. A current
source/backend GUI mission receipt is required before qualifying this workflow;
protocol checks and the availability of these buttons are separate evidence.

Read [the exact diagnostic scope and retained failures](../status/native-hands-2026-10-14.md)
and [the receipt](../status/native-hands-receipt-2026-10-14.json). Start from
the existing normal GUI producer on the SSD, not a replacement simulator or
direct pose writes. The initial hand implementation remains experimental.

For a new hand adapter, review the exact installed XML, actuator/plugin
semantics, source license, joint/tendon/equality bounds and transmission.
Bind the executed compiled motor/coupling arrays to the source before enabling
controls. Do not interpret a motor as a position servo because it has a
control range. Plugin proxies in the current Python binding caused a retained
segfault in a diagnostic; use compiled attributes and explicit source config.
Only connect/weld anchor coordinates tolerate 1e-12 numerical rounding after
global spawn; joint/tendon coefficients and motor parameters stay exact.

Later validation should cover all fingers/templates, full range and spread,
contact/effort interruption, heartbeat loss, Stop/reset during closure and
replay, GUI/process loss, and bounded source effort. For a grasp claim add a
real collision object, measure both fingers' contact, object pose/velocity,
slip, gravity release and repeated hold/lift behavior. A stationary mounted
hand needs an actual arm or the UMI placement axes for a lift. Do not use a
visual/physics weld as grasp evidence. Arbitrary-object scene editing, learned
grasp selection, raw torque/tendon-motor hands and additional backends remain
separate implementation tasks.

## Regulate an established grasp

November 13 adds **Requested hold force (N / side)** and **Regulate existing
object grasp**. First enable the documented object fixture, Run and establish
an actual two-sided grasp with **Grasp object**. Choose the same closing pose,
set a finite 0.2–5 N request, then explicitly click the regulation button.
Editing the field alone sends no motion.

The controller adjusts bounded source position targets from measured opposing
normal contacts; source PID gains, transmissions and hard actuator limits stay
unchanged. It regulates the weaker side, with a 10% / at least 0.05 N deadband,
normalized closure rate at most 0.03/s and gain 0.02/(N·s). These are declared
lab settings, not calibrated vendor force specifications. The requested effort
fraction still applies; UMI retains its existing 2 N × fraction lab threshold
where no source force clamp exists.

An existing grasp above twice the requested force enters bounded unloading.
Entry is refused above the declared 50 N observation bound. Further increases
above 110% of its measured entry peak stop it; after unloading, the normal
2× requested-force overload threshold applies. Established non-object fixture
contacts may remain, but rising force or worsening penetration there stops
regulation. New external finger contact, source-effort violation, loss of an
opposing contact, stale/non-finite timing/data, Stop, manual input or heartbeat
loss interrupt regulation. Its lease is bounded to 30 simulated / 60 wall
seconds; continuation requires an explicit new request.

Requested contact force is **not a hard physical force cap**. Stop holds
measured coordinates and may release a loaded object. No automatic regrasp or
object attachment is performed. The native Robotiq component fixture reduced
about 12 N per side to about 5.25 N for a 5 N request, with unchanged source
gains/force limits. This does not qualify the normal GUI mission, other hands,
lift/transport or hardware. [Receipt and retained failures](../status/hand-force-2026-11-13.md).
