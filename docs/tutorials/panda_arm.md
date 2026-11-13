# Native Panda arm and hand controls

Documentation reviewed October 9, 2026; current controls and exact source stages
are in the [extension evidence](../status/evidence/extensions-finish-2026-10-08/README.md).
Read [current status](../status/CURRENT_STATUS.md) for available workflows and
remaining qualification; evidence below retains its named source stages.

## Run

Select `menagerie_franka_emika_panda`, MuJoCo and Display in Launch. Choose a
map, then Run. The command includes `arm_control:=panda` and
`arm_planning:=moveit` for the existing profile. Fixture planning is now
implemented experimentally with measured object geometry and attach/detach;
its new mission validation is pending. Open **Launch → Arm** in the right-hand control column
to jog each joint, return Home, cancel a trajectory or Stop Arm. The displayed
positions come from the simulator. The base Drive/WASD controls are disabled
for this fixed arm. Compatible fixed native arms/hands have separate explicitly
enabled actuator/Servo/teach-replay controls; see [native controls](native-controls.md).
Other models keep their existing availability.

The actual seven position actuators, gains and force limits come from the
[pinned Menagerie Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/4d038b3feae26ec82b46a4d586379114012a8ac7/franka_emika_panda).
Targets advance through native MuJoCo dynamics; the controller does not write
joint positions or robot poses. The **Launch → Hand** page controls the original coupled
finger actuator through a bounded-force opening command.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=menagerie_franka_emika_panda simulator:=mujoco \
  mode:=display map_name:=nav_empty arm_control:=panda arm_planning:=moveit
```

Use `arm_control:=none` for the original passive authored-pose Display.
With `none`, `display_hold:=false` enables uncontrolled native dynamics.

The `/arm/follow_joint_trajectory` action uses the upstream
[FollowJointTrajectory interface](https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/action/FollowJointTrajectory.action).
This first implementation accepts **positions only**, all seven names exactly
once, zero header stamp, increasing waypoint times and a maximum duration of
15 seconds. Cubic segments start/end at rest and target speed stays within
0.5 rad/s. Targets retain a 0.005 rad margin inside the model's joint limits.
Invalid joints, limits, timing, velocity/effort arrays or unsupported tolerances
are rejected. Default path/goal tolerances are 0.12/0.03 rad, with one second
to settle at the final goal and an additional measured velocity check.

The GUI sends `/arm/heartbeat` (`std_msgs/Empty`) at 10 Hz while it owns this
launch and receives fresh arm state. External action clients must provide the
same heartbeat; stale input rejects new goals. During a trajectory, 0.8 seconds
without heartbeat aborts it and holds the measured position. `/arm/stop`
(`std_srvs/Trigger`) and action cancellation also stop and hold. A second
simultaneous goal is rejected. Stop is a controlled hold, so physical inertia
can continue briefly while the native actuators brake.

Actual moving-arm/environment contacts abort native joint motion reactively.
The optional grasp fixture allows contact between its cube and the source
finger pads; other moving-arm/environment contacts still abort the arm. Joint
jog/Home commands retain this native safety contract. The separate MoveIt
controls below predict collisions for their planned path.

## Cartesian Plan / Execute

Select the native Panda, MuJoCo, Display and `nav_empty`, leave the Hand grasp
fixture unchecked, and Run. In **Arm**, wait for the selected-world scene and
measured joint state. Press **Use current tool pose** to fill the measured XYZ
readout and clear the target offsets. Use the **X/Y/Z −/+** buttons to select
a small target from that position. The default step is **0.01 m (1 cm)**;
the step field accepts 1–50 mm. The readout shows the resulting XYZ target.
Press **Plan**, review the waypoints and duration, then **Execute Plan** to move.
Reading a pose and editing targets never starts a trajectory. Existing manual
offset fields remain available, for example Δz = 0.05 m or Δy = 0.04 m.
Offsets are relative to the measured TCP in `native_world`; orientation is
preserved. Each component is bounded to ±0.25 m; reachability and collision
checks can reject a request within that input range.

This is a position target with the current tool orientation held. A reachable
point may require another orientation or elbow configuration. Start with a
smaller step; use joint jogging to change the arm configuration if planning
still rejects the target. Coordinates are metres in `native_world`, not pixels
or the camera frame. This workflow currently applies to the qualified native
Panda on MuJoCo; other imported arms need their own controller and planner.

The October 8 normal GUI repeat measured 1 cm X/Y/Z movements and submillimetre
TCP endpoint errors. Model gravity/Coriolis feedforward corrects the previous
7 mm vertical sag through the original position actuators. Its command offset
is bounded to ±0.03 rad and retains native control/force limits, gains and
contact checks. It uses [MuJoCo's actuator and bias-force model](https://mujoco.readthedocs.io/en/stable/computation/index.html#actuation-model).
The full collision, unreachable, cancellation, Stop, watchdog and reset checks
also pass. A separate actual Hand repeat lifts the cube 7.81 cm and releases it.

This reuses installed [MoveIt 2 planning scenes](https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html),
KDL inverse kinematics, OMPL RRTConnect and FCL collision checks. The planning
URDF/SRDF is generated from the same pinned native MJCF: exact joint axes,
limits with the native 0.005 rad margin, source collision hulls and native
collision exclusions. A static selected-world scene is acknowledged before
planning. Moving links carry 1 cm environment padding. The TCP comes from
the original finger-pad midpoint, 0.1029 m along the hand's local z axis.

The GUI sends measured joints as the [MoveIt IK seed](https://github.com/moveit/moveit_msgs/blob/ros2/msg/PositionIKRequest.msg),
requests collision avoidance and plans to that joint solution. This avoids
far redundant elbow solutions for small offsets. Checked joint edges are
retimed at at most 0.35 rad/s and executed by the existing native action.
Paths exceeding its 15 s contract are rejected; no path is truncated or
executed automatically. Actual FK and independent physical TCP feedback
are checked in the [final control-column Cartesian evidence](../status/evidence/panda-cartesian-controls-column-2026-10-07/README.md).

A cached plan expires after ten wall seconds or a joint/finger change above
0.01 in the corresponding joint units. Jog/Home, Reset, Cancel, Stop,
selection changes and editing an offset also invalidate it. Stale joint/scene
feedback disables execution. Arm Cancel/Stop and heartbeat loss hold the
measured position. Live Monitor may remain connected during these controls.

The measured mission covers two targets and rejection/interruption/reset in
static `nav_empty`. Other static maps are operator experiments; actor/heightmap
scenes fail explicitly. October 9 adds experimental local **Cartesian jog**
and measured fixture planning. The free cube and pedestal enter the MoveIt
scene from actual native state; scene revisions invalidate saved plans. After
an idle measured grasp, **Attach grasped object** requires fresh joints, FK
and contact on both source fingers. The collision attachment uses actual
finger links and tool-relative geometry; it never welds the physical cube.
**Detach object** restores measured world geometry. Losing grasp contact
removes the planning attachment and interrupts an executing arm path.

Wait for scene acknowledgement before planning. Source/pose/contact changes
and pending scene updates disable execution. These new Servo/attachment
workflows await physical validation; historical static plans and joint cube
lifts remain scoped to their original sources. Arbitrary repeated pick/place,
other arm planning adapters/backends and coordinated mobile manipulation are
still missing initial tasks in R5.7–R5.9.

## Hand and physical grasp

Select the native Panda, MuJoCo, Display and `nav_empty`. Before Run, open
**Hand** and select **Add a supported grasp object on next Run**. The command
adds `grasp_fixture:=true`. It places a 3 cm, 50 g free cube between the home
finger pads on a pedestal. The source robot is unchanged; the cube moves through
actual contact forces and gravity.

Run and choose **Close** at the default 0.5 N force limit per finger. The
measured opening settles near 30 mm against the cube. In Arm, set the jog step
to 0.15 rad and click Joint 2's minus button. The recorded setup lifts the cube
7.6 cm and holds it for two simulation seconds. **Open** releases it to fall
under gravity. **Reset Robot** restores the arm, open fingers and supported
cube without rewinding the clock. This measured fixture is a first grasp
mission; other object geometry, masses, poses and grasp plans need their own
tests.

**Open**, **Close**, **Set Opening**, **Cancel** and **Stop Hand** use
`/gripper/gripper_action` (`control_msgs/action/GripperCommand`). The opening
is the sum of the two slide-joint displacements, 0–80 mm; pad geometry can make
the physical gap differ. The source split tendon and joint equality remain
active. Only actuator targets and their force bounds change. A force limit of
0.1–20 N in GUI bounds each finger's actuator effort; contact normal force is
measured separately and can include external loads. A zero API force request
uses the 0.5 N default. The opening target advances at 30 mm/s. A blocked
command succeeds as stalled only when both fingers have contact, measured
motion stops and actuator effort reaches the requested bound. Otherwise it
must reach its opening or abort after eight simulation seconds.

Arm and hand goals reject simultaneous execution. The GUI provides
`/gripper/heartbeat` only while it owns the selected launch and receives fresh
state. External clients must send this heartbeat too. Cancel, `/gripper/stop`
(`std_srvs/Trigger`) and 0.8 s heartbeat loss hold the measured opening.
GUI Reset aborts an active action before restoring the source home state.
See the [physical grasp and interruption evidence](../status/evidence/panda-gripper-2026-10-06/README.md).

The final normal control-column repeat measures 0.00649/0.00639 m and
0.780/0.765° TCP errors, preserves all original rejection/interruption/reset
checks and closes each owned child cleanly. Its strict source certificate
includes the shared workspace layout. Earlier repeats remain historical.

## Select a measured object and plan grasp stages

November 13 implementation: **Arm → Object grasp targets** exposes the actual
MuJoCo free-box catalog for the native Panda. Keep the normal Panda/MoveIt
launch above. The existing supported fixture works; a selected world can also
provide free bodies with one physical box collision shape. Static map objects,
meshes and compound bodies are not selectable grasp objects. The pose source
is explicitly simulator truth, not a camera detector.

1. Choose a named box and click **Select object** while the arm and hand are
   idle and any previous object is released/detached. Selection changes no
   physical state. Wait for the acknowledged measured collision scene.
2. Click **Generate candidates**, then choose a top grasp. Candidates use the
   measured collision geometry and the Panda's 80 mm opening. The recommended
   gap includes the actual planning padding; a 30 mm box with the default
   10 mm link padding recommends 54 mm.
3. **Set gap field** edits the Hand opening value only. In Hand, explicitly
   click **Set Opening**. Return to Arm and click **Plan approach**, then
   **Execute Plan** after inspecting the plan.
4. Click **Plan grasp**, inspect it, then **Execute Plan**. IK and OMPL still
   reject unreachable or colliding poses; a geometry candidate is not a
   successful grasp. Try another candidate or reposition the arm if needed.
5. Explicitly close the hand. Wait for measured contact on both source
   fingers, then use the existing **Attach grasped object** planning command.
   Attachment changes MoveIt's collision model only; it does not weld the
   physical object to the robot.
6. Regenerate candidates if the box moved, then **Plan lift** and explicitly
   **Execute Plan**. Lift planning requires acknowledged attachment and live
   two-sided contacts. Use the existing Cartesian controls to plan a transfer;
   release with Hand and detach the planning object after placement.

All stage buttons only request plans. Existing execution, cancellation,
heartbeat, source actuator limits and contact interruption rules still apply.
Stale poses, changed dimensions, movement over 5 mm or orientation change
invalidate candidates. The scene retires the selected world's static snapshot
only after binding its canonical model ID; an unknown binding leaves planning
unready. The contact observer changes its target only after the corresponding
scene acknowledgement.

This is an initial box-selection/stage-planning workflow, not automatic
arbitrary-object pick/place. Other arms/backends, mesh/compound adapters,
perception poses and a complete new physical pick/transfer/place mission remain
open in R5.7. [Implementation and scoped checks](../status/arm-objects-2026-11-13.md).
