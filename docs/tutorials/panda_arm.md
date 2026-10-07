# Native Panda arm and hand controls

Documentation reviewed October 7, 2026 against runtime checkpoint `091d388`.
Read [current status](../status/CURRENT_STATUS.md) for available workflows and
remaining qualification; evidence below retains its named source stages.

## Run

Select `menagerie_franka_emika_panda`, MuJoCo and Display in Launch. Choose a
map, then Run. The command includes `arm_control:=panda` and
`arm_planning:=moveit` for the qualified profile without a grasp fixture. Open **Launch → Arm** in the right-hand control column
to jog each joint, return Home, cancel a trajectory or Stop Arm. The displayed
positions come from the simulator. The base Drive/WASD controls are disabled
for this fixed arm. Other installed Panda/arm profiles remain Display imports.

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
measured joint state. Set the Cartesian offsets, for example Δz = 0.05 m with
Δx = Δy = 0, then press **Plan**. Review the reported waypoints and duration;
press **Execute Plan** to move. A subsequent Δy = 0.04 m was also measured.
Offsets are relative to the measured TCP in `native_world`; orientation is
preserved. Each component is bounded to ±0.25 m; reachability and collision
checks can reject a request within that input range.

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
scenes fail explicitly. The optional grasp fixture sets `arm_planning:=none`
because its free cube and pedestal are not yet represented as planning-scene
objects. Use the proven joint/Hand controls for that fixture. Attached payload
planning, Servo Cartesian jogging, arbitrary repeated pick/place and the
other arm backends remain R5.7–R5.9.

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
