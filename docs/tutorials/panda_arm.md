# Native Panda arm and hand controls

## Run

Select `menagerie_franka_emika_panda`, MuJoCo and Display in Launch. Choose a
map, then Run. The command includes `arm_control:=panda`. Open the **Arm** tab
to jog each joint, return Home, cancel a trajectory or Stop Arm. The displayed
positions come from the simulator. The base Drive/WASD controls are disabled
for this fixed arm. Other installed Panda/arm profiles remain Display imports.

The actual seven position actuators, gains and force limits come from the
[pinned Menagerie Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/4d038b3feae26ec82b46a4d586379114012a8ac7/franka_emika_panda).
Targets advance through native MuJoCo dynamics; the controller does not write
joint positions or robot poses. The **Hand** tab controls the original coupled
finger actuator through a bounded-force opening command.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=menagerie_franka_emika_panda simulator:=mujoco \
  mode:=display map_name:=nav_empty arm_control:=panda
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

Actual moving-arm/environment contacts abort motion reactively. The optional
grasp fixture allows contact between its cube and the source finger pads;
other moving-arm/environment contacts still abort the arm. This is **not
predictive collision checking** or a MoveIt planning scene. Cartesian targets,
MoveIt/Servo, self-collision planning, arbitrary pick/place and other
backends remain R5.7–R5.9. Do not enable those mission labels from this joint
screen alone. Measured joint/Home/Stop/cancel/watchdog results and negative
probes are in the [control evidence](../status/evidence/panda-turtlebot4-2026-10-05/README.md).

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
