# Native Panda joint controls

## Run

Select `menagerie_franka_emika_panda`, MuJoCo and Display in Launch. Choose a
map, then Run. The command includes `arm_control:=panda`. Open the **Arm** tab
to jog each joint, return Home, cancel a trajectory or Stop Arm. The displayed
positions come from the simulator. The base Drive/WASD controls are disabled
for this fixed arm. Other installed Panda/arm profiles remain Display imports.

The actual seven position actuators, gains and force limits come from the
[pinned Menagerie Panda](https://github.com/google-deepmind/mujoco_menagerie/tree/4d038b3feae26ec82b46a4d586379114012a8ac7/franka_emika_panda).
Targets advance through native MuJoCo dynamics; the controller does not write
joint positions or robot poses. The coupled finger actuator stays at its
authored open setting. Grasp control is R5.8.

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

Actual moving-arm/environment contacts abort motion reactively. This is **not
predictive collision checking** or a MoveIt planning scene. Cartesian targets,
MoveIt/Servo, self-collision planning, pick/place, hand contact and other
backends remain R5.7–R5.9. Do not enable those mission labels from this joint
screen alone. Measured joint/Home/Stop/cancel/watchdog results and negative
probes are in the [control evidence](../status/evidence/panda-turtlebot4-2026-10-05/README.md).
