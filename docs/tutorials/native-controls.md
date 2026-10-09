# Native walking, joint and Cartesian controls

October 9 integrations are implemented experimental controls. Physical
testing follows initial implementation. Existing measured variants remain
separate from the new policy models.

## Unitree policy models

Select Unitree G1/H1 → RL Gym policy variant (G1/H1/H1 v2) → MuJoCo → Display.
Enable **Native walking policy (experimental)** under advanced options before
Run. The preview includes `enable_native_locomotion:=true`. With the option
off, normal Display retains its passive authored pose.

Drive/WASD/strafe send bounded forward/lateral/yaw commands to the exact
lower-body policy. Startup command is zero; loss requests zero velocity.
After movement the balance policy retains joint support. Fall/non-finite
state latches zero effort until Reset. `/robot_lab/locomotion/state` reports
controller state/faults, not mission success. SLAM/navigation stay unavailable.

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=policy_unitree_g1 simulator:=mujoco mode:=display \
  map_name:=nav_empty enable_native_locomotion:=true
```

The adapter follows the pinned [Unitree RL Gym deployment contract](https://github.com/unitreerobotics/unitree_rl_gym/blob/276801e46c5d433564f24658bac64f254b7d2d4b/deploy/deploy_mujoco/deploy_mujoco.py)
with distinct checkpoints, actuator orders, gains and timing. These policies
cannot be substituted into arbitrary full-DOF descriptions.

## Native arm/hand joints

Select a compatible fixed-base native model → MuJoCo → Display. Enable
**Native joint controls (experimental)** before Run. Arm shows the dynamic
actuator panel. Select a channel and inspect measured value, units and bounds.
Jog ± changes a fraction of the range; Apply target executes a chosen value.
Moving the slider alone commands nothing. Source home restores the initial
control reference; Stop/heartbeat loss holds measured articulation.

Direct joints show radians/metres when the conversion is exact. Coupled
grippers keep source control units and tendon coupling. Reaching a control
target is separate from successful physical grasping.

## Cartesian jogging without typing coordinates

On Panda or compatible generic controls, select an authored tool site or actual terminal body origin in
**Cartesian jog**, enable it and hold XYZ/rotation ±. Axes use `native_world`;
translation is capped at 0.04 m/s and rotation at 0.2 rad/s. Release, leaving
the button or Stop sends zero; stale commands expire.

The local servo uses the executed model's Jacobian, limits and contact
prediction. It stops at bounds/contacts and is separate from a global path
planner. Panda's **Use current tool pose → target → Plan → Execute Plan**
remains available for MoveIt planned motion. Models without an authored tool site may use an explicitly labeled
`body:<source-name>` terminal origin; it is not a guessed TCP offset.
Incompatible transmissions retain joint-only or prior availability.

## Teach configurations and replay

The generic native Arm/Hand panel has **Teach / replay articulation**. Jog or
Servo to a configuration, wait for measured motion to stop, then **Teach
current**. This records every measured actuator coordinate; it commands no
movement. Repeat for the desired sequence, remove unwanted points, then
**Save sequence** on the SSD. **Load sequence** requires the exact source
model hash, channel set and current bounds. Loading does not execute it.

**Replay** advances original actuators at their declared bounded rates. Every
step must reach measured position/rate tolerances and remain settled for its
recorded dwell (default 0.5 simulation seconds). Missing heartbeat, Stop,
manual joint/Servo override, contact penetration, non-finite state or a
settling deadline interrupts replay. A step is limited to 120 wall seconds and
the sequence to 600; at most 64 configurations are accepted. This is taught
articulation, without global path planning or automatic grasp evaluation.
It works with the generic fixed-model and Stretch articulation controllers;
Panda retains its separate joint trajectory/MoveIt workflow.

## Stretch and Stretch 3 mobile controls

Select the normal complete Stretch family and exact original/Stretch 3
variant → MuJoCo → Display → enable **Native mobile controls** before Run.
Drive/WASD uses actual source wheel cylinders, axes, axle and transmissions:
original Stretch uses forward/turn tendon motors; Stretch 3 retains original
wheel velocity servos. Limits are declared lab settings: 0.09 m/s forward,
0.3 rad/s yaw, 0.2 m/s² linear and 0.5 rad/s² yaw acceleration. No body pose is
written for Drive. Neutral enablement, command loss and Reset request zero.

Stop the base before jogging/Servo/replay. Retract the arm tendon below
0.15 m before Drive; actual articulation motion and active sequences block
base commands. The robot's changing collision bounds produce a conservative
convex full-body footprint, including arm extension. These stability/ownership
rules are implemented; physical stability and mission validation remain pending.

## Experimental native mapping and navigation

Keep Launch's outer **Display** mode and explicit **Native mobile controls**.
Choose the separate **Native workflow** display/loc/slam/3d_slam/nav below it.
The command preview includes `enable_native_mobile:=true native_task:=...`.
This exposes an implemented candidate without changing measured support gates.
Select a world; AMCL/navigation require its generated occupancy YAML. Generate
missing grids in Worlds. A larger collision map does not by itself certify
navigation.

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=menagerie_hello_robot_stretch_3 simulator:=mujoco mode:=display \
  map_name:=nav_empty enable_native_mobile:=true native_task:=nav
```

- **loc:** EKF + AMCL and the selected occupancy map.
- **slam:** EKF + slam_toolbox; **Save Map** writes the actual occupancy result.
- **3d_slam:** EKF + RTAB-Map using actual authored-camera registered RGB-D;
  **Save Map** exports the current database using the normal robot/map paths.
- **nav:** EKF + AMCL + Smac2D + Regulated Pure Pursuit, low-speed caps and
  dynamic full-body footprint. RViz goals use the existing Nav2 goal relay.

Lidar originates at the source laser body plus a declared 35 mm lab mount
height. RGB and depth share the authored camera/intrinsics at 320×240/5 Hz.
Stacks wait for actual lidar messages; 3D SLAM also waits for actual camera
frames. A renderer failure is reported and leaves 3D readiness false.
Odometry/IMU derive from actual simulator state and bootstrap the local EKF;
they do not establish independent encoder accuracy or successful localization.
These new higher workflows await saved/reloaded map and physical route checks.
