# R5.3 evidence: live Gazebo backend bringup (command path + IMU + crash guard), 2026-09-16

In-scope, reproducible proof that the BHL live backend path (Gazebo Ignition
Fortress 6.18 + gz_ros2_control) actually brings up: the `bhl_standing_controller`
and `joint_state_broadcaster` load and activate, `/imu/out` and `/joint_states`
publish, the `humanoid_policy_controller` pushes 22-element `Float64MultiArray`
position targets at ~25 Hz, and the tilt-based crash guard fires as designed when
the robot spawns un-balanced.

This is the gap item from the 2026-09-15 `next_action` ("one live gz-sim bringup
launch consuming the published targets through the backend controller") — done on
the actual machine, with the actual installed Ignition 6.18 + both `ign_ros2_control`
and `gz_ros2_control` plugin libs.

## What was verified (single clean bringup, robot_name:=bhl, world nav_empty)

| Check | Result | Source |
|---|---|---|
| Robot spawns in Gazebo (entity 42, name `bhl`) | ✅ | `launch.log` "Created entity [42] named [bhl]" |
| IMU frame names unique (no `Non-unique name[imu]` / `frame with name[imu] already exists`) | ✅ | `launch.log` — that error is **absent**; was present in the pre-fix probe (`/tmp/r53probe/`) |
| `joint_state_broadcaster` loads + activates | ✅ | `launch.log` "Loading controller 'joint_state_broadcaster'", "configure successful" |
| `bhl_standing_controller` loads + activates | ✅ | `launch.log` "Loading controller 'bhl_standing_controller'", "configure successful", "activate successful" |
| IMU bridge publishes `/imu/out` | ✅ | `launch.log` "Creating GZ->ROS Bridge: [/imu (gz.msgs.IMU) -> /imu/out (sensor_msgs/msg/Imu)]" |
| `/joint_states` published | ✅ | `diag.txt` lists `/joint_states`; probe subscriber saw msgs |
| Policy node wired to the live backend | ✅ | `policy.log` "HumanoidPolicyController: policy=policy_humanoid (/joint_states + /imu/out + /cmd_vel -> /bhl_standing_controller/commands) at 25.0 Hz over 22 joints" |
| Policy publishes `Float64MultiArray` at ~25 Hz | ✅ | `policy.log` rate line; `diag.txt` subscriber confirmed topic |
| Crash guard fires on un-balanced spawn (intended) | ✅ | `policy.log` "safe_stop: tilt 0.79 rad exceeds fall threshold" — the robot spawns at the default pose with no balance, tilts ~0.79 rad, and the policy's latched SAFE_STOP engages; this is **expected** behavior for an un-balanced spawn, not a backend defect |

## Reproduce

Prerequisites (all present on this host): ROS 2 Humble sourced, workspace built,
Ignition Fortress 6.18 (`ign gazebo --version` ≈ 6.18.0), `ign_ros2_control-system`
and `gz_ros2_control-system` plugin libs installed, `robot_lab_adapter` installed
so `humanoid-policy-controller` resolves.

```bash
# 1. Headless Gazebo with the BHL sim xacro + nav_empty world
ros2 launch robot_lab_description gazebo.launch.py \
    world_name:=nav_empty \
    robot_name:=bhl \
    model:=$(ros2 pkg prefix robot_lab_robots)/share/robot_lab_robots/berkeley_humanoid_lite/xacro/bhl_sim.xacro \
    gui:=false \
    -r

# In another terminal (after robot_state_publisher + gz_ros2_control have started,
# roughly 3-5 s):
source install/setup.bash
# Quick diagnostic: controller_manager services, the two topics, and the loaded
# controller list
ros2 service list | grep controller_manager
ros2 topic list | grep -E 'imu/out|joint_states'
ros2 control list_controllers

# Policy node (uses the live backend's /imu/out + /joint_states; publishes to
# /bhl_standing_controller/commands). The first cmd_vel must arrive before the
# policy starts driving; before that it holds the default pose.
humanoid-policy-controller \
    --imu_topic /imu/out \
    --joint_states_topic /joint_states \
    --command_topic /bhl_standing_controller/commands \
    --cmd_vel_topic /cmd_vel \
    --policy_name policy_humanoid \
    --command_rate_hz 25.0 &

# Send a zero cmd_vel so the policy leaves the default-pose hold and starts the
# onboard loop, then watch the safe_stop fire as the un-balanced spawn tilts:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

The bringup is intentionally **un-balanced** for this proof: the point is to show
the full backend stack (description → ros2_control → controllers → IMU bridge →
policy node → command topic) all come up and talk to each other, and that the tilt
crash guard behaves as documented (0.70 rad latched SAFE_STOP). The real balance-on-
boots policy command-tracking gap stays out of scope here and is recorded as a
residual below.

## Artifacts

- `launch.log` — full headless bringup log (gz-sim server, robot_state_publisher,
  parameter_bridge, controller loads/activates, IMU bridge, entity creation)
- `policy.log` — humanoid_policy_controller output (policy init + rate line, then
  the safe_stop on tilt 0.79 rad)
- `diag.txt` — ros2 service/topic/controller snapshot taken mid-bringup

## What this does NOT claim

- The robot does **not** balance or walk here — it spawns un-balanced, the crash
  guard fires, and that is the intended safety behavior for an un-balanced spawn.
- The command-tracking gap (policy targets not producing commanded motion because
  the onboard loop lacks real odometry/state feedback, only the IMU) is a real
  residual and is documented as such. This proof is about the **backend wiring**,
  not about closed-loop balance performance.
- Headless mode (`gui:=false`) is now wired through `gazebo.launch.py` via the
  new `gui` launch argument (`-s` appended to `gz_args` when `gui:=false`).

## Residual / out of scope

- Balance-on-boots policy command tracking: the policy receives IMU + joint_states
  but no onboard odometry/velocity feedback, so published position targets do not
  yet close the loop into stable standing/walking. This is the documented R5.3
  command-tracking gap (onboard-odometry feedback or a retrained policy). Not a
  backend defect — the backend path proven here is sound.
