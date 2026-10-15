# Plan and execute motion on native fixed arms

October 14, 2026. This experimental adapter adds real MoveIt KDL/OMPL planning
and position-trajectory execution to 13 installed Menagerie arms on **MuJoCo
Display**. UR5e and SO101 have one actual GUI reach diagnostic each in
`nav_empty`; the other arms, objects, maps and backend missions remain pending.
See [implementation and evidence](../status/native-arm-planning-2026-10-14.md).
The existing [Panda](panda_arm.md) workflow remains separate.

## Use the GUI

1. Reopen the GUI after building. Choose the complete robot family and exact
   native variant below, **MuJoCo**, **Display**, and the selected world.
2. In Launch's advanced options enable **Native arm planning: MoveIt
   (experimental)**. It also enables native joint controls. The command
   automatically contains `enable_native_articulation:=true` and
   `arm_planning:=native_moveit`. KDL, OMPL RRTConnect and native trajectory
   execution are selected together.
3. Run the generated command. Open **Arm** in the Launch control column.
   Wait for fresh measured articulation and an acknowledged collision scene.
4. Click **Use current tool pose**. Edit a small X/Y/Z offset with the ± buttons
   or the offset fields. Coordinates use `native_world`, in metres. Editing
   requests no motion.
5. Click **Plan**, inspect the result, then **Execute Plan**. Use **Cancel
   planned motion** or **Stop / hold** to interrupt it. Joint jogging or local
   Servo commands also interrupt a running plan. A plan expires after ten wall
   seconds and is discarded when measured joints or the scene change.

Six/seven-joint arms keep the measured tool orientation in their IK request.
The three five-joint arms use KDL position-only IK; their orientation follows
the arm motion. An unreachable/colliding target remains rejected. Use smaller
steps or adjust a joint configuration first, then read the tool pose again.

| Exact installed variant | Authored target frame | Arm joints |
|---|---|---:|
| `menagerie_flexiv_rizon4` | `body:link7` origin | 7 |
| `menagerie_flexiv_rizon4s` | `body:link7` origin | 7 |
| `menagerie_franka_fr3` | `site:attachment_site` | 7 |
| `menagerie_franka_fr3_v2` | `body:fr3v2_link8` origin | 7 |
| `menagerie_kuka_iiwa_14` | `site:attachment_site` | 7 |
| `menagerie_rethink_robotics_sawyer` | `site:attachment_site` | 7 |
| `menagerie_robotis_omy_3m` | `site:attachment_site` | 6 |
| `menagerie_unitree_z1` | `body:link06` origin | 6 |
| `menagerie_universal_robots_ur5e` | `site:attachment_site` | 6 |
| `menagerie_universal_robots_ur10e` | `site:attachment_site` | 6 |
| `menagerie_low_cost_robot_arm` | `body:gripper_static_finger` origin | 5 |
| `menagerie_robotstudio_so101` | `site:gripperframe` | 5 |
| `menagerie_trs_so_arm100` | `body:Fixed_Jaw` origin | 5 |

Body-origin targets are labeled explicitly; they are not calibrated fingertip
TCPs. Passive/other channels stay in the full measured planning state. Moving
another channel invalidates the plan and interrupts execution. The GUI's Hand
page remains available for source position channels; this does not establish
grasp or payload readiness.

## Run: build and launch

From this workspace:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select robot_lab_utils robot_lab_mujoco robot_lab_bringup robot_lab_gui --symlink-install
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=menagerie_universal_robots_ur5e simulator:=mujoco mode:=display \
  map_name:=nav_empty enable_native_articulation:=true arm_planning:=native_moveit
```

Use one owned plant/planner per ROS domain. Other robot models and controller
types remain rejected by this adapter. Large exports and collision hulls live
under `$ROBOT_LAB_RUNTIME_ROOT/arm_planning/` on the SSD. The build uses the
existing installed ROS 2 MoveIt packages; no additional upstream checkout is
needed for the GUI.

## Execution contract and limitations

The separate planning URDF exports the source's joint anchors/references,
ordered joints, body transforms and actual collision hulls. Capsules use an
exact cylinder plus two spheres. Compiled kinematics, collision mesh vertices
and actuator parameters are fingerprinted against the executed plant. Free
bases, unexported equalities/flex mechanisms and explicit contact pairs reject
this planning adapter. Coupled mechanisms need their own kinematic export.

The action `/native_arm/follow_joint_trajectory` accepts 1–256 position-only
waypoints, at most 120 simulation seconds. Rest-to-rest cubic segments retain
the collision-checked straight joint-space edges. Source bounds and margins
apply; target speed is at most 0.3 rad/s or 0.03 m/s, acceleration at most
0.6 rad/s² or 0.06 m/s². Native source gains/transmissions/force limits remain
unchanged. Bounded bias compensation uses the original position actuators;
the live joint/body pose is never assigned to follow a path.

Stop, manual input, reset, contact, measured path error, stalled simulation
or 0.8 wall seconds without the operator heartbeat interrupt execution and
hold measured articulation. Native collision inspection has a 0.2 second
admission budget and ongoing current/lookahead checks. Long/expensive paths
can therefore reject. This is a static selected-world planning workflow,
with **1 mm generic link padding** and actual moving-link floor checking;
the existing Panda workflow keeps its 10 mm padding. Dynamic prediction,
attached payloads and globally safe manipulation remain separate work.

Planning reuses the official
[MoveIt KDL configuration](https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.rst)
and [MuJoCo joint semantics](https://mujoco.readthedocs.io/en/3.3.6/XMLreference.html).
The runtime remains on the locally installed Humble/MuJoCo versions.
