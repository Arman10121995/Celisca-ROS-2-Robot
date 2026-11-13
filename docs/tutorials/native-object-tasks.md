# Stretch object tasks: local pick, transport and place

This is an experimental implementation for the original native Stretch and
Stretch 3 on MuJoCo. It combines source-model reach planning, native motor
control, measured gripper contacts, object custody and Nav2 footprint receipts.
A complete physical pick/transport/place mission has not yet been qualified.

## Run

The `mobile_manipulation_lab` world is installed in the normal map list.
Restart the GUI if it was open during provisioning. To reproduce the installed
fixture on another SSD-equipped checkout:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_mobile_manipulation_scene.py
```

It contains two static benches and a free 30 mm, 35 g cube. These are declared
lab apparatus and material assumptions; vendor robot mechanics stay unchanged.
The occupancy grid blocks the benches and intentionally omits the movable cube.
MuJoCo retains its free joint; Gazebo has a dynamic SDF cube. Other importers
may flatten dynamic props, so their manipulation workflows are not granted.

Select **Stretch → original or Stretch 3 → MuJoCo → Display**, choose this
map and enable **Native mobile controls**. Select **Native workflow nav** if
your task will include base destinations. Display workflow suffices for local
arm/hand tasks. Run the autofilled command.

## Build a task

In the Arm/Hand native controls, find the mobile task panel. Select
`environment_pickup_cube` and the two `rubber_tip_left` / `rubber_tip_right`
collision pads. The choices come from the running model. A robot without a
free object cannot enable these object controls.

**Add local pick recipe** adds: open, approach 8 cm above the object, reach
its collision-geometry centre, close until both pads have contact, lift 10 cm,
confirm sustained measured grasp, and collision-check telescope retraction.
It adds steps only; **Run task** executes them. You can reorder/remove steps,
insert a different object-relative reach offset, or use taught configurations.

Insert a base navigation destination using the existing map X/Y/heading
controls. The arm must retract, the grasp must be confirmed, and both Nav2
costmaps must report the matching payload outline before transport starts.

Enter **Place XYZ · native world** for the gripper-pad midpoint at release.
These coordinates are metres in `native_world`, not a camera frame. For this
fixture, the second bench centre is approximately `(1.2, -0.62, 0.515)`;
reachability and support still require physical inspection. **Add local place
recipe** adds an approach above this point, lowering, gripper opening,
measured supported release, retreat and retraction. Reposition the base before
a local reach when needed; original Stretch has fewer wrist degrees of freedom
than Stretch 3. An unreachable or colliding local approach is refused.

## What the planner actually does

`native_control_kinematics.py` uses the compiled position-actuator gains,
transmissions and linear joint equality constraints. Fixed-tendon telescope
and mimic coordinates are included. It solves local position-only IK using
native FK/Jacobians and checks the motor-coordinate edge against the actual
scene in a private `MjData`. Unsupported spatial tendons, nonlinear/closed-loop
equalities or inconsistent source limits are refused. This does not replace
MoveIt/OMPL global arm planning or solve arbitrary grasp orientations.

Planning is an eight-second-budget worker. It cannot write live robot/object
poses. A late/canceled generation is ignored; object or actuator movement
invalidates the result. The original motors, rate/force limits and live
penetration/heartbeat guards execute the accepted targets. Predicted held
object poses exist only in the planning copy; the real object remains free.

Closing requires two measured opposing contacts for 0.5 simulated seconds.
Only that blocked gripper may bypass position-error settling while maintaining
those contacts; other joint, velocity, command and force checks remain.
Contact closure alone does not grant transport. Lift and the one-second grasp
checkpoint must establish no outside support and bounded relative slip first.
Opening removes the planning attachment; a successful physical release
checkpoint clears the conservative payload footprint.

## Footprint receipts and limits

The native footprint publisher includes the payload's physical bounds. In
navigation workflow, actual `/local_costmap/published_footprint` and
`/global_costmap/published_footprint` messages must match its padded shape and
current geometry epoch in odom/map respectively. Translation/yaw of the
published polygon are accounted for. Wrong shape, frame, padding, stale or
future timestamps cannot acknowledge it. Receipts expire after 2.5 simulated
seconds or 3 wall seconds; source geometry publication still has its existing
0.75 simulated-second freshness bound. Native Stretch's 1 cm Nav2 padding is
explicit. Editing that padding requires updating the receipt contract.

This is implemented consumer verification, not a completed loaded-navigation
certificate. Contact calibration, perception-based object estimates, global
arm planning, other grasp strategies, additional mobile-manipulator adapters
and full map/backend missions remain in ROADMAP R5.9/R5.7. Object pose input
here is explicitly simulator truth. Stop, manual intervention and GUI/task
heartbeat loss interrupt execution and retain conservative payload bounds.

[Implementation receipt](../status/mobile-custody-2026-11-13.md) ·
[Native controls](native-controls.md) · [Roadmap](../../ROADMAP.md#r59--mobile-manipulators).
