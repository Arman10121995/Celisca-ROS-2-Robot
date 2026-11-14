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
than Stretch 3. Unreachable targets and routes for which no collision-free
configuration path is found within the planning budget are refused.

## What the planner actually does

Before adding reach/pick/place steps, choose an orientation mode:

- **Position only** preserves the original task behavior: the target is the
  midpoint of the two selected physical collision pads.
- **Keep measured pad** captures the first selected pad body's actual world
  orientation at the start of each reach. Wrist controls stay held while the
  reviewed lift/telescope controls move. Planning samples must remain within
  0.05 rad, and a measured deviation over 0.08 rad interrupts execution.
- **World pad RPY** adds the entered roll/pitch/yaw to each new reach as a
  normalized world quaternion. Angles are degrees, extrinsic XYZ rotations
  relative to `native_world`, for the **first pad body's authored frame**.
  This constrains the endpoint, not every intermediate orientation. Source
  geometry and collision checks still determine the path. The measured final
  pad orientation must be within 0.08 rad or the task fails.

Editing fields changes no motion. Add recipe/step captures the selected mode
in the task document; Run task executes. Existing saved position-only tasks
remain valid. Keep mode is captured afresh for each reach, not once for the
whole recipe. Gripper open/close and telescope stow steps keep their original
contracts. Original Stretch cannot realize the same orientations as Stretch 3;
an unavailable orientation is refused rather than moving the base/head or
changing source kinematics. Prefer Keep measured pad for an initially taught
hand orientation; arbitrary zero RPY is not a universal downward grasp.

`native_control_kinematics.py` uses the compiled position-actuator gains,
transmissions and linear joint equality constraints. Fixed-tendon telescope
and mimic coordinates are included. It solves point or optional pad-pose IK
using native FK/Jacobians. Pose IK uses numerical orientation derivatives and
refuses a private FK position residual over 8 mm or angular residual over
0.05 rad. A clear direct motor-coordinate path is preferred; a blocked
path now triggers bounded bidirectional RRT-Connect search over the reviewed
lift, telescope and wrist channels. Base, head and gripper targets remain held.
The collision checker uses the actual scene in a private `MjData` and samples
the controller's per-channel motor rate clipping, including stopping instants.
Unsupported spatial tendons, nonlinear/closed-loop equalities or inconsistent
source limits are refused. This is an experimental local implementation of
the published [RRT-Connect method](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html),
not the Panda's OMPL binding. It does not infer a useful grasp orientation or
guarantee a solution for an arbitrary requested orientation.

Source IK first tries the measured controls. If that solution is unavailable
or blocked, it can try the source-range midpoint and four seed-0 samples of
the reviewed movable controls. Head/base/gripper and keep-mode wrists remain
held. Duplicate solutions and colliding endpoints are excluded; every accepted
solution must still satisfy the source FK pose tolerances. This searches a
bounded set of possible wrist configurations, not every IK branch.

A clear motor path to any candidate is preferred. Otherwise all accepted
goals seed one bidirectional configuration search, so a disconnected first
goal does not discard another reachable solution. The GUI execution phase
reports an alternate IK attempt; the journal records every attempt's result,
available goal count and selected attempt. A duplicate seed is not counted as
a new IK solution or a successful grasp.

Search uses seed 0, at most 384 tree nodes, 2,048 collision-edge queries and
12,000 native collision samples within the original eight-second worker
budget. Source control limits bound the search. A result must fit the existing
64-waypoint executor. The task journal records the planner choice, seed, nodes
and checks. A fixed seed repeats sampling on an identical snapshot; wall-time
timeouts can still vary. No route found within these bounds does not prove
that the target is geometrically unreachable. Try another approach or base
position. The measured-seed IK has a two-second sub-budget; alternate seeds
have up to 0.5 seconds each, and their collection stops with three seconds
reserved for configuration search under the default total budget. Any accepted
path remains usable if optional shortening exhausts its inspection budget;
only previously inspected edges are returned. Exhaustive IK, perception and
other source/backend adapters remain open.

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
certificate. Contact calibration, perception-based object estimates, other
grasp strategies, additional mobile-manipulator adapters
and full map/backend missions remain in ROADMAP R5.9/R5.7. Object pose input
here is explicitly simulator truth. Stop, manual intervention and GUI/task
heartbeat loss interrupt execution and retain conservative payload bounds.

[Implementation receipt](../status/mobile-custody-2026-11-13.md) ·
[Native controls](native-controls.md) · [Roadmap](../../ROADMAP.md#r59--mobile-manipulators).
