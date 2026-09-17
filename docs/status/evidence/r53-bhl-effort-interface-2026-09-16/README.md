# R5.3 evidence: BHL effort-interface control path, 2026-09-17

Status: **wiring + contract complete and unit-verified; live dispatch-path
validation of the effort path is pending** (see "Open item" below).

Switches the Berkeley Humanoid Lite standing controller from a **position
command** path (`forward_command_controller/ForwardCommandController` into the
ign_ros2_control Kirk position servo) to an **effort command** path
(`effort_controllers/JointGroupEffortController`, kp=10.0, kd=2.0) so the
policy's 22 position targets close a PD-effort loop inside gz-sim instead of
driving position targets into a free-floating base with no feedback path.

## Why this change

The R5.3 evidence chain established, in order:

1. **Actuation probe** (`../r53-bhl-actuation-2026-09-16/`): position commands
   *do* move the joints through the dispatch path (knees tracked 0.4 rad
   exactly, arms tracked 0.8 rad, clamped by URDF limits), and the
   straight-legged robot stands stably under zero commands (tilt 0.0 rad over
   10 s). This overturned the earlier "commands produce no joint motion"
   reading, whose real cause was the policy node's pre-`cmd_vel` bent-leg
   default-pose hold toppling the spawn.

2. **Startup-transient fix** (`../r53-bhl-settle-2026-09-16/`): measured-pose
   hold → bounded ramp on first `cmd_vel` → safe abort eliminated the
   pre-command fall (tilt 0.0 rad through the pre-policy and idle windows,
   both 2 s and 12 s ramps). But the squat transition itself topples the
   free-standing biped at any ramp speed — knees reached only ~0.17–0.26 rad
   in the 12 s run before SAFE_STOP. That README scoped the residual as
   closed-loop balance and named two options:

   > (a) drive the joints through an effort interface with the training PD
   > gains (kp 10–20, kd 2) so the policy's own control loop closes, or (b) a
   > dedicated standing/balance controller with CoM/odometry feedback.

   This evidence implements option (a). Rationale: it matches the control
   formulation the upstream policy was trained under (PD-effort joint targets),
   and the robot already demonstrates stable standing under zero commands.

## What was changed (verified against the files)

**`src/robot_lab_robots/berkeley_humanoid_lite/xacro/bhl_ros2_control.xacro`** —
the `bhl_joint` macro's single command interface changed from
`position` (min/max ±3.14159) to **`effort`** (min/max ±20.0 N·m, matching the
URDF effort limits and the balance controller's saturation constants). The
state interfaces are unchanged and complete: `position`, `velocity`, `effort`.

This **replaces** rather than *adds to* the position command interface: the
`bhl_joint` macro has exactly one `<command_interface>` element, and a
`JointGroupEffortController` claims the `effort` command interface. (An earlier
draft of this README described the change as additive with both interfaces
coexisting — that was wrong; the file is effort-only, which is why the
`forward_command_controller` position path is no longer usable against this
description. The position command interface was unused after the switch and is
not restored.)

**`src/robot_lab_robots/berkeley_humanoid_lite/config/bhl_controllers.yaml`** —
`bhl_standing_controller.type` changed from
`forward_command_controller/ForwardCommandController` to
`effort_controllers/JointGroupEffortController`, and `kp: 10.0` / `kd: 2.0`
were added. The 22-joint list is unchanged and still equals `BHL_JOINT_NAMES`
(12 legs, then 10 arms) — verified by
`test_backend_commands_the_canonical_22_joints_in_order`, which matters because
`JointGroupEffortController` maps command-vector entries onto its joint list
positionally.

The dispatch path already loads this exact file: `bhl_ros2_control.xacro`
points the `IgnitionROS2ControlPlugin` / `GazeboSimROS2ControlPlugin`
`<parameters>` at
`$(find robot_lab_robots)/berkeley_humanoid_lite/config/bhl_controllers.yaml`,
and `robots.yaml` declares the robot's controller list. So no separate
parameter-file wiring was needed — changing the one file is sufficient. (An
earlier draft created a redundant `bhl_controllers_effort.yaml` and described
pending dispatch wiring; that file was deleted — it referenced nothing, and its
ankle joint order differed from the canonical order.)

## Why these numbers

- **kp=10.0, kd=2.0** — the low end of the training-consistent band the settle
  README cited ("kp 10–20, kd 2"). Soft enough not to amplify free-floating-base
  wobble, non-zero so position targets close a PD loop instead of stalling.
- **±20 N·m** — the same bound as the URDF and `bhl_balance.py`
  (`EFFORT_LIMIT = 20.0 N·m`).

## Contract tests (unit-verified, no live graph)

- `src/robot_lab_bringup/test/test_sim_profiles.py::
  test_bhl_controller_config_declares_the_verified_controllers` — asserts the
  controller type is `JointGroupEffortController` with kp=10.0, kd=2.0 and 22
  joints.
- `src/robot_lab_adapter/test/test_r5_3_bhl_backend_contract.py` — asserts the
  backend is an effort controller, declares the PD gains, commands the
  canonical 22 joints in order, and that the `bhl_joint` macro declares the
  `effort` command interface plus full position/velocity/effort state.

Both suites pass under the repo's canonical runner
(`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:anyio`); the fast
tier (`scripts/test_fast.sh`) is green.

## What this does NOT claim

- **Not that the policy now balances or walks.** The effort interface only
  closes a PD-effort loop on the policy's position targets. The squat-to-stand
  transition still has no CoM/balance feedback, so the policy's default-pose
  squat may still destabilize a standing biped. Whether the PD-effort loop
  smooths that enough to hold is exactly what the pending live run must show.
- **Not a live-verified result yet.** No live sim run is bundled with this
  evidence: the changes are wiring/config + contract tests. The forward-command
  position path *was* verified live in `r53-bhl-actuation-2026-09-16`, but the
  effort path has not been run live, and since the effort command interface
  replaced the position one, the live proof in that README no longer applies to
  the current description. A live re-run is required before claiming the effort
  path actuates.
- **Gains are not tuned for this sim.** kp/kd come from the policy's training
  formulation; if the live robot is unstable or sluggish, gain tuning is the
  follow-up, not wiring.

## Open item (R5.3)

Run the policy chain live through the dispatch path with this effort controller
active:

```
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=display map_name:=empty robot_model:=berkeley_humanoid_lite_sim \
  simulator:=gazebo gui:=false start_rviz:=false
```

Success criteria: both controllers active with `bhl_standing_controller` an
`effort_controllers/JointGroupEffortController`; `/joint_states`, `/bhl/imu/out`
and `/bhl_standing_controller/commands` live; the policy's position targets
produce controlled motion rather than the "pinned near zero" stall; tilt stays
below the 0.70 rad fall threshold. If balance still fails, the residual is
option (b) — a dedicated balance controller. This is tracked in
`docs/status/platform-status.yaml` under R5.3 `next_action`.

## Relationship to prior evidence

- `r53-bhl-settle-2026-09-16` — direct predecessor: same dispatch path, same
  policy node, same robot; only the joint-control interface changed. Its
  option (a) is what this implements.
- `r53-bhl-actuation-2026-09-16` — proved the *position* path actuates; the
  effort path is a different actuation mode with velocity damping and is the
  one now wired.

## Artifacts

- This README (the record of the wiring rationale and honest scope).
- `../../../src/robot_lab_robots/berkeley_humanoid_lite/xacro/bhl_ros2_control.xacro`
- `../../../src/robot_lab_robots/berkeley_humanoid_lite/config/bhl_controllers.yaml`
- `../../../src/robot_lab_bringup/test/test_sim_profiles.py`
- `../../../src/robot_lab_adapter/test/test_r5_3_bhl_backend_contract.py`
- `../../../src/robot_lab_adapter/test/test_r5_3_bhl_policy_ros_graph.py`

