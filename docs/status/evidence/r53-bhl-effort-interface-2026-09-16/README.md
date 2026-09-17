# R5.3 evidence: BHL effort-interface control path, 2026-09-17

## Live balance runs (2026-09-17, second session) — 2 of 3 failure modes fixed

Three consecutive live runs of the dispatch path (`humanoid-standing-controller`
on `/imu/out`, effort interface, fresh sim each time) with a 40 s IMU/joint
trace starting before spawn. Findings, in the order they were fixed:

1. **SAFE_STOP latch on every cycle — FIXED.** The nodes defaulted to
   `imu_topic: /bhl/imu`, a topic with no publisher: the gz bridge publishes
   the trunk IMU as gz `/imu` → ROS `/imu/out`. With no IMU the body state
   fallback produced tilt = π, latching SAFE_STOP on the first cycle. Fix:
   default `imu_topic` is now `/imu/out` in both
   `humanoid_standing_controller.py` and `humanoid_policy_controller.py`
   (verified live: the bridge topic carries a unit quaternion, tilt ≈ 0 at
   spawn). `robots.yaml` (BHL `sensors.imu.topic`) and the
   `algorithms.yaml`/contract-test required-topics list were aligned to
   `/imu/out` — the registry previously declared `/imu`, which nothing
   publishes on ROS.

2. **Knee-buckle at the nominal stance — FIXED.** The draft nominal pose
   (knees 0.8 rad, ankle pitch 0.1, arms offset) actively dragged the biped
   out of its balanced spawn pose: the live trace showed tilt 0.000 until the
   PD began pulling the knees toward 0.8, then the knees buckled (20 N.m
   cannot hold that crouch) and the biped fell through 0.70 rad in ~1-2 s.
   Fix: `NOMINAL_STANDING_POSE` is now the URDF rest pose (all 22 joints 0).
   Live result: the robot stands in equilibrium at spawn (tilt 0.000, knees
   0.000) — the first live-stable stance of the R5.3 effort path.

3. **Rigid-body pivot topple — OPEN, control-design gap (not wiring).**
   With the stable stance achieved, the biped still toppled: tilt 0.005 at
   t≈0.16 s → 0.37 (0.5 s) → 0.66 (0.6 s) → 1.27 rad (0.8 s), then lying
   still at 1.27 rad. The joints barely moved during the fall (knees within
   0.02 rad): the body pivots about the foot edge like an inverted pendulum.
   Analysis: ankle PD restoring stiffness is 120 N.m/rad saturating at the
   URDF 20 N.m (≈0.167 rad), while the gravity toppling stiffness at the BHL
   CoM height (0.675 m, ~13 kg) is ≈86 N.m/rad — marginal even with a 1:1
   joint mapping, and the BHL's 45°-mounted leg axes mean body tilt does not
   map 1:1 onto ankle joint error, so the effective restoring stiffness drops
   below the toppling stiffness and the fall diverges. Holding stance
   therefore needs more than the current proportional tilt law: gravity
   compensation from the URDF dynamics, a proper leg Jacobian mapping
   (tilt → joint torques through the 45° axes), and/or a hip strategy, plus
   likely a pose with knees flexed inside (not at) the joint limits so the
   ankles retain motion authority. This is the documented option (b)
   fallback — a dedicated balance controller — now with quantitative
   justification.

Also fixed this session: `test_r5_3_bhl_balance.py::test_knee_softens_with_tilt`
exercised knee softening at the knee's 0.0 lower limit (clamped, no headroom);
it now runs at a flexed stance (0.5 rad) so the law is actually observable.

# R5.3 evidence: BHL effort-interface control path, 2026-09-17

Status: **wiring + contract complete and unit-verified; live dispatch-path
validation of the effort path is pending** (see "Open item" below).

Switches the Berkeley Humanoid Lite joint drive from a **position command**
path (`forward_command_controller/ForwardCommandController` into the
ign_ros2_control Kirk position servo) to an **effort command** path
(`effort_controllers/JointGroupEffortController`), and closes the PD-effort
loop in the publishing nodes so the policy's 22 targets are tracked as torques
rather than issued as raw position commands into a free-floating base.

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
`effort_controllers/JointGroupEffortController`. The 22-joint list is unchanged
and still equals `BHL_JOINT_NAMES` (12 legs, then 10 arms) — verified by
`test_backend_commands_the_canonical_22_joints_in_order`, which matters because
`JointGroupEffortController` maps command-vector entries onto its joint list
positionally. The file deliberately declares **no `kp`/`kd`**
(`test_backend_declares_no_control_law_gains`): see the correction below.

The dispatch path already loads this exact file: `bhl_ros2_control.xacro`
points the `IgnitionROS2ControlPlugin` / `GazeboSimROS2ControlPlugin`
`<parameters>` at
`$(find robot_lab_robots)/berkeley_humanoid_lite/config/bhl_controllers.yaml`,
and `robots.yaml` declares the robot's controller list. So no separate
parameter-file wiring was needed — changing the one file is sufficient. (An
earlier draft created a redundant `bhl_controllers_effort.yaml` and described
pending dispatch wiring; that file was deleted — it referenced nothing, and its
ankle joint order differed from the canonical order.)

## Where the PD law lives (correction to the first draft)

**`JointGroupEffortController` has no control law.** It derives from
`forward_command_controller::ForwardCommandController` and its header states it
"forwards the commanded efforts down to a set of joints"; the installed library
exposes only `interface_name` as a parameter name (no `kp`, `kd`, `ki` or
`gains`). A first draft of this evidence declared `kp: 10.0` / `kd: 2.0` on the
controller and claimed the controller closed the PD loop — that was **wrong**,
and worse, a silently-ignored gain would have left the policy's position values
(rad) being written to the effort interfaces as torques (N·m).

The loop is therefore closed **in the publishing nodes**, before publishing:

- `humanoid_policy_controller` and `humanoid_standing_controller` gained a
  `command_interface` parameter (`effort` by default, matching the description;
  `position` publishes raw targets and is kept for the position marshalling
  path).
- On the effort interface they convert their targets with
  `bhl_balance.pd_effort_command`: `tau = Kp*(q* - q) + Kd*(0 - qdot)`, clamped
  to ±20 N·m, with a joint that has no measurement receiving **zero effort**
  (never a blind drive). The standing node publishes the `efforts` the balance
  core already computes (zero on SAFE_STOP); the policy node computes them from
  its own targets with node parameters `kp_legs`/`kd_legs`/`kp_arms`/`kd_arms`
  (defaults `bhl_balance.STANCE_PD_LEGS` = (120, 4) legs, `STANCE_PD_ARMS` =
  (60, 2) arms).
- `pd_effort_command` gained optional `leg_gains`/`arm_gains` overrides so the
  nodes can select gains without changing the balance law (pinned by
  `TestPdGainOverrides` in `test_r5_3_bhl_balance.py`).

Gains are the repo's existing, unit-tested stance-PD convention rather than an
invented number: the vendored policy configs (`urdf/config.json`,
`mjcf/config.json`) declare **no** kp/kd, so the "kp 10–20, kd 2" figure in the
settle README's option (a) was a scoping estimate, not a value read out of the
checkpoints. The gains are node parameters precisely so they can be tuned
during live validation.

- **±20 N·m** — the same bound as the URDF and `bhl_balance.py`
  (`EFFORT_LIMIT = 20.0 N·m`), enforced by `clamp_effort` in the node.

## Contract tests and results

- `src/robot_lab_bringup/test/test_sim_profiles.py::
  test_bhl_controller_config_declares_the_verified_controllers` — asserts the
  controller is `JointGroupEffortController` with 22 joints and that the file
  carries no dead `kp`/`kd`.
- `src/robot_lab_adapter/test/test_r5_3_bhl_backend_contract.py` — asserts the
  backend is an effort controller, declares no control-law gains, commands the
  canonical 22 joints in order, and that the `bhl_joint` macro declares the
  `effort` command interface plus full position/velocity/effort state.
- `src/robot_lab_adapter/test/test_r5_3_bhl_balance.py::TestPdGainOverrides` —
  the gain-override API the nodes use, including clamping under large gains.
- `src/robot_lab_adapter/test/test_r5_3_bhl_policy_ros_graph.py` — live rclpy
  graph with real ONNX inference. The four original tests run the
  `position` interface (regression cover for target marshalling); two new tests
  run the deployed `effort` default and assert that the pre-command hold is
  **zero drive** (not the measured pose, proving the wire semantics changed) and
  that driving produces finite efforts inside ±20 N·m.

Results (2026-09-17): `scripts/test_fast.sh` **430 passed, 1 skipped, PASS** with
registry cross-reference validation PASS; the full adapter suite (rclpy +
onnxruntime) **243 passed** including all 6 graph tests; integration tier
(xacro expansion + BHL qualification) **114 passed**.

## What is still NOT verified

The sim has not been run live against this corrected wiring. Every claim above
is static or process-level: the xacro expands to 22 effort interfaces (0
position, 66 state interfaces), the plugin is installed and registers the type,
the node tests run on a real rclpy graph, but **no gz-sim physics run has
executed with the node publishing efforts**. Confirming that the PD-effort loop
actually holds the biped (and tuning the gains if not) is the remaining R5.3
item; the earlier live actuation proof applied to the position path this change
supersedes.

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
- **Gains are not tuned for this sim.** The nodes default to the repo's
  stance-PD convention (legs 120/4, arms 60/2); the vendored checkpoints
  declare no gains, so these are a starting point. If the live robot is
  unstable or sluggish, gain tuning is the follow-up — the gains are node
  parameters for exactly that reason.

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

