# Stabilization and remaining work: agent execution guide

Updated 2026-09-30. This guide implements the user's requested patch queue
before further project expansion. Read `docs/AGENT_HANDOFF.md`,
`docs/status/platform-status.yaml`, and `ROADMAP.md` first. Drone integration is
excluded from the current run. Existing Bumperbot/Labbot workflows and assets
must remain available.

## Establish the actual baseline

Read [the 2026-10-02 audit](status/audit-2026-10-02.md) and
[storage guide](STORAGE.md). Generated demonstration scores cannot qualify a
mission. Keep all large artifacts on the mounted workspace SSD.

1. Inspect `git status --short` and the task ledger. Preserve existing edits.
   Claim the next patch and record any additional scope paths before changing
   shared code. Current source baseline is `e3d63b3` plus documented working-tree
   patches; an installed package can still be older than source.
2. Source `/opt/ros/humble/setup.bash` and `install/setup.bash`. Build changed
   packages with `colcon build --packages-select ... --symlink-install`.
   Python packages on this host sometimes copy files despite that option;
   verify the installed module/launch actually contains the new code.
3. Run source tests with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest`.
   This avoids the host's incompatible auto-loaded anyio plugin. GUI tests
   require `xvfb-run -a`. Use `scripts/test_fast.sh` for the final combined
   source check; do not substitute source tests for live missions.
4. Run one physics qualification at a time. Use a distinct `ROS_DOMAIN_ID`
   below 233 and a distinct `IGN_PARTITION`/`GZ_PARTITION`. Check for orphaned
   simulator/controller processes. Record the generated GUI command, not only
   a handwritten command that happens to work.
5. Retain both failed and passed artifacts under `docs/status/evidence/`.
   Record source hashes, launch/probe arguments, backend/host, duration, message
   counts, measured state, limits, real-time factor and cleanup. A zero process
   return code alone does not prove a robot moved or remained upright.

## P1: preserve the repaired MuJoCo feedback loop

Start from `r52-r53-mujoco-regression-2026-09-30/README.md` and its before/after
JSON. The resolver must forward only requested spawn overrides. Bringup owns
map defaults and robot-specific heights; BHL uses `z=-0.038`, and Celisca starts
at `(0, 1.32)`. Saved manifests containing stale explicit zeroes need regeneration.

MuJoCo's effort-controlled profiles must not replay multiple physics ticks on
one stale torque measurement cycle. `mujoco_simulator.launch.py` forces catch-up
to zero for BHL and Go2. Wheel velocity drives can retain the configured budget.
Do not re-enable blind effort catch-up to make a speed metric look better.
Before any timing change is accepted, repeat a startup hold, bend, walk, stop
and command-loss trial and inspect tilt, joint velocities and effort cadence.

Acceptance already recorded here is bounded: BHL forward walk/stop and Go2
policy walk in Celisca floor 1. BHL's held-turn/low-speed stall and Go2's
terrain/get-up remain open under R5.3/R5.2. Never relabel those as complete
because this regression is repaired.

## P2: GUI, keyboard and joystick control

Inspect `robot_lab_gui/drive_control.py` and `launcher.py`'s Drive callbacks.
The displayed linear/angular values are increments. Held GUI buttons and WASD
ramp up by these increments; releasing input ramps to zero. A second button
click releases its latch. Space must stop immediately. Arming keyboard/joystick
must publish nothing until operator input, and joystick startup axis events
must calibrate neutral rather than become motion commands.

Run `test_command_autofill.py` through Xvfb. Then record a physical joystick
trial: arm untouched; move each axis independently; return to neutral; disconnect;
press Space; switch robot/backend; close the GUI. Observe `/key_vel` and the
post-mux command. Check for another `/joy_vel` publisher masking GUI results.
Missing joystick hardware means this physical acceptance remains pending.

## P3: Celisca scale, spawn and navigation

The world/map scale is already enlarged by 20%. Use these checks before editing:

```bash
python3 src/robot_lab_maps/tools/resize_celisca_maps.py --factor 1.2 --check
python3 src/robot_lab_maps/tools/celisca_extents.py --align
python3 src/robot_lab_maps/tools/gen_mjcf_worlds.py --check
```

Read `r61-furniture-collision-2026-09-30/README.md`. Plain and furniture STLs have
different source dimensions; applying the same literal scale is wrong.
Furniture collision proxies exclude the floor surface overlapping the world's
physical floor. Keep full meshes as visuals and the smaller wall/furniture
proxies for collision. Regenerate derived MJCF after any SDF geometry change.

Verify robot and map-only displays for all six Celisca variants in every backend.
Check 3D/map alignment in RViz and collision clearance at the selected spawn.
Then use `scripts/sim_nav_check.sh` for Bumperbot and Labbot in plain/furnished
floors 1 and 2. The existing six results cover short clear-corridor goals only.
Add an obstacle route with at least two turns, a goal beside furniture, goal
cancellation, and a second goal in the same run. Inspect controller activation,
URDF errors, localization, TF ownership, costmap obstacles and `/cmd_vel` routing.

The furnished PGMs currently duplicate plain-floor occupancy. Generate/check a
furniture-aware static occupancy map, or explicitly qualify local-costmap
avoidance of furniture absent from the global map. Record that choice. Follow
with GUI Drive checks in Localization, 2D SLAM and RGB-D SLAM. An unavailable
2D map error must identify a truly missing asset, not reject every selection.

## P4: improve MuJoCo speed without destabilizing control

Measure simulated `/clock` advancement against monotonic wall time on each
named map, with and without viewers. Profile collision, scan raycasts, camera
rendering and publication cost. The repaired effort path can run slowly on a
large mesh; this is an explicit open item.

Prefer reducing validated collision geometry or moving expensive sensing off
the feedback loop. Preserve visual assets and sensor obstacle fidelity. If
control is moved into a simulator-synchronous callback, document its observation,
inference and PD cadence and repeat P1's matched trials. Require both improved
real-time factor and unchanged safety/motion acceptance. CPU utilization alone
is not a speed result.

## P5: four-wheel steering

Existing code is in `robot_lab_utils/drive_kinematics.py`,
`robot_lab_robots/holonomic_wheels/`, and the Gazebo
`robot_lab_controller/holonomic_controller.py`. October 2 exact rolling/pivot
and navigation results supersede the earlier weak-pivot diagnosis; read
[the continuation report](status/continuation-2026-10-02.md) first. The steering drive supports
opposite-phase/Ackermann, crab/in-phase and pivot targets. The launch/GUI
selector now exists; verify its value also reaches the resolved manifest. Selecting a mode must change
the controller law, not merely a label.

Gazebo needs joint-group position/velocity controllers and measured wheel/steer
odometry. The hardware plugin class is `ign_ros2_control/IgnitionSystem` for
Fortress and `gz_ros2_control/GazeboSimSystem` for the gz path. The system plugin
class name is not a hardware interface class. Ensure new scripts are executable
when symlink-installed. Check all controllers are active and their joints exist.

For each backend and steering pattern, record forward/reverse, turn or crab,
stop, command loss, reset and joint limits. Use physics truth to detect wheel
slip; kinematic wheel odometry alone cannot prove body travel. Pivot must change
yaw with bounded translation; crab must show lateral body motion at nearly
constant yaw. Then record Localization, 2D SLAM, RGB-D SLAM and Nav2 goals.
Planner/controller defaults must suit the selected steering pattern. The GUI
now selects Smac2D/DWB for this base. Crab/in-phase use `omni_parallel` with
`ParallelSteeringCritic`, which rejects mixed translation/yaw. Never switch
back to unconstrained RPP for parallel steering: the diagonal trial aborted
with no progress. Include a final-heading goal in every repeat.
The named crab screen passes on all four backends after Isaac's 4WS `vy`
forwarding repair; this does not close the other-pattern matrix. The newer
probe reports independent final yaw explicitly. Isaac's 16.13° truth-heading
error remains a precision gap even when Nav2 accepts its localization estimate.

## P6: physical mecanum wheels

The MIT FUJI passive roller model is now imported with a pinned source/license,
and display hold no longer pins its roller joints. Physical lateral travel is
measured on MuJoCo and PyBullet; GUI strafe controls exist. Preserve that plant
and inspect `status/evidence/r56-holonomic-measure-2026-10-01/README.md` before
changing contact parameters. Gazebo now measures 0.278 m/s lateral body motion and a short Nav2 goal
with the validated 68-face convex contact proxy. Remaining backend/mode/map
missions are listed in the continuation report.
The same proxy now has MuJoCo/PyBullet lateral movement repeats at
0.253/0.276 m/s; qualify their navigation and obstacle routes separately.
Keep roller geometry, wheel axis signs and joint order consistent across
URDF/MJCF/USD imports.

Measure pure `linear.y` in both directions, forward/reverse, diagonal travel,
yaw, stop and command loss against physics truth. Check bounded vertical motion
and roller contact. Do not implement lateral motion by directly setting the
robot pose. Keep the tested lateral Drive controls and use
holonomic Nav2 controller/velocity limits; then test all mapping/localization/
navigation modes on a clear and obstacle map per backend.

## P7: additional humanoid and quadruped policies

Use `docs/status/policy-px4-candidates-2026-09-30.md` and the primary upstream
[Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite)
and [Unitree RL Mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab)
projects as starting points. Search model-specific upstream sources when a
listed robot lacks a deployable checkpoint. Record negative searches too.

Before importing code, record license, commit, robot model, checkpoint source,
joint/action order, observations/history, normalization, command ranges,
action scale, initial pose, policy rate, PD gains, actuator losses and limits.
Test inference parity and a zero-command state on the exact local model.
A checkpoint for Go2 or BHL is not interchangeable with another robot.

Adapt descriptions/sensors/effort commands and verify measured joint states
reach RViz. Start with bounded stance and stop, then walk/turn, then terrain.
Only enable SLAM/navigation after a reliable velocity interface and actual
sensor/estimation stack pass. PyBullet passive localization does not imply a
MuJoCo walking policy runs there. Gazebo/Isaac modes require their own actuation
and sensing proof; retain a clear unavailable reason until that exists.

## Documentation, GUI and CI completion

After each patch, update evidence, the ledger, roadmap and tutorials in the same
logical change. Health → Platform Status displays `priority_patches` and task
states; command preview must match what Run executes, including steering/policy
options and selected algorithms. Rebuild copied GUI modules and reopen the GUI.

Run required focused tests, the combined fast suite and registry validation,
then inspect GitHub Actions on the exact published revision when publishing is
authorized. Diagnose the actual failing job/log; do not silence a test or label
an untested backend successful. R5.4 now has measured native/ROS2 X500 Flight acceptance. Use the
[PX4 guide](tutorials/px4_x500.md); broader aerial mapping/planning remains open.
