# Stabilization and remaining work: agent execution guide

Updated 2026-10-08. This guide implements the user's requested patch queue
and preserves its remaining acceptance. The latest user instruction prioritizes
the new extensions first, then resumes this older queue; follow
[ASSET_EXTENSION_GUIDE](ASSET_EXTENSION_GUIDE.md) and the live handoff/task owners.
Read `docs/AGENT_HANDOFF.md`,
`docs/status/platform-status.yaml`, and `ROADMAP.md` first. The user includes
drone integration; retain the measured PX4 Flight path. Existing Bumperbot/Labbot workflows and assets
must remain available.

## Establish the actual baseline

The October 8 instruction sets the next robot workflow order: motion control
for every complete variant, then real SLAM, then navigation across compatible
installed maps. Use the staged acceptance in ROADMAP.md. Complete controller
and sensor contracts before changing GUI mode gates. Missing upstream
checkpoints, unsafe gait trials, unsuited floor/ceiling geometry and failed
routes stay explicit gaps. A stationary manipulator's arm/hand controls are
its motion stage; only a qualified mobile base gains ground navigation.

Read [the 2026-10-02 audit](status/audit-2026-10-02.md) and
[storage guide](STORAGE.md). Generated demonstration scores cannot qualify a
mission. Keep all large artifacts on the mounted workspace SSD.
Use [the staged robot execution guide](AI_ROBOT_READINESS_GUIDE.md) for the
current motion → saved SLAM → navigation order, exact-source mode promotion,
upstream policy reuse and physical regression requirements.

1. Inspect `git status --short` and the task ledger. Preserve existing edits.
   Claim the next patch and record any additional scope paths before changing
   shared code. Use the [October 8 checkpoint](status/continuation-2026-10-08.md), with the
   [current status](status/CURRENT_STATUS.md) and documented source stages; installed packages can still be older
   than source. Read [the latest continuation](status/continuation-2026-10-07.md)
   before repeating completed TurtleBot 3/4 or native Panda implementation.
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

The current read-only Labbot/MuJoCo/hospital-v4 navigation observation measures
1.892 simulated seconds over 14.499 monotonic wall seconds: 0.1305× real time,
with nine scans and ten depth frames. Its
[actual samples and producer](status/evidence/extensions-finish-2026-10-08/mujoco-hospital-performance/report.json)
are retained. This single window establishes neither the cause nor a speed
improvement. Use matched baseline/viewer/sensing conditions before optimizing;
retain the new eight hospital route passes and the unchanged endpoint gates.

Prefer reducing validated collision geometry or moving expensive sensing off
the feedback loop. Preserve visual assets and sensor obstacle fidelity. If
control is moved into a simulator-synchronous callback, document its observation,
inference and PD cadence and repeat P1's matched trials. Require both improved
real-time factor and unchanged safety/motion acceptance. CPU utilization alone
is not a speed result.

The October 8 packing optimization is lossless: four ROS wire cases and the
full 151-case local integration tier pass; actual Labbot navigation meets the
unchanged physical endpoint gates. Whole-simulator clock windows remain about
0.13×. The actual native stage counters average 10.794 ms per step, including
9.353 ms kinematics, 1.234 ms collision and 0.016 ms constraints. All 200 rigid
world flexes contain 109,918 world-bound vertices. Static-world update is the
leading candidate inferred from those measurements. The isolated 3.15 trial
shows no speed gain; keep the default 3.12 environment.

For the next P4 patch:

1. Reproduce the real initialized `(0.55, 12.45, 0.1)` launch and retained
   [native-timer hook](status/evidence/extensions-finish-2026-10-08/mujoco-hospital-performance/native-loop/sitecustomize.py).
   Saved model XML alone loses the runtime spawn; preserve the earlier offline
   default-pose result as a separate diagnostic.
2. Inspect the rigid-world kinematic/flex update or lossless collision-mesh
   simplification. Preserve original visual/sensor surfaces and compare actual
   floor, wall and furniture contacts. Do not skip dynamic body kinematics,
   alter actuator/solver limits or reduce sensing merely to raise a rate.
3. Measure repeated matched clock windows and native stages, with identical
   map, robot, engine, viewer, camera/scan rates and background workload.
   Record callback/profiling overhead separately from ordinary operation.
4. Require both a reproducible real-time-factor improvement and unchanged
   normal GUI forward/reverse/turn/Stop/loss/reset, independent body/floor,
   sensor calibration and short/obstacle navigation gates. If shared effort
   timing or engine behavior changes, repeat P1's BHL/Go2 hold/walk/stop checks.
   Retain every failed candidate; one successful static timing probe cannot
   qualify a robot mission or the full roadmap.

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
with no progress. Include a final-heading goal in every repeat. Use `steering_mode:=PATTERN`;
the bounded wrappers query the running `drive_config` to catch launch typos.
Generic plugin defaults must load before the robot overlay, so DWB cannot
erase its footprint critic or arrival window.
The clear-map screen matrix has all 16 pattern/backend cells and all four
mecanum cells. October 5 crab repeats pass stricter independent terminal limits
(0.15 m / 5°) on every backend. The continuous endpoint-distance critic avoids
the grid-cost plateau that stalled parallel steering near a goal. These predate the final obstacle-scoring changes. Repeat each pattern and
route with current settings and preserve any body-heading failure. Read the
[October 5 measurements](status/continuation-2026-10-05.md) and retain the
earlier aborted hard-gate/latched-critic trials as negative evidence.

Use `scripts/sim_workflow_check.sh` for live localization, mapping, watchdog and
reset checks. A robot reset preserves monotonic `/clock`, then resets the EKF
from fresh raw odometry. Humble SLAM Toolbox 2.6.10 has no graph-reset service:
`slam_supervisor.py` restarts the actual upstream process and acknowledges a
fresh accepted `/pose`. Clearing its queue is insufficient. RTAB-Map uses its
real reset service. Observe motion after reset: a stationary matching pose
alone cannot prove the estimator resumed tracking.

The GUI's **Reset Robot** is enabled for an owned wheeled Display/Localization/
SLAM/3D SLAM launch. It is deliberately unavailable for an active navigation
goal or flight until those reset workflows are qualified. Gazebo restores the
robot's settled pose and controller commands, preserving the world clock;
resetting actors or the entire world requires restarting the launch.

For 2D SLAM, count accepted poses and occupied cells, not merely `/map`
publications. Small bases need 0.1 m / 0.1 rad scan spacing for short Drive
legs. For RGB-D SLAM, require a changing finite XYZ cloud with actual height.
**Save 3D map** calls RTAB-Map's acknowledged backup service, copies the stable
database and exports the real `/cloud_map` to PCD. Verify both files; no
placeholder collision world counts as a reconstructed map.

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

The exact published `091d388` CI passes core build, 933 fast checks (four
skips/four deselections), 20 physics checks, 134 integration checks (twelve
skips) and registry validation. Final local GUI/Drive/Panda evidence is separate.
Keep [current status](status/CURRENT_STATUS.md), README, roadmap, tutorial
indexes, workflow, architecture, testing and package guides consistent. Mark
older reports historical; preserve their actual measurements and failures.

The October 7 redesign places controls/limits in Launch's right-hand column.
See [GUI workspace](tutorials/gui-workspace.md). Controller widgets now belong
to `control_notebook`; keep old `show_tab('Arm'/'Hand'/'Drone'/'Drive')` routes.
After selection/layout changes, verify actual filtered selection, exact command,
neutral enablement, accessible controls at 1600 and 1024 pixels, and owned
Drive/native Panda callbacks. A screenshot alone does not establish motion.

After each patch, update evidence, the ledger, roadmap and tutorials in the same
logical change. Health → Platform Status displays `priority_patches` and task
states; command preview must match what Run executes, including steering/policy
options and selected algorithms. Rebuild copied GUI modules and reopen the GUI.

Run required focused tests, the combined fast suite and registry validation,
then inspect GitHub Actions on the exact published revision when publishing is
authorized. Diagnose the actual failing job/log; do not silence a test or label
an untested backend successful. R5.4 now has measured native/ROS2 X500 Flight acceptance. Use the
[PX4 guide](tutorials/px4_x500.md); broader aerial mapping/planning remains open.

## October 5 world and robot extensions

Follow [ASSET_EXTENSION_GUIDE](ASSET_EXTENSION_GUIDE.md) and R3.6, R5.7–R5.10,
R6.5–R6.7. Catalogs and generated occupancy previews are available in GUI.
All-map flight selection, imports, terrain conversion and manipulation
acceptance remain distinct. New source listings must not close existing
stabilization, measured comparison or clean-host tasks.

The later P4 camera helper uses ROS typed byte arrays with identical serialized
RGB/depth. Four wire cases, 151 sourced integration checks and a new physical
Labbot hospital route pass. Native-step profiling is retained; actual clock
windows stay about 0.13×, so broader MuJoCo optimization remains open. See
[the exact timing/source stages](status/evidence/extensions-finish-2026-10-08/mujoco-hospital-performance/README.md).
