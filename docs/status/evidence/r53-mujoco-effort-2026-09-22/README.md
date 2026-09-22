# Live MuJoCo humanoid effort control — 2026-09-22

The BHL standing node now controls the actual MuJoCo ROS backend through the
common `simulated_robot.launch.py` entrypoint. Previously the spawner accepted
only differential-wheel velocity commands; native-model controller probes did
not establish a working articulated command path in this backend.

Base revision: `4edd8c9`, with the working-tree changes fingerprinted in
`manifest.json`. This is live stance/effort/reset qualification, not walking,
perturbation recovery, terrain, localization or whole-humanoid qualification.

## Implementation

- Read the effort-controller YAML joint order, create one torque motor per
  named revolute joint, and apply URDF effort limits, friction and damping.
  Reject missing/already-actuated joints and incomplete/non-finite commands.
- BHL launch automatically selects its effort configuration and 250 Hz physics
  ticks/state publications. The existing balance node reads IMU and joint states
  and publishes its PD efforts; it receives no ground-truth odometry feedback.
- Clear effort after 0.5 wall seconds without a valid command and on reset.
  Publish measured generalized actuator effort instead of a fabricated zero.
- Import the standing node's missing shutdown exception type. Shield MuJoCo's
  thread cleanup against duplicate SIGINT delivery from a process-group stop
  followed by ROS launch forwarding the signal.

## Measured comparison

Both conditions use the URDF model, corrected -0.038 m spawn height, nav_empty,
the same joint losses and 12 simulated seconds. Only the active condition starts
`humanoid-standing-controller` before spawning. No random seeds are injected.

| Measurement | Active balance | Passive comparison |
|---|---:|---:|
| Maximum tilt | 0.001816 rad | 1.904760 rad |
| Maximum actuator effort | 0.757827 N.m | 0 N.m |
| Effort after stopping controller and watchdog expiry | 0 N.m | 0 N.m |
| First observed joint-state time after reset | 0.004 s | 0.004 s |
| Effort after reset | 0 N.m | 0 N.m |
| All owned processes exit cleanly | Yes | Yes |
| Stance criterion: tilt < 0.2 rad | Pass | Fail, expected control condition |

The passive robot is already at 0.962 rad tilt by 2 simulated seconds. This
rules out passive joint friction alone as the explanation for the active hold.
It does not by itself prove recovery from an externally applied disturbance.

Full traces, reports and process logs are in `active/` and `passive/`. The
probe checks message presence, complete finite 22-joint state, the simulation
budget, effort bounds, watchdog, reset and process cleanup. Passive mode exits
1 because it fails the same stance criterion; this failure is retained.

The initial active trial held stance but crashed during shutdown: launch's
second SIGINT interrupted the physics-thread join and native teardown segfaulted.
`initial-shutdown-failure.log` preserves that negative finding. After rebuilding
the cleanup fix, both final trials exit cleanly. Only those final reports qualify.

## Reproduce

From the workspace root, after building the changed packages:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 MUJOCO_GL=egl
export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:$PYTHONPATH"
ROS_DOMAIN_ID=186 .venv/bin/python \
  docs/status/evidence/r53-mujoco-effort-2026-09-22/probe.py \
  /tmp/bhl-active-new active
ROS_DOMAIN_ID=187 .venv/bin/python \
  docs/status/evidence/r53-mujoco-effort-2026-09-22/probe.py \
  /tmp/bhl-passive-new passive
```

For an interactive run, source the same environment and use the same isolated
domain in two terminals. Start the standing node first:

```bash
ros2 run robot_lab_adapter humanoid-standing-controller \
  --ros-args -p use_sim_time:=true
```

Then start the robot (set `gui:=true` for the viewer):

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=display map_name:=nav_empty robot_model:=berkeley_humanoid_lite_sim \
  simulator:=mujoco gui:=false start_rviz:=false
```

## Checks and remaining work

Changed packages built successfully. Focused effort/reset tests: **14 passed**.
Bringup suite plus BHL balance tests: **383 passed, 2 skipped in 27.43 s**;
the 14 focused cases overlap this total. The new effort suite is registered
with CMake/CTest and also passed there: **10 passed** (`ctest.log`), another
repeat of a subset of the 383. The two skips are opt-in live backend
tests; they are not included in the live comparison above. `tests.log` records
the command output. Existing packaging warnings remain.

R5.3 remains active. Next: qualify bounded perturbations and the locomotion
policy on this URDF/ROS effort path, with appropriate PD scheduling and the
policy's operating limits. Earlier native-MJCF policy results are a different
plant/control path. Walking, turning, command tracking, controller-state reset,
other maps and GUI controller orchestration still require their own evidence.
