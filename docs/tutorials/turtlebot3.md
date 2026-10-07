# TurtleBot3 in Robot Lab

## Run

The official Humble Burger, Waffle and Waffle Pi descriptions and their
simulation definitions are already downloaded on the workspace SSD. Select
`asset_turtlebot3_burger`, `asset_turtlebot3_waffle` or
`asset_turtlebot3_waffle_pi` in the normal **Launch** tab. Choose a simulator,
`nav_empty` and Display. The command fills automatically; use **Run**, then
open **Drive**. An already running GUI must be restarted after rebuilding.

Enabling keyboard/joystick input sends no motion until operator input. Hold
**W/S** for forward/reverse or **A/D** to rotate. GUI Drive buttons use the
same channel. Release ramps the requested velocity down; **Space/Stop** sends
zero immediately. The physical wheel controller still obeys its acceleration
limits. The displayed linear/angular values are increments, not current
speed. Physical joystick qualification remains separate from these measured
keyboard trials.

The lab controller caps translation at 0.18 m/s, acceleration at 0.3 m/s²,
rotation at 1.2 rad/s and angular acceleration at 2 rad/s². Its conservative
caps preserve the small original chassis and caster geometry. No base-pose
writes implement driving.

From a sourced workspace:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_turtlebot3_burger simulator:=mujoco \
  map_name:=nav_empty mode:=display gui:=true
```

Use the Waffle/Pi IDs above or `gazebo`, `pybullet` and `isaac` for the other
installed engines. The named `nav_empty` spawn is (-4,-4,0 yaw), where the
original short-range LDS can see walls. GUI preview, copied command, execution
and estimator initialization use that same spawn. Explicit `spawn_x`,
`spawn_y`, `spawn_z` or `spawn_yaw` launch arguments override defaults. This
robot-specific override applies only to `nav_empty`; other maps retain their
own spawn metadata.

## Sensors, mapping and navigation

The source LDS produces 360 rays in `base_scan` with a 0.12–3.5 m range at
5 Hz. The IMU uses `imu_link`; actual TF mounts are checked against the
executed URDF. The source simulation files define RGB-only cameras. Camera
geometry is retained, but RGB rendering is pending and no depth stream is
fabricated. **3D SLAM remains unavailable** until an appropriate camera and
actual mapping/export workflow exist.

Localization, 2D SLAM and Navigation availability is determined by immutable
backend-specific runtime reports and control/geometry fingerprints. Use the
modes enabled in Launch. Compatible defaults are AMCL, Slam Toolbox and
Nav2 A* / Pure Pursuit respectively, and the command is filled automatically.
The [mode evidence](../status/evidence/turtlebot3-modes-2026-10-07/README.md)
names the measured maps and backends; a selectable additional map does not
inherit those results.

In Localization or SLAM, use Drive to move and **Reset Robot** to return to
the selected spawn. Reset preserves monotonic simulation time and reseeds
the estimator. SLAM restarts its actual upstream graph and resumes accepting
poses; **Save Map** writes the measured occupancy YAML/PGM. In Navigation,
give a goal through RViz. Clear and blocked-direct-path routes use independent
engine body truth, 0.15 m / 5° final limits and at least 0.02 m swept static
obstacle clearance. These bounded screens do not qualify longer multi-goal
missions or every imported/furnished world.

The 3.5 m lidar range matters: starting at the center of a large empty arena
can leave no visible walls for AMCL or SLAM. Choose a free spawn near real
geometry rather than silently extending the vendor sensor range. Burger's
navigation envelope is 0.185 m; Waffle/Pi use 0.26 m. These conservative
envelopes include body corners, rather than only wheel separation.

## Source and physics contracts

Sources are pinned and licensed Apache 2.0:

- [ROBOTIS description](https://github.com/ROBOTIS-GIT/turtlebot3/tree/90a68bd2e3c61c12966779da89d8eeaec82730e9).
- [ROBOTIS simulation definitions](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/a35a56c8b04877dc89772b598084d8ce648a9023).

The original files remain unchanged. A separate `drive.urdf` and controller
YAML add lab velocity control. Wheel radius is 0.033 m; the executed tracks
are 0.160 m for Burger and 0.288 m for Waffle/Pi. Waffle's source SDF rounds
the track to 0.287 m; control follows the actual URDF wheel centers. The
Fortress derivative attaches the source scan/IMU at their local mounted
origins, avoiding a second model-relative sensor offset.

Undefined root/sensor-frame inertias receive a 1e-6 kg / 1e-9 kg·m² explicit
regularizer so PyBullet does not invent 1 kg / 1 kg·m². Authored inertias are
preserved; source camera mounts also omit inertia. MuJoCo uses profile-local
small-wheel armature/gain and ramps watchdog deceleration. Defaults for
existing robots remain unchanged. The [final Drive evidence](../status/evidence/turtlebot3-sensors-2026-10-07/README.md)
retains the original negative inertia/braking stages and actual physical
traces, source/installed hashes and measured RTF.

This is source-backed lab simulation control. It does not integrate OpenCR
firmware, vendor navigation tuning, camera rendering, docking or hardware.
Other maps, longer routes, contacts, material parity and hardware need their
own evidence before broader support claims.

## Agent/bootstrap refresh

Operators do not download models in the GUI. To reproduce the installation:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_extension_assets.py --kind robots --source turtlebot3_vendor
# Reuse the installed pinned sources without network downloads:
python3 scripts/provision_extension_assets.py --kind robots --source turtlebot3_vendor --no-download
```

Read [the extension execution guide](../ASSET_EXTENSION_GUIDE.md) before
changing physics or mode claims. An import failure preserves the existing
working profile; it does not qualify the failed replacement. Archive actual
failed trials and exact pre-trial geometry/parameter hashes, then add only
passing named mode cells to the support certificate. Recheck normal GUI
availability/defaults/autofill after provisioning and keep the roadmap partial
until its full acceptance passes.
