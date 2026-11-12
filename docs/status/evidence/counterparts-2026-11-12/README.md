# R14.1 source counterparts — November 12, 2026

## TurtleBot4 Lite cleaning derivative

`asset_turtlebot4_lite_cleaning` is a separate installed variant of the pinned official
TurtleBot4/Create3 description. Original profiles and certificates are unchanged.
Source chassis, wheels, spring suspension and hazard geometry are preserved. The lab
restores the source bumper plus four native GPU cliff scans at 20 Hz, bridges their
actual source names and pauses cleaning on missing/stale cliff feedback. The 0.08 m
cliff threshold comes from the source Create3 sensor contract. Cleaning, battery and
coverage remain laboratory models; the robot has no physical vacuum brush.

Actual run: Gazebo Fortress, `nav_empty`, source compliant suspension with a 2 ms
maximum step, AMCL/Nav2 and one zone `[-1,-1,0.2,0.2]`, detector disabled. It completed
and docked: 96.8% reported target coverage, 2.08 m² reported cleaned area, 13.2 m
reported travel, 99.6 s cleaning duration. One stall was detected and recovered.
All four physical cliff scans were fresh (0.04 s age) at the end. Bumper advertised,
but no contact occurred; this does **not** qualify bumper impact detection.

[Measured summary](turtlebot4-cleaning.json) preserves independent body samples and
final truth separately from estimated pose. [Source receipt](turtlebot4-provision.json)
pins original/derived bytes and dependencies. Raw trajectory/logs remain on the SSD:
`/workspace/molar/robot_lab_runtime/r14-implementation-20261112/`.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_robot_counterparts.py --robot turtlebot4_cleaning
ROS_DOMAIN_ID=144 TIMEOUT=220 bash scripts/sim_vacuum_mission.sh gazebo \
  asset_turtlebot4_lite_cleaning nav_empty /workspace/molar/robot_lab_runtime/new-cleaning-run.json \
  '{"mode":"zones","zones":[[-1,-1,0.2,0.2]],"edge":false}' \
  allow_experimental_assets:=true vacuum_detector:=none
```

The GUI Vacuum Setup exposes the installed source variant and an explicit experimental
opt-in; launch autofills the command and synchronizes the session mode. Fresh runs keep
the shared installed assets visible while isolating their cleaning history directory.
Other maps/backends, bumper contact, cliff edges and learned-camera cleaning remain
unqualified. This is one measured zone mission, not completion of R14.1.

## VRX WAM-V H configuration

`asset_vrx_wamv_h` installs the complete Apache-2.0 VRX WAM-V H source, revision
`dc30ed8d17aa1083fd872edad9c77c69896d2b07`: 258 kg total mass, original hull/engine/
propeller/battery geometry and source collision primitives. Its upstream Garden
Surface, Wavefield and SimpleHydrodynamics code builds in a private SSD Fortress
prefix. Only header/library/namespace names are ported; the force equations remain.
Root license, source files, derivative hashes and binary hashes are retained in the
[provision receipt](wamv-provision.json). No system Gazebo replacement is installed.

The lab adds massless RGB-D/ENU IMU frames, existing marine estimation/mission API,
zero azimuth hold for H differential thrust, profile-specific feedback gains and
source drag feedforward. Native thrust bounds also respect the stricter source
propeller joint limit: 100 rad/s yields ±70.752 N with the source propeller model.
The native wall-clock watchdog preserves asymmetric force bounds and removes stale
commands. Water density/absolute surface are taken from the selected world; the
upstream analytic surface plugin supplies buoyancy, without a second buoyancy system.
The source's geometric collision volume is **not** its analytic displaced-water volume.

The clear-water run `wamv-drive-4` completed neutral, forward, neutral brake, turn and
neutral brake in 46.057 wall seconds: 1.8785 m forward travel and 0.7413 rad yaw during
the turn; maximum measured propeller speed remained at 100 rad/s. Fresh real 320×240
camera images and all four joint states were received. Final neutral-brake planar
motion was 0.0475 m over 3.90 s. This is not a navigation, contact, native-publisher-loss
or wave qualification. [Measured summary](wamv-drive.json), [actual session](wamv-session.json).

Retained trials: `wamv-drive-1` failed on a preserved massless camera link; `-2`
exposed missing battery visual URI resolution and insufficient small-craft yaw gains;
`-3` exposed the large hull reaching the dock from the generic small-vessel spawn.
The source-specific `surface_harbor` spawn is now (-5,0,0), with the original world
geometry retained. Raw reports/logs/source snapshots remain in the SSD directory above.
Other water worlds and acquired-map missions remain unqualified.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_robot_counterparts.py --robot wamv --download
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_vrx_wamv_h simulator:=gazebo mode:=marine \
  map_name:=surface_harbor allow_experimental_assets:=true
```

The normal GUI lists **VRX WAM-V → Source H layout**, under **Surface vessels**.
Enable experimental source assets, select Gazebo/Marine and `surface_harbor`.
The run command, source-specific spawn and optional RGB-D SLAM settings autofill.
Connect Marine telemetry, then Arm; surge/yaw start neutral. Depth and strafe are
unavailable on this surface craft. Stop removes propulsion; buoyancy and coasting remain physical.
