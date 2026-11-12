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

## BlueROV2 Heavy source candidate

The separate `asset_bluerov2_heavy` is installed with the original Heavy hull, eight
propellers, 13.016 kg mass, 0.01313875 m³ collision displacement, source inertias and
mesh scales. SDF 1.6 link-frame joint poses convert explicitly to URDF; unreviewed
relative frames/topologies are rejected. Native Fortress buoyancy/hydrodynamics and
thrusters retain source coefficients. Source normalized control endpoints yield
±50 N motor bounds; the existing lab allocator replaces ArduSub, with a native timeout.
Original FRD IMU data is kept separate from the declared massless lab ENU IMU/RGB-D kit.

The exact pinned package declares MIT but lacks a root license file; third-party
mesh redistribution review remains pending. This is a locally cached experimental
candidate, not an unconditionally adopted/redistributable source release.

[Provision receipt](bluerov-provision.json) records source/model/mesh hashes and the
full allocation matrix. [Measured summary](bluerov-drive.json) records `bluerov-drive-2`:
50.826 wall seconds, 1.1373 m forward travel, 1.4250 rad yaw and real 320×240 images.
All source joints are reported. Final neutral hold continued 0.1718 rad yaw over
3.90 s; no instantaneous Stop claim. Actual URDF, active world, native watchdog
binary hashes and source snapshots are preserved on SSD. The first run's relative
world texture error is retained; cloned worlds now resolve PBR texture paths.

Two source-coordinate tests and a two-package build (2.78 s) passed. The normal GUI
includes the source family/classification, explicit experimental Marine gate and
optional RGB-D mapping command autofill. Depth/strafe, maps/routes, message loss,
contact, original FRD/ArduSub parity and hardware acceptance remain unqualified.
[Operator guide](../../../tutorials/bluerov2.md).

## Robotnik RB-SUMMIT-STEEL source mecanum base

`asset_robotnik_summit_steel` preserves the complete BSD-3-Clause source base at
`1a7aff1310dbc4427e0b2ae5ef83e0ffd2094504`: 151 kg, original geometry/inertias,
four wheel motors and the manufacturer's sphere/anisotropic-contact model.
The Fortress ros2_control ABI adapter retains the original ±10 rad/s front and
±20 rad/s rear command interfaces. A declared massless lab sensing kit adds actual
LiDAR, ENU IMU and RGB-D without claiming the optional vendor sensor hardware.

The source `gz:expressed_in` spelling is ported to Fortress's literal
`ignition:expressed_in`. The source direction vectors, coefficients and original
`base_footprint` frame remain unchanged. That frame is the physical DART body after
fixed-joint merging. The XML asset resolver preserves the native prefix instead of
rewriting it to `ns0`, which previously made the SDK ignore the friction reference.
Physical kit poses and the bridge account for the same fixed-joint merging.

Actual `summit-drive-4`: one Gazebo `nav_empty` Display source control screen, through
`/key_vel`, 31.362 wall seconds. Measured forward 0.250 m/s, reverse -0.243 m/s,
lateral 0.250 m/s and yaw 0.250 rad/s. Eight actual images and eight LiDAR samples
arrived, with 476 recorded wheel-state samples. Maximum wheel rate was 1.969 rad/s.
Input-loss travel was 0.112 m over 3.0 s, with zero final forward/yaw speed. The lease
now resets wheel motion immediately on expiry, including after a frozen ROS clock;
four-wheel steering retains its last angles. No body pose is written to implement driving.

[Measured summary](summit-drive.json), [source/model receipt](summit-provision.json),
[GUI command receipt](summit-gui.log). Actual runtime URDF, retained source snapshots
and full body/joint trajectories remain on the SSD under the R14 directory above.
The measured summary pins their hashes. Metadata/algorithm autofill was checked after
the physical Display run; no Navigation/SLAM mission is claimed from that GUI check.

Retained negatives: `summit-drive-1` exposed a version-specific friction-frame failure;
`-2` still lost the SDK prefix at description loading and had a camera attachment bug;
`-3` fixed source motion/sensing but travelled 0.183 m after publisher loss because the
old controller added a normal deceleration ramp to an expired command. `-4` corrects
that behavior without weakening the existing 0.15 m travel criterion. Root/namespace,
watchdog, sensor, description and kinematics checks: 78 passed; adapter/dispatch checks:
71 passed. Four-package builds completed in 13.2 and 16.7 seconds. These are separate
software results, not full source mission qualification.

The normal GUI classifies one complete Robotnik family with strafe controls. Higher
modes require explicit experimental opt-in and autofill compatible algorithms, including
the actual holonomic wheel controller rather than the differential-drive PID label.
A source-sized Nav2 footprint is implemented. Contact fidelity, other maps/backends,
physical joystick, SLAM/3D SLAM/localization and navigation missions remain unqualified.
[Operator guide](../../../tutorials/summit-steel.md).
