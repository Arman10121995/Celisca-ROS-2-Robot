# myAGV Pro source integration — 2026-11-12

User top priority R14.2. Baseline `df829517` plus the source hashes and actual
profile/world/producer snapshots in each raw GUI report. Host: Jetson AGX Orin,
ROS 2 Humble, Gazebo Fortress/DART. Original and failed artifacts are preserved.
One simulation ran at a time; no builds/tests ran during its recording.

## Source and implementation

Official `elephantrobotics/agv_pro_ros2`, pinned
`d3851c768d2e887eef7ad79fc4dcb9a928b070ac`, BSD-3-Clause. Source CAD chassis,
all wheel inertias/visual meshes and sensor mounts are retained. The source
model is about 20.316 kg, whereas the vendor hardware table lists 40 kg; no
hardware-dynamics calibration claim. See [the guide](../../../tutorials/myagv-pro.md).

The incoming uncommitted two-wheel circular prototype did not represent this
robot. It and its diff are preserved on SSD under `incoming-stub/`; its false
all-backend/SDK/support assertions are not registered. The new complete source
profile is `asset_elephant_myagv_pro`, grouped/classified in the normal GUI.

Declared lab adapter: resolve CAD corners from positions (vendor labels differ),
0.0762 m cylindrical wheel envelopes with chassis-frame directional friction,
four passive tire-compliance surrogates (±12 mm, 10,000 N/m, 100 N·s/m,
0.02 kg total carriers), bounded native wheel control, source-frame simulated
LiDAR/IMU/RGB-D and proper passive-joint feedback. No root-pose driving.
Configured motor envelope is 5 Nm / 8 rad/s, a conservative lab choice rather
than a vendor rating. Combined twists are uniformly scaled to the wheel limit.
The actual converted SDF is checked for all four directions and three sensors
before publishing an installed profile. [Provenance/recipe](provision.json).

## Actual Display motion and sensing

[Run 4](drive.json): 31.921 wall seconds in `nav_empty`, normal `/key_vel` → mux
→ relay → holonomic controller → native wheels, with separate physical body
truth. Measured forward 0.250 m/s, reverse −0.215 m/s (phase average includes
the direction transition), strafe 0.250 m/s and turn 0.234 rad/s. Neutral motion
was zero at reporting precision. Publisher loss coasted 0.110 m and ended at
zero forward/yaw speed. Eight real RGB frames and eight LiDAR messages were
received, plus drive and four passive-joint states. These are named screens,
not an all-map or calibrated tire-dynamics qualification.

## Actual normal-GUI navigation

[Run 3](nav-3.json) selected the source robot, Gazebo, `nav_empty`, Nav and the
normal experimental opt-in in Tk. It invoked Run with autofilled A*, DWB,
native holonomic control, AMCL, EKF and wheel/IMU fusion. A real RViz-style
topic goal traversed `[-0.5,-0.5]` → `[1.5,-0.5]`. Stack readiness 8.3 wall
seconds, action 14.1 wall seconds; success and a settled simulation second were
recorded. Final independent body error **0.125 m / 2.23°**, fresh truth age
0.001 s, 600 body samples. The declared acceptance was 0.15 m / 5°. Original
truth and estimated pose remain separate in the raw report.

This qualifies only that static route on that map/backend/profile/revision.
Localization/2D/3D acquisition, furnished routes, other maps/backends and
hardware operation have no new mission qualification here. Higher modes stay
explicitly experimental in the GUI.

## Retained failures and focused repairs

- [Drive 1](drive-negative-1.json): dropped collision-direction extensions and
  missing native sensor systems; no camera/scan data, no useful strafe/yaw.
  Unnamed URDF collisions now match the sdformat extension target; model sensor
  systems and the native camera topic are configured explicitly.
- [Drive 2](drive-negative-2.json): sensors/yaw arrived but translation coupled
  along a diagonal. [Drive 3](drive-negative-3.json): baking the cylindrical
  envelope into a mesh did not repair control. That mesh approach is not the
  delivered plant. Passive compliance resolved the four-point contact defect
  in run 4; its measured travel is published without commanding the springs.
- [Nav 1](nav-1.json): Nav2 reported success at 0.238 m / 7.65°, failing the
  independent arrival limits. The checker was not weakened.
- [Nav 2](nav-2.json): tighter goal-checker tolerance alone left DWB's own
  RotateToGoal window at 0.25 m and it aborted without enough progress. The
  [upstream critic](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp)
  reads a separate threshold and latches rotation. Both thresholds now agree;
  fewer sampled rollouts and 10 Hz control reduce Jetson overload. Run 3 passes.
- The first source importer attempted to read URDF mass as element text;
  corrected to the `value` attribute before registration. The first converter
  test used `/dev/stdin`, which this sdformat CLI cannot read; it now uses an
  actual SSD test file.

[Sixty focused checks](checks.log) pass (source frames/inertias, actual
sdformat directions/systems/springs, wheel-limit inverse consistency, existing
drive kinematics and timeout handling). Changed ROS packages build. These
software checks do not qualify more missions.

The probes invoked bounded owned-process cleanup. Gazebo sometimes required
SIGTERM after its SIGINT timeout; shutdown-only RViz/joy-teleop messages are
retained. Their success/exit status is not treated as motion evidence.
Raw traces, actual descriptions/world hashes, full source manifests, joint
and image records, producer scripts and logs remain on SSD:
`/workspace/molar/robot_lab_runtime/myagv-pro-20261112/`.

## Next implementation boundary

Keep R14.2 ahead of terrain/older tasks. Other backends need portable wheel
contact/actuator adapters before being enabled; this Fortress-specific
anisotropy is not silently reused there. Mass/tire calibration, vendor
charging/serial operation and complete arm-equipped composites remain
separate. Preserve the current source variant and its scoped receipts.
