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

## Portable roller variant: MuJoCo, PyBullet, Isaac (2026-11-12/13)

`asset_elephant_myagv_pro_rollers` is a separate profile for the three
self-contained backends; the Gazebo Fortress profile is unchanged. Each driven
wheel keeps the vendor CAD visual and the original wheel mass, centre of mass
and inertia, decomposed into a core plus 15 free passive rollers in the
FUJI mecanum geometry (`DaiGuard/fuji_mecanum` at
`646431a5e107448e0e2bdeeba4575db9aa3665ff`, MIT). It is a declared lab
derivative (`robot_lab_utils/myagv_rollers.py`), not vendor tyre calibration.

PyBullet first imported 25.3 kg instead of the source 20.336 kg: Bullet gives
every empty TF frame a default 1 kg ([retained import report](rollers-bullet-import-2.json)).
`urdf_inertia.bullet_frame_inertials` now gives geometry-free fixed frames zero
inertia (and the floating root 1e-6 kg), refuses to invent inertia for any
geometry or movable link, and the spawner stops if the imported mass differs
from the source. The repeat imported **20.335907 kg (expected 20.335907)**.

Display drive screens, `nav_empty`, the normal `/key_vel` input path,
independent simulator truth (`scripts/screen_ground_counterpart.py`, 0.25 m/s
or rad/s requests); raw traces stay on the SSD, summaries here:

| Backend | Forward | Reverse | Strafe | Turn | Neutral | Publisher loss | Sensors | Summary |
|---|---|---|---|---|---|---|---|---|
| MuJoCo (Codex run 4) | 0.238 m/s | −0.213 m/s | 0.216 m/s | 0.238 rad/s | still | stopped | RGB, LiDAR, joints | [summary](rollers-mujoco-4.summary.json) |
| PyBullet | 0.248 m/s | −0.202 m/s | 0.234 m/s | 0.248 rad/s | still | coasted 0.156 m, stopped | 8 RGB, 8 scans | [summary](rollers-pybullet-1.summary.json) |
| Isaac Sim | 0.247 m/s | −0.248 m/s | 0.246 m/s | 0.246 rad/s | still | coasted 0.104 m, stopped | 8 RGB, 8 scans | [summary](rollers-isaac-1.summary.json) |

Navigation (experimental opt-in), PyBullet, `nav_empty`,
`scripts/sim_nav_mission.sh`: **passed**: detour route, cancel, resume
(0.034 m from goal A), near-obstacle goal B (0.118 m). [Report](rollers-pybullet-nav-1.json).

Scope: named screens on one empty map; other maps, furnished routes, SLAM and
MuJoCo/Isaac navigation are not yet measured, and higher modes stay
experimental in the GUI.

## myAGV Pro + myCobot Pro630 (R14.3, 2026-11-13)

`asset_elephant_myagv_pro_pro630` is the roller base with Elephant Robotics'
official six-axis Pro630 fixed to the deck rails
(`provision_myagv_pro.py --variant pro630`, [receipt](pro630-provision.json)).
Arm source: `elephantrobotics/mycobot_ros2` at
`d42ff61a78122c79246623391540d75738b03b23` (BSD-2-Clause), the
`mycobot_pro_630.urdf` the official `mycobot_630_moveit2` package uses.
Kinematics, axes, position limits, collision primitives and visual meshes are
the source's; meshes are converted from millimetre Collada to metre STL.

The source has no inertials and zero velocity limits, and the vendor publishes
no arm weight or joint speeds. Declared lab estimates
(`robot_lab_utils/myagv_pro630.py`): 8.0 kg spread by convex-hull volume
(base 3.22, links 0.48/1.60/2.00/0.30/0.37/0.03 kg), 1.0 rad/s per joint,
effort caps 1.5 × (gravity torque with the rated 2 kg payload + inertia × 2
rad/s²) with a 5 N·m floor (J2 43.4, J3 18.9 N·m), and the mount centred on
the rails at 0.220 m. Composed mass 28.336 kg; the PyBullet import matched it.

Arm control is the shared URDF articulation contract
(`/molar/articulation/*`, 0.5 s heartbeat watchdog, Stop, base interlock).
The arm's reaction rocks the light compliant base up to 0.06 m/s while it
stands still, so this profile declares `base_motion_limits: [0.10, 0.20]`
(contract cap 0.15 m/s, 0.3 rad/s) instead of the default 0.025/0.05.

Display screens on `nav_empty`: drive (`screen_ground_counterpart.py`, the
normal `/key_vel` path, simulator truth) followed by the arm
(`screen_urdf_arm.py`: each joint +0.5 rad and back, Stop part-way through a
1.5 rad move, heartbeat withheld during a long move):

| Backend | Forward | Reverse | Strafe | Turn | Arm joints settle | Stop holds (drift) | Watchdog holds | Summary |
|---|---|---|---|---|---|---|---|---|
| PyBullet | 0.244 m/s | −0.206 m/s | 0.228 m/s | 0.243 rad/s | 1.4–2.1 s, ≤ 22 mrad | yes (0.2 mrad) | yes, 3.0 rad short | [summary](pro630-pybullet-2.summary.json) |
| MuJoCo | 0.238 m/s | −0.211 m/s | 0.223 m/s | 0.235 rad/s | 0.6–0.7 s, ≤ 29 mrad | yes (0 mrad) | yes, 2.8 rad short | [summary](pro630-mujoco-2.summary.json) |
| Isaac Sim | 0.247 m/s | −0.247 m/s | 0.246 m/s | 0.243 rad/s | 1.1–1.3 s, ≤ 20 mrad | yes (0.2 mrad) | yes, 2.9 rad short | [summary](pro630-isaac-2.summary.json) |

Every run published 8 RGB images and 8 scans; neutral stayed still and the
publisher-loss phase stopped the base.

**Normal GUI**: the new *Mobile arm* page (`robot_lab_gui/urdf_arm_controls.py`)
appears for any profile whose articulation declares `panel: arm`, one row per
configured joint. `scripts/qualify_urdf_arm_gui.py` drove it on PyBullet through
the launcher's own Run path and the page's widgets: Apply J1 0.5 rad (settled
1.45 s), Jog J2 +0.2 rad (0.95 s), Home (1.72 s), Stop part-way through a 2 rad
move (held at 0.325 rad, 0.1 mrad drift), Home again. Runs 8 and 9 both
passed ([report](pro630-gui-pybullet-9.json)).

Retained failures and repairs:

* [PyBullet run 1](pro630-pybullet-1.summary.json): J2 and J5 froze; the arm's
  reaction tripped the default base-motion interlock. Repair: the per-profile
  limits above.
* MuJoCo run 1 did not spawn (`'URDFArticulation' object has no attribute
  'get'`); `ArticulationROS` now reads the limits from the parsed config.
* [Isaac run 1](pro630-isaac-1.summary.json): forward 0.019 m/s, reverse
  −0.014 m/s. The screen began while the arm still settled after spawn, so the
  interlock (correctly) held the base. A probe that waited showed 0.245 m/s
  forward with the interlock never tripping. Repair: the drive screen now waits
  for the interlock to release (30.6 s of wall time in Isaac).
* GUI runs 2, 4–7: the check script's own screenshots blocked the Tk loop for
  about 1 s. The page then refused commands on stale feedback, and once the
  plant's heartbeat watchdog held and rejected Home. Both refusals are the
  designed behaviour; the script now waits for fresh feedback after each
  screenshot.

Scope: Display screens on one empty map. Not yet done: a gripper (Elephant
publishes no Pro630 gripper description; myGripper F100 pending), MoveIt
planning, mobile pick-and-place, navigation with the arm, and physical
qualification. Mass, speeds, efforts and mount are declared estimates.
