# Native tracked vehicle

## Run

Select **Mobile robots → Tracked vehicle** in the GUI. Then choose a
simulator (Gazebo, MuJoCo or PyBullet), a terrestrial map and a mode:
Display, Localization, SLAM, 3D SLAM or Navigation. Drive uses the normal
control column. From a terminal:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py robot_model:=lab_tracked \
  mode:=nav map_name:=nav_obstacle simulator:=mujoco    # or gazebo / pybullet
```

Three variants are nested under the family:

| Variant | Flippers | Use it for |
|---|---|---|
| `lab_tracked` | the source's, fixed at zero | flat-ground driving, mapping and Nav2 |
| `lab_tracked_articulated` | four ±0.8 rad joints, 4 Nm force PID | flipper control; too weak to climb |
| `lab_tracked_strong_flippers` | the same joints, 40 Nm PID | steps and stairs under automatic flippers |

All three keep the complete Gazebo tracked source's geometry, mass, belts
and sensors (a 360-ray lidar, an IMU and a 320×240 RGB-D camera).

## Tracks and flippers

Choose an articulated variant to open **Tracks** in the right control column.
Connect sends no angle or Drive input.

- **Use measured angles** copies real feedback. Edit the four degree targets
  and **Apply bounded targets**, or **Raise ends**.
- Targets slew at 0.2 rad/s within ±0.8 rad.
- **Hold flippers / stop Drive** keeps the current motor targets and stops
  Drive.

**Automatic terrain flippers** runs a policy on the robot's own depth camera
(the terrain 0.55–1.4 m ahead, plus the floor beyond an edge out to 2 m) and
on IMU pitch and roll. Its states:

- travel: flippers at rest;
- approach: front flippers up, rear pressing;
- climb: front up, rear down; with `flipper_full_press` (the strong
  variant) both pairs press to the joint limit;
- descend: front flippers reaching down;
- ledge: forward drive stops before a drop that is unseen or deeper than
  0.25 m;
- hold: Drive stops when roll goes above 0.35 rad.

Any manual target or Hold ends the policy.

What it does, measured by ground truth:

| Course | Result | Receipt |
|---|---|---|
| 0.18 m step, 4 Nm flippers | detected and sequenced, but the hull back-drives the flippers and rears; fixed flippers do the same | [11-14](../status/evidence/flipper-stairs-2026-11-14/README.md) |
| 0.18 m step, 40 Nm, auto | up, along the tread, down the far side (2/2) | [11-19](../status/evidence/flipper-strong-2026-11-19/README.md) |
| Five-step flight (`terrain_stairs_flight`) | climbs to 0.9 m and stops at the ledge (3/3) | [11-20](../status/evidence/r107-tracked-flight-2026-11-20/README.md) |
| Up and over (`terrain_stairs_over`) | five up, five down (2/2, about 97 s) | [11-20](../status/evidence/r107-tracked-over-2026-11-20/README.md) |
| Wide step at 15°, 30°, 45° off square (`terrain_step_wide`) | climbs at all three; descends only at 15°, while 30° and 45° stop on the roll hold | [11-20](../status/evidence/r107-tracked-angled-2026-11-20/README.md) |

Use the normal latched Drive panel for the belts. A lost ROS owner leaves
the bounded servos holding their last targets, and the plant's own Drive
watchdog (0.4 s) removes stale belt commands.

## Simulators

- **Gazebo Fortress:** native contact-surface belts (TrackedVehicle,
  TrackController) on a private Physics 5.4/DART plugin, on SSD at
  `$ROBOT_LAB_RUNTIME_ROOT/native_tracks`. On a clean installation, build it
  with `python3 scripts/provision_tracked_physics.py` after sourcing
  `scripts/ssd_env.sh`. The script extracts pinned OSRF packages and builds
  only the required plugin; it does not install over system packages.
- **MuJoCo and PyBullet (2026-11-21):** a shared track law
  (`robot_lab_utils/track_drive.py`). The track geometry has normal-only
  contacts, and the law pushes each track contact towards its belt's speed,
  within friction bounds (1.0 along the belt, 0.7 across it).
  - The flipper servos are Gazebo's JointPositionController law, with the
    gains read from each variant's `model.sdf`.
  - The command path is the same: owner, watchdog, caps and slews.
  - Measured on both: turns in place within 0.001 rad of the odometry, stops
    on operator loss, and maps `nav_obstacle` (100 % of occupied cells
    within 0.1 m).
  - Nav2 reaches the obstacle-route goal 2/2 on each.
  - The strong variant crosses the step, climbs the flight and goes up and
    over it, as on Gazebo.
  - On the wide step it passes at 15° and 30°, but at 45° it is deflected
    along the face and does not climb (Gazebo climbs).
  - They run at 0.8–0.9× real time, and on stairs the vehicle drifts
    sideways more than on Gazebo.
  - [Receipt](../status/evidence/r107-tracks-backends-2026-11-21/README.md).
- **Isaac:** not implemented.

Navigation autofills A* / regulated pure pursuit and the **Native tracked
command owner**. Track dead reckoning from the commanded belt speeds
supplies planar motion. The IMU supplies the turn rate to the EKF, and AMCL
corrects the map pose. Simulator truth (`/odom/ground_truth`) is a separate
observer.

## Limits

- Angled descents beyond 15° stop on the roll hold. The policy does not
  steer, so squaring up to an edge is up to the operator or a planner.
- One step height (0.18 m). The 40 Nm actuator is a lab choice, not a vendor
  rating.
- Belt slip is not measured by the odometry.

Upstream references: [TrackedVehicle](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1TrackedVehicle.html),
[TrackController](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1TrackController.html),
[Fortress DART requirement](https://github.com/gazebosim/gz-sim/issues/2495).
Source hashes, licenses and the small-command adaptation are kept with
the model and the vendored controller. [Measured scope](../status/native-workflows-2026-10-27.md).
