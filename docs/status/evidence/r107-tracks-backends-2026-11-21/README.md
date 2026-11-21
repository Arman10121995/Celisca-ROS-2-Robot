# R10.7: the tracked vehicle on MuJoCo and PyBullet (November 21, 2026)

**Status: implemented, experimental.** `lab_tracked` (fixed flippers) now
runs on MuJoCo and PyBullet through the normal launch
(`simulated_robot.launch.py simulator:=mujoco|pybullet`) with the same
command owner, watchdog, EKF, slam_toolbox and Nav2 stack as on Gazebo.
Until now it ran on Gazebo only. On both new backends it drives, turns in place,
stops on operator loss, maps `nav_obstacle` and reaches a Nav2 goal across
it, measured by ground truth. The articulated and strong-flipper variants
stay Gazebo-only.

## What changed

- **`robot_lab_utils/track_drive.py`:** a backend-neutral track law. MuJoCo
  and PyBullet have no moving contact surface (Gazebo's TrackController), so
  the track geometry gets normal-only contacts. The law supplies the
  tangential force at each track contact:
  - Along the belt (`axle x normal`: forward on the ground, upward at a riser
    met by the front sprocket), the target is the track speed at the
    contact's own lateral offset. Across the belt, the target is the
    commanded yaw about the hull origin, as TrackController sets surface
    motion about the vehicle's centre of rotation.
  - Each component is a velocity servo (gain 50/s on a load-weighted share
    of the vehicle mass), bounded by Coulomb friction: 1.0 along the belt,
    0.7 across it (Gazebo's lateral `mu`).
  - Each track integrates its load-weighted belt slip into a traction share
    of the normal force (40 per m/s per s, held within ±`mu_along`). This
    removes steady slip without the step-size limit of a stiffer servo.
  - The command path is the Gazebo plugins': a 0.4 s watchdog, caps of
    0.3 m/s and 0.5 rad/s, 0.4 m/s² and 0.4 rad/s² slews, and odometry
    dead-reckoned from the commanded track speeds.
- **`robot_lab_mujoco/track_plant.py`:** condim-1, raised-priority track
  geoms, with normal forces from `efc_force`. Point velocities are measured
  from each body's centre of mass, and one wrench per body goes into
  `qfrc_applied`. The step is vectorized.
- **`robot_lab_pybullet/track_plant.py`:** zero-friction track links with
  one `applyExternalForce` per contact. The source's zero-width flipper
  limits are held by position motors, because Bullet does not hold them.
- **Spawners:** a `track_config` parameter subscribes `/tracked/cmd_vel`
  and publishes `/robot_lab_controller/odom`.
- **`tracked_simulator.launch.py`:** a `simulator` argument. MuJoCo and
  PyBullet get the Gazebo session's sensor frames, its track block, a
  360-ray 10 Hz lidar, an IMU and an optional RGB-D camera.
- **`simulated_robot.launch.py`** forwards the simulator. The
  `lab_tracked` profile lists Gazebo, MuJoCo and PyBullet; the two flipper
  variants are pinned to Gazebo.

## Measured

### Drive probe (`track_probe.py`, `nav_empty`, `/key_vel`)

Truth along/lateral is in the heading at phase start.

| Run | Forward 8 s (truth / odom) | Turn 0.4 rad/s, 6 s (truth yaw / odom yaw, drift) | Reverse 6 s | Real-time factor |
|---|---|---|---|---|
| Gazebo (reference) | 1.539 / 1.543 m | 1.495 / 2.188 rad, 0.000 m | −0.866 m | 1.0 |
| MuJoCo, first (gain 50, no traction term) | 0.333 / 0.337 m | 0.149 / 0.408 rad, 0.013 m | −0.212 m | 0.25 |
| MuJoCo, vectorized + traction | 1.318 / 1.317 m | 1.624 / 1.693 rad, 0.130 m | −0.749 m | 0.85 |
| MuJoCo, final | 1.395 / 1.394 m | 1.840 / 1.839 rad, 0.0003 m | −0.722 m | 0.88 |
| PyBullet, first | 1.255 / 1.255 m | 1.762 / 1.832 rad, 0.107 m | −0.677 m | 0.81 |
| PyBullet, final | 1.243 / 1.238 m | 1.789 / 1.788 rad, 0.0004 m | −0.654 m | 0.80 |

Distances on the ported plants are shorter because they run below real time
for the same wall-clock phases. Gazebo's own odometry over-reports its turn
by 46 %; the ported plants turn at the commanded rate.

### Nav2 route (`tracked_nav_probe.py`, `nav_obstacle`, spawn (−7, −7) → goal (−3, −4), contract 0.15 m / 5°)

| Run | Result | Position error | Heading error | Path |
|---|---|---|---|---|
| Gazebo | passed | 0.039 m | 0.014 rad | 6.41 m |
| MuJoCo 1 (before the final law) | **failed** (heading) | 0.088 m | 0.113 rad | 6.48 m |
| MuJoCo 2 | passed | 0.029 m | 0.042 rad | 6.45 m |
| MuJoCo 3 | passed | 0.029 m | 0.029 rad | 6.46 m |
| PyBullet 1 | passed | 0.011 m | 0.038 rad | 6.46 m |
| PyBullet 2 | passed | 0.034 m | 0.008 rad | 6.43 m |

MuJoCo 1 ran with the earlier law: in place, the hull turned about its
centre of mass while the odometry reported a turn about the origin. It is
kept.

### Operator loss (`tracked_loss_probe.py`, `nav_empty`)

The command owner is frozen with SIGSTOP after 7 s at 0.1 m/s.

| Backend | Travel before | Travel after the fault | Final 2 s | Result |
|---|---|---|---|---|
| MuJoCo | 0.582 m | 0.056 m | 0.000 m | passed |
| PyBullet | 0.543 m | 0.054 m | 0.000 m | passed |

The 0.05 m after the fault is the 0.4 s watchdog plus the 0.4 m/s² slew.

### 2D SLAM (`tracked_slam_probe.py`, `nav_obstacle`, slam_toolbox)

The route is forward, a 90° turn, then forward. Each SLAM occupied cell is
placed in the world through the spawn pose and scored against the installed
map.

| Backend | Known cells | Occupied cells | Within 0.1 m | p95 error |
|---|---|---|---|---|
| Gazebo (reference) | 10,349 | 537 | 98.9 % | 0.039 m |
| MuJoCo | 9,849 | 428 | 100 % | 0.001 m |
| PyBullet | 10,461 | 525 | 100 % | 0.025 m |

## Found on the way

- **A vectorized step was needed.** A per-contact Python loop over about 24
  contacts cost 5 ms per 2 ms step: 11.5 ms per 2-step tick with the plant,
  against 0.93 ms without. That ran MuJoCo at 0.25× real time.
- **A pure velocity servo cannot skid-turn these tracks.** At gain 50/s the
  servo needs about 0.15 m/s of belt slip to push hard enough, more than
  the 0.08 m/s track speed of the turn. The traction integral fixed this.
- **`mj_objectVelocity` reports a body's linear velocity at its centre of
  mass (`xipos`), not its frame origin.** Measuring from the origin made a
  turn about the centre of mass look like one about the origin, which left
  0.07 m of uncorrected drift.

## Limits

- Fixed flippers only. The 0.18 m step and the stair missions need the
  flipper servos, which exist on the Gazebo plant only. The MJCF
  conversions of the stair worlds also do not exist yet.
- No Isaac track plant yet.
- Belt friction is 1.0 along the belt (Gazebo uses 150, effectively no
  slip) and 0.7 across. One flat route world and one empty world were
  measured. Real-time factors are 0.8–0.9.
- The odometry is dead-reckoned from the commanded speeds, as Gazebo's. The
  EKF fuses its velocities with the IMU yaw rate.

## Reproduce

```bash
python3 docs/status/evidence/r107-tracks-backends-2026-11-21/track_probe.py mujoco <out>
LAB_EXTENSION_RUN=<dir> python3 docs/status/evidence/r107-tracks-backends-2026-11-21/tracked_nav_probe.py pybullet nav_obstacle -3 -4
LAB_EXTENSION_RUN=<dir> python3 docs/status/evidence/r107-tracks-backends-2026-11-21/tracked_loss_probe.py mujoco
LAB_EXTENSION_RUN=<dir> python3 docs/status/evidence/r107-tracks-backends-2026-11-21/tracked_slam_probe.py pybullet
```

Traces are on SSD under `robot_lab_runtime/tracks-backends-20261121`
(hashes in `runtime-sha256.txt`).
