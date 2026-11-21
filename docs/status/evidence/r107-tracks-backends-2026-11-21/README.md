# R10.7: the tracked vehicles on MuJoCo and PyBullet (November 21, 2026)

**Status: implemented, experimental.** `lab_tracked` (fixed flippers) now
runs on MuJoCo and PyBullet through the normal launch
(`simulated_robot.launch.py simulator:=mujoco|pybullet`) with the same
command owner, watchdog, EKF, slam_toolbox and Nav2 stack as on Gazebo.
Until now it ran on Gazebo only. On both new backends it drives, turns in place,
stops on operator loss, maps `nav_obstacle` and reaches a Nav2 goal across
it, measured by ground truth. **Later the same day** the ported plants got
the flipper servos. The 40 Nm variant then crosses the 0.18 m step, climbs
the five-step flight and goes up and over it on both backends, with the
same automatic flipper policy as on Gazebo
([flippers](#flippers-later-on-november-21)).

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

## Flippers (later on November 21)

### What changed

- **`FlipperServo` (`track_drive.py`):** Gazebo's JointPositionController
  law in force mode. The gains (p 80, i 12 clamped to ±7.5 Nm, d 4, ±40 Nm
  for the strong variant) are read from the variant's own `model.sdf`
  (`sdf_flippers`). The plants apply its torque and subscribe the same
  `/tracked/flippers/<joint>/target` topics.
- **Flipper policy input:** `tracked_control` takes a `depth_topic`. The
  ported plants render the Gazebo camera (320×240, 1.25 rad, at
  .05 0 .34) on `/oakd/depth/image_raw` (32FC1).
- **Servo mass cap:** each body's contacts servo at most 4× that body's own
  mass. Before the cap, a 0.75 kg flipper pressed into a riser got a 25 kg
  servo share. The explicit force then diverged in one step: the flipper
  went to 0.92 rad past its 0.8 limit, and the vehicle flew off the flight
  (`flight-mujoco-1`, kept).
- **MJCF conversions:** `terrain_stairs_flight`, `terrain_stairs_over` and
  `terrain_step_wide` (by `gen_mjcf_worlds.py`; the other worlds' existing
  conversions were left as they are).
- **Profiles:** both flipper variants list Gazebo, MuJoCo and PyBullet.

### Measured (automatic flippers, the Gazebo courses and probes of 11-14, 11-19 and 11-20)

**Single 0.18 m step** (`terrain_stairs`, spawn (−6.65, −5.9) heading +y,
`stairs_case.sh`, scored by `stairs_crossing.py`). Crossed means level
samples on the tread, then the floor beyond the far edge. The rule
reproduces the Gazebo verdicts: auto-full 1 and 2 cross; default climb and
fixed do not.

| Vehicle / mode | Gazebo 11-19 | MuJoCo | PyBullet |
|---|---|---|---|
| 40 Nm, auto (full press) | crossed 2/2 | crossed 3/3 (incl. the final law) | crossed 3/3 |
| 40 Nm, fixed flippers | rears to 0.658 rad, stops | rears to 0.657 rad, stops | rears to 0.690 rad, stops |
| 4 Nm (`lab_tracked_articulated`), auto | rear flippers back-driven (−0.03 rad) | not crossed; rear back-driven to −0.025 rad | not crossed; −0.014 rad |

**Five-step flight** (`terrain_stairs_flight`, spawn (−7.3, −4.0) heading +x, 100 s, `course_case.sh`):

| Run | Top reached | Ledge stop | Max roll |
|---|---|---|---|
| Gazebo 11-20 `auto-4` | 5 steps (1.01 m base_link) | x 2.55 | 0.001 rad |
| MuJoCo 1 (before the cap) | **failed**: diverged on tread 1 | — | π |
| MuJoCo 2, 3 | 5 steps (0.93 m footprint) | x 2.53, 2.54 | 0.02, 0.06 rad |
| PyBullet 1, 2 | 5 steps (0.94 m) | x 2.36, 2.58 | 0.04, 0.06 rad |

**Up and over** (`terrain_stairs_over`, the drive stops at x 6.0, 180 s):

| Run | Result | Time to x 6.0 | End y (start −4.0) | Max roll |
|---|---|---|---|---|
| Gazebo 11-20 `auto-4`, `auto-5` | passed | 97.0, 96.8 s | — | ≤ 0.002 rad |
| MuJoCo 1, 2 | passed: 5 up, 5 down, on the floor | 92.8, 92.8 s | −4.35, −4.37 | 0.09, 0.07 rad |
| PyBullet 1, 2 | passed | 94.2, 94.0 s | −3.68, −4.22 | 0.09, 0.12 rad |

**Angled approaches** (`terrain_step_wide`, the 11-20 spawns, 90 s, the drive stops at x 4.5):

| Angle off square | Gazebo 11-20 | MuJoCo | PyBullet |
|---|---|---|---|
| 15° | up and down to the floor, 2/2 | **passed** (51.8 s, roll 0.16 rad) | **passed** (51.8 s, roll 0.18 rad) |
| 30° | climbs; stops on the roll hold at the far edge | **passed**: rolled past the hold (0.40 rad) on the climb, recovered and descended (72.4 s) | **passed**: roll 0.33 rad, under the hold (56.1 s) |
| 45° | climbs; stops on the roll hold at the far edge | **not passed**: never climbs; deflected along the face to y 6.3 | **not passed**: the same deflection, to y 6.3 |

At 45° the ported belts do not pull the hull up the face. It turns
along the face instead, so the behaviour differs from Gazebo in both
directions: further at 30°, short at 45°. Gazebo's belts are effectively
no-slip along the belt (mu 150), while the ported law bounds them at 1.0.
That was the first suspect, and it is ruled out. With a new
`track_overrides` launch argument, mu_along 2 and 4 end within millimetres of
the 1.0 run (`angled-mujoco-45deg-mu2`, `-mu4`). So does a trial law giving
riser-face contacts full Coulomb traction, which was reverted
(`angled-mujoco-45deg-riser`). The hull turns parallel to the face before any
climbing force matters. The next step is logging which contact (the front
flipper or sprocket on the step edge) turns it.

**Regression on fixed-flipper `lab_tracked` after the cap:**
- Turn: truth 1.840 / odom 1.832 rad on MuJoCo, 1.755 / 1.745 on PyBullet,
  under 0.5 mm of drift.
- Nav2 route: 0.077 m / 0.035 rad on MuJoCo and 0.033 m / 0.051 rad on
  PyBullet; both pass.

On the ported plants the vehicle drifts sideways by up to 0.33 m across
the flight and rolls more than on Gazebo, though it stays on the treads.
Gazebo's implicit solver holds the track straight. The 0.15 m/s drive is
open loop in these probes, with no heading hold.

## Limits

- No Isaac track plant yet.
- 45° step approaches do not climb on the ported plants (Gazebo climbs).
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
docs/status/evidence/r107-tracks-backends-2026-11-21/stairs_case.sh mujoco lab_tracked_strong_flippers <dir> auto
python3 docs/status/evidence/r107-tracks-backends-2026-11-21/stairs_crossing.py <dir>
docs/status/evidence/r107-tracks-backends-2026-11-21/course_case.sh pybullet terrain_stairs_over -7.3 -4.0 0.0 <dir> 180 6.0
docs/status/evidence/r107-tracks-backends-2026-11-21/course_case.sh mujoco terrain_step_wide -3.0 -5.73 0.5236 <dir> 90 4.5   # 30°
```

Traces are on SSD under `robot_lab_runtime/tracks-backends-20261121`
(hashes in `runtime-sha256.txt`).
