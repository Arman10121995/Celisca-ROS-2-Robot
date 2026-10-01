# R5.6 — Physical mecanum travel and per-pattern 4WS steering (2026-10-01)

## Three defects stood between the models and measured motion

1. **The display hold pinned the mecanum's passive rollers.** In `auto` mode the
   spawner holds every joint it does not drive, to keep legged/humanoid display
   robots upright. A mecanum is not a differential drive, so `_lw`/`_rw` are
   unset, `has_drive` was false, and **all 60 roller joints got spawn-pose
   spring-dampers**. A pinned roller cannot spin, the roller wheel acts as a
   solid tire, and the strafe the drive model commands is resisted. Measured
   through the MuJoCo bridge: **0.014 m/s lateral of a commanded 0.30 m/s**
   (`mujoco/mecanum_strafe_before_fix.json`). The same model in a plain MuJoCo
   loop (`offline/offline_mecanum_probe.py`) reached **0.280 m/s**, which is what
   identified the hold — not the contact model — as the blocker.
   The `fuji_mecanum` roller assets (15 passive barrel rollers per hub, MIT,
   revision pinned in `third_party/fuji_mecanum/README.md`) were already
   imported: 60 roller bodies, 60 roller hinge joints, 60 roller collision geoms.
2. **The PyBullet bridge crashed on any non-differential drive.** It evaluated
   `self._drive.left` for every drive type; `MecanumDrive`/`FourWheelSteerDrive`
   have no `left`, so the physics thread died with `AttributeError` before
   publishing (`pybullet/mecanum_crash_before_fix.log`). The generalized branch
   that commands every wheel by velocity and every steering joint by position
   already existed two lines below; only the diff-only lookup was misplaced.
3. **The chassis collision box overlapped the steered wheels** (the same defect
   as `r55-car-wheelwell-2026-10-01/`): the holonomic chassis collision is now a
   0.20 m wheel-well box under a 0.30 m visual shell.

Fixes: `robot_lab_utils.urdf_joints.driven_assembly_joints()` (new, hermetic,
pinned by `test_driven_assembly_hold.py` — 60 rollers for the shipped mecanum),
applied in both bridges' hold lists; the diff-only lookup moved into the diff
branch of the PyBullet loop; the narrower holonomic chassis collision.

## Measured mecanum, MuJoCo, simulator ground truth (ROS domain 224)

Command → measured vx / vy / wz (m/s, rad/s), speeds from odometry **header
stamps** (sim time):

| phase | command | vx | vy | wz | travel |
|---|---|---|---|---|---|
| straight | 0.4, 0 | +0.397 | −0.000 | −0.000 | 1.07 m |
| left arc | 0.4, +0.5 | +0.348 | −0.003 | **+0.490** | 1.25 m |
| right arc | 0.4, −0.5 | +0.347 | +0.003 | **−0.490** | 1.26 m |
| reverse left | −0.3, +0.3 | −0.290 | +0.002 | +0.294 | 0.79 m |
| tight left (clamped) | 0.3, +1.5 | +0.135 | −0.005 | +1.472 | 0.37 m |
| spin in place | 0, +1.0 | −0.000 | −0.000 | **+0.983** | 0.00 m |
| **strafe** | vy = +0.3 | −0.000 | **+0.283** | +0.000 | **1.03 m lateral** |
| stop | 0, 0 | 0.000 | −0.000 | −0.000 | 0.00 m |

The strafe is the first *measured physical lateral motion* for this base: no
pose is set directly, the wheels are commanded by the mecanum matrix and the
rollers slip. Before the fix the same phase measured −0.005 m/s
(`mujoco/mecanum_strafe_before_fix.json`).

## Measured four-wheel steering, MuJoCo, one run per pattern

`steering_mode:=ackermann|in_phase|crab|pivot` is a new launch argument
(`simulated_robot.launch.py`) that overrides the `steering_mode` in the robot's
drive block; empty keeps `robots.yaml`. Domains 213–216.

| pattern | straight 0.4/0 | left arc 0.4/+0.5 | spin 0/+1.0 | steering joints (left arc) |
|---|---|---|---|---|
| ackermann (opposite phase) | 0.398 / 0.000 | 0.350 / **+0.464** | **0.972** | front +0.192, rear −0.202 |
| in_phase | 0.398 / 0.000 | 0.379 / +0.092 (vy 0.076) | 0.972 | all four ≈ +0.196 |
| crab | 0.398 / 0.000 | 0.380 / +0.095 (vy 0.075) | 0.972 | all four ≈ +0.196 |
| pivot | 0.398 / 0.000 | 0.395 / +0.105 (vy 0.003) | **0.216** | all ≈ 0.000 |

Readings: opposite-phase follows the curvature (93 % yaw tracking) with the two
axles in opposite phase; in_phase and crab translate with a lateral component
and almost no yaw (the wheels point the same way, so the base slides — the
expected physical signature); pivot holds the wheels straight and changes yaw
with no translation. **`pivot` zero-turn reaches only 0.216 rad/s of a commanded
1.0 rad/s**: a pivot turn is pure lateral scrub, and with this plant's contact
friction the skid-limited rate is well under the kinematic target. That is a
measured limitation of the pattern on this backend, not a pass.

## Second backend: PyBullet (the bridge that used to crash)

With the diff-only lookup moved out of the common path, PyBullet runs these
bases at all (before the fix the physics thread died on `MecanumDrive.left`,
`pybullet/mecanum_crash_before_fix.log`). Ground-truth odometry, ROS domains
230–232:

**Mecanum** — `pybullet/mecanum_after_fix.json`

| phase | command | vx | vy | wz |
|---|---|---|---|---|
| straight | 0.4, 0 | +0.408 | 0.000 | 0.000 |
| left / right arc | 0.4, ±0.5 | +0.367 / +0.366 | −0.002 / +0.002 | **+0.507 / −0.506** |
| reverse left | −0.3, +0.3 | −0.299 | +0.001 | +0.305 |
| tight left | 0.3, +1.5 | +0.138 | −0.003 | +1.515 |
| spin in place | 0, +1.0 | +0.000 | 0.000 | **+1.017** |
| **strafe** | vy = +0.3 | −0.000 | **+0.301** | 0.000 |

1.07 m of lateral travel at 100 % of the commanded 0.30 m/s. The mecanum now
has **two** backends with measured physical lateral motion: MuJoCo 0.283 m/s
(94 %) and PyBullet 0.301 m/s (100 %).

**Four-wheel steering** — `pybullet/4ws_crab.json`, `pybullet/4ws_pivot.json`

| pattern | left arc 0.4/+0.5 | spin 0/+1.0 | steering joints (left arc) |
|---|---|---|---|
| crab | 0.383 / 0.218, vy +0.073 | **+1.011** | all four +0.197…+0.198 (parallel) |
| pivot | 0.411 / 0.028 | **+0.072** | all four ≈ 0.000 |

Crab reproduces the MuJoCo signature (four parallel steered wheels, lateral
velocity, near-full spin when the steering can be arranged). **Pivot's zero-turn
is weak on both backends** — 0.216 rad/s of 1.0 on MuJoCo and 0.072 rad/s on
PyBullet — with the wheels held straight and the base not translating: a pivot
turn is pure lateral scrub, and the skid-limited rate is far below the
kinematic target on both plants. Recorded as a measured limitation of the
pattern, not a pass.

## Lateral crab command (closed)

The four-wheel-steer drive now carries a lateral term for the patterns that can
use it. `FourWheelSteerDrive.targets` aims `crab` (all four wheels parallel) and
`in_phase` (both axles the same way) along the commanded body velocity when
`vy` is non-zero; `ackermann` (a turning circle) and `pivot` (an in-place skid)
ignore it rather than pretend. Both bridges forward `Twist.linear.y` for the
four-wheel-steer drive as well as the mecanum. With `vy = 0` every pattern's
previous behaviour is unchanged — all 37 pre-existing kinematics tests still
pass, and three new ones pin the lateral behaviour.

Measured, MuJoCo ground truth (`mujoco/4ws_crab_lateral_command.json`,
ROS domain 228, `--strafe`):

| phase | command | vx | vy | wz | steering |
|---|---|---|---|---|---|
| **strafe (crab)** | vy = +0.3 | **+0.149** | **+0.149** | **−0.000** | all four +0.785 |
| left arc (unchanged) | 0.4, +0.5 | +0.379 | +0.075 | +0.092 | all four +0.196 |

The base translates **diagonally with no yaw** — the steering limit (±0.785
rad = 45°) caps a pure lateral command, so a `vy` command becomes a 45° crab
rather than a 90° one. That is the physical limit of this base, and the unit
tests pin it (`0.3 · sin(0.785) / r` wheel rate, all four angles equal and
clamped).

## GUI and command surface

* `steering_mode:=` is appended by the launcher for the four-wheel-steer base
  only, and is selectable in the GUI through the new **4WS pattern** combo
  (`ackermann`/`in_phase`/`crab`/`pivot`, disabled for every other robot).
* The Drive pad gains **Strafe L / Strafe R** latched buttons, enabled only for
  a mecanum drive; they publish `Twist.linear.y` with the same bounded 10 Hz
  ramp as the other axes. A stale strafe latch is cleared when another robot is
  selected, and `_stop_drive` zeroes all three axes.
* Pinned by `test_command_autofill.py::test_mecanum_strafe_publishes_lateral_motion_only_for_mecanum`
  and `::test_four_wheel_steer_pattern_is_appended_to_the_command`.

## Reproduction

```bash
ROS_DOMAIN_ID=224 MODE=display ODOM=/odom/ground_truth TIMEOUT=120 WARMUP=2 \
  CHECK_ARGS='--strafe' bash scripts/sim_drive_check.sh mujoco mecanum_car nav_empty /tmp/mec.json
ROS_DOMAIN_ID=215 MODE=display ODOM=/odom/ground_truth TIMEOUT=120 WARMUP=2 \
  bash scripts/sim_drive_check.sh mujoco four_wheel_steer_car nav_empty /tmp/4ws.json steering_mode:=crab
python3 offline/offline_mecanum_probe.py     # no ROS: lateral/yaw/forward in the raw plant
```

Revision `906d7fe` plus working-tree changes; MuJoCo 3.12, PyBullet 3.2.7,
ROS 2 Humble, Jetson Orin. Note the installed `robot_lab_mujoco` /
`robot_lab_pybullet` are *copies* on this host: rebuild with
`colcon build --packages-select … --symlink-install` or a live run silently uses
the previous bridge.

## Still open (not claimed here)

* Gazebo and Isaac cells for these two bases, including a Gazebo pattern run
  against independent truth (the existing Gazebo evidence uses controller
  odometry).
* Nav2 / localization / SLAM missions per pattern; only the empty-floor drive
  phases are measured above.
* `pivot` zero-turn rate, and in_phase/crab distinguished from each other by a
  pure-lateral command (the drive probe's lateral phase currently strafes the
  mecanum only).

