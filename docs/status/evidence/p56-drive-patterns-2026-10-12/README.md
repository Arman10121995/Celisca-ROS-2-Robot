# P5/P6: four-wheel-steer patterns and mecanum in all four backends (2026-10-12)

Every 4WS steering pattern of `four_wheel_steer_car` and the mecanum base
`mecanum_car` was driven in MuJoCo, PyBullet, Gazebo and Isaac. All runs
used localization mode on nav_empty, one simulator at a time:

```bash
MODE=loc WARMUP=6 CHECK_ARGS="--strafe" scripts/sim_drive_check.sh <sim> mecanum_car nav_empty <out>.json
MODE=loc WARMUP=6 [CHECK_ARGS=--strafe for crab] scripts/sim_drive_check.sh <sim> four_wheel_steer_car nav_empty <out>.json steering_mode:=<pattern>
```

`sim_drive_check.py` runs each `/cmd_vel` phase in simulated time and
measures body velocity from simulator truth (`/odom/ground_truth`),
excluding each phase's first second. The steering joints are recorded per
phase. All 20 cells ran. The files are `<sim>_<pattern>.json`.

## Measured (truth; commands in parentheses)

| Simulator | Pattern | straight vx (0.4) | left arc vx, wz (0.4, 0.5) | tight left wz (1.5) | spin wz (1.0) | strafe vy (0.3) | stop vx, wz |
|---|---|---|---|---|---|---|---|
| mujoco | ackermann | 0.398 | 0.336, 0.491 | 1.343 | 0.971 | n/a | -0.000, 0.000 |
| mujoco | in_phase | 0.398 | 0.398, 0.000 | 0.000 | 0.973 | n/a | -0.000, -0.000 |
| mujoco | crab | 0.398 | 0.398, -0.000 | 0.000 | 0.972 | 0.269 | 0.000, 0.000 |
| mujoco | pivot | 0.398 | 0.336, 0.491 | 1.337 | 0.970 | n/a | -0.000, 0.000 |
| mujoco | mecanum | 0.390 | 0.338, 0.468 | 1.433 | 0.951 | 0.254 | 0.001, 0.001 |
| pybullet | ackermann | 0.407 | 0.341, 0.506 | 1.286 | 0.926 | n/a | 0.000, -0.000 |
| pybullet | in_phase | 0.407 | 0.407, 0.000 | 0.000 | 0.929 | n/a | -0.000, 0.000 |
| pybullet | crab | 0.407 | 0.407, -0.000 | 0.000 | 0.986 | 0.271 | 0.000, -0.000 |
| pybullet | pivot | 0.407 | 0.341, 0.506 | 1.282 | 0.955 | n/a | -0.000, 0.000 |
| pybullet | mecanum | 0.402 | 0.339, 0.504 | 1.508 | 1.015 | 0.274 | -0.000, -0.000 |
| gazebo | ackermann | 0.400 | 0.338, 0.497 | 1.325 | 0.990 | n/a | 0.000, 0.000 |
| gazebo | in_phase | 0.400 | 0.400, -0.000 | 0.000 | 0.991 | n/a | 0.000, 0.000 |
| gazebo | crab | 0.400 | 0.400, 0.000 | -0.000 | 0.992 | 0.268 | 0.000, 0.000 |
| gazebo | pivot | 0.400 | 0.338, 0.497 | 1.347 | 0.992 | n/a | 0.000, 0.000 |
| gazebo | mecanum | 0.397 | 0.336, 0.497 | 1.487 | 0.992 | 0.282 | 0.000, -0.000 |
| isaac | ackermann | 0.401 | 0.337, 0.494 | 1.446 | 0.954 | n/a | -0.000, -0.000 |
| isaac | in_phase | 0.401 | 0.401, -0.000 | 0.000 | 0.961 | n/a | -0.000, -0.000 |
| isaac | crab | 0.401 | 0.400, -0.000 | 0.000 | 0.958 | 0.300 | -0.000, 0.000 |
| isaac | pivot | 0.401 | 0.337, 0.493 | 1.443 | 0.960 | n/a | -0.000, -0.000 |
| isaac | mecanum | 0.403 | 0.331, 0.498 | 1.538 | 1.007 | **0.142** | -0.000, -0.000 |

## What each pattern does

The behaviour is the same in every backend; the Gazebo steering joint
angles are in `gazebo_*.json`.

- **ackermann** (opposite phase): the rear wheels steer opposite to the
  front (+0.24/+0.17 front, -0.24/-0.17 rear rad on the left arc). Yaw
  follows the command: 0.49-0.51 of 0.5 rad/s, and 1.29-1.45 rad/s on the
  1.5 rad/s tight turn.
- **in_phase**: all four wheels steer to the same angle, so an angular
  command becomes a sideways drift without yaw: vy 0.080 m/s on the
  0.5 rad/s arc, 0.24 m/s on the tight turn, wz 0.000.
- **crab**: angular commands are ignored (wz 0.000). A lateral command
  turns all wheels to 1.57 rad and strafes at 0.268-0.300 m/s of 0.3.
- **pivot**: like ackermann in motion. With no forward command, every
  pattern pivots in place: wheels at ±0.79 rad, 0.93-0.99 rad/s.
- **mecanum**: drives, arcs and spins like the 4WS patterns. It strafes at
  0.254-0.282 m/s in MuJoCo, PyBullet and Gazebo, but only **0.142 m/s in
  Isaac**, where the lateral roller grip is lower. This remains a
  backend-specific limit.

All 20 cells stop at zero.

## Isaac strafe follow-up (2026-10-13)

The Isaac URDF importer gives every revolute joint a drive, so the 60
passive rollers were held by it. `42d1ccb` gives roller joints zero
stiffness and the URDF's bearing-level damping; the launch log confirms
"Free roller joints: 60". The repeat
([isaac_mecanum_drive_free_rollers_2026-10-13.json](isaac_mecanum_drive_free_rollers_2026-10-13.json))
strafed at **0.119 m/s** of 0.3, no better than 0.142. The roller drives
were not the cause. The roller contact (friction combination, roller
collision shapes) is the next suspect. The other phases are unchanged:
straight 0.403, arcs 0.334/0.488, spin 1.016 rad/s, stop at zero.

The same build passed the mecanum obstacle mission in Isaac (0.06 m and
0.09 m from truth).

A second suspect was the damping the fix gave the rollers. USD angular
drive damping is per degree, so "1e-4" was 0.0057 N·m·s/rad, 570 times
the URDF's bearing damping. At the ~33 rad/s a strafe needs, that resists
with about 30 N at a 6 mm roller's rim.

- `6a88bce` set the URDF value in per-degree units. It also broke the
  wheel and steering drives (an undefined name), so every Isaac wheeled
  robot failed to spawn until `a84ab0c`. A static name test now guards
  the file.
- The repeat with the corrected damping
  ([isaac_mecanum_drive_roller_damping_per_degree_2026-10-13.json](isaac_mecanum_drive_roller_damping_per_degree_2026-10-13.json))
  strafed at **0.148 m/s**, with 0.066 m/s of forward drift: about the
  same as the 0.142 and 0.119 m/s before. Damping was not the cause
  either.
- The obstacle mission still passes (0.07 m and 0.05 m from truth).

**Roller spin probe** (`scripts/roller_spin_probe.py`, 12:32). It commands
a 0.3 m/s strafe and reads every hub and roller rate from
`/joint_states`. Ideal 45° mecanum wheels turn their hubs at 6.0 rad/s,
and the roller touching the ground at about 67 rad/s.

| | Lateral speed | Hub rate | Fastest roller, mean per wheel |
|---|---|---|---|
| MuJoCo ([json](mujoco_mecanum_rollers_2026-10-13.json)) | 0.251 m/s (forward drift -0.002) | 5.68-5.71 rad/s | 63.9-65.4 rad/s (95-97% of ideal) |
| Isaac ([json](isaac_mecanum_rollers_2026-10-13.json)) | 0.154 m/s (forward drift 0.094) | 6.02-6.05 rad/s | 27.5-38.2 rad/s (41-57% of ideal) |

In Isaac the hubs turn at the ideal rate, but the rollers turn at only
about half the rate the strafe needs, and unevenly between wheels (hence
the forward drift). The rollers neither turn freely nor are held still:
something resists them or lets them slip against the floor. Their joint
drives are now free, so the remaining suspects are in PhysX:

- the articulation's solver iterations (none are set; about 65 links);
- contact offsets that are large next to 6 mm rollers;
- the imported roller joint axes.

**Two of them tried (2026-10-13 afternoon).**

| Isaac change | Lateral speed | Forward drift | Fastest roller, mean per wheel |
|---|---|---|---|
| Free rollers, bearing damping (above) | 0.154 m/s | 0.094 m/s | 27.5-38.2 rad/s |
| + 32/8 solver iterations ([json](isaac_mecanum_rollers_iterations_2026-10-13.json)) | 0.167 m/s | 0.078 m/s | 26.9-39.9 rad/s |
| + 2 mm contact offset on the 60 roller collisions ([json](isaac_mecanum_rollers_iterations_contact_offset_2026-10-13.json)) | 0.167 m/s | -0.034 m/s | 31.8-40.3 rad/s |

- Neither brings the rollers near the 67 rad/s they need. The strafe
  stays at about 55% of the command.
- The contact offset first reached no prims at all: the importer's
  collision meshes are instanceable. The roller instances are now made
  local first (`175fa1c`, `cdcb01c`).
- The full drive check with both changes
  ([json](isaac_mecanum_drive_roller_tuning_2026-10-13.json)) strafes at
  0.148 m/s with less forward drift (0.022 m/s). Every other phase is
  unchanged, and the obstacle mission still passes (0.12 m and 0.09 m).
- The changes stay: they do no harm and reduce the drift.
- At this point Isaac mecanum strafing was still at about half speed.

**Cause found: faceted roller collisions.** Each roller's collision is a
convex hull with 12 sides around its axis. MuJoCo rolls it smoothly, but
PhysX rests it on a flat facet and has to tip it over each edge, which
resists rolling. `e0e496e` replaces each roller's collision with an
analytic capsule sized from the mesh's bounds (the mesh stays as the
visual).

| Isaac change | Lateral speed | Forward drift | Fastest roller, mean per wheel |
|---|---|---|---|
| Capsule rollers ([json](isaac_mecanum_rollers_capsules_2026-10-13.json)) | **0.293 m/s** | 0.002 m/s | 70.2-79.6 rad/s (104-118% of ideal) |

With capsules, Isaac strafes at 98% of the command: better than MuJoCo
(0.251 m/s), with no forward drift. The full drive check and the obstacle
mission are being rerun with this change.

## Not covered here

These are open-loop drive screens on a flat map. Obstacle routes,
mapping, navigation per pattern and contact measurements stay in R5.6.
