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

## Not covered here

These are open-loop drive screens on a flat map. Obstacle routes,
mapping, navigation per pattern and contact measurements stay in R5.6.
