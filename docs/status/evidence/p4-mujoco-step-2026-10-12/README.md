# P4: MuJoCo static-world step cost and range-sensor cost (2026-10-12)

Jetson AGX Orin, MuJoCo 3.12, headless. Source: the MuJoCo bridge changes in
`robot_lab_mujoco/world_collision.py` and `mujoco_spawner.py` (committed
in `ff88509`, "Claude work"), with the benchmark and probe scripts below.

## What was slow

A live Labbot/celisca_floor_1 run sampled every 5 ms (scratch sampler on the
spawner threads) spent 71 % of its time in `mj_step` and 25 % casting the
360-ray scan. Native timers on the captured hospital scene (194k static
flex elements) gave 10.6 ms per 2 ms step, 9.0 ms of it in position
kinematics: the per-step refit of the collision midphase over static
world triangles.

## Changes (geometry, contacts and sensing unchanged)

| Change | Effect | How it was checked |
|---|---|---|
| Disable MuJoCo's midphase when the world has more than 5000 static flex elements (`world_midphase:=auto`) | It only accelerates; contact results are identical. Captured hospital: 10.55 -> 4.73 ms per step, identical final `qpos` and contact count after 350 steps | `world_step_midphase_{on,auto}.json`, unit test `test_midphase_is_disabled_only_for_large_static_flex_worlds` |
| Rigid-flex pieces of 1000 instead of 3000 triangles | Without the midphase a contact tests the whole piece: celisca_floor_1 narrowphase 0.24 -> 0.05 ms | `scripts/mujoco_world_step_benchmark.py` |
| Range sensors cast against exact copies of the world mesh split into 200-triangle pieces (group 3, not drawn, not colliding); the original meshes stay as visuals (group 2) | `mj_ray` tests every triangle of a mesh it reaches; celisca_floor_1 360 rays 65 -> 18 ms, distances equal to 1e-6 m | `test_ray_proxies_return_the_original_mesh_distances` |
| One `mj_multiRay` call per scan, with per-ray stepping only for rays that hit the robot | Removes the per-ray Python loop; identical ranges | `test_vectorized_scan_matches_per_ray_casting_and_skips_the_robot` |

Per-step world cost (probe box at the map spawn, 200 steps, `world_step_midphase_*.json`):

| Map | Flex elements | Midphase on (ms) | Auto (ms) | Physics bound on RTF (2 ms step) |
|---|---:|---:|---:|---|
| celisca_floor_1 | 70,029 | 4.24 | 1.95 | 0.47 -> 1.02 |
| celisca_floor_2 | 58,235 | 3.50 | 1.68 | 0.57 -> 1.19 |
| celisca_floor_1_furniture | 30,665 | 1.79 | 1.00 | 1.12 -> 1.99 |
| celisca_floor_2_furniture | 31,729 | 1.76 | 0.90 | 1.13 -> 2.24 |
| small_house | 3,835 | 0.22 | 0.18 (midphase kept) | |

## Whole-simulator real-time factor (live, `scripts/sim_rtf_check.sh`, nav mode)

| Robot / map | Before | After |
|---|---:|---:|
| Labbot / celisca_floor_1 | 0.309 (nav), 0.317 (display) | 0.675 |
| Bumperbot / celisca_floor_1 | | 0.693 |
| Labbot / celisca_floor_2_furniture | | 0.842 |
| Labbot / dataset_hospital | 0.13 (2026-10-08 report) | 0.344 |
| Bumperbot / nav_obstacle | | 0.877 |

The remaining gap to real time is the rest of the physics step (flex vertex
update ~1.2 ms on celisca_floor_1), the scan and the camera render, plus the
navigation stack sharing the CPU.

## Behaviour after the change

The Celisca navigation mission (detour route, cancel/stop, resume, a second
near-obstacle goal; `scripts/sim_nav_mission.sh`) passes for Labbot on
celisca_floor_1 in MuJoCo after the change: cancel stop in 0.18 simulated s,
goals 0.057 and 0.029 m from independent truth. The full Celisca matrix is
in `../p3-celisca-missions-2026-10-12/`.

## Rejected: a 4 ms model step for wheel-only robots

Implemented as `wheel_physics_timestep` and measured with
`scripts/sim_drive_check.sh` on nav_empty (`wheel_timestep_drive_checks.json`):
Bumperbot tracked identically, but Labbot drifted (straight-line yaw -0.047
rad/s, 0.419 instead of 0.458 rad/s on an arc, residual motion after Stop).
It stays off by default (parameter 0.0).
