# P7 / R5.2: walking robots map with slam_toolbox on leg odometry (2026-10-13)

Status: **implemented, experimental**. Every walking robot run builds an
accurate 2D map of nav_obstacle in slam mode, in MuJoCo, Isaac and Gazebo,
with leg kinematic odometry as the only odometry. One run per cell.

## How it is measured

`scripts/slam_mapping_check.sh <sim> <robot> nav_obstacle <out.json>` launches
`mode:=slam`. Slam mode selects the robot's policy, leg odometry feeds the
EKF, and slam_toolbox maps from the robot's own laser scan.
`scripts/slam_mapping_check.py` then drives the robot:

- along an A* loop through the reference map: start, then (-3.2, -3.8),
  then (3.4, -3.7) moved into open space, then back to the start;
- the loop is 26-27 m;
- a pure-pursuit drive steers it on simulator truth. Only the drive uses
  truth; slam_toolbox never sees it.

Scores, all against simulator truth and the reference map
(`robot_lab_maps/maps/nav_obstacle`). The slam frame is aligned to the
world once, at the start:

- **trajectory:** slam pose (`map -> base`) against truth, every 0.5 s;
- **precision:** share of slam occupied cells within 0.10 m of a reference
  occupied cell (no invented walls);
- **surface distance:** from slam occupied cells to the nearest reference
  wall surface;
- **recall:** share of reference wall surfaces within 3 m of the driven
  path that have a slam cell within 0.10 m. Walls hidden behind other
  walls count as misses, so 100% is not reachable.

## Results (`v1/`, 16:39-17:3x, from `e0e496e`)

| Robot | Simulator | Route done | Slam pose RMSE / max | Precision | Surface distance mean / p95 | Recall |
|---|---|---|---|---|---|---|
| A1 | MuJoCo | yes, 83 s | 4.7 / 7.8 cm (`v2/`) | 1.00 | 5.5 / 12.0 cm | 0.65 |
| B2 | MuJoCo | yes, 77 s | 4.6 / 15.4 cm | 1.00 | 5.4 / 10.7 cm | 0.68 |
| G1 | MuJoCo | yes, 179 s | 4.6 / 8.7 cm | 1.00 | 5.5 / 12.0 cm | 0.70 |
| Go2 | MuJoCo | yes, 85 s (`v2/`) | **4.3 / 7.2 cm** | 1.00 | 5.5 / 10.7 cm | 0.70 |
| Go2 | Isaac | yes, 243 s | 5.4 / 9.1 cm | 1.00 | 5.5 / 11.0 cm | 0.70 |
| A1 | Isaac | yes, 217 s | 6.1 / 11.0 cm | 1.00 | 5.4 / 11.0 cm | 0.63 |
| G1 | Isaac | yes, 321 s | 7.1 / 11.2 cm | 1.00 | 7.0 / 13.8 cm | 0.51 |
| Go2 | Gazebo | yes, 73 s | 4.5 / 7.8 cm | 0.95 | 10.9 / 16.2 cm | 0.68 |
| Bumperbot (wheels) | Gazebo | yes, 88 s | 11.3 / 16.1 cm | 0.86 | 14.2 / 59.1 cm | 0.63 |

Map scores come from the saved maps (`v1/*.pgm` with their `*.yaml`,
rescored with `slam_mapping_check.py --rescore`), collected in
[v1/scores.json](v1/scores.json).

- **Go2 MuJoCo** ran with the first version of the drive harness. At the
  second waypoint, a near-obstacle nav goal, it wedged itself 0.16 m from
  an obstacle and pushed until the 900 s limit, after 20.5 m. The harness
  now moves waypoints into open space and backs off when stuck.
- **A1 MuJoCo** saved no map YAML yet (added right after), so only its
  trajectory and precision are comparable.
- Both cells were rerun with the final harness (`v2/`, 17:58-18:03); the
  table shows the reruns. The first Go2 drive (20.5 m, wedged) had held
  9.5 cm RMSE.

## Wheeled robots (`wheels/`, 18:04-18:26)

The same loop, for the mecanum car in all four backends and the 4WS car
in its ackermann and pivot patterns. Outside Gazebo the odometry was the
wheels (`odometry_source:=wheels`), not the simulator's truth.

| Robot | Simulator | Slam pose RMSE / max | Precision | Surface distance mean | Recall |
|---|---|---|---|---|---|
| Mecanum | MuJoCo | 1.1 / 2.3 cm | 1.00 | 5.8 cm | 0.57 |
| Mecanum | PyBullet | 1.8 / 4.0 cm | 1.00 | 6.0 cm | 0.72 |
| Mecanum | Gazebo | 2.5 / 5.2 cm | 1.00 | 5.9 cm | 0.72 |
| Mecanum | Isaac | 1.6 / 3.4 cm | 1.00 | 5.8 cm | 0.58 |
| 4WS, ackermann pattern | MuJoCo | 2.5 / 4.2 cm | 1.00 | 5.5 cm | 0.72 |
| 4WS, pivot pattern | MuJoCo | 1.4 / 3.0 cm | 1.00 | 5.7 cm | 0.72 |
| 4WS, ackermann pattern | Gazebo | 1.8 / 4.4 cm | 1.00 | 6.4 cm | 0.72 |
| 4WS, pivot pattern | Gazebo | 1.8 / 3.9 cm | 1.00 | 6.3 cm | 0.72 |

All eight completed the loop, and every slam wall cell lies on a reference
wall. The drive harness steers with forward speed and turn rate only, and
both 4WS patterns pivot when stopped and arc when moving. So in this drive
the two patterns move the same way (identical paths); their sideways
patterns (crab, in_phase) are not exercised.

## What it shows

- **Leg odometry is good enough for mapping.** Slam_toolbox kept every
  walking robot within 4.5-7.1 cm RMSE of truth over 25-27 m, never more
  than 15 cm away.
- **The maps invent nothing in MuJoCo and Isaac.** Every slam wall cell
  lies on a reference wall, and the mean distance to the true surface is
  5-7 cm, one cell.
- **Wheeled robots map tighter still:** 1.1-2.5 cm for the mecanum and
  4WS cars in every backend. Bumperbot in Gazebo is the exception (11.3 cm,
  13.1 cm on a second run), and 14% of its map cells are more than 0.10 m
  from any reference wall (Go2 in the same world: 5%); the cause is not
  established.
- **Bumperbot's slam pose is worse than its own odometry.** Checked
  2026-10-15 on a later Gazebo drive (round 22, same loop):
  - slam_toolbox's pose was 9.6 cm RMSE from truth;
  - the EKF odometry it was given (TF `odom -> base_footprint`, aligned
    at the start) was 2.7 cm;
  - for the mecanum car, slam corrected its odometry from 24.9 cm to
    2.4 cm.
  - Bumperbot's slam heading drifted to about -0.9° with its first 90°
    turn, and the position error peaked at 18 cm on the far side of the
    loop.
  - Both robots carry the same lidar model. Bumperbot's chassis sits
    pitched 0.6° on its casters, which at 0.15 m keeps the beams off the
    floor within the 12 m range.
  - The cause is not established.
- **Gazebo maps show cells off the reference walls in the same places for
  both robots** (around (-3.5, 3.5), far from the route). That points at
  a Gazebo world/map mismatch there; not verified.
- **The Go2 walks the whole loop in Gazebo** in 73 s since its physics
  step was capped at 2 ms. It walked only lurching at the worlds' own
  10 ms step (P7 navigation, round 4).
- **The drive times are wall-clock.** Isaac ran at 0.24-0.27 of real time
  here, MuJoCo with the G1 at 0.43, and Gazebo at 0.97. That accounts for
  the 217-321 s Isaac drives.

## Limits

- One run per cell, one world, flat ground.
- The drive harness steers on truth, so this measures mapping and
  localization, not autonomous exploration.
- Recall depends on what the route can see; it compares cells between
  runs, not maps against an ideal.
