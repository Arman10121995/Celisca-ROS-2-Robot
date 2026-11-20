# R10.6: saved-map Nav2 goals in a second world, `nav_obstacle` (November 20, 2026)

**Status: implemented, experimental.** The 11-19 saved-map goals
([receipt](../r106-gait-saved-map-2026-11-19/README.md)) were measured in
`nav_empty`. This repeats them in the obstacle world `nav_obstacle` on
MuJoCo, with the same runner, workloads and 0.15 m / 0.15 rad settle
contract. Each model first maps the world and saves the map. In a fresh
launch it then reaches a Nav2 goal 0.5 m ahead on that saved map, judged
by independent truth.

- From the open spawn, the hexapod (0.128 m) and the octopod (0.099 m)
  pass.
- From the world's corner spawn, the octopod passes (0.105 m).
- The centipede does not pass from either spawn.

## What changed

`scripts/qualify_gait_saved_map.py`:

- **`--spawn x,y,yaw`** passes the pose to both launches. The world spawn
  (−7, −7) is 0.65 m from the nearest obstacle. The only open area on the
  map (clearance 2.2 m) is around (−2.6, 2.5); the open runs start at
  (−3.0, 2.5).
- **A fallback for the mission result:** the runner left one `run.json`
  without `mission_result`, so the qualifier now also reads the workload's
  own `extension-mission-result.json`. The hexapod's open run had passed;
  its report first showed no result.

## Measured (MuJoCo, `nav_obstacle`, goal 0.5 m ahead on the model's own saved map)

| Spawn | Model | Map cells | Result | End to goal | Heading error | Wall time |
|---|---|---|---|---|---|---|
| corner (−7, −7) | hexapod | 6877 | **not passed**: stopped 0.152 m short at t ≈ 95 s; Nav2 never finished; timed out | 0.152 m | 0.110 rad | 220 s |
| corner | octopod | 6854 | **passed** | 0.105 m | 0.083 rad | 77 s |
| corner | centipede | 6871 | **not passed**: Nav2 aborted at once, "Starting point in lethal space" (its long footprint within the corner obstacle's inflation) | 0.711 m (did not move) | — | 9.5 s |
| open (−3, 2.5) | hexapod | 21311 | **passed** | 0.128 m | 0.011 rad | 97 s |
| open | octopod | 21316 | **passed** | 0.099 m | 0.010 rad | 64 s |
| open | centipede | 21362 | **not passed**: settled 0.129 m away but 0.172 rad off heading (contract 0.15); Nav2 never finished; timed out | 0.129 m | 0.172 rad | 220 s |

The contract was not loosened. Both centipede failures are about its body.
Its footprint is too long for the corner spawn, and it ends a short goal
turned 0.17 rad. On `nav_empty` it passed 1 of 2, and the other run stopped
0.157 m off.

## Limits

- MuJoCo only. Saved-map goals on PyBullet or Isaac need a sensor
  configuration in their gait launch branch, which they do not have yet.
- One short goal per run (0.5 m ahead).
- The centipede still lacks a passing run in this world.

## Reproduce

```bash
python3 scripts/qualify_gait_saved_map.py --output <new dir> --map nav_obstacle --spawn=-3.0,2.5,0.0 \
  --robots hexapod_ros2 lab_octopod lab_centipede
```

`summary.json` lists every run. Logs are on SSD under
`robot_lab_runtime/gait-saved-map-obstacle{,-open}-20261120` (hashes in
`runtime-sha256.txt`).
