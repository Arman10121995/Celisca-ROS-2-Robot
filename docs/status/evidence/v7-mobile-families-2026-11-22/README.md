# V7: multi-leg and tracked missions repeated on current code (November 22, 2026)

**Status: 14 of 14 passed.** `run_round.sh` repeats the November 20–21
protocols unchanged, one run at a time under the shared physics lock, at
revision 21d6d202 (installed build).

## Multi-leg saved-map goals (`scripts/qualify_gait_saved_map.py`)

Each model maps the world with its gait, saves the map and reaches a Nav2
goal 0.5 m ahead. Pass requires the goal to settle within 0.15 m and
0.15 rad.

| Backend, map | Model | Result | End to goal | Heading error | Map cells |
|---|---|---|---|---|---|
| MuJoCo, nav_obstacle (open spawn −3, 2.5) | hexapod | **passed** | 0.144 m | 0.002 rad | 21355 |
| | octopod | **passed** | 0.117 m | 0.009 rad | 21258 |
| | centipede | **passed** | 0.143 m | 0.033 rad | 21423 |
| PyBullet, nav_empty (600 s budget) | hexapod | **passed** | 0.146 m | 0.003 rad | 37829 |
| | octopod | **passed** | 0.099 m | 0.001 rad | 37589 |
| | centipede | **passed** | 0.095 m | 0.036 rad | 41161 |

Compared with November 20: the first PyBullet round passed only the
octopod. Later that day the gait was changed:
- turns now go about the root frame, with one stride scale for all legs;
- the spine legs are placed from the spine angles.

After that change, each model passed once on PyBullet. On MuJoCo
nav_obstacle the hexapod and octopod had passed, and the centipede had
reached its goal once (0.113 m) after an earlier heading miss. These
repeats confirm those single passes on current code.

## Tracked vehicle on the ported track plants

| Protocol | MuJoCo | PyBullet |
|---|---|---|
| Nav2 route, nav_obstacle (−7, −7) → (−3, −4), 0.15 m / 5° (`tracked_nav_probe.py`) | **2/2**: 0.010 m / 2.4°, 0.017 m / 0.9° | **2/2**: 0.016 m / 3.0°, 0.059 m / 2.5° |
| Command loss (`tracked_loss_probe.py`): the command owner is frozen while driving | **2/2**: stopped within 0.056 m, 0 mm in the final 2 s | **2/2**: within 0.054 m, 0 mm |

November 21 measured one run of each, and all passed.

## Limits

- Only these protocols are repeated. Fleet, swarm, marine and form
  missions, and resets, are not part of this round.
- One goal per multi-leg model and backend.
- Gazebo and Isaac were not re-run.

## Reproduce

```bash
docs/status/evidence/v7-mobile-families-2026-11-22/run_round.sh <evidence-dir>
```

Reports are copied here. Logs, maps and run folders are on SSD under
`robot_lab_runtime/v7-mobile-families-20261122`; their hashes are in
`runtime-sha256.txt`.
