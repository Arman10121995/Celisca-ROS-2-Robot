# V8: MOLAR protocols repeated on current code (November 22, 2026)

**Status: 12 of 13 runs passed.** All 10 transfers passed, and 2 of 3
Nav2 mission runs passed. In the failed run, route A, cancel and resume
passed; goal B aborted, and the run is kept.

`run_round.sh` repeats the November 17–18 MOLAR (`molar_hri`) receipts
with their own arguments, at revision 4caa7f87, from 07:50 to 08:12.
Each run took one shared lock hold.

## Payload transfer to a station (`scripts/molar_station_transfer_check.py`)

MOLAR lifts the 0.5 kg box, extends the telescope and runs the deck belt.
It then retracts onto a passive station, through the normal articulation
interface.

| Backend, map | Wait scale | Runs | Payload centre ahead of base | Retract drag | Earlier |
|---|---|---|---|---|---|
| Gazebo, nav_empty | 2 | **2/2** | 0.678, 0.678 m | 0 mm | 0.678 m (11-17) |
| MuJoCo, nav_empty | 2 | **2/2** | 0.672, 0.672 m | 0 mm | 0.672 m (11-17) |
| PyBullet, nav_empty | 4 | **2/2** | 0.681, 0.681 m | 0 mm | 0.687 m, 1.0 mm (11-17) |
| Isaac, nav_empty | 3 | **2/2** | 0.672, 0.671 m | 0 mm | 0.671–0.673 m (11-18) |
| Gazebo, celisca_floor_1_detailed | 4 | **2/2** | 0.678, 0.678 m | 0 mm | 0.669 m (11-17) |

Every run left the box on the station and off the deck.

## Nav2 on CELISCA floor 1 (`scripts/sim_nav_mission.sh`, as on 11-18)

The run uses `CHECK_ARGS="--base base_link --robot-radius 0.49"`, seed 1,
and the same goals as November 15 and 18:
- route A: a 9.6 m detour, cancelled after 1 m and then resumed;
- goal B: 0.60 m from an obstacle.

| Run | Cancel A | Resume A (truth) | Goal B (truth) | Estimate − truth at the end of A |
|---|---|---|---|---|
| 1 | passed | 0.070 m, 5.8° | **aborted** after 17 s, "Starting point in lethal space" | 0.158 m |
| 2 | passed | 0.075 m, 10.7° | **0.042 m**, 4.4° | 0.128 m |
| 3 | passed | 0.177 m, 8.7° | **0.082 m**, 5.0° | 0.202 m |

- On November 18, after MOLAR's AMCL overlay, goal B passed 2/2 (0.09 m).
  Today it passed 2 of 3.
- Run 1 is the November 15 failure. From its estimated pose, the planner
  put MOLAR's footprint in lethal cost and aborted.
- The localization error at the end of route A does not separate the
  failed run from the two that passed (0.158 m against 0.128 and
  0.202 m). The run is kept and the cause is open.
- The resume heading errors (5.8–10.7°) do not count against a pass in
  this protocol, as before.

## Limits

- Mounted sensors still lack the real robot's layout (R11, I17).
- Navigation was repeated on Gazebo only. The transfer used one box and one
  station.
- The fleet lift, Open-RMF and cargo receipts (November 19–20) are not
  part of this round.
- "Identified dynamics" in the V8 row needs measurements from the real
  robot, which are not available.

## Reproduce

```bash
docs/status/evidence/v8-molar-repeats-2026-11-22/run_round.sh
NAV_RUNS=3 docs/status/evidence/v8-molar-repeats-2026-11-22/run_round.sh   # the third Nav2 run
```

Reports are copied here. Launch logs, TF bags and traces are on SSD under
`robot_lab_runtime/v8-molar-20261122`. Their hashes are in
`runtime-sha256.txt`. `progress.txt` also logs the transfers that the
third-run invocation skipped (`exit=0` lines).
