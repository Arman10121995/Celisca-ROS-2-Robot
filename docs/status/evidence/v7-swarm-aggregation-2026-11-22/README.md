# V7: swarm aggregation repeated (November 22, 2026)

**Status: the three Bumperbots gathered in 3 of 3 runs. They stayed
together for 33–71 % of the phase, and in 1 of 3 runs they ended all
together.** `run_round.sh` repeats run 6 of the
[November 18 receipt](../r103-swarm-aggregation-2026-11-18/README.md)
three times, at revision 3491fc1a:
- Gazebo `swarm_arena` with Nav2 localization;
- three Bumperbots spread 3.6–4.0 m apart, beyond their 3.5 m sensing
  range;
- 180 s of ROS2swarm-style `local_aggregation`, each member using only its
  own scan;
- measured by the same `swarm_probe.py`.

"Together" means all three are within 1.5 m of each other
(`together.py` on the probe trace).

| Run | First together | Together | Together, second half | Mean distance start → end | Closest pair | End pairs |
|---|---|---|---|---|---|---|
| 1 | 29.6 s | 33 % | 7 % | 3.88 → 1.40 m | 0.35 m | 0.68, 2.08, 1.45 m |
| 2 | 22.0 s | 43 % | 43 % | 3.88 → 2.00 m | 0.51 m | 3.00, 1.11, 1.90 m |
| 3 | 22.2 s | 71 % | 94 % | 3.88 → 1.06 m | 0.52 m | 0.82, 1.36, 1.01 m |
| 11-18 run 6 | 21.9 s | 56 % | 50 % | 3.88 → 0.96 m | 0.39 m | all within 1.1 m |

- **Gathering repeats.** Every run brought the spread members together
  within 22–30 s of simulated time, as on November 18.
- **Staying together varies.** ROS2swarm's aggregation is probabilistic:
  members stay, leave and rejoin. Over three runs the time together ranged
  from 33 to 71 %, around the single November 18 value (56 %). The end
  state is a snapshot of that cycle, and only run 3 ended all together.
- No pair came closer than 0.35 m (radii 0.24 m), and no launch process
  was left behind.

## Limits

- One arena, one start and three members. Attraction and flocking are not
  repeated here (on November 18 flocking did not keep the group).
- The probe stops the members at the end; the result is the measured
  state, not a pass/fail contract.

## Reproduce

```bash
docs/status/evidence/v7-swarm-aggregation-2026-11-22/run_round.sh
python3 docs/status/evidence/v7-swarm-aggregation-2026-11-22/together.py <run>/run-*/agg/trace.jsonl
```

Probe summaries are copied here. Traces and launch logs are on SSD under
`robot_lab_runtime/v7-swarm-20261122`. Their hashes are in
`runtime-sha256.txt`.
