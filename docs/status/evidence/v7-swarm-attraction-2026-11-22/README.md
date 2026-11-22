# V7: swarm attraction repeated; flocking still fails (November 22, 2026)

**Status: attraction 2 of 2 passed; flocking 0 of 1, as on November 18.**
`run_round.sh` repeats two runs of the
[November 18 receipt](../r103-swarm-aggregation-2026-11-18/README.md) on
Gazebo at revision 7bb36b6b:
- the passing attraction run 3, twice;
- flocking run 4, once.

Both use `session-open-triangle.yaml`: three Bumperbots in a 2.5 m
triangle in `nav_empty`, with walls beyond sensing range. The runner is
the aggregation round's `run_case.py` (`SWARM_SESSION`, `SWARM_PHASES`).

| Run | Behaviour | Mean distance start → end | Closest pair (radii 0.24 m) | All within 0.6 m after | November 18 |
|---|---|---|---|---|---|
| attraction 1 | `local_attraction` 60 s | 2.49 → 0.41 m | 0.32 m | 5.7 s | 2.5 → 0.39 m in about 7 s, closest 0.36 m |
| attraction 2 | `local_attraction` 60 s | 2.49 → 0.40 m | 0.35 m | 5.2 s | |
| flocking 1 | `local_flocking` 120 s | 2.49 → **11.1 m** | 0.54 m | — | 2.5 → 7.6 m (not passed) |

- **Attraction repeats.** The members close in from 2.5 m and hold
  together without contact.
- **Flocking still disperses.** As on November 18, the members drift
  apart, this time further. The ROS2swarm flocking port does not keep three
  scan-only members together. This is open.

## Reproduce

```bash
docs/status/evidence/v7-swarm-attraction-2026-11-22/run_round.sh
```

Results are copied here. Traces and launch logs are on SSD under
`robot_lab_runtime/v7-swarm-attraction-20261122`. Their hashes are in
`runtime-sha256.txt`.
