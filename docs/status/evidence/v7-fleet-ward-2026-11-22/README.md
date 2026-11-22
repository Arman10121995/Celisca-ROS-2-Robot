# V7: fleet_ward doors, docking and energy, repeated (November 22, 2026)

**Status: 3 of 3 passed.** `run_round.sh` repeats run 3 of the
[November 18 R10.2 receipt](../r102-fleet-ward-2026-11-18/README.md)
three times on Gazebo, at revision 3508dd8c:
- the same session: Bumperbot `alpha` in the west room, Labbot `beta` in
  the east room;
- the same `ward_probe.py`: both robots are sent through the one door at
  once, in opposite directions, then each docks at the far room's charger;
- each robot must be docked and still for 1 s, with its modeled energy
  rising over 10 s.

| Run | Verdict | Crossing | Door reservation | Closest approach (radii sum 0.30 m) | Contacts | Docked after (α / β) | Modeled charge over 10 s |
|---|---|---|---|---|---|---|---|
| 1 | **passed** | both succeeded | alpha waited 0.99 m before `door:ward` while beta held it | 1.01 m | 0 | 19.3 / 19.4 s | 0.7988 → 0.8053 / 0.8052 at 120 W |
| 2 | **passed** | both succeeded | the same (0.98 m) | 1.01 m | 0 | 19.9 / 19.5 s | 0.7988 → 0.8052 / 0.8054 |
| 3 | **passed** | both succeeded | the same (0.98 m) | 1.00 m | 0 | 19.9 / 19.7 s | 0.7988 → 0.8052 / 0.8053 |
| 11-18 run 3 | passed | both succeeded | alpha waited | 1.05 m | 0 | 21.6 s each | 0.7988 → 0.8047 |

The door went closed → moving → open in every run. Each docking was about
2 s faster than on November 18. The modeled charge rose slightly more over
the same 10 s window because charging started earlier in that window.
No launch process was left behind.

## Limits

- One door and two docks, two robots, Gazebo only.
- Energy is the authored battery model with sampled wheel effort, not a
  hardware battery.

## Reproduce

```bash
docs/status/evidence/v7-fleet-ward-2026-11-22/run_round.sh
```

Probe results are copied here. Traces, journals and launch logs are on SSD
under `robot_lab_runtime/v7-ward-20261122`. Their hashes are in
`runtime-sha256.txt`.
