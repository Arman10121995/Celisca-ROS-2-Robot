# V7: fleet lift transport L0 → L1 → L0, repeated (November 22, 2026)

**Status: 3 of 3 passed.** MOLAR passed twice and Bumperbot once.
`run_round.sh` repeats the
[November 20 receipt](../r11-molar-transport-2026-11-20/README.md) at
revision 9a86a7b4:
- Gazebo `fleet_facility`, with the same `transport_trial.py`;
- the operator sends only the two Facility `transport` requests;
- the infrastructure drives the approach, the landing gate, boarding, the
  ride, the floor and map switch, and a settled exit, each judged by body
  truth.

| Case | Result | Time | L1 exit (x, y, yaw) | L0 exit | November 20 |
|---|---|---|---|---|---|
| MOLAR 1 | **passed** | 203.0 s | 2.178, 0.004, 3.12 | 2.185, 0.041, −3.14 | runs 3–4 passed in 182.9 / 188.6 s, L1 exit 2.143 |
| MOLAR 2 | **passed** | 201.4 s | 2.168, −0.004, −3.11 | 2.172, −0.016, −3.13 | |
| Bumperbot | **passed** | 183.8 s | 2.203, 0.004, 0.01 | 2.388, 0.004, −0.01 | passed in 174.2 s; exits L1 2.215, L0 2.398 |

- The MOLAR exits keep the arrival heading of π, as on November 20,
  within 3 cm.
- MOLAR took 13–20 s longer than November 20. The L1 landing came at
  109 s, against 94 s. Bumperbot took 10 s longer.
- The Bumperbot session uses the November 2 spawn (2.5, 0) with RGB-D on.
  The receipt does not keep its session file.
- No launch process was left behind.

## Reproduce

```bash
docs/status/evidence/v7-fleet-transport-2026-11-22/run_round.sh
```

Results are copied as `<case>-result.json`. Logs and traces are on SSD under
`robot_lab_runtime/v7-transport-20261122`. Their hashes are in
`runtime-sha256.txt`.
