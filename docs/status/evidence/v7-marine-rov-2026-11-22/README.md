# V7: the lab ROV on four backends, repeated (November 22, 2026)

**Status: 8 of 8 runs reproduce the November 18–19 rounds.**
`run_round.sh` runs `lab_rov` twice per backend at revision ae47019d:
- `marine_simulator.launch.py` on `marine_tank`, with the USV round's case
  runner (`MARINE_MODEL=lab_rov`);
- the same `rov_probe.py`: hold; surge at 0.2 m/s for 10 s; yaw at
  0.3 rad/s for 6 s; 15 s in a 0.1 m/s current; operator loss.

| Phase | Gazebo | MuJoCo | PyBullet | Isaac | November 18–19 (G / M / P / I) |
|---|---|---|---|---|---|
| Hold drift | 2.2, 2.2 cm | 0.9, 1.1 cm | 0.7, 0.7 cm | 0.5, 0.6 cm | 2 / 5.7 / 7.9 / 8.2 cm (while arming) |
| Surge 10 s | 1.073, 1.073 m | 1.037, 1.045 m | 1.037, 1.036 m | 1.052, 1.044 m | 1.074 / 1.030 / 1.041 / 1.038 m |
| Yaw 6 s | 1.478, 1.483 rad | 1.426, 1.407 rad | 1.448, 1.425 rad | 1.460, 1.450 rad | 1.495 / 1.437 / 1.439 / 1.452 rad |
| 15 s current | 1.0, 1.0 cm | 1.7, 3.1 cm | 3.2, 1.0 cm | 0.7, 0.8 cm | 1.0 / 0.9 / 3.5 / 3.2 cm |
| Operator loss | disarms ×2 | disarms ×2 | disarms ×2 | disarms ×2 | disarms |

- Surge agrees within 1.5 % and yaw within 2 % of the earlier rounds on
  every backend. The current feedforward still holds station to a few
  centimetres.
- Hold drift on the ported plants is now 0.5–1.1 cm, against 5.7–8.2 cm
  on November 18–19, with the same probe. Gazebo is unchanged (2 cm). The
  change since then that improved the ported hold is not identified.
- No launch process was left behind.

## Reproduce

```bash
docs/status/evidence/v7-marine-rov-2026-11-22/run_round.sh
```

Results are copied as `<backend>-<run>-result.json`. Logs and traces are on
SSD under `robot_lab_runtime/v7-rov-20261122`. Their hashes are in
`runtime-sha256.txt`.
