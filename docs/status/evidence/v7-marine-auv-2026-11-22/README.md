# V7: Tethys AUV on four backends, repeated (November 22, 2026)

**Status: 8 of 8 runs completed the probe, and every run disarmed.
Cruise and turn reproduce November 19 on every backend. Gazebo's depth
during the turn has changed.** `run_round.sh` repeats the
[November 19 receipt](../r104-tethys-mujoco-2026-11-19/README.md) twice
per backend at revision 7b0b5ed4:
- `auv_simulator.launch.py`, spawning 1.5 m deep in `marine_tank`;
- the same `auv_probe.py`: neutral 5 s; cruise at 0.3 m/s for 10 s; turn
  at 0.1 rad/s while cruising for 25 s; climb at 0.1 m/s for 10 s;
  operator loss.

Values are distance, yaw change and depth change (positive = rose). Each
cell gives run 1 / run 2.

| Phase | Gazebo | MuJoCo | PyBullet | Isaac |
|---|---|---|---|---|
| Cruise | 2.45 / 2.46 m | 2.42 / 2.42 m | 2.43 / 2.41 m | 2.42 / 2.42 m |
| Turn | 6.97 / 6.96 m, +1.19 rad, rose 0.004 / 0.015 m | 7.16 m, +0.86 / +0.87 rad, rose 0.31 m | 7.43 / 7.44 m, +0.51 rad | 7.44 m, +0.49 rad |
| Climb | 0.07 / 0.24 m (at the tank wall) | 2.91 / 2.92 m, rose 0.59 m | 2.99 / 2.98 m | 2.99 m |
| End | disarmed: operator loss / **attitude-depth envelope exceeded** | disarmed: operator loss | disarmed: operator loss | disarmed: operator loss |
| **November 19** | cruise 2.45 m; turn 6.93 m, +1.19 rad, **rose 0.24 m**; climb 0.32 m | 2.43; 7.17 m, +0.86 rad, rose 0.32; 2.92 m | 2.42; 7.43 m, +0.51 rad | 2.43; 7.44 m, +0.49 rad |

- **Cruise and turn reproduce.** Distances agree within 0.04 m and yaw
  within 0.01 rad of November 19 on every backend. The ports still turn
  less than Gazebo; the November 20 identification explains this as the
  opposite added-mass sign in Fortress.
- **Gazebo depth during the turn changed.** On November 19 the Gazebo AUV
  rose 0.24 m while turning. Now it holds depth within 1.5 cm. The MuJoCo
  port still rises 0.31 m, as before. This round does not identify which
  change since November 19 improved the Gazebo depth hold.
- **Gazebo run 2's climb** ended at the tank wall. The vehicle disarmed on
  its attitude/depth envelope before the operator-loss phase. It was
  disarmed either way, but the reason is not the probe's. On November 19
  the climb also ended at the wall, and the receipt marks the climb as not
  comparable.
- The Isaac run 1 teardown left two processes briefly. They had exited
  when checked seconds later. No other leftovers.

## Limits

- One tank; the Gazebo climb reaches the wall. The ported plants have only
  a seabed.

## Reproduce

```bash
docs/status/evidence/v7-marine-auv-2026-11-22/run_round.sh
```

Results are copied as `<backend>-<run>-result.json`. Logs and traces are on
SSD under `robot_lab_runtime/v7-auv-20261122`. Their hashes are in
`runtime-sha256.txt`.
