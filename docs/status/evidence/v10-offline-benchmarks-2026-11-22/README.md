# V10: the offline R7 benchmarks repeated at one revision (November 22, 2026)

**Status: all five reproduce October's accuracy results exactly.** At
revision b803693, `run_all.sh` re-ran the October 13 offline benchmarks
for perception (R7.2), localization (R7.3), state estimation (R7.4),
sensor fusion (R7.5) and control (R7.8), with October's settings.

| Benchmark | October revision | Methods | Accuracy vs October | Compute time |
|---|---|---|---|---|
| [R7.2 perception](r72/report.md) | 42d1ccb | laser obstacle detection and scan clustering, RANSAC ground removal, Euclidean and DBSCAN object clustering | identical, every value | −5 % to +7 % |
| [R7.3 localization](r73/report.md) | 42d1ccb | dead reckoning, ICP point-to-point and point-to-line, NDT, correlative | identical | −9 % to +1 % |
| [R7.4 state estimation](r74/report.md) | 65ae417 | CV Kalman, CTRV EKF and UKF, particle filter, IMM, window smoother | identical | −2 % to +20 % (p99 of IMM and window smoother) |
| [R7.5 fusion](r75/report.md) | 303e01a | complementary, Mahony, Madgwick, ESKF attitude; wheel/IMU EKF, EKF and UKF with GNSS | identical | −13 % to −1 % |
| [R7.8 control](r78/report.md) | 58b8a70 | PID, LQR, linear and nonlinear MPC, feedback linearization, backstepping | identical tracking, disturbance, envelope and saturation tables | see below |

The benchmarks are seeded and deterministic, so identical accuracy shows
that the algorithm code these benchmarks exercise did not change between
the revisions. They are model and ray-cast benchmarks. Live simulator
replays are not part of this round. The October receipts remain the
reference for the method descriptions and protocols:
- [R7.2](../r72-perception-benchmark-2026-10-13/README.md)
- [R7.3](../r73-localization-benchmark-2026-10-13/README.md)
- [R7.4](../r74-state-estimation-benchmark-2026-10-13/README.md)
- [R7.5](../r75-fusion-benchmark-2026-10-13/README.md)
- [R7.8](../r78-control-benchmark-2026-10-13/README.md)

**Control timing.** In October the benchmark ran with a host load average
of 11–16. Here the load was 2–3. About 34 300 control steps per method:

| Controller | p99 / max (ms), October | p99 / max (ms), now | Deadline misses, October → now |
|---|---|---|---|
| pid | 0.169 / 17.33 | 0.054 / 0.19 | 0 → 0 |
| lqr | 0.244 / 15.75 | 0.057 / 5.42 | 0 → 0 |
| linear_mpc | 6.403 / 30.17 | 1.765 / 2.87 | 0 → 0 |
| nonlinear_mpc | 13.258 / 70.74 | 7.290 / 10.14 | 2 → 0 |
| feedback_linearization | 0.071 / 9.19 | 0.022 / 0.14 | 0 → 0 |
| backstepping | 0.041 / 7.67 | 0.022 / 0.13 | 0 → 0 |

October's two nonlinear-MPC deadline misses came from host load: on a
quieter host there were none. The host was still not idle (other agents'
gazebo processes were present).

## Run notes

- The first pass failed at once in every benchmark (`exit=127`). The
  wrapper called `/usr/bin/time`, which this host lacks. The wrapper now
  times with the shell.
- R7.2's default changed from October's 100 scenes to 50. The 50-scene
  run is kept in `r72-default50/`. `r72/` holds the 100-scene run that
  is compared with October.
- R7.3 reuses October's `trajectories.json` (the same Gazebo truth traces).

## Reproduce

```bash
docs/status/evidence/v10-offline-benchmarks-2026-11-22/run_all.sh
python3 scripts/r7_2_perception_benchmark.py --out <dir> --scenes 100
```
