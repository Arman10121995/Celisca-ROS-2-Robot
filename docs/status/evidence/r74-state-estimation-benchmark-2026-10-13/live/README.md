# R7.4 on simulator AMCL fixes (2026-10-13)

The [offline benchmark](../README.md) feeds the estimators a modelled
10 Hz sensor with white 0.3 m noise, where every estimator halves the
position error. This is the next step that benchmark named: the same
estimators (`robot_lab_algorithms.tracking_filters`) on the real
`/amcl_pose` stream that AMCL published live during the R7.3 simulator
runs, scored against the `/odom/ground_truth` truth recorded in the same
bags (`scripts/r7_4_live_replay.py`). Bumperbot drove the R4.3 goals on
nav_maze (A*, seeds 23/37/53, Regulated Pure Pursuit, AMCL), once in Gazebo
and once in MuJoCo; the bags are the R7.3 live-scan bags
(`EXTRA_BAG_TOPICS=/scan scripts/planner_comparison.sh`, kept on the SSD in
`robot_lab_runtime/claude-r73-live-{gazebo,mujoco}/`).

Each estimator is replayed offline over every run's fixes in time order:
`predict(dt)` with the wall-clock gap between fixes, `update((x, y))` with
each fix, scored against truth interpolated to the fix's stamp. The first
10 s are skipped as warm-up. Gains are the node defaults
(`measurement_std` 0.3, `accel_std` 0.5); the raw AMCL fixes are the
baseline. Code as in `681939c`.

The input is not the modelled sensor. AMCL publishes at ~1.6 Hz with bursty
timing (gaps 0.2-7.8 s, median 0.4-0.6 s), and its fixes are the output of
a scan matcher, so they are correlated over time and carry the ~4 cm fixed
map-registration offset R7.3 measured (mean offset here -0.038 to -0.039 m
in x). It is a real, low-rate, correlated localizer, not white noise.

## Results (position RMSE against simulator truth)

| Estimator | Gazebo | MuJoCo |
|---|---|---|
| Raw AMCL fixes (baseline) | **0.080 m** | **0.090 m** |
| Linear Kalman (CV) | 0.083 m | 0.093 m |
| CTRV EKF | 0.084 m | 0.095 m |
| CTRV UKF | 0.113 m | 0.121 m |
| Particle filter | 0.284 m | 0.241 m |
| IMM | 0.086 m | 0.099 m |
| Window smoother | 0.083 m | 0.093 m |

Raw AMCL: p95 0.115/0.131 m, max 0.141/0.155 m. The estimators' worst case
grows with model order (EKF/UKF p95 up to 0.209 m, particle filter up to
0.68 m). Full per-run and per-seed numbers are in the `results.json`
files. Baselines cross-check the R7.3 live scan replay, which scored the
same AMCL output at 0.079 m (Gazebo) and 0.088 m (MuJoCo).

## What it shows

- **On real AMCL fixes the tracking filters do not help.** Every estimator
  is 3-28% worse than the raw fixes; the simple CV Kalman and window
  smoother are closest, the particle filter and UKF are far worse. The
  offline benchmark had every estimator cut the error in half.
- **Why.** AMCL is already a good, low-rate, *correlated* localizer. A
  filter that assumes independent white noise only averages away noise that
  is not there; at ~1.6 Hz with 0.4-0.6 s gaps a motion model has little to
  predict over, and its process noise adds lag at the turn points the CV and
  window models cannot follow. The gains that win on white noise lose here.
- **The model's own error is what the benchmark exposed offline.** The
  UKF and particle filter, which gained nothing on the modelled sensor,
  actively hurt on the real one - their sampling and linearisation noise is
  now the dominant error, 2-3x the raw fixes.
- **Limits.** One robot and one map, three seeds each, and the replay runs
  offline (the live nodes publish the same estimates, but their timing in a
  running stack is not measured here). Global localization and recovery are
  not tested; AMCL already ran live and converged.

## Reproduce

```bash
python3 scripts/r7_4_live_replay.py --out <out>/gazebo \
  /workspace/molar/robot_lab_runtime/claude-r73-live-gazebo/*.bag
python3 scripts/r7_4_live_replay.py --out <out>/mujoco \
  /workspace/molar/robot_lab_runtime/claude-r73-live-mujoco/*.bag
```

The bags stay on the SSD in `robot_lab_runtime/claude-r73-live-{gazebo,mujoco}/`.
