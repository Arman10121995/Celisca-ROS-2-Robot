# R7.4: state estimators on noisy position fixes (2026-10-13)

Status: **implemented, experimental**. The estimators are measured on a
modelled trajectory and sensor with known truth, and on the real AMCL
fixes the R7.3 simulator runs recorded ([live](live/README.md)).

## What was built

`robot_lab_algorithms.tracking_filters` (revisions `0df755d`, `65ae417`).
The state estimation node ids now run it:

| Selection (`state_estimation:=`) | Method |
|---|---|
| `linear_kalman_filter` | Linear Kalman filter, constant velocity `[x, y, vx, vy]`, white-acceleration process noise |
| `ekf_3d_estimator` | Extended Kalman filter on the constant turn rate and velocity (CTRV) model `[x, y, θ, v, ω]` |
| `ukf_estimator` | Unscented Kalman filter on the CTRV model |
| `particle_filter` | Particle filter on the CTRV model: 500 particles, systematic resampling, log-domain weights, sensor resetting, roughening |
| `motion_model_estimator` | Interacting multiple model: a straight-line and a turning CTRV EKF, Markov switching 0.05 |
| `pose_graph_estimator` | Sliding-window least squares over the last 20 fixes (a linear Gaussian pose graph) |

**How the nodes run.** Each node (`state_estimation.StateEstimatorNode`):

- takes position fixes from `/amcl_pose` (any
  `PoseWithCovarianceStamped` topic);
- or, for matched-input runs, reads truth at 10 Hz and adds seeded
  Gaussian noise (`inject_noise_std`);
- publishes position, heading, speed and turn rate as `nav_msgs/Odometry`
  after every fix, on `/odometry/linear_kf`, `ctrv_ekf`, `ukf`,
  `particle_filter`, `imm` or `window_smoother`.

**What the ids ran before.** The earlier nodes filtered noiseless ground
truth, so nothing was estimated. The classes remain for their tests only.
`error_state_ekf` is retired here; the error-state formulation now runs
for attitude as sensor fusion `attitude_eskf`
([R7.5](../r75-fusion-benchmark-2026-10-13/README.md)).

**Fixes found by this benchmark.**

- The UKFs stopped with "Matrix is not positive definite" at small process
  noise. They now clip the covariance eigenvalues when the Cholesky
  factorisation fails (`0df755d`).
- The particle filter lost the track on some seeds (position RMSE 0.42,
  1.14 and 7.76 m): far fixes underflowed every weight to the same floor.
  - Weights are now normalised in the log domain.
  - A fifth of the particles is re-seeded around a fix that no particle
    explains.
  - Resampled particles get a small jitter.
  - Seeds 0-3 now give 0.18-0.22 m (`65ae417`).

## Benchmark (`scripts/r7_4_state_estimation_benchmark.py`)

**Truth.** A ground robot drives a 300 s CTRV route that repeats every
100 s: straight runs; turns of 0.2-0.6 rad/s both ways; speeds 0.15-0.8 m/s;
and a 5 s stop.

**Sensor.** Position fixes at 10 Hz with 0.3 m noise per axis. The raw
fixes have 0.425 m RMSE.

**Tuning.** One gain per estimator (the process noise `accel_std`) is
chosen on seed 0 from {0.1, 0.2, 0.35, 0.5, 1, 2}. Results are means over
seeds 1-3. The first 5 s are skipped.

| Estimator | Position RMSE | Speed RMSE | Heading RMSE, v > 0.3 m/s | Turn-rate RMSE | Compute p50 / p99 |
|---|---|---|---|---|---|
| Raw fixes | 0.425 m | - | - | - | - |
| Linear Kalman (CV) | 0.167 m | 0.112 m/s | **16.7°** | 0.261 rad/s | **0.06** / 0.07 ms |
| CTRV EKF | **0.164 m** | **0.111 m/s** | 17.3° | 0.234 rad/s | 0.17 / 0.19 ms |
| CTRV UKF | 0.169 m | 0.122 m/s | 23.1° | 0.271 rad/s | 0.23 / 0.27 ms |
| Particle filter | 0.191 m | 0.128 m/s | 24.3° | 0.335 rad/s | 0.22 / 0.36 ms |
| IMM | 0.166 m | 0.111 m/s | 19.6° | **0.213 rad/s** | 0.45 / 0.48 ms |
| Window smoother | 0.187 m | 0.136 m/s | 18.5° | 0.261 rad/s | 0.22 / 0.25 ms |

Per-seed numbers and the tuning grid are in [results.json](results.json);
the table with heading error at v > 0.1 m/s is in [report.md](report.md).
Position RMSE per seed varies by at most 0.017 m.

## What it shows

- **Every estimator cuts the position error by more than half** (0.16-0.19 m
  against 0.43 m).
- **The motion model matters less than expected.** The constant-velocity
  Kalman filter is within 3 mm of the CTRV EKF. At 10 Hz the turns are
  gentle enough that a straight-line model with tuned process noise keeps
  up.
- **The turn rate is where the models differ.** The IMM, which switches
  between a straight and a turning model, estimates ω best (0.213 rad/s).
- **Heading from position alone is poor** (16-24° at speed). Heading is only
  observable through motion, and 0.3 m noise at 10 Hz hides short motions.
  A gyro or wheel odometry is needed for a usable heading (see
  [sensor fusion](../r75-fusion-benchmark-2026-10-13/README.md)).
- **The UKF and the particle filter gain nothing here.** With position
  measurements the problem is nearly linear, so the EKF linearisation is
  accurate. The particle filter's sampling noise costs 2-3 cm.
- **The window smoother lags.** The newest pose of a 2 s window has no
  future fixes to refine it, and the constant-velocity factors over the
  window smooth through the turns.
- **Limits.** The sensor noise is white and Gaussian, with no outliers,
  delays or multipath. AMCL fixes are correlated over time. A run on
  simulator AMCL output is in [live](live/README.md).

## Reproduce

```bash
python3 scripts/r7_4_state_estimation_benchmark.py --out <dir>
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/robot_lab_algorithms/test/test_tracking_filters.py
```
