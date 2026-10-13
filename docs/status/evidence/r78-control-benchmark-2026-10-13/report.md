# R7.8 control benchmark (model plant)

Revision `58b8a70`, molar1 aarch64, Python 3.10.12, NumPy 1.26.4, SciPy 1.15.3.

Plant: unicycle, actuator lag 0.10 s, |a| <= 0.5 m/s^2, |alpha| <= 2.0 rad/s^2, v in [0.0, 0.3] m/s, |w| <= 1.0 rad/s; control period 0.05 s.

Host load average (1/5/15 min) [11.39, 12.91, 12.89] at start, [16.37, 15.23, 13.86] at end, 12 CPUs; other processes were running, so compute times are not from an idle host.

## Tracking

| Path | Controller | Finished | Time (s) | RMS lateral (m) | Max lateral (m) | RMS heading (deg) | int w^2 dt | Command TV |
|---|---|---|---|---|---|---|---|---|
| s_curve | pid | yes | 42.5 | 0.0001 | 0.000 | 0.01 | 0.102 | 0.30 |
| s_curve | lqr | yes | 42.5 | 0.0002 | 0.000 | 0.01 | 0.102 | 0.30 |
| s_curve | linear_mpc | yes | 42.5 | 0.0003 | 0.000 | 0.01 | 0.102 | 0.29 |
| s_curve | nonlinear_mpc | yes | 42.5 | 0.0007 | 0.001 | 0.02 | 0.102 | 0.29 |
| s_curve | feedback_linearization | yes | 42.5 | 0.0032 | 0.005 | 0.09 | 0.101 | 0.32 |
| s_curve | backstepping | yes | 42.5 | 0.0003 | 0.000 | 0.01 | 0.102 | 0.29 |
| rectilinear_turns | pid | yes | 48.2 | 0.0337 | 0.098 | 7.63 | 6.091 | 8.56 |
| rectilinear_turns | lqr | yes | 48.2 | 0.0348 | 0.098 | 7.48 | 6.010 | 8.43 |
| rectilinear_turns | linear_mpc | yes | 47.8 | 0.0325 | 0.086 | 7.22 | 5.673 | 8.25 |
| rectilinear_turns | nonlinear_mpc | yes | 48.5 | 0.0041 | 0.015 | 2.86 | 5.832 | 9.13 |
| rectilinear_turns | feedback_linearization | yes | 50.0 | 0.0216 | 0.069 | 10.77 | 3.712 | 11.61 |
| rectilinear_turns | backstepping | yes | 48.4 | 0.0326 | 0.088 | 7.16 | 6.043 | 8.63 |
| tight_arc | pid | yes | 8.0 | 0.0024 | 0.007 | 0.76 | 2.607 | 0.95 |
| tight_arc | lqr | yes | 8.0 | 0.0027 | 0.008 | 0.80 | 2.606 | 0.85 |
| tight_arc | linear_mpc | yes | 8.0 | 0.0034 | 0.009 | 0.89 | 2.598 | 0.83 |
| tight_arc | nonlinear_mpc | yes | 8.1 | 0.0073 | 0.009 | 0.76 | 2.577 | 0.85 |
| tight_arc | feedback_linearization | yes | 8.3 | 0.0345 | 0.040 | 2.14 | 2.412 | 2.10 |
| tight_arc | backstepping | yes | 8.0 | 0.0026 | 0.007 | 0.76 | 2.604 | 0.87 |

## Disturbance (0.25 m, 0.3 rad kick at 8 s on the S-curve)

| Controller | Peak lateral after kick (m) | Recovery (s) | int w^2 dt (10 s) |
|---|---|---|---|
| pid | 0.2812 | 4.5 | 0.605 |
| lqr | 0.2815 | 4.95 | 0.539 |
| linear_mpc | 0.2817 | 5.4 | 0.442 |
| nonlinear_mpc | 0.2719 | 5.1 | 0.525 |
| feedback_linearization | 0.2699 | 3.9 | 0.897 |
| backstepping | 0.2763 | 6.05 | 0.417 |

## Stability envelope (straight path, settle within 20 s)

Initial lateral offsets [-1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0] m x heading offsets [-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150] deg.

| Controller | Converged | Median settle (s) | Not converged (lateral m, heading deg) |
|---|---|---|---|
| pid | 77/77 | 6.85 | - |
| lqr | 71/77 | 6.65 | (-1, 90), (-1, 120), (-1, 150), (1, -150), (1, -120), (1, -90) |
| linear_mpc | 77/77 | 7.1 | - |
| nonlinear_mpc | 77/77 | 6.25 | - |
| feedback_linearization | 77/77 | 4.9 | - |
| backstepping | 77/77 | 7.35 | - |

## Saturation recovery (0.8 m lateral start, |w| <= 0.3 rad/s)

| Controller | Overshoot (m) | Settle (s) | Saturated steps | int w^2 dt |
|---|---|---|---|---|
| pid | 0.000 | 8.0 | 49 | 0.357 |
| lqr | 0.000 | 8.5 | 53 | 0.384 |
| linear_mpc | 0.000 | 8.8 | 47 | 0.327 |
| nonlinear_mpc | 0.000 | 8.45 | 43 | 0.285 |
| feedback_linearization | 0.000 | 6.95 | 72 | 0.455 |
| backstepping | 0.000 | 9.75 | 37 | 0.215 |
| pid_without_anti_windup | 0.035 | 10.3 | 60 | 0.478 |

## Compute time against the 50 ms period

| Controller | Steps | p50 (ms) | p95 (ms) | p99 (ms) | Max (ms) | Misses |
|---|---|---|---|---|---|---|
| pid | 34334 | 0.041 | 0.049 | 0.169 | 17.33 | 0 (0.000%) |
| lqr | 34333 | 0.046 | 0.062 | 0.244 | 15.75 | 0 (0.000%) |
| linear_mpc | 34324 | 1.371 | 2.839 | 6.403 | 30.17 | 0 (0.000%) |
| nonlinear_mpc | 34341 | 3.502 | 8.097 | 13.258 | 70.74 | 2 (0.006%) |
| feedback_linearization | 34376 | 0.021 | 0.024 | 0.071 | 9.19 | 0 (0.000%) |
| backstepping | 34335 | 0.020 | 0.022 | 0.041 | 7.67 | 0 (0.000%) |
