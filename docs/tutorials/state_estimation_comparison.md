# State Estimation Comparison: six filters on noisy position fixes

Measured October 13, 2026, at revision `65ae417`. The evidence and raw
results are in
[r74-state-estimation-benchmark-2026-10-13](../status/evidence/r74-state-estimation-benchmark-2026-10-13/README.md).

**Category:** state_estimation
**Type:** measured comparison on a modelled trajectory and sensor

## What is compared

A sensor reports where the robot is, with noise: AMCL, a GNSS or UWB fix.
Each estimator turns those noisy positions into a clean position, plus a
heading, a speed and a turn rate that the sensor never measures.

| Selection (`state_estimation:=`) | Method | Idea |
|---|---|---|
| `linear_kalman_filter` | Linear Kalman filter | Assume the robot keeps its velocity; the model is linear, so the Kalman filter is exact |
| `ekf_3d_estimator` | Extended Kalman filter | Assume constant speed and turn rate (CTRV); linearise that curved motion at each step |
| `ukf_estimator` | Unscented Kalman filter | The same model; push sigma points through it instead of linearising |
| `particle_filter` | Particle filter | The same model; 500 sampled hypotheses, weighted by how well each explains the fix |
| `motion_model_estimator` | Interacting multiple model | A straight-line and a turning filter run side by side; their estimates mix by how likely each model is |
| `pose_graph_estimator` | Sliding-window smoother | Least squares over the last 2 s of fixes, with smooth-motion constraints between them |

The equations, parameters and sources are in
`src/robot_lab_algorithms/config/r7_method_subrecords.yaml` (state
estimation) and the module `robot_lab_algorithms.tracking_filters`.
`robot_localization`'s EKF (`ekf_localization_node`) is a separate
selection that fuses odometry and IMU; it is not part of this run.

## Run

Benchmark (under a minute; no simulator):

```bash
python3 scripts/r7_4_state_estimation_benchmark.py --out <dir>
```

On a robot, the estimators filter the AMCL pose:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav simulator:=gazebo \
  robot_model:=bumperbot map_name:=nav_maze state_estimation:=motion_model_estimator
ros2 topic echo /odometry/imm
```

To compare on matched input instead, set the node parameter
`inject_noise_std` (for example 0.3). The node then reads the truth
odometry and adds seeded noise itself. The output topics are
`/odometry/linear_kf`, `ctrv_ekf`, `ukf`, `particle_filter`, `imm` and
`window_smoother`. In the GUI, pick the method in the **State estimation**
slot.

## The benchmark

- **Truth:** a 300 s drive that repeats every 100 s: straight runs, left and
  right turns (0.2-0.6 rad/s), speeds from 0.15 to 0.8 m/s and a stop.
- **Sensor:** a position fix 10 times a second, with 0.3 m noise per axis.
- **Fairness:** each estimator gets one tuned gain (its process noise),
  chosen on a separate seed. Scores are means over three other seeds.

## Results

| Estimator | Position error (RMSE) | Speed error | Heading error at speed | Turn-rate error | Compute |
|---|---|---|---|---|---|
| Raw fixes | 0.425 m | - | - | - | - |
| Linear Kalman | 0.167 m | 0.112 m/s | **16.7°** | 0.261 rad/s | **0.06 ms** |
| Extended Kalman (CTRV) | **0.164 m** | **0.111 m/s** | 17.3° | 0.234 rad/s | 0.17 ms |
| Unscented Kalman (CTRV) | 0.169 m | 0.122 m/s | 23.1° | 0.271 rad/s | 0.23 ms |
| Particle filter | 0.191 m | 0.128 m/s | 24.3° | 0.335 rad/s | 0.22 ms |
| Interacting multiple model | 0.166 m | 0.111 m/s | 19.6° | **0.213 rad/s** | 0.45 ms |
| Sliding-window smoother | 0.187 m | 0.136 m/s | 18.5° | 0.261 rad/s | 0.22 ms |

Heading error is measured while the robot moves faster than 0.3 m/s.

## What the numbers say

- **Filtering pays.** Every estimator more than halves the position error.
- **The simplest model is nearly as good.** The linear constant-velocity
  filter is 3 mm behind the best and three times cheaper. Tuned process
  noise lets it follow these gentle turns.
- **Multiple models help where models disagree.** The IMM estimates the turn
  rate best, because it can switch between "straight" and "turning".
- **Heading needs another sensor.** From positions alone, heading is only
  visible through motion; with 0.3 m noise every 0.1 s it is off by
  16-24°. Fuse a gyro or wheel odometry for heading (see
  [sensor fusion](sensor_fusion_comparison.md)).
- **Heavier machinery does not help a nearly linear problem.** The UKF and
  the particle filter are no better than the EKF here. The particle
  filter's sampling noise costs 2-3 cm.
- **Particle filters need care.** Before the fixes in this release, far
  fixes underflowed every particle weight to the same floor and the filter
  lost the robot (7.76 m RMSE on one seed). Log-domain weights and
  re-seeding particles around unexplained fixes cured it.

## Limits

- The noise is white and Gaussian, with no outliers, delays or multipath.
  Real AMCL fixes are correlated over time.
- The motion is modelled, not recorded from a simulator.
- A run on simulator AMCL output is the next step.

## Related tutorials

- [Tutorial index](README.md)
- [Numerical learning notes](state_estimation.md)
- [Sensor fusion comparison](sensor_fusion_comparison.md)
- [Measured-run and failure workflow](../WORKFLOW.md)

*Part of R9.2: Seven Real Comparison Tutorials. See [ROADMAP.md](../../ROADMAP.md).*
