# State Estimation: a diagonal numerical filter

The class is named `EKF3DEstimator`, but its present implementation is a
simplified constant-velocity predictor with independent scalar Kalman-style
measurement updates over `[px, py, pz, vx, vy, vz]`. It stores only six diagonal
covariance values, without full covariance propagation or nonlinear Jacobians.
Do not describe this as a validated general-purpose 3D EKF.

This tutorial calls its numerical methods directly. The current executable
starts an idle ROS node; it does not yet connect these methods to sensor inputs
or publish an estimated state.

## Run

Follow the workspace-root setup in the [tutorial index](index.md). This example
uses noiseless position and velocity measurements from a consistent straight
trajectory. It needs no ROS initialization and writes no results to disk.

```bash
python3 - <<'PY'
from robot_lab_algorithms.state_estimation import EKF3DEstimator

estimator = EKF3DEstimator(process_noise=0.1, measurement_noise=0.2)
dt = 0.1
for k in range(20):
    t = (k + 1) * dt
    estimator.predict(dt)
    estimator.update([0.5 * t, 0.0, 0.0, 0.5, 0.0, 0.0])

state = estimator.state()
print(f'Estimated x: {state[0]:.4f} m; reference x: {0.5 * t:.4f} m')
print(f'Estimated vx: {state[3]:.4f} m/s')
PY
```

Expected output, checked against the current source:

```text
Estimated x: 0.9998 m; reference x: 1.0000 m
Estimated vx: 0.5000 m/s
```

Agreement with this one noiseless trajectory does not validate noise modeling,
covariance consistency, nonlinear motion or numerical robustness. The package's
`PoseGraphEstimator` is also an approximation: it exponentially averages
incoming increments; it does not optimize a pose graph.

## Path to a controlled comparison

1. State the process model, observation model, units and timestep convention;
   implement full matrix/covariance behavior for algorithms claiming it.
2. Add analytical/reference tests, covariance symmetry/positive-semidefiniteness
   checks where applicable, and tests for missing observations, outliers and
   irregular sampling. Correct or rename simplified implementations.
3. Generate or record shared noisy trajectories with explicit seeds and ground
   truth; compare genuinely distinct methods under the same observation set
   and tuning protocol.
4. Measure position/velocity error, convergence, latency and memory, plus
   innovation/estimation consistency when meaningful. Add real ROS adapters
   and input-to-output integration tests before simulator comparison claims.

Follow the [roadmap](../../ROADMAP.md) and [agent handoff](../AGENT_HANDOFF.md)
for the broader implementation work. This is not a completed EKF benchmark.
