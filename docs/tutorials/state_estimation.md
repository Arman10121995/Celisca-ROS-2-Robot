# State Estimation Tutorial

EKF 3D state estimator: predict and update cycle.

## Run

```python
from robot_lab_algorithms.state_estimation import EKF3DEstimator

ekf = EKF3DEstimator()

# Simulate: object moving along x at 0.5 m/s
for _ in range(20):
    ekf.predict(0.1)
    ekf.update([0.5, 0.0, 0.0, 0.5, 0.0, 0.0])

state = ekf.state()
print(f"Position: ({state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f})")
print(f"Velocity: ({state[3]:.2f}, {state[4]:.2f}, {state[5]:.2f})")
```

## Expected Output

```
Position: (0.50, 0.00, 0.00)
Velocity: (0.50, 0.00, 0.00)
```
