# Localization Tutorial

Dead reckoning: integrate velocity commands into a pose estimate.

## Run

```python
from robot_lab_algorithms.localization import DeadReckoning

dr = DeadReckoning()

# Move forward at 0.5 m/s for 2 seconds
for _ in range(20):
    dr.integrate(0.5, 0.0, 0.1)

# Rotate at 0.3 rad/s for 2 seconds
for _ in range(20):
    dr.integrate(0.0, 0.3, 0.1)

x, y, theta = dr.integrate(0.0, 0.0, 0.0)
print(f"Pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
```

## Expected Output

```
Pose: x=1.00, y=0.00, theta=0.60
```
