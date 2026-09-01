# Sensor Fusion Tutorial

Complementary IMU filter: blend gyro integration with accelerometer tilt.

## Run

```python
from robot_lab_algorithms.sensor_fusion import ComplementaryImu

imu = ComplementaryImu(alpha=0.98)

# Simulate: level flight with slight gyro rotation
for _ in range(50):
    pitch, roll = imu.update(
        ax=0.0, ay=0.0, az=9.81,  # accelerometer (level)
        gx=0.01, gy=0.0,           # gyro (slight pitch rate)
        dt=0.01,
    )

print(f"Pitch: {pitch:.4f} rad")
print(f"Roll:  {roll:.4f} rad")
```

## Expected Output

```
Pitch: 0.0049 rad
Roll:  0.0000 rad
```
