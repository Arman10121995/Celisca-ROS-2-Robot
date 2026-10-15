# Sensor Fusion Comparison: attitude and planar pose

Measured October 13, 2026, at revision `303e01a`. The evidence and raw
results are in
[r75-fusion-benchmark-2026-10-13](../status/evidence/r75-fusion-benchmark-2026-10-13/README.md).

**Category:** sensor_fusion
**Type:** measured comparison on modelled sensors (offline and over ROS)

## What is compared

**Attitude from an IMU.** Five attitude filters estimate roll and pitch
from gyro and accelerometer readings:

| Selection (`sensor_fusion:=`) | Method | Idea |
|---|---|---|
| `complementary_imu` | Complementary filter | Integrate the gyro; pull roll and pitch toward the accelerometer tilt with time constant tau |
| `mahony_filter` | Mahony (2008) | Feed the cross product of measured and predicted gravity back into the gyro (proportional and integral) |
| `madgwick_filter` | Madgwick (2010) | One normalised gradient-descent step toward the measured gravity per sample |
| `attitude_eskf` | Error-state Kalman filter | Estimate attitude and gyro bias; weigh the accelerometer by how much the platform accelerates |
| `imu_complementary_filter` | imu_tools Madgwick (upstream C++) | The ROS package's filter, for reference |

**Planar pose.** Three pose filters estimate (x, y, heading) from wheel
odometry, a gyro and GNSS:

| Selection | Method |
|---|---|
| `wheel_imu_fusion` | EKF: gyro heading with bias, wheel speed, zero angular rate update at standstill |
| `gps_odom_fusion` | The same EKF plus GNSS position |
| `wheel_imu_gnss_ukf` | Unscented Kalman filter on the same model |

All of them read standard messages (`sensor_msgs/Imu`, `nav_msgs/Odometry`,
`sensor_msgs/NavSatFix`) and publish beside the lab's own estimator. Select
one in the GUI's **Sensor fusion** slot, or with `sensor_fusion:=`.

## Run

```bash
python3 scripts/r7_5_fusion_benchmark.py --out <dir>      # offline, about 1 minute
scripts/r7_5_attitude_replay.sh <dir>/ros_replay.json     # the five attitude nodes over ROS, 2 minutes
```

The benchmark generates sensors with known truth:

- **Attitude:** a body that rolls, pitches and turns, with constant gyro
  bias, sensor noise, and horizontal accelerations of 0.5 m/s². These rise
  to 2 m/s² for 20 s.
- **Pose:** a 0.5 m/s drive with stops, wheel scale errors, two wheel-slip
  episodes and 1 Hz GNSS.

Each attitude filter's gain is chosen on a separate tuning seed. The
results average three other seeds.

## Results

**Attitude** (tilt error after 10 s):

| Filter (tuned gain) | Tilt RMS | Under 2 m/s² (RMS / max) | Settle from 20° off | Yaw drift |
|---|---|---|---|---|
| ESKF (16 m/s²) | **0.46°** | **0.53° / 0.98°** | **4.3 s** | -24°/min |
| Mahony (kp 0.25) | 1.49° | 2.46° / 4.75° | 15.2 s | **-0.6°/min** |
| Complementary (tau 4 s) | 1.90° | 2.69° / 6.18° | 11.4 s | 40°/min |
| Madgwick (beta 0.02) | 2.03° | 2.96° / 6.82° | 15.7 s | 39°/min |

Over ROS, with each node at its default gain:

- Mahony 1.33°;
- ESKF 1.41° (default 2.0 m/s²);
- complementary 1.90°;
- Madgwick 1.95°;
- upstream imu_tools Madgwick 3.73° (gain 0.1).

**Pose** over 300 s:

| Method | Position RMSE | Final error | Heading RMSE |
|---|---|---|---|
| Wheel dead reckoning | 17.8 m | 40.3 m | 50° |
| EKF (wheels + gyro) | 0.42 m | 0.93 m | 1.6° |
| EKF + GNSS | **0.20 m** | 0.17 m | 1.8° |
| UKF + GNSS | **0.20 m** | 0.17 m | 1.8° |

## What the numbers say

- **Estimating the gyro bias is what matters.** The ESKF learns the bias,
  so it can rely on the gyro and hardly lean when the platform accelerates.
  The fixed-gain filters must trade noise against acceleration error
  through one gain.
- **Gains decide the ranking among the fixed-gain filters.** Upstream
  Madgwick at its default gain (0.1) scores 3.73°. The lab's Madgwick
  scored 3.72° at the same gain. Implementations agree; tuning differs.
- **No filter can hold yaw without a heading reference.** Mahony's
  integral term happened to pick up the z bias while the body tilted. The
  ESKF learned it and then lost it under acceleration.
- **For a wheeled robot, heading should come from the gyro, not the
  wheels.** Wheel yaw rate carries track-width error and slip, which cost
  dead reckoning 50° of heading. The gyro bias is learned whenever the
  robot stands still.
- **The UKF matches the EKF here.** The unicycle model is mildly nonlinear
  at these rates, and the UKF costs three times the compute.

## On Gazebo's IMU

Replayed over recorded Gazebo drives
([evidence](../status/evidence/r75-fusion-benchmark-2026-10-13/README.md)):

- **Bumperbot:** the wheel+gyro EKF holds 1.3 cm over 25 m, where wheel
  dead reckoning reaches 9.8 cm. Every attitude filter stays within 0.3°
  RMS.
- **Walking Go2:** the trunk rolls and pitches 2.9° RMS. The
  complementary filter tracks it best (0.25° RMS) and the ESKF worst
  (0.78°). On real gait accelerations, the modelled benchmark's ranking
  turns around.

## Limits

- The ranked tables above use modelled sensors: constant biases, white
  noise, no vibration and no magnetometer. The Gazebo replay is one drive
  per robot. The MuJoCo and Isaac bridges publish a perfect orientation and
  a constant accelerometer, so they cannot test attitude filters; Gazebo's
  IMU sensor can.
- The pose filters need wheel odometry: Gazebo publishes it, and the
  other backends do with `odometry_source:=wheels`.

## Related tutorials

- [Tutorial index](README.md)
- [Numerical learning notes](sensor_fusion.md)
- [Measured-run and failure workflow](../WORKFLOW.md)

*Part of R9.2: Seven Real Comparison Tutorials. See [ROADMAP.md](../../ROADMAP.md).*
