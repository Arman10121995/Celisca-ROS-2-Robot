# R7.5: sensor fusion filters on modelled sensors (2026-10-13)

Status: **implemented, experimental**. The filters are measured on synthetic
sensors with known truth, offline and over ROS. Simulator runs are pending.

## What was built

`robot_lab_algorithms.fusion_filters` (revision `303e01a`) contains the
filters below. The sensor_fusion node ids now run them.

| Selection (`sensor_fusion:=`) | Filter | Output |
|---|---|---|
| `complementary_imu` | Gyro integration with roll and pitch pulled toward the accelerometer tilt (time constant tau) | `/imu/complementary` |
| `mahony_filter` | Mahony explicit complementary filter (PI on `a_hat x v_hat`) | `/imu/mahony` |
| `madgwick_filter` | Madgwick gradient-descent filter, IMU form | `/imu/madgwick` |
| `attitude_eskf` (new) | Error-state Kalman filter on attitude and gyro bias | `/imu/eskf` |
| `imu_complementary_filter` | The upstream imu_tools Madgwick node (the id predates the name), now wired to `/imu/out` | `/imu/madgwick_upstream` |
| `wheel_imu_fusion` | EKF on `[x, y, theta, v, b_g]`; gyro heading, wheel speed and yaw rate, zero angular rate update at standstill | `/odometry/wheel_imu` |
| `gps_odom_fusion` | The same EKF with GNSS position updates | `/odometry/gps_odom` |
| `wheel_imu_gnss_ukf` | UKF on the same model and measurements | `/odometry/wheel_imu_gnss` |

**What the ids ran before.** The earlier node code had:

- the complementary filter's roll and pitch gyro axes swapped;
- a "Madgwick" filter that applied Mahony's error with a nonstandard step;
- a "UKF" that dead-reckoned and blended GNSS at a fixed weight;
- wheel/GNSS "fusion" by fixed weighting.

Those classes remain only for their old unit tests.

## Attitude benchmark (`scripts/r7_5_fusion_benchmark.py`)

**Sensors and motion.** 100 Hz IMU for 120 s.

- The body rolls ±17°, pitches ±11° and turns.
- Gyro bias is (0.010, -0.008, 0.012) rad/s, with 0.003 rad/s noise.
- Accelerometer noise is 0.05 m/s².
- Horizontal accelerations are 0.5 m/s² throughout and 2 m/s² from 60 to
  80 s.
- Every filter starts 20° (roll) and 17° (pitch) off.

**Tuning.** One gain per filter is chosen on tuning seed 0: the gain with
the lowest tilt RMS after 10 s, disturbance included. The chosen gains are
complementary tau 4 s, Mahony kp 0.25, Madgwick beta 0.02 and ESKF
`linear_accel_std` 16 m/s². The ESKF value is the top of its grid; it
trusts the bias-corrected gyro almost entirely.

**Evaluation.** Mean over seeds 1-3 ([report.md](report.md)):

| Filter | Tilt RMS after 10 s | Settle to 2° | RMS / max under 2 m/s² | Yaw drift |
|---|---|---|---|---|
| ESKF | **0.46°** | **4.3 s** | **0.53° / 0.98°** | -24°/min |
| Mahony | 1.49° | 15.2 s | 2.46° / 4.75° | **-0.6°/min** |
| Complementary | 1.90° | 11.4 s | 2.69° / 6.18° | 40°/min |
| Madgwick | 2.03° | 15.7 s | 2.96° / 6.82° | 39°/min |

- **The ESKF learns the gyro bias,** so under the 2 m/s² phase it keeps
  tilt within 1°. The fixed-gain filters lean with the acceleration.
- **Yaw has no absolute reference** (no magnetometer):
  - Complementary and Madgwick drift with the 0.012 rad/s z bias (41°/min).
  - Mahony's integral term picked up the z bias through the tilting motion
    and nearly stopped the drift.
  - The ESKF's z-bias estimate was right (0.0122) at 50 s, but the 2 m/s²
    phase corrupted it, so its yaw drifted -24°/min.
- **Compute** is 0.04-0.17 ms per update.

## Same stream over ROS (`scripts/r7_5_attitude_replay.sh`)

Seed 1 was replayed at 100 Hz in real time on `/imu/out`. All five
attitude nodes ran at their node defaults
([ros_replay.json](ros_replay.json)). Each node processed all 12,000
messages.

| Node | Default gain | Tilt RMS after 10 s | RMS under 2 m/s² |
|---|---|---|---|
| mahony_filter | kp 0.25 | 1.33° | 2.57° |
| attitude_eskf | linear_accel_std 2.0 | 1.41° | 2.86° |
| complementary_imu | tau 4 | 1.90° | 2.69° |
| madgwick_filter | beta 0.02 | 1.95° | 2.96° |
| imu_filter_madgwick (upstream) | gain 0.1 | 3.73° | 7.52° |

**Why the ESKF node's default is 2.0.** The node default is 2.0 m/s², not
the benchmark's 16. With a bias that drifts, the accelerometer must keep
control of tilt. Its replay score (1.41°) is the same as the offline
tuning-seed score at 2.0 (1.41°).

**The upstream node confirms the implementation.** At its default gain of
0.1, upstream Madgwick scores 3.73°. The robot_lab Madgwick scored 3.72° at
the same beta on the offline tuning seed. The two implementations agree,
and the difference in the table is the gain.

## Planar pose benchmark

**Sensors and motion.** 300 s at 50 Hz.

- The drive is 0.5 m/s with a 10 s stop every minute.
- Wheel odometry has +2% speed and -3% yaw-rate scale errors.
- There are two slip episodes: +0.1 rad/s for 10 s and for 5 s.
- Gyro bias is 0.01 rad/s.
- GNSS is 1 Hz with 0.5 m noise.

| Method | Position RMSE | Final error | Heading RMSE | Gyro bias estimate |
|---|---|---|---|---|
| Wheel dead reckoning | 17.77 m | 40.32 m | 50.2° | - |
| wheel_imu_fusion (EKF) | 0.42 m | 0.93 m | 1.6° | 0.0099 |
| gps_odom_fusion (EKF + GNSS) | **0.20 m** | 0.17 m | 1.8° | 0.0099 |
| wheel_imu_gnss_ukf (UKF + GNSS) | **0.20 m** | 0.17 m | 1.8° | 0.0099 |

- **Heading from the gyro** removes the wheel yaw-rate scale error and slip.
  At standstill the gyro reading is the bias, which the zero angular rate
  update learns.
- **GNSS bounds the position.** On this mildly nonlinear model the UKF
  matches the EKF and costs three times the compute (0.23 ms vs 0.07 ms).

## On simulator data ([live/](live/report.md))

Two Gazebo drives around the 26 m nav_obstacle loop were recorded on
2026-10-13 (`scripts/slam_mapping_check.sh` with
`EXTRA_BAG_TOPICS="/imu/out /robot_lab_controller/odom"`):

- Bumperbot on wheels, with its 100 Hz IMU (gyro and accelerometer noise
  2e-4);
- the Go2 walking, with its 200 Hz trunk IMU.

`scripts/r7_5_live_replay.py` runs the filters over them offline, at their
node defaults, from the true start, and scores them against
`/odom/ground_truth`.

**Attitude** (roll and pitch error after 5 s):

| Filter | Bumperbot: tilt RMS / max | Go2 walking: tilt RMS / max |
|---|---|---|
| Complementary (tau 4 s) | **0.17° / 0.41°** | **0.25° / 1.14°** |
| Mahony (kp 0.25) | 0.24° / 0.48° | 0.32° / 1.21° |
| Madgwick (beta 0.02) | 0.26° / 1.26° | 0.53° / 1.30° |
| ESKF (linear_accel_std 2.0) | 0.20° / 0.69° | 0.78° / 2.76° |

The truth tilt was 0.6° and constant for Bumperbot. For the walking Go2 it
was 2.9° RMS (6.2° max).

**Planar pose, Bumperbot** (25.4 m):

| Filter | Position RMSE | Final | Heading RMSE |
|---|---|---|---|
| Wheel + gyro EKF | **1.3 cm** | 2.3 cm | **0.10°** |
| Wheel + gyro UKF | 4.7 cm | 1.3 cm | 0.10° |
| Wheel dead reckoning | 9.8 cm | 4.3 cm | 1.05° |

- On real IMU data, **the complementary filter is the most accurate on a
  walking trunk**, and the ESKF the least: the reverse of the modelled
  benchmark. The ESKF's accelerometer weighting (linear_accel_std 2.0) was
  chosen on modelled data with slow 2 m/s² pushes; a gait's quick trunk
  accelerations are a different regime, likely the reason. It was not
  retuned here.
- **The gyro still fixes the wheel heading.** On Gazebo's flat floor the
  wheel odometry is good, and the EKF cuts its heading error tenfold and
  its position error from 9.8 to 1.3 cm.
- One drive per robot, one world; gains as shipped.

## Limits

- The sensors are modelled, not simulated: constant biases, white noise and
  no vibration or magnetometer.
- Gains are tuned on the tuning seed of the same scenario.
- The pose filters need a wheel odometry topic: Gazebo publishes
  `/robot_lab_controller/odom`, and the other backends do with
  `odometry_source:=wheels` (wheel_odometry node).

## Reproduce

```bash
python3 scripts/r7_5_fusion_benchmark.py --out <dir>
scripts/r7_5_attitude_replay.sh <dir>/ros_replay.json
python3 scripts/r7_5_live_replay.py --out <dir>/live <recorded run>.tf.bag ...
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/robot_lab_algorithms/test/test_fusion_filters.py
```
