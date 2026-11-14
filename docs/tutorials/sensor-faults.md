# Seeded sensor faults on live topics

`sensor_fault_relay` puts repeatable faults on a robot's live sensor
streams, so a baseline run and a degraded run can share the same task, map
and budgets. Truth topics are never faulted.

## Run

Write a scenario:

```yaml
seed: 7
relays:
- input: /scan_clean          # where the bridge now publishes the clean scan
  output: /scan               # what AMCL, SLAM and Nav2 read
  type: LaserScan             # LaserScan, Imu, Odometry or Image (32FC1 depth or 8-bit colour)
  faults:
  - {kind: noise, std: 0.02, start_s: 20}          # active from 20 s of simulation time
  - {kind: occlusion, fraction: 0.25, start_s: 40, end_s: 60}
  - {kind: dropout, probability: 0.05}
  - {kind: delay, seconds: 0.2}
- input: /imu/out_clean
  output: /imu/out
  type: Imu
  faults:
  - {kind: drift, rate: 0.002, start_s: 30}
```

Start a Gazebo session with it:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py simulator:=gazebo \
  robot_model:=bumperbot map_name:=nav_obstacle sensor_faults:=/path/scenario.yaml
```

For each relay, the launch renames the bridge's ROS topic (`/scan` becomes
`/scan_clean`) and starts the relay on the original name. Every consumer
then sees the faulted data without any change to its configuration.

## Faults

| Kind | Parameters | Effect |
|---|---|---|
| `noise` | `std` | Gaussian noise on every value |
| `bias` | `value` | A constant offset |
| `drift` | `rate` | An offset growing at `rate` per second since the window opened |
| `delay` | `seconds` | The message is released late, with its original stamp (simulation clock) |
| `dropout` | `probability` | The whole message is dropped |
| `occlusion` | `fraction` | A contiguous sector of scan rays (no return), a band of image rows (NaN depth / black), or the first components of a vector |
| `outlier` | `probability`, `scale` | Values multiplied by `scale` |

Every fault accepts `start_s` and `end_s` in seconds of simulation time
since the relay saw its first clock. Each fault has its own generator,
seeded by the scenario seed, the relay index and the fault index. It
advances once per message, so the same input sequence and seeds give
identical output.

The relay writes every message's outcome (dropped, release time, an output
digest and which faults acted) to
`$ROBOT_LAB_RUNTIME_ROOT/sensor_faults/<robot>-<pid>.jsonl`. Comparing two
traces checks that a degraded run repeats.

## Limits

- Gazebo only. MuJoCo, PyBullet and Isaac publish their sensors from their
  own bridges, and the launch refuses `sensor_faults` there.
- Faults apply to the message values only. A physical effect such as glare
  or motion blur is not modelled.
- Recovery-time measurements are validation work (V9). The relay has unit
  tests only so far.
