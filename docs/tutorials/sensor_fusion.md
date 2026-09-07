# Sensor Fusion: complementary-filter numerical response

This example exercises `ComplementaryImu.update()` with manually supplied
numbers. It starts neither a simulator nor a ROS sensor-fusion node. The
current executable wrapper is an idle ROS node, not a complete IMU subscriber
and filtered-orientation publisher.

The implementation blends a gyro-integrated angle with accelerometer-derived
tilt. It estimates two angles only: there is no yaw, quaternion state,
gyro-bias estimate or covariance.

## Run

Follow the workspace-root setup in the [tutorial index](index.md). Use synthetic
level acceleration and a small constant value in the API's `gx` input:

```bash
python3 - <<'PY'
from robot_lab_algorithms.sensor_fusion import ComplementaryImu

imu = ComplementaryImu(alpha=0.98)
for _ in range(50):
    pitch, roll = imu.update(
        ax=0.0, ay=0.0, az=9.81,
        gx=0.01, gy=0.0,
        dt=0.01,
    )
print(f'Implementation pitch: {pitch:.4f} rad')
print(f'Implementation roll:  {roll:.4f} rad')
PY
```

Expected output, checked against the current source:

```text
Implementation pitch: 0.0031 rad
Implementation roll:  0.0000 rad
```

The names above deliberately describe the implementation. Its `gx` input
updates the variable named `pitch`, while `gy` updates `roll`. Do not assume
that this matches ROS/body-axis conventions or connect raw IMU axes without a
validated frame/sign mapping. The previous tutorial's `0.0049` result was not
the response after the stated 50 updates.

This input illustrates numerical response to conflicting tilt/rate evidence;
it is not a physically validated flight trajectory. A constant `alpha` also
changes its effective time response when the sample rate changes. Accelerometer
tilt is not a reliable gravity measurement during strong linear acceleration.

## Path to a controlled comparison

1. Specify frame orientation, signs, gravity convention, units and timestamps.
   Add single-axis rotation tests against a known orientation trajectory before
   wiring the real ROS IMU adapter.
2. Decide whether the task is tilt estimation, full attitude estimation or
   position/velocity fusion; only compare algorithms solving the same task
   with the same available sensors.
3. Use replayable datasets with independent orientation truth, bias, noise,
   acceleration, rate changes and dropouts. Record every injected-noise seed
   and parameter set.
4. Compare angle/orientation error, drift, transient response, latency and
   resource use. Require valid output messages, timestamps and frame IDs in
   integration tests; an idle node is not evidence of sensor fusion.

See the [roadmap](../../ROADMAP.md) and [agent handoff](../AGENT_HANDOFF.md) for
the implementation and qualification plan. No comparative performance result
is claimed by this example.
