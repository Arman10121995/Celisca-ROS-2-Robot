# Localization: manually driven dead reckoning

This example integrates supplied body linear/angular velocities. It does not
localize against a map or consume a live odometry topic. The current
`DeadReckoning` class inherits `rclpy.node.Node`, but its integration method is
called manually and its executable does not yet connect that method to sensor
subscriptions and pose publications.

## Run

Unlike the other numerical examples, this one requires ROS 2 `rclpy` and a
properly initialized ROS context. Constructing `DeadReckoning` before
`rclpy.init()` fails when real ROS dependencies are present. Do not hide that
failure by forcing the module's optional-import fallback.

On a host with the documented Humble installation, source ROS, then follow the
workspace-root Python-path setup in the [tutorial index](index.md):

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/robot_lab_algorithms${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
```

Use the Python interpreter matching that ROS installation (`/usr/bin/python3`
for the Ubuntu 22.04/Humble baseline). This briefly creates a ROS node; do not
run it in a domain shared with a real robot or another active experiment. Use
an isolated, unused `ROS_DOMAIN_ID` for your environment. There are no velocity
publishers, simulator launches or hardware commands in this snippet.

```bash
/usr/bin/python3 - <<'PY'
import rclpy
from robot_lab_algorithms import localization

if localization.rclpy is not rclpy:
    raise RuntimeError('Real rclpy is required; do not use the fallback node')

rclpy.init(args=[])
dr = None
try:
    dr = localization.DeadReckoning()
    # Move straight at 0.5 m/s for two seconds, then rotate in place.
    for _ in range(20):
        dr.integrate(0.5, 0.0, 0.1)
    for _ in range(20):
        dr.integrate(0.0, 0.3, 0.1)
    x, y, theta = dr.integrate(0.0, 0.0, 0.0)
    print(f'Pose: x={x:.2f}, y={y:.2f}, theta={theta:.2f}')
finally:
    if dr is not None:
        dr.destroy_node()
    rclpy.shutdown()
PY
```

The analytical expected final pose is below; ROS can also print a startup log.
This ROS-node example was not executed during the documentation-only update.

```text
Pose: x=1.00, y=0.00, theta=0.60
```

This ideal-input exercise does not measure localization accuracy. Integrating
commands assumes the robot executes them perfectly: slip, actuator limits and
disturbances break that assumption. The method does not estimate covariance,
perform map correction or normalize the heading angle.

## Path to a controlled comparison

1. Separate the numerical integrator from its ROS adapter so math tests do not
   depend on ROS initialization; keep explicit ROS lifecycle tests separately.
2. Define the actual velocity source (command, wheel odometry or measurement),
   timestamps, frame convention and output pose/TF contract. Avoid multiple
   nodes publishing the same TF transform.
3. Use trajectories with turns, slip, bias and dropouts, synchronized independent
   ground truth, and recorded seeds for injected errors. Compare methods only
   when they receive equivalent allowable information.
4. Evaluate absolute/relative trajectory error, drift per distance, latency,
   recovery and uncertainty consistency where covariance is available. Do not
   treat perfect-input integration as an AMCL/SLAM comparison.

See the [roadmap](../../ROADMAP.md) and [agent handoff](../AGENT_HANDOFF.md) for
the implementation and qualification plan.
