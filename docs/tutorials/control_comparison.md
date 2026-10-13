# Control Comparison: six path-tracking controllers on one plant

Measured October 13, 2026, at revision `58b8a70`. The evidence, raw results
and method notes are in
[r78-control-benchmark-2026-10-13](../status/evidence/r78-control-benchmark-2026-10-13/README.md).

**Category:** control
**Type:** measured comparison (model plant and Gazebo)

## What is compared

All six controllers drive the same plant: a differential-drive unicycle
`x' = v cos θ, y' = v sin θ, θ' = w`, with Bumperbot's limits.

- **Speed limits:** 0.3 m/s and 1 rad/s.
- **Actuator model:** a 0.1 s first-order lag, 0.5 m/s² and 2 rad/s²
  acceleration limits, then saturation.

Each controller reads the path and the robot pose and returns `(v, w)`
every 50 ms.

| Selection (`control:=`) | Method | Idea |
|---|---|---|
| `pid_controller` | PID | Steer toward `θ_r − atan(k e_lat)`; curvature feedforward; anti-windup by conditional integration |
| `lqr_controller` | LQR | Optimal linear feedback on lateral and heading error, linearised at the current speed |
| `mpc_controller` | Constrained linear MPC | The same linear model, optimised over a 2 s preview of the path with hard turn-rate bounds |
| `nonlinear_mpc` | Nonlinear MPC | The full unicycle model over 1.5 s, bounded speed and turn rate |
| `feedback_linearization` | Feedback linearisation | Makes a point 0.2 m ahead of the axle a linear system, then steers it with a P loop |
| `backstepping_controller` | Backstepping | Lyapunov-designed law (Jiang and Nijmeijer 1997) with a guaranteed decreasing error energy |

The equations, parameters and sources are in
`src/robot_lab_algorithms/config/r7_method_subrecords.yaml` (control) and
the module `robot_lab_algorithms.tracking_control`.

## Run

Model plant (about 5 minutes on the Jetson):

```bash
python3 scripts/r7_8_control_benchmark.py --out <dir>
```

On a robot: in nav mode, a control selection replaces the local planner
as Nav2's FollowPath server:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav simulator:=gazebo \
  robot_model:=bumperbot map_name:=nav_maze control:=lqr_controller
```

In the GUI, choose the controller in the **Motion controller** slot of
nav mode. These controllers assume a robot that can turn in place, so the
GUI does not offer them for car-like robots.

## Results on the model plant

**Tracking** (RMS / worst lateral error from the path):

| Controller | S-curve | 90° corners | 0.5 m arc |
|---|---|---|---|
| PID | 0.1 mm / 0.3 mm | 34 mm / 98 mm | 2.4 mm / 6.6 mm |
| LQR | 0.2 / 0.3 | 35 / 98 | 2.7 / 7.6 |
| Linear MPC | 0.3 / 0.3 | 33 / 86 | 3.4 / 8.8 |
| Nonlinear MPC | 0.7 / 1.1 | **4 / 15** | 7.3 / 9.4 |
| Feedback linearisation | 3.2 / 5.0 | 22 / 69 | 35 / 40 |
| Backstepping | 0.3 / 0.4 | 33 / 88 | 2.6 / 7.3 |

**Disturbance, envelope, saturation and compute:**

| Controller | Recovery after a 0.25 m + 0.3 rad kick | Envelope (77 starts) | Overshoot from 0.8 m with w ≤ 0.3 rad/s | p99 compute |
|---|---|---|---|---|
| PID | 4.5 s | 77 | 0 | 0.17 ms |
| PID without anti-windup | - | - | **3.5 cm** | - |
| LQR | 4.95 s | **71** | 0 | 0.24 ms |
| Linear MPC | 5.4 s | 77 | 0 | 6.4 ms |
| Nonlinear MPC | 5.1 s | 77 | 0 | 13.3 ms (2 misses of 34,341) |
| Feedback linearisation | 3.9 s | 77 | 0 | 0.07 ms |
| Backstepping | 6.05 s | 77 | 0 | 0.04 ms |

## Results in Gazebo (Bumperbot, nav_maze)

The six controllers drove Bumperbot through Nav2 on the same 15 seeded goals
as the [local planning comparison](local_planning_comparison.md). The global
planner was Smac 2D A* and localization was AMCL. The evidence is in
[gazebo/](../status/evidence/r78-control-benchmark-2026-10-13/gazebo/report.md).

| Controller | Success | Sim time median (s) | Clearance min (m) | Cross-track RMS / p95 (m) |
|---|---|---|---|---|
| Backstepping | 15/15 | 27.06 | 0.283 | **0.053** / 0.100 |
| LQR | 15/15 | 27.08 | 0.310 | 0.056 / 0.108 |
| Linear MPC | 15/15 | 27.16 | 0.327 | 0.057 / 0.113 |
| Feedback linearisation | 15/15 | 26.78 | **0.244** | 0.061 / 0.114 |
| PID | 15/15 | 26.88 | 0.304 | 0.063 / 0.126 |
| Nonlinear MPC | 15/15 | 26.60 | 0.280 | 0.063 / 0.126 |

Cross-track error is measured from Gazebo truth to the latest global plan
(`scripts/path_tracking_error.py`). For comparison, Nav2 Regulated Pure
Pursuit took 38.9 s median with 6.6 cm RMS in the R7.7 runs.

## What the numbers say

- **Preview matters at corners.** Only the nonlinear MPC sees a corner
  coming. It tracks the 90° turns about 8 times closer than the others.
  The linear MPC also previews, but it optimises only the turn rate around
  a linearisation, and the speed follows a fixed profile.
- **On smooth paths every law is near perfect.** With curvature
  feedforward, the remaining error is the actuator lag.
- **Feedback linearisation trades accuracy for gentleness.** It controls a
  point ahead of the axle, so the axle cuts inside curves (35 mm on the
  tight arc). In return it recovers fastest and uses the least turn effort
  at corners.
- **LQR has a false equilibrium far from the path.** About 1 m off the path
  and turned 90° or more, its command vanishes while the robot faces the
  path. The speed `v_r cos θ_err` is zero there, so it never moves. The
  shared 45° rotate-in-place supervisor on the robot prevents this. The
  envelope test leaves the supervisor out on purpose.
- **Anti-windup is visible.** With the turn rate capped, PID without
  anti-windup overshoots by 3.5 cm and settles 2.3 s later.
- **On the robot the differences shrink.** All six reached every goal in
  Gazebo, within about 1% of each other's time. The shared speed profile
  and supervisor set the pace, so the laws differ only in how closely they
  hold the plan. Backstepping and LQR hold it best.
- **NMPC's corner advantage does not carry over to the maze.** The plan is
  a grid path that the reference smooths over 0.25 m, so previewing a
  corner gains little.
- **Compute.** The nonlinear MPC is the only law near the 50 ms budget.
  Under a load average of 16 it missed 2 of 34,341 deadlines.
  Multi-threaded BLAS made it much worse (2% misses), so BLAS is pinned to
  one thread.

## Limits

- The model plant has no wheel slip, delay or sensor noise. The gains were
  chosen on it, not identified on Bumperbot.
- Gazebo: one robot, one map, five seeds, three goals each.
- Compute times come from a loaded host.
- The unicycle plant covers differential drives. Car-like robots need the
  bicycle model with a minimum turning radius. These controllers are gated
  off for them (`not_for_features: car_steering`).

## Related tutorials

- [Tutorial index](README.md)
- [Local planning comparison](local_planning_comparison.md)
- [Numerical learning notes](control.md)
- [Measured-run and failure workflow](../WORKFLOW.md)

*Part of R9.2: Seven Real Comparison Tutorials. See [ROADMAP.md](../../ROADMAP.md).*
