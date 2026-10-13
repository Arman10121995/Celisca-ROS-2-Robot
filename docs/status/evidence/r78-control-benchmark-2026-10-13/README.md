# R7.8: six path-tracking controllers on one plant (2026-10-13)

Status: **implemented, experimental**. This directory holds the model-plant
benchmark. The Gazebo comparison of the same controllers driving Bumperbot
through Nav2 is queued and will be linked here when it has run.

## What was built

`robot_lab_algorithms.tracking_control` (revision `58b8a70`) has six
controllers for one differential-drive (unicycle) plant. Each turns a path
and the robot pose into `(v, w)`:

| Selection (`control:=`) | Class | Law |
|---|---|---|
| `pid_controller` | `PidTracker` | PID on the heading error, which includes `atan(k_lat e_lat)`. Curvature feedforward, filtered derivative, conditional-integration anti-windup |
| `lqr_controller` | `LqrTracker` | Discrete LQR on `[e_lat, e_theta]` linearised at the reference speed. Gains come from the DARE, cached per 0.01 m/s |
| `mpc_controller` | `LinearMpcTracker` | Linear time-varying MPC over a 2 s path preview with hard bounds `\|w\| <= w_max`. The box-constrained QP is solved exactly by BVLS |
| `nonlinear_mpc` | `NonlinearMpcTracker` | Unicycle NMPC over 1.5 s with bounded `(v, w)`. Single shooting with an adjoint gradient; L-BFGS-B, at most 30 iterations, warm-started |
| `feedback_linearization` | `FeedbackLinearizationTracker` | Static I/O linearisation of a point 0.2 m ahead of the axle, with a linear outer loop |
| `backstepping_controller` | `BacksteppingTracker` | The Jiang-Nijmeijer law; `V = (e_x^2+e_y^2)/2 + e_th^2/(2 gamma)` gives `V' = -c_x e_x^2 - (c_th/gamma) e_th^2` |

**Path reference.** The path is resampled every 0.05 m and smoothed over
0.25 m. The reference speed is the smallest of three limits:

- 0.3 m/s;
- `sqrt(a_lat/|kappa|)`, with `a_lat` = 0.25 m/s²;
- `sqrt(2 a_dec (L - s))`, with `a_dec` = 0.25 m/s².

**Running on a robot.** The same server that runs the robot_lab FollowPath
laws, `nav2_local_planner`, runs these controllers, selected with
`planner:=pid|lqr|...`. In nav mode a control selection that serves
FollowPath replaces the local planner's. The server adds one supervisor
that is the same for every controller:

- it rotates in place while the heading error exceeds 45°;
- it stops before collisions seen in the scan;
- it applies the Nav2-style progress check;
- a pose watchdog stops the robot when its TF pose is older than 0.5 s, and
  aborts the goal after 5 s.

**Earlier versions.** Before this revision the same ids started
single-axis demonstration nodes. They published on `/cmd_vel_lqr` and
similar side topics. `pid_controller` started a pure-pursuit node
listening to `/a_star/path`.

## Model-plant benchmark

`scripts/r7_8_control_benchmark.py` writes `results.json` and
[`report.md`](report.md).

**Plant.** A unicycle with:

- a first-order actuator lag of 0.1 s;
- |a| <= 0.5 m/s² and |alpha| <= 2 rad/s²;
- v in [0, 0.3] m/s and |w| <= 1 rad/s.

The control period is 50 ms. The plant integrates the motion as exact arcs.

**Limits of this benchmark.**

- It measures the control laws only, not a simulator or a robot.
- Gains were chosen on this plant; none were identified on Bumperbot.
- The compute times come from a loaded host (load average 11-16 on 12
  CPUs, with a Gazebo mission and a stray PX4 SITL running). They show
  behaviour under load, not on an idle machine.

### Results

**Tracking, starting on the path.**

- The smooth S-curve is tracked to under 1 mm RMS by every law except
  feedback linearisation, which reaches 3.2 mm. Its point offset cuts
  inside the curve.
- On the rectilinear path with 90° corners:
  - NMPC keeps 4 mm RMS (15 mm worst), because it previews the corner;
  - the four reactive or one-step laws keep 32-35 mm RMS (86-98 mm worst);
  - feedback linearisation keeps 22 mm RMS with the least turn effort
    (∫w² dt 3.7 against 5.7-6.1), at the cost of the largest heading error.
- On the 0.5 m-radius arc all except feedback linearisation stay within
  7-9 mm.

**Disturbance.** The robot is kicked 0.25 m and 0.3 rad at 8 s on the
S-curve.

- All recover; the time to settle within 3 cm and 0.05 rad for 1 s ranges
  from 3.9 s (feedback linearisation) to 6.05 s (backstepping).
- Backstepping uses the least effort (0.42) and feedback linearisation the
  most (0.90).

**Stability envelope.** The robot starts on a straight path with lateral
offsets of ±1 m and heading offsets of ±150°, 77 cells in all. There is no
supervisor in this test; the law acts alone.

- Five laws converge from every cell.
- LQR misses 6 cells: 1 m off the path and turned 90° or more.
- Its gains at 0.3 m/s are K = [3.02, 1.82]. The commanded turn rate
  vanishes at `e_theta = K1 e_lat / K2`, which is 1.65 rad (94.8°) for a
  1 m offset. There the speed `v_r cos(e_theta)` is clamped to zero.
- The robot therefore stops facing the path at about 95° and never moves.
  This is the linearisation's false equilibrium. The server's 45°
  rotate-in-place supervisor exists for this case.

**Saturation recovery.** The robot starts 0.8 m off the path with the turn
rate capped at 0.3 rad/s.

- No anti-windup law overshoots.
- PID without anti-windup integrates through 60 saturated steps. It
  overshoots by 3.5 cm and settles in 10.3 s, against 0 cm and 8.0 s with
  conditional integration.
- The unit test `test_pid_anti_windup_bounds_the_integral_and_reduces_overshoot`
  uses `ki` 0.6 and pins the same effect.

**Compute.** Times are per 50 ms step, about 34,000 steps per controller.

- PID, LQR, feedback linearisation and backstepping: p99 at most 0.25 ms.
- Linear MPC: p99 6.4 ms.
- NMPC: p50 3.5 ms, p99 13.3 ms, maximum 70.7 ms, with **2 deadline
  misses** (0.006%) at a load average of 16.
- An earlier run with multi-threaded BLAS gave a p99 of 72.6 ms and missed
  2% of deadlines. The server and the benchmark now pin BLAS to one thread.

### Watchdog and server check

`scripts/tracking_server_check.py` (a ROS graph, so not part of the unit
suite): a kinematic fake robot (the same `ActuatorPlant`) publishes
`map -> base_footprint` with wall time, and the server runs with
`planner:=lqr` and with `planner:=nonlinear_mpc`.

- **L-shaped path** (2 m, then 1.5 m): SUCCEEDED in 12.1 s; final error
  0.08 m (LQR) and 0.09 m (NMPC), within the 0.10 m tolerance.
- **TF stopped 3 s into a second path:** every command after 0.8 s was
  zero, and the goal ABORTED 5.6 s after the TF loss ("robot pose stale").

## Reproduce

```bash
python3 scripts/r7_8_control_benchmark.py --out <dir>
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/robot_lab_algorithms/test/test_tracking_control.py
```
