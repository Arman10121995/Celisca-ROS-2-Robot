# Experimental bicycle tracking for front/rear-steer cars

Implemented October 13, 2026. This adds a separate car controller family;
it does not replace the existing car defaults or Claude's measured differential
controllers. Simulator mission comparisons remain pending.

Select **Ackermann car**, **Rear-steer car** or **Anti-Ackermann car**, a
compatible world/backend, and **Navigation**. Keep the **Hybrid A\*** global
planner, then choose an experimental bicycle controller in **Motion controller**:

- **Bicycle Pure Pursuit:** lookahead geometry, with source steering/speed limits.
- **Bicycle Stanley:** heading/cross-track correction with curvature feedforward.
  Rear steering uses a fixed-axle feedback adaptation, rather than applying the
  front-only steered-axle feedback unchanged.
- **Bicycle Steering-Rate MPC:** linear time-varying lateral/heading/steering
  prediction with angle/rate constraints and an analytic quadratic gradient.
  The ten-step horizon uses 150 ms prediction blocks; it replans/applies at
  20 Hz. A 35 ms solver budget or failed solve selects bounded pursuit for
  that cycle and reports the reason.

The command box autofills `control:=bicycle_pure_pursuit`,
`control:=bicycle_stanley` or `control:=bicycle_mpc`. The car-only dispatcher
rewires Nav2 FollowPath to `/robot_lab/bicycle_follow_path` and configures
Hybrid A\* with **DUBIN** for forward-only paths. Other controllers retain
their existing search defaults. Explicit incompatible lattice/global planner
choices fail clearly. Reverse cusps are rejected; reversing controllers and
parking manoeuvres remain further work.

The server uses the actual robot profile's wheelbase, virtual steering/rate,
speed/acceleration limits, and navigation overlay's fixed-axle frame and
full rectangle. Front steering has a fixed rear reference; rear steering a
fixed front reference. The existing drive bridge supplies the proper physical
rear-steer sign. Anti-Ackermann tire scrub is outside the ideal bicycle model.
The model is `x'=v cos(yaw), y'=v sin(yaw), yaw'=v tan(delta)/wheelbase`.

Command slew uses elapsed **simulation** time, while solver/watchdog budgets
use wall time. A slow simulator does not multiply the allowed steering rate.

Every command is curvature/rate bounded. The supervisor checks fresh TF,
measured `/odom`, and the local collision grid. It sweeps the entire footprint
along both proposed and measured stopping arcs, including corners, unknown
cells and map boundaries. Stop/cancel/preemption/pose loss remain implemented;
a preempted worker cannot issue commands to a new goal. A car never receives
an in-place rotation. An XY endpoint with wrong heading aborts and asks for a
feasible approach. This geometry check is not a measured contact guarantee.

Drive/Space input while tracking pauses this controller and requests cancellation
of local NavigateToPose goals accepted before the stop. This is the lab's
single-robot isolated ROS graph. A new RViz `/goal_pose` or
`/robot_lab/bicycle/resume` releases the pause; Resume requests no movement.
`/robot_lab/bicycle/stop` provides the same explicit pause/stop service.
Parent cancellation is asynchronous; its physical completion needs validation.

`/robot_lab/bicycle/status` reports the controller, actual observed velocity,
requested twist, virtual steering estimate, solver/fallback and compute time.
Virtual steering is inferred from odometric curvature above 0.03 m/s and
retains the prior command below that speed; it is not claimed a physical
steering-joint measurement. Disturbed/slipping tires require later validation.

The equations and examples were reviewed against the upstream
[PythonRobotics Stanley implementation](https://atsushisakai.github.io/PythonRobotics/modules/6_path_tracking/stanley_control/stanley_control.html)
and [bicycle MPC derivation](https://atsushisakai.github.io/PythonRobotics/modules/6_path_tracking/model_predictive_speed_and_steering_control/model_predictive_speed_and_steering_control.html).
This implementation reuses the lab's shared path reference and SciPy's bounded
solvers, with source profiles and ROS safeguards; it does not bundle another
simulator or change wheel geometry.

For later physical acceptance, retain exact robot/map/backend/revision,
source geometry and selected algorithm. Record real body and steering feedback,
clear/obstacle curved approaches, cancellation/Stop/loss, infeasible/reverse
path rejection, contact/clearance, solver deadlines and end errors. Numerical
constraint tests and installation checks are separate from those missions.
See [current status](../status/CURRENT_STATUS.md) and [the ledger](../status/platform-status.yaml).

## Run

From a sourced workspace, the Ackermann car on the obstacle map with the
bicycle Pure Pursuit tracker (Hybrid A\* stays the car's default planner):

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=ackermann_car simulator:=mujoco mode:=nav map_name:=nav_obstacle \
  control:=bicycle_pure_pursuit
```

`control:=bicycle_stanley` or `control:=bicycle_mpc` select the other two.
