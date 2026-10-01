# Four-wheel steering and mecanum runtime audit (2026-09-30)

The newly registered `four_wheel_steer_car` and `mecanum_car` have xacros,
drive kinematics and source tests, but their advertised mode lists had no live
mission evidence. These probes used `scripts/sim_drive_check.sh` against the
actual simulator odometry, with `/cmd_vel` commands held for 3–5 seconds.

Four-wheel steering initially drove straight but had near-zero yaw for 0.5
rad/s arc commands: the URDF put all four steering hub joints at the chassis
origin. Moving the hubs to `(±0.16, ±0.16)` m restored the wheelbase/track.
The default MuJoCo pattern then measured +0.431/-0.431 rad/s on +0.5/-0.5
rad/s arcs, -0.269 m/s reverse motion, and zero travel on stop. Correcting
the pure-yaw wheel tangent angles and rates improved a 1.0 rad/s spin from
0.226 to 0.911 rad/s, with zero measured translation. The `pivot` pattern's
idle wheel command was also fixed to zero; source tests cover it and the
four distinct hub positions. This is a drive probe on an empty world, not a
SLAM/navigation/obstacle qualification for every pattern or simulator.

The Gazebo four-wheel drive probe did **not** start its drive controller: its
configuration declares `four_wheel_steer_controller` and
`four_wheel_wheel_controller`, but the shared launch tries to spawn
`robot_lab_controller`. The joint-state broadcaster activation also failed;
there was no `/robot_lab_controller/odom`. A dedicated Gazebo controller
launch and wheel/steering odometry are still required before Gazebo mapping
or navigation should be called working.

The mecanum drive initially measured ~0 yaw on 0.5 rad/s turn commands because
its wheel equation opposed the front and rear axles instead of the left and
right sides. Correcting the ideal 45-degree wheel matrix yielded about
+0.197/-0.216 rad/s on ±0.5 rad/s arcs and 0.502 rad/s on a 1.0 rad/s spin
in MuJoCo. A real lateral command (`Twist.linear.y=0.3` m/s for five seconds)
then produced **0.000 m/s lateral truth velocity and 0.000 m displacement**.
Its current collision wheels are solid cylinders with visual roller markings;
it is not yet a physical mecanum base. The `holonomic_base` capability was
removed from its profile. The new input forwarding permits a proper roller
model later but does not make the present model strafe. Gazebo has the same
controller-launch mismatch as the four-wheel-steering profile.

JSON files in this directory are the direct probe results. Source drive tests
and the expanded URDF check pass. Remaining: physical roller-wheel/contact
model, GUI lateral input and steering-pattern selector, dedicated Gazebo
controller path, and per-backend mapping/navigation missions.

## Gazebo controller repair later on 2026-09-30

The new `holonomic_controller.py` starts the joint-group controllers for 4WS
and mecanum, forwards stamped velocity commands, and derives odometry from
measured wheel/steer joints. A first launch failed because the symlink-installed
script lacked executable permission. After that correction, the hardware
loader exposed a second defect: the description used the system plugin class
as its ros2_control hardware class. The macro now selects
`ign_ros2_control/IgnitionSystem` or `gz_ros2_control/GazeboSimSystem` to match
the selected Gazebo path.

`4ws-gazebo-controller.json` records the resulting quick forward/stop check:
0.4 m/s wheel odometry, 1.196 m integrated travel, then zero velocity. All
required joint groups activated. Command:

```bash
ROS_DOMAIN_ID=97 MODE=loc ODOM=/robot_lab_controller/odom \
  CHECK_ARGS='--quick' TIMEOUT=100 bash scripts/sim_drive_check.sh \
  gazebo four_wheel_steer_car nav_empty /tmp/robotlab-4ws-gz-ready.json
```

This verifies controller activation, feedback and the forward/stop command
path. The measurements are wheel odometry; independent Gazebo body truth,
steering patterns and mapping/navigation missions remain to be measured.
The log records Gazebo requiring SIGTERM after its five-second SIGINT grace
period and a joy_teleop shutdown exception; no simulator/controller remains.
Mecanum's physical lateral-motion limitation is unchanged by the shared
hardware/controller repair.
