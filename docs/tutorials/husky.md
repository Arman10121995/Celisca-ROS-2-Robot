# Husky in Robot Lab

Updated October 8, 2026. The source is the official Humble Husky description
at `729f8aa45ccd86fa33a05e07ef698c52c451cd9c`, installed on the workspace SSD.
The original geometry, inertias and four independent wheel joints are retained.
This integration uses a declared lab controller and sensor kit; Clearpath
firmware, hardware, payload and outdoor traversal require separate work.

## Run

Select **Husky / `asset_husky`** in Launch, choose a simulator and `nav_empty`,
and start Display with the autofilled command. Open **Drive & limits** in the
right control column. Enable keyboard input, then hold **W/S** or **A/D**.
Enabling input alone publishes no movement. Release decelerates; **Space/Stop**
sends zero immediately, while the physical motors obey acceleration limits.
The displayed velocity values are increments.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_husky simulator:=mujoco \
  map_name:=nav_empty mode:=display gui:=true
```

The four fixed axles use skid steering: both wheels on each side receive
separate motor targets. Translation is capped at 0.4 m/s, rotation at 1 rad/s,
with 0.5 m/s² and 1.5 rad/s² acceleration limits. The lab motor envelope is
50 N·m / 10 rad/s. MuJoCo and Isaac use bounded body-rate feedback to
compensate skid slip; Bullet retains the source tires' lower axial friction
with explicitly declared independent friction constraints. Driving never
writes the base pose or anchors the chassis.

The default source has no enabled lidar/camera. The **lab kit** adds a 360-ray
0.1–12 m lidar at `(0.30, 0, 0.40)` in `base_link`, a 320×240 RGB-D camera at
`(0.35, 0, 0.45)` and the source IMU frame. Lidar/RGB-D run at 5 Hz, IMU at
50 Hz. These are simulation mounts, not OEM sensor calibration. Actual wheel
changes, sensor geometry, mount transforms, neutral input, both turns, Stop,
command loss and owned Stop/Run relaunch pass the four named Drive screens.

Use the higher modes enabled for the selected backend. Their availability
comes from hashed physical workflow reports, with AMCL, Slam Toolbox,
RTAB-Map and Nav2 A* / Pure Pursuit chosen automatically. **Save Map** exports
the live occupancy grid or database/PCD; **Reset Robot** requires the measured
robot/estimator reset path. A selectable additional world is an experiment
until its own source/spawn/sensing and mission are qualified.

The 0.62 m navigation circle encloses the original collision envelope.
Gazebo estimates translation from encoders and yaw rate from the IMU, with
lidar AMCL correcting the map pose. Encoder yaw overstates rotation during
skid turns; the Husky-specific EKF overlay excludes that conflicting input.
The other three backends currently use the platform's reference body-twist
input to the EKF. Those screens test physical motion and workflow operation;
they do not benchmark noisy encoder estimation or sim-to-real accuracy.

Read [the current status](../status/CURRENT_STATUS.md) and
[October 8 evidence](../status/evidence/extensions-finish-2026-10-08/README.md)
for exact modes, maps, source stages and retained failures. Short successful
routes do not qualify all maps, long missions, payloads or contact sensing.
