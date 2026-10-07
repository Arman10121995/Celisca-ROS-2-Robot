# TurtleBot 4 in Robot Lab

Documentation reviewed October 7, 2026 against runtime checkpoint `091d388`.
Read [current status](../status/CURRENT_STATUS.md) for available workflows and
remaining qualification; evidence below retains its named source stages.

## Run

The official ROS 2 Humble **Standard** and **Lite** descriptions are installed
on the workspace SSD. Select `asset_turtlebot4_standard` or
`asset_turtlebot4_lite` in Launch, select a map and simulator, and use Display.
The command and compatible algorithms fill automatically. Restart an already open GUI after rebuilding;
Installed Extensions → Refresh Installed Assets reloads the model catalog.

Both variants have measured GUI Run/Stop, clock, joint state, description and
TF checks on Gazebo Fortress, MuJoCo, PyBullet and native Isaac Sim. The October 6
continuation adds real source-mounted RPLidar/OAK-D sensing, Localization,
2D SLAM, 3D SLAM, reset/resume and clear/obstacle Nav2 trials. Mode availability
comes from hashed backend-specific measurements. Original visual-material
parity, vendor hazard/docking behavior and other-map missions remain open.

For a first drive test, choose `nav_empty`, Display and the desired simulator.
Run, open **Launch → Drive & limits** in the right-hand column and enable keyboard control. Enabling it publishes no motion.
Hold **W/S** for forward/reverse and **A/D** for left/right rotation. Releasing
input ramps to zero; **Space** or **Stop** sends zero immediately. The GUI
buttons use the same control path. The displayed linear/angular values are
increments, while the actual controller limits speed to 0.3 m/s and 1.2 rad/s.
Physical joystick validation remains separate from the measured keyboard path.

The same selection can be launched from a sourced workspace:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_turtlebot4_standard simulator:=mujoco \
  map_name:=nav_empty mode:=display gui:=true
```

Use `asset_turtlebot4_lite` for Lite and `gazebo`, `pybullet` or `isaac` for
the other installed backends. Choose Localization, SLAM, 3D SLAM or Navigation
in Launch where enabled. The GUI selects AMCL, Slam Toolbox, RTAB-Map and the
compatible Nav2 controller as appropriate and fills the run command. Support
records require real measurements; unavailable combinations retain a reason.

Use **Reset Robot** to restore the selected spawn without rewinding simulation
time. In 2D SLAM, **Save Map** writes the actual occupancy YAML/PGM. In 3D SLAM,
it saves an intact RTAB-Map database and finite PCD points and waits for the
export process to exit. These are measured on `nav_empty`; the navigation
screens also include a blocked direct path in `nav_obstacle`. A different map
is selectable for testing but does not inherit qualification from those two.

The source pins and licenses are retained:

- [Official TurtleBot 4](https://github.com/turtlebot/turtlebot4/tree/a6ee13b63cbd524500cc6a68cd20dbef69f326a9),
  Apache-2.0: Standard/Lite URDF, OAK-D, RPLidar, original meshes.
- [Official Create 3 dependency](https://github.com/iRobotEducation/create3_sim/tree/7d7a4f69a503d4eef22f13aeb8755c86955b8cc5),
  BSD-3-Clause: base, wheels, wheel-drop joints, caster and sensor descriptions.

The agent/bootstrap installer downloads them without GUI download controls:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_extension_assets.py --kind robots --source turtlebot4_vendor
```

It expands Xacro against an SSD source-backed ament index, resolves actual
mesh resources and checks both resulting models with PyBullet. Derived
Display files omit upstream simulator/control plugins; the original source,
license and four movable wheel/wheel-drop joints remain. A separate `drive.urdf`
and controller YAML retain the original geometry and the Humble Create 3
450 N/m wheel-drop springs, 50 Ns/m damping and 0.03 m equilibrium/travel.
Gazebo uses these original spring declarations; MuJoCo, PyBullet and Isaac
apply the same passive spring law through their actual physics engines.
No robot pose writes implement driving. The derivative adds finite wheel
actuator limits and the lab controller; it does not install the upstream
docking, hazard or firmware stack.

## Measured controls and remaining work

[Drive evidence](../status/evidence/turtlebot4-drive-2026-10-05/README.md)
records exact variant/backend commands, independent body and joint feedback,
neutral enable, forward/reverse/turn, Stop and silent-publisher trials in
`nav_empty`. Stop acceptance uses a 2.5 s simulation settling window and the
final 0.5 s average; it does not claim an instantaneous physical halt.
The physics bridges use a 0.5 s wall timeout; Gazebo's upstream controller uses
its ROS simulation clock with the same configured interval. Isaac differential
drive now sends timed wheel targets on message loss and advances acceleration using
actual SDK time, preserving behavior when physics runs slower than real time.
The original Isaac no-timeout and tipping trials remain negative evidence.

The lab sensor configuration uses the real source transforms for
`rplidar_link` and the OAK-D optical frame: a 640-ray scan and 320×240 calibrated
RGB-D images. Trials require changing depth geometry and finite measured wheel
and suspension state. These simulate the selected lab rates and ranges; they
do not reproduce the vendor firmware's complete noise/timing/hazard stack.

Gazebo caps coarse selected worlds at a 2 ms physics step for the compliant
Create 3 suspension. The cached derivative preserves the selected world name,
geometry and plugins and never increases a finer authored step. Isaac headless
rendering samples the camera separately from 60 Hz physics. Its ideal body
odometry derives twist from successive actual root poses: the raw contact
solver reported nonzero velocity while the Lite body was stationary, which
previously drifted the EKF. This remains ideal simulated odometry, not a claim
about noisy wheel or hardware localization.

Gazebo uses Humble's upstream
[diff_drive_controller](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
with the original wheel names/dimensions; the other backends use the existing
lab physics bridges. `/odom/ground_truth` comes from engine body poses,
independently of wheel-controller odometry. This is manual simulation control,
not Create 3 vendor firmware or TurtleBot 4 hardware qualification.

For agents continuing integration: read the R3.6 ledger, exact evidence and the
[official Humble simulator guide](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html).
Use `left_wheel_joint`, `right_wheel_joint`, wheel radius 0.03575 m and track
0.233 m from the pinned Create 3 description. Preserve the actual suspension
and caster, measured command-loss behavior and neutral GUI enable. Preserve the
October 6 named reset/Save Map/estimate and strict Nav2 screens when changing
timing, collision geometry or sensor adapters. Require a fresh pre-trial source
manifest, independently measured physical body position and heading, and no
orphan plant or export process before the next Run. Next test other imported
and furnished maps, goal cancellation, second goals and contact telemetry.
Keep vendor docking/hazards, full visual parity and hardware qualification as
separate tasks.
Do not enable a navigation mode from an import count.

The [October 6 mode trials](../status/evidence/turtlebot4-modes-2026-10-06/README.md)
and [final sensors/Drive trials](../status/evidence/turtlebot4-sensors-2026-10-06/README.md)
record exact commands, truth limits, saved-map measurements and retained
failures. MuJoCo performance remains below real time on this host.
