# TurtleBot 4 in Robot Lab

The official ROS 2 Humble **Standard** and **Lite** descriptions are installed
on the workspace SSD. Select `asset_turtlebot4_standard` or
`asset_turtlebot4_lite` in Launch, select a map and simulator, and use Display.
The command fills automatically. Restart an already open GUI after rebuilding;
Installed Extensions → Refresh Installed Assets reloads the model catalog.

Both variants have measured GUI Run/Stop, clock, joint state, description and
TF checks on Gazebo Fortress, MuJoCo, PyBullet and native Isaac Sim, using
`dataset_room2`. These checks cover display/state. Original visual-material
parity, driving, simulated sensor parity, SLAM and navigation remain open.
Drive/WASD/joystick controls are disabled for these profiles until a base
controller is integrated. Selecting a lidar mesh does not establish `/scan`.

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
license and four movable wheel/wheel-drop joints remain. This does not install
the upstream docking, hazard or firmware stack.

For agents continuing control integration: read the R3.6 ledger and the
[official Humble simulator guide](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html).
Use `left_wheel_joint`, `right_wheel_joint`, wheel radius 0.03575 m and track
0.233 m from the pinned Create 3 description. Preserve the actual suspension
and caster. Implement each backend's physical differential drive and measured
odometry, stop/watchdog/reset; then verify real lidar, OAK-D optical frames,
camera calibration and changing depth/point clouds. Add Localization/SLAM/
3D SLAM/Nav2 only after their real adapters resolve and execute. Qualify
forward/reverse/turn, obstacles, cancel and a second navigation goal against
physics truth in clear and furnished maps. Record docking/hazard behavior as
separate vendor integration rather than implying it from the imported URDF.
