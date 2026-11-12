# Robotnik RB-SUMMIT-STEEL in Robot Lab

## Run

The complete source base is installed as `asset_robotnik_summit_steel`. In **Run a robot**,
choose **Mobile robots → Four-wheel mecanum base → Robotnik Summit**, Gazebo, Display
and `nav_empty`. The command fills automatically. Use the Drive column for forward,
reverse, turning and **Strafe L/R**. It uses the same latched pad and keyboard path as
the other bases. Enabling input begins neutral; Stop clears input.

Display Drive is implemented. To experiment with Localization, SLAM, 3D SLAM or
Navigation, enable experimental source assets and select the mode. The GUI selects
compatible algorithms and fills the command, including **Native Holonomic Wheel Control**
for Navigation. These higher robot missions remain unqualified.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_robot_counterparts.py --robot summit_steel --download
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_robotnik_summit_steel simulator:=gazebo \
  mode:=display map_name:=nav_empty
```

Restart the GUI or refresh installed assets after agent provisioning. There is no
operator download step in the GUI.

## What the controller uses

The source BSD-3-Clause Robotnik base keeps its 151 kg total mass, chassis and wheel
meshes, inertias, four driven wheels, source sphere contacts and anisotropic friction.
It models the source's roller effect through friction; it does not add passive roller
joints. Original wheel interface bounds are ±10 rad/s front and ±20 rad/s rear.
The lab limits manual body commands to 0.3 m/s per linear axis and 0.4 rad/s yaw.

The input path is Drive → `/key_vel` → priority mux → stamped relay → holonomic
wheel controller → ROS2 velocity group → Fortress physics. Actual wheel feedback
supplies odometry; simulator body truth is observed separately during measurements.
An expired command lease clears wheel motion rather than adding another deceleration
ramp. Four-wheel-steering robots retain their steering angles on that watchdog stop.

The derivative adds an explicitly declared massless lab LiDAR/IMU/RGB-D kit. It does
not reproduce Robotnik's optional sensor hardware. Source fixed-joint merging is
accounted for in physical sensor poses, TF and the real camera bridge path. Nav2 uses
a source-sized footprint with a small margin; no source geometry is replaced.

## Measured scope

One Gazebo `nav_empty` Display run measured 0.250 m/s forward, -0.243 m/s reverse,
0.250 m/s sideways and 0.250 rad/s turning. Camera images, LiDAR scans and source wheel
states arrived. Publisher-loss travel was 0.112 m, with zero final forward/yaw speed.
The largest measured wheel rate was 1.969 rad/s, within the source bounds.

Other worlds/backends, contact fidelity, saved maps and full SLAM/Nav2 missions remain
unqualified. Original authored mecanum and four-wheel-steering robots are preserved.
[Exact source/measurement receipt](../status/evidence/counterparts-2026-11-12/README.md) ·
[Upstream source](https://github.com/RobotnikAutomation/robot_description/tree/1a7aff1310dbc4427e0b2ae5ef83e0ffd2094504).
