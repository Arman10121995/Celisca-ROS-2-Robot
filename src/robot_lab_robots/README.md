# Robots Package

This package stores robot descriptions and robot launch profiles. The simulator
selects robots through:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py robot_model:=bumperbot
```

The physics backend is selected with `simulator:=gazebo|isaac|pybullet|mujoco`
(default `gazebo`; the ISAAC/PyBullet/MuJoCo adapters mirror the Gazebo spawn
interface — see P7.7 in the ROADMAP):

Robot profiles are defined in:

```text
src/robot_lab_robots/config/robots.yaml
```

The GUI launcher reads the same profiles:

```bash
ros2 run robot_lab_bringup robot_lab_gui
```

## Directory Layout

Robot files live directly under this package:

```text
src/robot_lab_robots/
  config/
    robots.yaml
  bumperbot/
    urdf/
    meshes/
  berkeley_humanoid_lite/
    urdf/
    meshes/
    mjcf/
    usd/
  unitree/
    a1_description/
    go2_description/
    ...
```

The original upstream checkouts are kept for provenance under:

```text
src/robot_lab_robots/_upstream/
```

That folder is excluded from installation. Use the cleaned robot folders above
for runtime launch files.

## Add A New Robot

1. Create a folder:

```bash
mkdir -p src/robot_lab_robots/my_robot/urdf
mkdir -p src/robot_lab_robots/my_robot/meshes
```

2. Add your URDF or Xacro:

```text
src/robot_lab_robots/my_robot/urdf/my_robot.urdf.xacro
```

3. Use package-relative mesh paths that point back to this package:

```xml
<mesh filename="package://robot_lab_robots/my_robot/meshes/base_link.stl"/>
```

4. Register a profile in `src/robot_lab_robots/config/robots.yaml`:

```yaml
robots:
  my_robot:
    package: robot_lab_robots
    xacro: my_robot/urdf/my_robot.urdf.xacro
    name: my_robot
    supported_modes: [display]
    features: []
    supports_room_vacuum: false
```

5. Rebuild:

```bash
colcon build --packages-select robots robot_lab_bringup
source install/setup.bash
```

6. Run it:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=display robot_model:=my_robot
```

## Run Robots In The Modes

The GUI disables modes that are not listed in a robot profile's
`supported_modes`. Some modes also require robot `features`; for example,
`3d_slam` requires `rgbd_camera`.

Display any valid robot description:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=display robot_model:=unitree_go2
```

Localization with a robot that has the full simulation stack:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=loc map_name:=small_house robot_model:=bumperbot
```

SLAM with a robot that has the full simulation stack:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=slam map_name:=small_house robot_model:=bumperbot
```

3D RGB-D SLAM with a robot that has an RGB-D camera:

```bash
sudo apt-get install ros-humble-rtabmap-ros
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=3d_slam map_name:=small_house robot_model:=bumperbot
```

Navigation with a robot that has the full simulation stack:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav map_name:=small_house robot_model:=bumperbot
```

## Runtime Requirements

`display` only needs a valid URDF/Xacro.

`loc`, `slam`, `3d_slam`, and `nav` need more than a visual model:

- a base frame compatible with the rest of the stack;
- a simulated sensor publishing `/scan` for localization, SLAM, and Nav2;
- odometry and TF from `odom` to the robot base;
- a controller that accepts velocity commands or another robot-specific command bridge.
- for `3d_slam`, an RGB-D camera publishing RGB image, depth image, camera info, and TF.

The imported Unitree and Berkeley models are description assets. They can be
selected by `robot_model`, but their locomotion, sensors, and control plugins
may need robot-specific work before they can drive through Nav2 like bumperbot.

## Current Robot Profiles

- `bumperbot`
- `berkeley_humanoid_lite`
- `berkeley_humanoid_lite_biped`
- `unitree_a1`
- `unitree_aliengo`
- `unitree_b1`
- `unitree_b2`
- `unitree_b2_mujoco`
- `unitree_b2w`
- `unitree_g1`
- `unitree_g1_29dof`
- `unitree_go1`
- `unitree_go2`
- `unitree_go2w`
- `unitree_h1_2`
