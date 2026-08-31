# Maps Package

This package stores simulation worlds, Nav2 map files, and map-specific assets.
The launch system reads map defaults from:

```text
src/bumperbot_bringup/config/sim_maps.yaml
```

The GUI launcher reads the same map profiles:

```bash
ros2 run bumperbot_bringup sim_launcher_gui
```

## Directory Layout

Each map lives under `maps/<map_name>/`:

```text
src/maps/
  maps/
    my_map/
      worlds/
        my_map.world
      maps/
        map.yaml
        map.pgm
      meshes/
        optional_mesh.stl
```

Required files depend on the mode:

- `display`: no map files required.
- `slam`: requires `worlds/<map_name>.world`.
- `3d_slam`: requires `worlds/<map_name>.world` and a robot with an RGB-D camera.
- `loc`: requires `worlds/<map_name>.world` and `maps/map.yaml`.
- `nav`: requires `worlds/<map_name>.world` and `maps/map.yaml`.

Maps without a real 2D occupancy map should set `has_2d_map: false`.
The GUI and launch file will then disable `loc` and `nav` for that map until
you create and register a saved map.

## Add A New Map

1. Create the map folder:

```bash
mkdir -p src/maps/maps/my_map/worlds
mkdir -p src/maps/maps/my_map/maps
mkdir -p src/maps/maps/my_map/meshes
```

2. Add the Gazebo world:

```text
src/maps/maps/my_map/worlds/my_map.world
```

3. Add the Nav2 occupancy map when using `loc` or `nav`:

```text
src/maps/maps/my_map/maps/map.yaml
src/maps/maps/my_map/maps/map.pgm
```

The `image:` field inside `map.yaml` should usually be relative:

```yaml
image: map.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

4. Register the map profile in `src/bumperbot_bringup/config/sim_maps.yaml`:

```yaml
maps:
  my_map:
    gazebo:
      world_package: maps
      world_name: my_map
      world_path: maps/my_map/worlds/my_map.world
    map:
      has_2d_map: true
      package: maps
      path: maps/my_map/maps/map.yaml
    spawn:
      x: "0.0"
      y: "0.0"
      z: "0.0"
      yaw: "0.0"
    initial_pose:
      x: "0.0"
      y: "0.0"
      yaw: "0.0"
```

`initial_pose` is the AMCL pose in the map frame. For maps generated from the
same Gazebo world frame, set it to the same `x`, `y`, and `yaw` as `spawn`.

5. Rebuild:

```bash
colcon build --packages-select maps bumperbot_bringup
source install/setup.bash
```

## Run The Four Modes

Display robot only:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=display robot_model:=bumperbot
```

Localization on a saved map:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=loc map_name:=my_map
```

SLAM on a Gazebo world:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=slam map_name:=my_map
```

3D RGB-D SLAM on a Gazebo world:

```bash
sudo apt-get install ros-humble-rtabmap-ros
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=3d_slam map_name:=my_map
```

Navigation on a saved map:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py mode:=nav map_name:=my_map
```

Room vacuum simulation is separate:

```bash
ros2 launch bumperbot_bringup simulated_room_vacuum.launch.py mode:=nav map_name:=my_map
```

## Current Map Profiles

- `celisca_floor_1`
- `celisca_floor_2`
- `celisca_f1_actor`
- `celisca_f2_actor`
- `simple_box`
- `small_house`
- `small_warehouse`
