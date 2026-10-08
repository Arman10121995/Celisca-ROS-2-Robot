# Robot Lab worlds and occupancy maps

Updated October 8, 2026; exact source stages are in the current status and checkpoint. Read
[current status](../../docs/status/CURRENT_STATUS.md),
[the extension guide](../../docs/ASSET_EXTENSION_GUIDE.md) and
[the roadmap](../../ROADMAP.md) for exact world/backend mission acceptance.

## Select and inspect maps

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_lab_gui robot_lab_gui
```

Launch uses a complete map family and exact **World variant**. Registry and
Worlds/maps open native **Preview 3D** inside the GUI. Celisca shell, furniture
and actors remain variants of their floor; Room2/3/4 static/dynamic worlds and
hospital source variants retain exact IDs and files.

The installed Jetson snapshot contains 26 core plus 17 extension world
profiles under 33 map families. Fourteen dataset worlds and three Gazebo
examples are installed on SSD. The remaining 97 example SDFs include plugin/
robot fixtures requiring review. Loading or previewing a world does not qualify
its collisions, spawn, actors or navigation/flight missions.

## Geometry and mode contracts

Core definitions are in
[sim_maps.yaml](../robot_lab_bringup/config/sim_maps.yaml); installed extension
profiles live under `$ROBOT_LAB_RUNTIME_ROOT/external_assets/installed/`.
Grouping is in [asset_groups.yaml](../robot_lab_bringup/config/asset_groups.yaml).
Gazebo loads SDF directly; shared SDF/include/mesh conversion feeds PyBullet,
MuJoCo and Isaac. Source scale/poses and physics collision geometry must agree.
Native preview is a separate source-visual inspection path.

| Mode | Required map data |
|---|---|
| `display` with a world | Valid resolved world; robot-only display uses `map_name:=none` |
| `loc` / `nav` | World plus a reviewed 2D occupancy map with compatible origin/scale/spawn |
| `slam` | World and compatible robot lidar/odometry/TF; existing occupancy is unnecessary |
| `3d_slam` | World and compatible robot RGB-D/camera-info/state pipeline |
| `flight` | Actual PX4 plant/world with spawn and floor/ceiling clearance; a 2D Nav2 map is unnecessary |

`has_2d_map: false` prevents known-map localization/navigation until a real map
is created and registered. A generated connected height slice is not an
all-floor occupancy map. Explicit launch spawns override named robot/map
defaults; estimator initialization must use the same resolved frame and pose.

Celisca's 20% scaling, spawn alignment and furniture collision fixes have named
measured trials. Remaining furnished/actor/map/backend cells are still open.
Derived hospital variants repair six SHA-pinned escaped fixtures to their
authored floors, preserve original sources and Gazebo Collada node frames, and
freeze furniture consistently across engines. Spawn uses the reviewed lobby
floor support. Read [current geometry and route evidence](../../docs/status/evidence/extensions-finish-2026-10-08/README.md);
upper-floor travel, dynamic actors and complete contact/visual parity remain open.

## Generate a 2D occupancy grid

In **Worlds/maps**, select a world, resolution, height slice and fresh SSD
output directory, then **Generate 2D Occupancy Grid**. Inspect the result before
navigation registration. Stop Generation closes its owned generator/Gazebo
process. The GUI fills the selected profile's spawn as its generation seed;
edit **Seed X/Y** to select another connected room. Existing saved maps remain intact.

Equivalent example:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ROBOT_LAB_GRID_RUN_DIR=$(mktemp -d "$ROBOT_LAB_RUNTIME_ROOT/occupancy-XXXXXX")
ros2 run robot_lab_maps generate_occupancy_map.py   --world "$PWD/src/robot_lab_maps/maps/nav_obstacle/worlds/nav_obstacle.world"   --output-dir "$ROBOT_LAB_GRID_RUN_DIR"   --resolution 0.1 --height 0.3 --seed-x -7 --seed-y -7
```

The pinned UPO Fortress plugin and actual collision-mesh slice produce real
PGM/YAML/report files. Check world/map metres, origin, composed poses, rotated
geometry, seed-connected free regions and robot footprint. Record registration
and actual Nav2 goals separately. Actors/dynamic obstacles and multilevel
terrain need their own representation and mission checks.

## Add a world

1. Claim the task and retain source/license/dependency pins on SSD. Resolve
   includes, meshes/materials and backend plugin migrations without altering
   the preserved upstream source.
2. Add `maps/<id>/worlds/<id>.world`, its resources and a profile. Use free
   spawn/goals measured from actual collision geometry.
3. Supply or generate a reviewed occupancy map for localization/navigation.
   `image:` should be relative to the YAML; resolution/origin must match world
   geometry and estimator initialization.
4. Generate/check backend representations and actual load/display/physics.
   Then record class-specific movement, contacts, sensors, reset and mission
   outcomes per advertised backend. Do not inherit support from another world.
5. Add the family/variant, update exact evidence and rebuild changed packages.

Example core profile (replace the example with real installed files/poses):

```yaml
maps:
  my_map:
    gazebo:
      world_package: robot_lab_maps
      world_name: my_map
      world_path: maps/my_map/worlds/my_map.world
    map:
      has_2d_map: true
      package: robot_lab_maps
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

```bash
source scripts/ssd_env.sh
colcon build --packages-select robot_lab_maps robot_lab_bringup --symlink-install
source install/setup.bash
```

## Existing arena and CLI examples

Five deterministic navigation arenas have companion worlds/occupancy geometry:
`nav_empty`, `nav_obstacle`, `nav_maze`, `nav_narrow_passage`, `nav_warehouse`.
The generator and validator are in this package's `tools/`; regenerating assets
is an agent/development change, separate from GUI operation.

World-only display and robot-only display use explicit choices:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py   robot_model:=none map_name:=nav_obstacle simulator:=gazebo mode:=display
```

For measured robot/mapping/navigation or flight commands, use
[the operator guides](../../docs/tutorials/README.md) and exact named source
variants. Wider actor, terrain, imported-world and flight-map acceptance remains
R6.5–R6.7/R5.10. Keep actual export/trial buffers and logs on SSD.
