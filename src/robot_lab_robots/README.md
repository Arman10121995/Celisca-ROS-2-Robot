# Robot Lab robot descriptions and profiles

Updated October 8, 2026; exact source stages are in the current status and checkpoint. Read
[current status](../../docs/status/CURRENT_STATUS.md),
[the support matrix](../../docs/status/support-matrix.md) and
[the extension guide](../../docs/ASSET_EXTENSION_GUIDE.md) for measured scope.
Descriptions, structural tags and controller/mission support are separate.

## Select and inspect robots

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_lab_gui robot_lab_gui
```

In Launch, choose a category/type, complete parent family and exact
**Model/source variant**. Commands and compatible algorithms fill automatically.
Drive, Arm, Hand, Drone and their limits are in the right-hand control column.
Registry retains nested parts and variants and opens native **Preview 3D**.
A component may be inspected without being offered as a standalone robot.

The installed Jetson snapshot has 24 core and 113 extension profiles: 137 raw
profiles, consolidated into 72 complete families/112 selectable variants;
25 inspection components/reference profiles stay out of standalone Launch.
This is an inventory, not 137 working robot missions.

## Profile and source ownership

- Core profiles: [config/robots.yaml](config/robots.yaml).
- Grouping: [asset_groups.yaml](../robot_lab_bringup/config/asset_groups.yaml).
- Structural tags: [robot_taxonomy.yaml](../robot_lab_bringup/config/robot_taxonomy.yaml).
- Exact measured extension guards: [asset-runtime-support.yaml](../../docs/status/asset-runtime-support.yaml).
- Installed upstream sources/derivatives: SSD
  `$ROBOT_LAB_RUNTIME_ROOT/external_assets/`; large checkouts remain outside Git.
- Historical vendored sources: `_upstream/`, retained for provenance and excluded
  from runtime installation; third-party licenses remain in force.

Core profiles include Bumperbot/Labbot, Ackermann/rear/anti-Ackermann, 4WS,
mecanum, Berkeley/Unitree models, PX4 X500 and the legacy quadrotor fixture.
Official TurtleBot3/TurtleBot4, Husky, native Menagerie and robot-assets entries
come from the pinned SSD installation. Source alternatives retain their exact
IDs, frames, sensors and backend/controller limits.

## Measured controller workflows

| Models | Recorded scope | Next work |
|---|---|---|
| Bumperbot/Labbot and wheel bases | Named physical Drive, mapping/reset and Nav2 screens | Remaining map/mode/backend/pattern cells and repeats |
| TurtleBot3 Burger/Waffle/Waffle Pi | Four-backend Display/Drive, localization, 2D SLAM/export and navigation | Other maps/routes, RGB rendering/vendor firmware; RGB-only 3D SLAM unavailable |
| TurtleBot4 Standard/Lite | Four-backend Display/Drive, localization, 2D/3D SLAM/export and navigation | Other maps/routes, materials, docking and vendor behavior |
| Husky | Four-backend physical four-wheel Drive, localization, 2D/3D SLAM/export and clear/obstacle navigation with a declared lab kit | Other maps/routes, outdoor/payload/vendor hardware and broader estimator qualification |
| Native Menagerie Panda | MuJoCo joint/Hand/cube grasp and static-world MoveIt Plan/Execute | Servo, attached/dynamic scenes, other arms/hands/backends and mobile manipulation |
| PX4 X500 | Gazebo Harmonic FCU flight in named worlds | Wider flight-world/clearance/planning/mapping matrix |
| Go2/BHL | MuJoCo startup and bounded walks with model-specific policies | Sustained/terrain/recovery and further sensor/goal missions |
| Other imported descriptions | Declared Display/import/preview paths | Individual rest pose, materials/license, actuation and missions |

See [operator tutorials](../../docs/tutorials/README.md). Native MJCF profiles
remain MuJoCo-specific. A URDF derivative or shared family label does not transfer
native controls or a gait to another plant/backend.

## Add a robot

1. Claim the task in the ledger. Add a licensed URDF/Xacro/native model with
   resolved geometry, authored inertias/collisions, joint limits and source pin.
2. Add a core profile or use the pinned SSD provisioning workflow. Start with
   Display and explicit available backends. Do not grant motion from asset import.
3. Add reviewed grouping and structural tags. Preserve canonical profile IDs;
   keep partial assemblies nested until their actual mounting topology is known.
4. Declare sensors, frames, commands, state, stop/reset and controller limits.
   Implement the matching backend/controller interface and measure actual body
   response against simulator truth.
5. Complete class-specific sensor/control and mission checks. Archive exact
   commands, source/installed hashes, failures, metrics and owned cleanup before
   enabling additional modes. Keep other maps/backends unqualified.

Example core profile:

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

Use package-relative geometry resources, for example
`package://robot_lab_robots/my_robot/meshes/base_link.stl`. Build changed packages:

```bash
source scripts/ssd_env.sh
colcon build --packages-select robot_lab_robots robot_lab_bringup --symlink-install
source install/setup.bash
```

Example CLI inspection for an existing core robot, without a map:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py   robot_model:=unitree_go2 simulator:=mujoco mode:=display map_name:=none
```

Localization/mapping/navigation require compatible map/sensors/odometry/TF and
an actual motion controller where movement is requested. `3d_slam` needs RGB,
depth, camera information and their measured frame/time contract. Flight and
manipulation have separate controllers and acceptance. The GUI/common resolver
retain the reason for unsupported combinations.
