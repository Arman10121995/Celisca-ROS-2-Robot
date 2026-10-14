# Plan drone routes with mapped and live depth obstacles

October 14, 2026. This experimental integration supplements the complete
selected **static world** with RTAB-Map obstacle occupancy and current depth
observations. It uses the existing native PX4 X500/Gazebo Harmonic Flight
workflow. No new mapped-route flight mission has been qualified; see
[implementation and diagnostics](../status/aerial-obstacles-2026-10-14.md).

## Operator workflow

1. Reopen the rebuilt GUI. Choose **PX4 X500**, **Gazebo**, **Flight** and a
   compatible installed static world.
2. Enable **Drone route: mapped + live depth obstacles (experimental)** in
   Launch's advanced options. RGB-D and RTAB-Map mapping are enabled together.
   The preview contains `px4_vehicle:=x500_depth`, `aerial_mapping:=rtabmap` and
   `aerial_obstacles:=mapped_depth`. A saved database remains an optional
   localization input. Configure these options before Run.
3. Run. Use the existing **Check takeoff clearance → Takeoff** workflow.
   In Drone, wait for the observed route layer to report ready and inspect
   the map/depth cell counts and observation ages.
4. **Use current FCU position**, edit a small target, then **Plan route**.
   Plan runs 3D A* against the canonical world plus observed occupancy.
5. **Execute route** first rechecks the entire route asynchronously against
   current observations. A clear, unchanged FCU/operator/resource context
   starts the actual Offboard waypoint controller.
6. A detected blockage or stale sensor/map/intrinsics interrupts that route
   and requests Hold at the current FCU position. Move/inspect the scene or
   restore sensing, then explicitly Plan and Execute again. **Cancel / Hold**,
   Space, manual input, direct goals and Land interrupt pending work too.

The existing manual Drive/yaw/altitude and Takeoff controls remain separate
control paths. Enabling the observation option requests no takeoff or motion.
Data recovery does not resume an interrupted route automatically.

## Command

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=px4_x500 simulator:=gazebo mode:=flight map_name:=nav_empty \
  px4_vehicle:=x500_depth aerial_mapping:=rtabmap aerial_obstacles:=mapped_depth
```

The launch selects the existing SSD PX4 Python environment and native `-d`
FCU runtime. It rejects observation mode without RGB-D, RTAB-Map and Flight.
No new upstream model or download step is required.

## Data and execution contract

- RTAB-Map's `/rtabmap/cloud_obstacles` supplies mapped occupancy in
  `aerial_map` or `map`; the capture-time TF converts it to the FCU's `map`.
- `/x500/camera/depth/image_raw` and matching fresh CameraInfo supply registered
  pinhole depth. Every valid pixel between the native 0.2 m near clip and
  the declared 8 m observation range is projected; no sparse pixel sampling.
  The native camera itself renders to 19.1 m, while mapping uses an 8 m range.
- A latest-message worker performs decoding, transforms and voxel/tree building
  outside ROS/FCU callbacks. Oversized/malformed/untransformed observations
  reject. Old/replayed stamps do not renew health. Point clouds have a 300,000
  point/16 MB admission limit and occupancy a 100,000-cell cap; overflow
  rejects instead of silently removing obstacles.
- Occupancy uses 0.15 m cells and conservative half-diagonal inflation around
  cell centres. It adds to the existing 0.7 m spherical planning envelope.
- The FCU route controller checks current map/depth occupancy along 2 m of
  the upcoming route. Depth expires after 0.75 wall seconds, CameraInfo after
  2 seconds, mapped occupancy after 10 seconds. A failed or near-contact query
  requests Hold. The existing setpoint rate/speed and telemetry-loss Land
  behavior remain.
- Background Plan/Execute rechecks bind the operator command generation. A
  late worker result cannot start an old route after manual input, Stop or a
  direct goal. World resource changes or >5 cm FCU drift reject execution.

This is observed occupancy and interruption, not future actor prediction.
Unseen space is not certified clear by an absent cloud point; the complete
canonical static world remains the planning baseline. Unsupported actor/world
geometry still rejects that baseline. Detouring is done by another explicit
Plan/Execute. It does not establish autonomous dynamic navigation, tracking
clearance or obstacle-aware manual/Takeoff control.

The integration uses the official
[RTAB-Map obstacle publishers and frame contract](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_util/src/MapsManager.cpp)
and [Humble PointCloud2 decoder](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs_py/sensor_msgs_py/point_cloud2.py).
SciPy supplies the nearest-neighbour tree; the existing canonical planner
checks collision edges. Installed library versions remain in the receipt.

## Next agent's qualification work

Read the ledger and claim the validation scope separately. First run a short
normal-GUI `nav_empty` flight, record actual FCU and independent Gazebo body
pose, RGB-D/TF/RTAB-Map messages, planner states and the exact source/command.
Measure observed versus world obstacle alignment, current transform latency,
Offboard setpoint cadence, route/body tracking and minimum physical clearance.

Then introduce an **actual collision model** in the running Gazebo scene ahead
of the camera/route, retaining the insert request/model hash and independent
pose trace. Verify the physical interruption and actual flight clearance;
publishing a fabricated PointCloud2 is only a software test. Stop the camera
producer to measure real sensor-loss/hold behavior and restore it to check
that execution stays interrupted until a new operator request. Repeat manual/
Space/direct-goal/Cancel/Land during both planning and the execution recheck.

Keep existing Takeoff/body/contact/landing criteria; do not infer success from
arming, cell counts, framework availability or action/state strings alone.
Repeat saved-map/localization, furnished/ceiling/terrain worlds and source
changes only after this first mission. Other flight backends, future actor
prediction, automatic detours and full world coverage remain separate tasks.
