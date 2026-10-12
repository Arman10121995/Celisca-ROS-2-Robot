# Aerial RGB-D sensing and 3D mapping: implementation diagnostics (2026-10-12)

Scope: implementation smoke runs on the Jetson AGX Orin host, PX4 v1.16.2 SITL
with Gazebo Harmonic, `nav_obstacle`, headless, one run each. These show the
new path works end to end; they are not a qualification (no repeats, one
world, no map-accuracy or contact measurement).

Command (GUI: Drone RGB-D camera + Drone 3D mapping):

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py robot_model:=px4_x500 \
  simulator:=gazebo mode:=flight map_name:=nav_obstacle gui:=false start_rviz:=false \
  px4_vehicle:=x500_depth aerial_mapping:=rtabmap [aerial_map_database:=<saved .db>]
python3 scripts/aerial_mapping_smoke.py   # check -> takeoff -> 30 s yaw sweep -> 6 s forward -> land
```

| Run | Takeoff check | Flight | RTAB-Map | Files |
|---|---|---|---|---|
| Mapping (`mapping_flight.json`) | clear, 3.0 m rise | takeoff, sweep, forward, land, disarm | 67 nodes; cloud_map 1314 points | `aerial_map.db` 3.5 MB; exported `aerial_map_cloud.ply` 3456 points (5 cm voxels); hashes in `artifacts.sha256` |
| Localization in that map (`localization_flight.json`) | clear | same sequence | 66 localization poses, 14 loop/proximity closures | the saved database was copied, never written (SHA-256 unchanged) |

Camera bridge: 5 Hz colour and depth, 0 rejected frames over the runs;
intrinsics fx = fy = 216.25 px, cx = 160, cy = 120 for 320x240 at 1.274 rad.

## Defects found and fixed while doing this

- RTAB-Map did not start: the ROS executables load a `/usr/local` RTAB-Map core
  that links a RealSense library absent from this host. The runtime puts the
  ROS-shipped core first on `LD_LIBRARY_PATH` (as `robot_lab_mapping` already
  does for ground 3D SLAM), and runs `rtabmap-export` the same way.
- RTAB-Map parameters with `/` (e.g. `Grid/3D`) cannot be `-p` overrides; they
  go through a parameter file (`rtabmap_parameters.yaml`).
- Takeoff check rejected a landed drone on its own floor (October 9 code): a
  first run measured the landed FCU start at z = 0.218 m, putting the 0.227 m
  feet 9 mm into the ground plane (`takeoff_low_start_flight.json`, checked
  with the fix). The lowest 0.05 m of the starting envelope is no longer swept.

## Not covered

Map accuracy against the world, other worlds, longer or 3D routes, dynamic
actors, saving while airborne, the GUI-run path on a desktop display and
repeats. The FCU estimate is the odometry; RTAB-Map's correction is published
as `aerial_map -> map` and is not fed back to the flight controller.
