# TurtleBot3 lidar and GUI Drive — October 7, 2026

Twelve final serial GUI trials passed: official Burger, Waffle and Waffle Pi
on Gazebo Fortress, MuJoCo, PyBullet and native Isaac. These extend `18ecdd2`
on Jetson AGX Orin / ROS 2 Humble, with the exact source, installed-module,
executed URDF and producer hashes captured before each launch. Each producer
and owned launch returned zero; the archived batch runner checked that their
owned plants were gone. These are engine body/joint observations, not a
command echo or wheel odometry substituted for physical travel.

The normal installed GUI selects Display/`nav_empty`, autofills the command
and starts its owned launch. Keyboard enable emits zero commands and body
motion stays bounded. W/S/A/D move forward/reverse and turn both ways. Space
and publisher loss settle below 0.025 m/s and 0.05 rad/s in the final 0.5 s of
a 2.5 simulation-second window. These limits do not describe an instantaneous
physical halt. Both wheel joints have finite changing physical state; the
complete raw trace, including shutdown, stays below 0.3 rad body tilt.

The actual source LDS has 360 rays, a 0.12–3.5 m range, `base_scan`, and 5 Hz
lab publication. `base_scan` and `imu_link` TF positions match their executed
URDF mounts within 10 µm. The pinned simulation definitions provide RGB-only
cameras. Their geometry is retained, but RGB rendering is pending and no
depth stream or 3D SLAM support is claimed. Other backends use the existing
ideal sensor bridge; Gazebo retains the pinned IMU definition at 50 Hz.

The original ROBOTIS sources are immutable: description revision
`90a68bd2e3c61c12966779da89d8eeaec82730e9` and simulation revision
`a35a56c8b04877dc89772b598084d8ce648a9023`, Apache 2.0. See the
[source manifest](../../asset-sources-2026-10-05.yaml),
[official simulation source](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
and [vendor simulation guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).
The lab derivative uses 0.033 m wheels and executed tracks of 0.160 m for
Burger and 0.288 m for Waffle/Pi. Its 0.18 m/s, 0.3 m/s², 1.2 rad/s and
2 rad/s² caps are simulation controller settings, not OpenCR qualification.

Source frames without authored inertia receive an explicit 1e-6 kg mass and
1e-9 kg·m² diagonal inertia. PyBullet otherwise invents 1 kg / 1 kg·m² for
each such frame. Authored physical inertias are preserved; this regularization
also covers source camera mounts that omit inertia. MuJoCo gets profile-local
0.0002 wheel armature, 0.1 velocity gain and acceleration-limited watchdog
deceleration. Other robots retain the existing defaults. `nav_empty` uses
the named robot spawn (-4,-4,0) so the 3.5 m LDS sees the walls; other maps
retain their own spawn metadata and explicit operator overrides take priority.

## Retained failures and claim boundary

The stage-1 DDS domain was outside this host's valid port range. The stage-2
PyBullet turn stop exceeded 0.05 rad/s with implicit frame inertias. MuJoCo
stages 4–7 exceeded the unchanged 0.3 rad tilt limit: reducing the speed cap
or wheel armature alone did not resolve it. The final profile's ramped
watchdog stop passed a diagnostic repeat and then these twelve fresh trials.
All earlier reports/logs remain on SSD; failed reports are copied under
`negative/`. They are excluded from final support, rather than overwritten.

The producer's [original reports](asset_turtlebot3_burger-mujoco/report.json)
are preserved. `report-measured.json` adds joint ranges and maximum body tilt
calculated from the retained raw physical trace, with both source checksums.
[collect_sensors.py](collect_sensors.py) is that derivation, not a simulator.
[manifest.json](manifest.json) indexes original trace/model/log hashes and
all retained stages. The batch runner and its actual outcomes are archived.
Large traces remain at the named mounted SSD paths.

These are bounded Display/keyboard/Drive/sensor screens. Mapping/navigation
have separate evidence. Longer routes, other maps, materials, RGB cameras,
vendor firmware/noise/docking and physical joystick remain unqualified.

## Measured final runs

RTF is simulation advancement divided by observed wall time on this host,
including concurrent source-check load. It is not a backend performance
benchmark. The larger TurtleBot4 MuJoCo performance gap remains open.

| Variant | Backend | Measured RTF | Peak body tilt (rad) | LDS messages |
|---|---|---:|---:|---:|
| burger | gazebo | 0.930 | 0.0052 | 141 |
| burger | isaac | 0.694 | 0.2467 | 138 |
| burger | mujoco | 0.705 | 0.2662 | 142 |
| burger | pybullet | 0.905 | 0.0332 | 139 |
| waffle | gazebo | 0.835 | 0.0026 | 141 |
| waffle | isaac | 0.854 | 0.0757 | 140 |
| waffle | mujoco | 0.721 | 0.1503 | 142 |
| waffle | pybullet | 0.827 | 0.0081 | 139 |
| waffle_pi | gazebo | 0.854 | 0.0026 | 141 |
| waffle_pi | isaac | 0.551 | 0.0762 | 138 |
| waffle_pi | mujoco | 0.751 | 0.1503 | 142 |
| waffle_pi | pybullet | 0.852 | 0.0115 | 138 |
