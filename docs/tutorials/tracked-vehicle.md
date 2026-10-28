# Native tracked vehicle

Select **Mobile robots → Tracked vehicle → lab_tracked** in the GUI, choose
Gazebo and a terrestrial map, then Display, Localization, SLAM, 3D SLAM or
Navigation. Drive uses the normal control column. The initial `nav_empty`
reference has measured forward motion, controller-loss stopping and Nav2
arrival; other configurations remain experimental.

The robot is the complete Gazebo tracked/flipper source. It uses native
contact-surface belts. Four flippers retain the source joints and measured
joint-state visualization. Source flippers are passive; a dedicated flipper
actuation/editor workflow remains separate.

Navigation autofills A* / regulated pure pursuit and **Native tracked command
owner**. Track dead reckoning supplies planar motion; the actual IMU supplies
turn rate to the EKF, with AMCL correcting the map pose. Simulator truth is a
separate observer. Display uses the same source geometry and calibrated
footprint height.

The installed private physics prefix is on SSD:
`$ROBOT_LAB_RUNTIME_ROOT/native_tracks`. Agents preparing a clean installation
build it with `python3 scripts/provision_tracked_physics.py` after sourcing
`scripts/ssd_env.sh`. The script extracts pinned OSRF packages and builds only
the required Physics 5.4/DART plugin. It does not install over system packages.
The GUI uses the locally provisioned result; there is no asset-download step
inside its Launch flow.

The native watchdog stops stale commands even if the ROS command process is
suspended. Terrain, slip, more maps, full contact/clearance measurements and
other simulator implementations remain open.

Upstream references: [TrackedVehicle](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1TrackedVehicle.html),
[TrackController](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1TrackController.html),
[Fortress DART requirement](https://github.com/gazebosim/gz-sim/issues/2495).
Source hashes, licenses and the small-command adaptation are preserved with
the model and vendored controller. [Measured scope](../status/native-workflows-2026-10-27.md).
