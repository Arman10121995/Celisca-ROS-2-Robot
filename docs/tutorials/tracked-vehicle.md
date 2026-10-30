# Native tracked vehicle

Select **Mobile robots → Tracked vehicle → lab_tracked** in the GUI, choose
Gazebo and a terrestrial map, then Display, Localization, SLAM, 3D SLAM or
Navigation. Drive uses the normal control column. The initial `nav_empty`
reference has measured forward motion, controller-loss stopping and Nav2
arrival; other configurations remain experimental.

The robot is the complete Gazebo tracked/flipper source. It uses native
contact-surface belts. Four flippers retain the source joints and measured
joint-state visualization. The original source fixes their limits at zero.
An explicit **articulated flippers** variant, `lab_tracked_articulated`, is
nested under the same family; its geometry, mass and belts are preserved.

Choose that variant to open **Tracks** in the right control column. Connect
sends no angle or Drive input. **Use measured angles** copies real feedback;
edit the four degree targets and **Apply bounded targets**, or **Raise ends**.
Targets slew at 0.2 rad/s within ±0.8 rad. Native force PIDs apply at most
4 Nm per joint; they do not set ideal joint velocity or body pose.
**Hold flippers / stop Drive** retains current motor targets and stops Drive.
Use the normal latched Drive panel for belts. A lost ROS owner leaves the
bounded native servos holding their last targets; the native Drive watchdog
removes stale belt commands.

On `nav_empty`, the articulated variant reaches approximately ±0.270 rad for
±0.3 rad requests, settles for 1.016 simulated seconds and drives the free
body forward 0.3005 m. [Receipt](../status/evidence/native-forms-2026-10-30/README.md).
Its SLAM/localization/Nav2 launch paths and commands are integrated, but the
original fixed-flipper Nav2 result does not qualify articulated terrain or
navigation. Those missions remain pending.

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
