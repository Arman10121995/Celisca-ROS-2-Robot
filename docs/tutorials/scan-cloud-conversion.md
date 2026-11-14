# Scan and point-cloud conversion

## Run

The Perception algorithm `laser_scan_to_pointcloud` now starts the official
Humble `laserscan_to_pointcloud_node`: `/scan` is its input and `/cloud` its
output. Choose it in Launch's algorithm options; the normal command preview
includes the selection. It converts the measured rays into XYZ points in the
scan frame. It does not add vertical observations to a 2D laser.

The converter subscribes lazily: a PointCloud2 consumer must subscribe to
`/cloud` before it processes `/scan`. Set RViz's PointCloud2 display to that
topic, or connect the downstream perception component. The
[upstream Humble interface](https://github.com/ros-perception/pointcloud_to_laserscan/tree/humble)
documents `target_frame`, `queue_size` and `transform_tolerance`. An explicit
target frame needs valid timestamped TF.

On a normal ROS installation, the bringup package declares
`pointcloud_to_laserscan` as a runtime dependency. On this Jetson, it can also
be provisioned without root onto the mounted SSD:

```bash
source scripts/ssd_env.sh
python3 scripts/provision_perception_runtime.py
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
ros2 pkg executables pointcloud_to_laserscan
ros2 run robot_lab_gui robot_lab_gui
```

The provisioner checks the configured apt repository's SHA256, records exact
package versions, retains notices from the binary package, and publishes a
completed overlay under `$ROBOT_LAB_RUNTIME_ROOT/perception/current`.
It changes neither `/opt` nor dpkg's installed-package database. Non-ROS
dependencies must already be supplied by the host package manager. Source
`ssd_env.sh` after ROS/workspace setup in the shell that starts the GUI;
running the provisioner alone does not alter an already open GUI's environment.

For a standalone converter with real input and an active output subscriber:

```bash
ros2 run pointcloud_to_laserscan laserscan_to_pointcloud_node --ros-args \
  -r scan_in:=/scan -r cloud:=/cloud
```

The November 14 check used an actual ROS node and three synthetic laser rays;
the output points agreed with independently calculated XYZ within 0.14 µm.
That checks the installed conversion interface. It does not qualify a robot's
sensor calibration, a perception algorithm or a navigation mission.
[Receipt](../status/takeover-2026-11-14.md).
