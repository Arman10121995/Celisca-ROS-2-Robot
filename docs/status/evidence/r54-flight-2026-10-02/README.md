# R5.4 measured PX4 X500 flight — October 2

Host: Jetson AGX Orin, Ubuntu 22.04, ROS Humble arm64, Gazebo Harmonic 8.15.
Robot Lab baseline `2a0aae2` plus recorded source hashes. PX4 upstream
v1.16.2 revision `54f0455ffcd755534539a7cf33a09a20bf71d29d`, unmodified FCU
source, native X500 motor/sensor model; pymavlink 2.4.50 in the SSD venv.

The old `0xFC7` MAVLink mask ignored XYZ and requested zero velocity: arming
correctly held the vehicle on the ground. `0x9F8` requests position and yaw;
waypoint XYZ are forwarded instead of hard-coded zero. Native pinned PX4
startup inserts the model and flies it without the manual include flattener.
The earlier claim that the motor plugin produced no thrust is superseded.

`fcu-flight.json` records `acceptance-run4`: normal arming checks, takeoff,
6 s settled hover at 3 m, three changing NED waypoints with 3D errors
0.084/0.071/0.036 m, a six-second setpoint-stream loss, Land and automatic
disarming. Offboard-loss parameters were read back as Land (4) and 0.5 s.
All acceptance fields pass; the CLI exits zero only for measured acceptance.

`ros-flight.json` records the actual bringup/ROS service path in domain 190,
map `nav_empty`, GUI and RViz off:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py robot_model:=px4_x500 \
  simulator:=gazebo mode:=flight map_name:=nav_empty gui:=false start_rviz:=false
python3 scripts/px4_ros_check.py --out /workspace/molar/robot_lab_runtime/r54-flight-2026-10-02/ros-flight-final.json
```

Independent Gazebo base-link truth is stationary and unarmed during idle
input. Height rises from 0.227 to 3.194 m. The 3D goal is reached within
0.25 m; a 0.4 m/s body-forward command moves the body 1.105 m. Release holds
within 0.192 m over the following three seconds. Land returns to 0.227 m and
disarms. Measured rotor joint speed reaches 88.05 rad/s (Gazebo's visual
rotor slowdown is 10). All seven ROS acceptance fields pass.

The pose publisher includes the upstream merged base-link offset of 0.24 m.
FCU NED origin is aligned once to configured spawn; truth is not controller
feedback. The preceding successful prototype is retained as
`ros-flight-before-frame-fix.json`, with its known pose-origin mismatch.
Two native relay attempts failed on integer JSON-to-ROS float fields; one
repeat accidentally used a copied stale installed module. A first checker
attempt used ROS Node's reserved `clients` property. These were fixed before
the final run; failed raw logs remain on SSD.

Raw flight `.ulg`, full logs and failed attempts stay under
`/workspace/molar/robot_lab_runtime/r54-flight-2026-10-02/` and
`/workspace/molar/robot_lab_runtime/px4/ros-flight-*`. Cleanup ended the launch,
FCU, native Gazebo and robot-state publisher; internal storage remains 70%
used. GUI tests record 39 passing checks, including Flight autofill, backend
correction and neutral enable; protocol/frame/capability/resolver checks
record 55 passing tests. Broader fast-suite and later navigation results are
tracked in the continuation report.

This completes the named SITL flight integration acceptance. It does not
qualify obstacle-aware drone navigation, aerial SLAM, other maps/backends,
hardware operation, concurrent FCUs, long hover drift or exhaustive failsafes.
