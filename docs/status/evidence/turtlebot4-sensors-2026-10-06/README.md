# TurtleBot 4 sensor and Drive checks — October 6, 2026

Baseline `61335fe`, with each exact pre-trial source/installed stage in the
case manifest. Jetson AGX Orin / ROS 2 Humble. Eight serial actual Tk GUI trials
passed with producer exit 0, owned launch exit 0 and no remaining owned plant.
These are physical engine body/joint observations, not wheel odometry alone.

Each trial uses normal installed `nav_empty`, Display, command autofill and
GUI Run. Enabling keyboard control emits zero commands; W/S/A/D produce
forward/reverse/both turns. Space/Stop settles below 0.025 m/s and 0.05 rad/s;
a silent publisher meets those same final-window limits. The window is 2.5
simulation seconds with the final 0.5 s average, not an instantaneous halt.

| Variant | Backend | Measured RTF | Depth messages | Independent body samples |
|---|---|---:|---:|---:|
| lite | gazebo | 0.891 | 141 | 1406 |
| lite | isaac | 0.665 | 136 | 2047 |
| lite | mujoco | 0.365 | 142 | 1274 |
| lite | pybullet | 0.503 | 140 | 1358 |
| standard | gazebo | 0.834 | 141 | 1412 |
| standard | isaac | 0.558 | 136 | 2445 |
| standard | mujoco | 0.263 | 142 | 1278 |
| standard | pybullet | 0.431 | 139 | 1357 |

The RPLidar scan has 640 rays and 0.164–12 m bounds. The OAK-D image is
320×240 with measured calibration and changing finite depth geometry. The
actual source-mounted lidar and optical TF positions agree within 10 µm.
All four wheel/wheel-drop joint names and finite changing physical state are
observed. The simulator adapters use lab sensor rates, not the full vendor
noise, hazard, docking or firmware stack. These RTF values describe this
headless run and host; MuJoCo performance remains a measured open gap.

The first repeated Standard/Gazebo trial met motion/sensor limits but its
producer exited 139 after bypassing normal GUI shutdown. It is retained and
excluded. The final producer closes through `_on_close()`. Later Live Monitor
work gives its worker a dedicated executor and destroys ROS handles after
spinning stops; its own actual ROS/Tk regression check is recorded separately.

Each case contains the actual producer, report and pre-trial hashes.
[manifest.json](manifest.json) records raw SSD trace/log/model hashes and
producer exit codes; [batch.json](batch.json) records the serial outcomes.
Large raw data remain at the named SSD paths. Physical joystick hardware,
other maps and full material parity remain unqualified. See the
[TurtleBot guide](../../../tutorials/turtlebot4.md).
