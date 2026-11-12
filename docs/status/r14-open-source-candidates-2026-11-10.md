# R14.1: open-source candidates for the self-made robots (2026-11-10)

First search pass for [roadmap R14.1](../../ROADMAP.md#r141--open-source-counterparts-for-self-made-robots).
These are **candidates, not integrations**. Licenses marked *listed* come from
an index or package page; confirm each repository's own LICENSE file, pin a
revision, and check the model (inertias, collision, actuators, sensors) and
its Gazebo/ROS 2 Humble compatibility before provisioning.

| Self-made profile | Candidate | License | Fit and open questions |
|---|---|---|---|
| `oomwoo` (vacuum) | iRobot Create 3 (`iRobotEducation/create3_sim`: `irobot_create_description`, `irobot_create_gz_*`) | BSD-3-Clause (listed on index.ros.org) | Round robot-vacuum-class base with bumper, cliff and IR sensors. The lab already installs TurtleBot 4, whose base is the Create 3. Check its Gazebo version (Ignition/Classic) and sensor plugins. Best candidate: run `robot_lab_vacuum_cleaning` on it (`supports_room_vacuum`) |
| `lab_usv` | VRX WAM-V (`osrf/vrx`) | Apache-2.0 (listed) | VRX 2.x moved to the new Gazebo (Garden) and ROS 2; the lab's marine work uses Fortress 6.18, so a Fortress port or a Garden/Harmonic session is needed. Thrusters and wave/buoyancy plugins included |
| `lab_rov` | BlueROV2: `CentraleNantesROV/bluerov2` (Gazebo + hydrodynamics, ROS 2), `clydemcqueen/bluerov2_gz` (Harmonic), `bluerov2_ignition` (Garden, legacy), Orca3/Orca4 (MIT, listed) | mixed; Apache-2.0 for one mirror, others not shown | No Fortress-specific model found. Harmonic is the PX4 session's version here. Check licenses and thruster/hydrodynamics plugin versions |
| `mecanum_car`, `four_wheel_steer_car` | Robotnik Summit-XL (`robot_description` / `robotnik_sensors` ros2-devel; `easynav_playground_summit_*`) | BSD-3-Clause (model, listed) | ROS 2 Summit-XL description with ros2_control; Harmonic worlds. The mecanum variant is documented for ROS 1; confirm a ROS 2 mecanum (Summit-XL Steel/OMNI) description |
| `ackermann_car`, `rear_steer_car`, `anti_ackermann_car` | F1TENTH (`f1tenth/f1tenth_gym_ros`, ROS 2; the old `f1tenth_simulator` is Melodic); third-party Humble + Gazebo Sim Ackermann models (`alitekes1/ros2-ackermann-vehicle-gz-sim-harmonic-nav2`, `cihataltiparmak/mit_racecar_gazebo_ros2`) | unverified | No official F1TENTH Gazebo URDF for Humble found. The third-party models are small projects; check build, license and steering geometry. The rear-steer and anti-Ackermann variants have no known open-source model, so they likely stay self-made |
| `lab_tracked`, `lab_tracked_articulated` | none found: Gazebo Classic `tracked_vehicle_simple` example (ODE), ROS 1 projects | — | No maintained ROS 2 Gazebo Sim tracked/flipper robot found. Search Gazebo Fuel and robot_descriptions next |
| `lab_snake`, `lab_amphibious`, `lab_octopod`, `lab_centipede` | not searched in this pass (octopod/centipede: none found on 2026-10-14) | — | Search ROS index, Fuel, robot_descriptions, Menagerie |

## Already installed: Create 3 and TurtleBot 4 Lite

`/opt/ros/humble/share/irobot_create_description` (version 2.1.0, BSD) is
installed, with `bumper.urdf.xacro`, `sensors/cliff_sensor.urdf.xacro`, IMU,
IR sensors and the standard dock model (`dock/standard_dock.urdf.xacro`),
plus `irobot_create_ignition_*` (Fortress). The Create 3 has no LiDAR, which
the cleaning system needs for localisation. The installed and measured
`asset_turtlebot4_lite` is a Create 3 with a 2D LiDAR and a camera, so it is
the quickest open-source robot vacuum-class platform for R13.

## November 12 implementation update

TurtleBot4 cleaning is implemented as the separate `asset_turtlebot4_lite_cleaning` variant, with native source bumper/four-cliff bridges and required fresh cliff feedback. One actual Gazebo zone-clean and dock completed; see the [receipt](evidence/counterparts-2026-11-12/README.md). Original TurtleBot4 profiles/certificates stay unchanged. Other backends/contact/edge cases remain unqualified. The steps below are the original search plan; continue with WAM-V, BlueROV2 and Summit-XL.

## Original search plan

1. TurtleBot 4 Lite first: enable `supports_room_vacuum` with a `vacuum:`
   block (radius 0.17 m) on the installed profile, and map its Create 3
   bumper and cliff topics onto the `vacuum_cleaner.py` bumper/cliff
   parameters (create3_sim uses `/hazard_detection` and `/cliff_intensity`;
   check what the lab's TurtleBot 4 bridge publishes). Run one
   `scripts/sim_vacuum_mission.sh` job per backend. Keep `oomwoo` and its
   evidence.
2. Then WAM-V and BlueROV2, which need a Gazebo version decision (port to
   Fortress, or run them in a Harmonic marine session).
3. Then Summit-XL for the mecanum/omni class.
4. Record each outcome (adopted / rejected and why / none found) in the
   roadmap R14.1 table.

Sources: [osrf/vrx](https://www.github.com/osrf/vrx), [vrx on ecosyste.ms](https://awesome.ecosyste.ms/projects/github.com%2Fosrf%2Fvrx),
[create3_sim](https://github.com/iRobotEducation/create3_sim), [irobot_create_description](https://index.ros.org/p/irobot_create_description),
[CentraleNantesROV/bluerov2](https://tgfed.nhyvvfeijnh.workers.dev/CentraleNantesROV/bluerov2), [bluerov2_gz](https://relatedrepos.com/gh/clydemcqueen/bluerov2_gz),
[orca4](https://github.com/clydemcqueen/orca4), [easynav_playground_summit_description](https://index.ros.org/p/easynav_playground_summit_description/),
[summit_xl_description](https://index.ros.org/p/summit_xl_description/), [f1tenth_simulator](https://github.com/f1tenth/f1tenth_simulator),
[ros2-ackermann-vehicle-gz-sim-harmonic-nav2](https://awesome.ecosyste.ms/projects/github.com%2Falitekes1%2Fros2-ackermann-vehicle-gz-sim-harmonic-nav2),
[tracked robots in Gazebo](https://answers.ros.org/question/249874/tracked-robot-gazebo-ros).
