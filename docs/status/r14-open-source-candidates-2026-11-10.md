# R14.1: open-source candidates for the self-made robots (2026-11-10)

## November 12 source decisions

The initial implementation/search pass is complete. TurtleBot4 cleaning,
WAM-V, Summit-STEEL and the MIT source octopod each have a named physical
screen. Wider mission acceptance is separate. BlueROV2 remains a local
experimental candidate: package-only MIT metadata does not resolve its absent
license text and third-party mesh attribution chain. Adoption/distribution is
on hold under R9.1. Continue R6.7 initial implementation next.

| Existing robot | Decision in this bounded source review |
|---|---|
| Vacuum | Installed separate source TurtleBot4 cleaning; one zone/dock measured |
| USV | Installed complete Apache-2.0 WAM-V H; upstream water equations ported to Fortress |
| ROV | Native-force BlueROV2 Heavy candidate installed; retain local experimental status pending asset licensing |
| Mecanum | Complete BSD-3-Clause Summit-STEEL source plant and declared sensing kit installed |
| Octopod | Complete MIT Light Robotics 7.5 kg/27-servo native plant and scripted gait installed; actual MuJoCo control/joint/TF screen |
| Tracked/flippers | Already imported Apache-2.0 Gazebo Sim source model and declared articulated derivative. Seven link and six joint subtrees match exactly. The earlier self-made/no maintained Sim model classification was wrong; this origin audit adds no runtime certificate |
| Ackermann/rear/anti-steer | Retain benchmarks. Original Racecar description license is TODO and has no root text. The ROS2 mirror has MIT text for its author but shares 36 mesh blobs, including chassis/wheels, with that original; model-asset licensing remains unresolved |
| 4WS | Retain benchmark. Ranger model packages have template author/license metadata without license text; its separate BSD driver is not a CAD license. MIT omni4WS is a real alternative, but its steering/rolling links have coincident complete wheel colliders and lack physical effort limits. It needs a declared plant redesign/envelope review |
| Mobile snake | Retain benchmark. Slither has package-only MIT; thesis packages have template BSD/TODO authors; the common seven-joint model is world-fixed. OPPAS supplies CC-BY design/CAD without a complete bounded simulation plant in the inspected tree |
| Amphibious | Retain authored wheeled land/water benchmark: reviewed Polymander research describes a different legged form and did not establish a suitable licensed wheeled plant |
| Centipede | Retain twelve-leg/six-segment benchmark: no complete matching licensed plant found in this bounded pass. The source octopod is an eight-leg segmented alternative, not a twelve-leg replacement |
| Quadrotor display | Already superseded by real PX4 X500; preserve exact flight certificates |

Exact source pins/reasons are in [the source catalog](asset-sources-2026-10-05.yaml).
[Identity audits and actual receipts](evidence/counterparts-2026-11-12/README.md)
retain the distinctions between origin checks, local candidates and physical screens.
These outcomes do not claim that no other models can exist.

Primary sources:
[octopod](https://github.com/light-robotics/robot_rl_lab/tree/c2cc2437393155760eeea5f588c5912f4654067c/robot_rl/bodies/octopod),
[tracked example](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/tracked_vehicle_simple.sdf),
[Racecar original](https://github.com/mit-racecar/racecar_gazebo/tree/effd29f9edc20a03c8a1f666fb6fb6c0cbbee8ff),
[ROS2 mirror](https://github.com/cihataltiparmak/mit_racecar_gazebo_ros2/tree/77cea07f1c15e9ee7117ba2e178467af0bccc8ce),
[Ranger models](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/27633a956c845903ee630538afeb17fe70afdd84),
[omni4WS](https://github.com/kshibata-g1412049/omnidirectional_four_wheeled_robot/tree/b40ae465683c4d2889c45e07936f658bd364a02a),
[Slither](https://github.com/daniel-s-ingram/slither/tree/ceb388f0bd0b3a37a4edfeb82a97ee0303340711),
[snake thesis](https://github.com/vinicius-cgomes/TG-snake-robot/tree/52ea9b436af31334de14e2fdf8649ddc083ea8d7),
[world-fixed snake](https://github.com/guzhaoyuan/snake_robot/tree/631d758680fde32e0dcd08af173ffad458d07fc2),
[OPPAS](https://github.com/FabReyesMecha/OPPAS/tree/1e7cd22f98b3faee9cc5d65deea4993864f39f84),
[Polymander](https://www.epfl.ch/labs/biorob/research/amphibious/polymander/).

## Historical November 10 first pass


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

VRX WAM-V H is now installed as `asset_vrx_wamv_h`, using the pinned Apache-2.0 upstream hull/propellers/water equations with a private Fortress ABI port. One physical source control screen completed; see [guide](../tutorials/wamv.md). Continue BlueROV2 and Summit-XL next.

Summit-STEEL now has a source-preserving native mecanum integration and one measured Gazebo Display control/sensing screen; [guide](../tutorials/summit-steel.md). It does not replace a 4WS robot. Source-search decisions for cars, 4WS and unusual forms remain, plus the BlueROV2 source-license review.

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
