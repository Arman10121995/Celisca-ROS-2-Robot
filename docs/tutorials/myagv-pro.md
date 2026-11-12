# Elephant Robotics myAGV Pro

The normal installed catalog now includes the complete official myAGV Pro
CAD base, from Elephant Robotics' BSD-3-Clause
[ROS 2 Humble source](https://github.com/elephantrobotics/agv_pro_ros2/tree/d3851c768d2e887eef7ad79fc4dcb9a928b070ac).
It is a four-wheel omnidirectional robot. Its source wheel names do not match
their geometric corners; the adapter resolves those positions explicitly.
It is grouped as one complete robot family, rather than listing its wheel
and sensor parts separately.

## Run

1. Open **Launch → Mobile robots → Four-wheel mecanum base**.
2. Choose **Elephant Robotics myAGV Pro** and its official CAD variant.
3. Choose **Gazebo**, **Display** and `nav_empty` for the measured starting case.
4. Run the automatically filled command. Use the Drive column for forward,
   reverse, turning, strafe, WASD/joystick and Stop. Existing latched button
   behaviour and incremental speed controls apply.
5. For localization, SLAM, 3D SLAM or navigation, first enable **Experimental
   imported-asset modes**. Algorithms and commands fill automatically. These
   pipelines remain experimental; a named physical screen is not universal
   map or mode qualification.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_elephant_myagv_pro simulator:=gazebo \
  mode:=display map_name:=nav_empty
```

For experimental navigation, use `mode:=nav allow_experimental_assets:=true`.
Choose a land world with its matching registered 2D occupancy map. Display can
also select no map. Other simulator control adapters are not advertised as
working by this integration.

The existing Registry preview opens this complete source model inside the GUI.
**Learn & diagnose → Documentation library** includes this page. Health uses
the same task ledger as the roadmap; prototype pipelines and measured missions
remain distinct.

## How control and sensing connect

```mermaid
flowchart LR
    GUI[Drive / WASD / joystick] --> Mux[Twist mux]
    Nav[Nav2 commands] --> Mux
    Mux --> Relay[Stamped twist relay]
    Relay --> Control[Native holonomic controller]
    Control --> Wheels[Four wheel velocity interfaces]
    Wheels --> Physics[Gazebo native physics]
    Physics --> Joints[Measured wheel and passive-joint states]
    Joints --> Control
    Joints --> TF[Robot state publisher / RViz]
    Physics --> Sensors[LiDAR / IMU / RGB-D]
    Sensors --> Bridge[ROS bridge]
    Bridge --> Estimate[Wheel-IMU estimation / AMCL / mapping]
    Estimate --> Nav
    Physics --> Truth[Separate ground-truth odometry]
```

The physical contact model drives the body; neither the controller nor its
odometry moves the root pose. Command loss expires the wheel controller's
wall-clock lease. A common scale factor bounds combined forward/strafe/yaw
commands at 8 rad/s per wheel while preserving their twist direction.

The source visual meshes, original body masses/inertias and sensor mounts
are retained. The lab declares these simulation changes:

- Cylindrical wheel envelopes of 0.0762 m radius and directional contact
  replace rigid STL roller collisions. The vendor's Gazebo sample uses
  0.05 m despite the approximately 0.152 m CAD wheel diameter.
- Four passive vertical compliance joints distribute tire contact. They have
  ±12 mm travel, 10,000 N/m stiffness and 100 N·s/m damping, with 0.02 kg of
  added carriers. These are lab approximations, not vendor suspension claims.
  Their measured states are published for correct wheel TF; they are not driven.
- Conservative simulation limits are 5 N·m motor effort, 8 rad/s wheel speed,
  0.3 m/s per translation axis and 0.4 rad/s yaw. These are not hardware ratings.
- Native simulated LiDAR/IMU/RGB-D instruments use the original source frame
  mounts. The camera is a declared lab sensor model, not a calibrated vendor
  camera. The broken joint/controller names in the vendor Gazebo example are
  replaced by the actual source joint names and the existing bounded controller.

The source CAD model has about **20.316 kg**, while the
[hardware product table](https://github.com/elephantrobotics/myagvPro_docs/blob/d2b23d72fa92ce322e9c39617ca04d40685c6f62/MYAGV_PRO_EN/2-ProductFeature/2.1-BasicEdition.md)
lists 40 kg. The discrepancy is unresolved. This integration preserves the
published model rather than presenting it as calibrated hardware dynamics.

## Measured scope and remaining work

One actual Gazebo `nav_empty` Display run measured forward/reverse, 0.25 m/s
strafe, about 0.234 rad/s turn, neutral hold and command-loss stop. Eight real
camera frames, eight LiDAR messages and measured drive/passive-joint feedback
were recorded. Publisher-loss travel was about 0.11 m with zero final speed.

Navigation receipts and negative runs are kept in the
[integration evidence](../status/evidence/myagv-pro-2026-11-12/README.md).
Other maps, furnished obstacle routes, reset/loss histories, calibrated tire
and mass dynamics, other backends, hardware serial/charging operation and
the manual's arm-equipped composites remain separate work. The Basic and
Visual-Navigation product editions are not duplicated as falsely qualified
simulation profiles.

## Install on another host

Assets are already downloaded on this host. Provisioning is an installation
step, with no GUI download requirement:

```bash
python3 scripts/provision_myagv_pro.py --download
```

Source checkouts, CAD assets, generated URDF/SDF, license, resource hashes,
contact/control recipe and installation receipts stay under
`$ROBOT_LAB_RUNTIME_ROOT/external_assets/` on the SSD. Re-provisioning rejects
edited source checkouts. The incorrect incoming two-wheel prototype is
preserved in the SSD integration directory; it is not registered as myAGV Pro.

For agents: keep R14.2 ahead of other implementation tasks. Preserve the
geometric corner mapping, source CAD/inertias, source sensor frames, observed
passive joints, compiled contact-direction checks and matching Nav2 goal
thresholds. Portable wheel physics is required before enabling another
backend. Use real body/sensor feedback when adding support claims.
