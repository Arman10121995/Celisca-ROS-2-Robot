# MBARI Tethys native AUV adapter

The original model is **MBARI Tethys LRAUV**, Fuel owner `accurrent`, model
author **Mabel Zhang / Open Robotics**, Fuel version **3**:
[source model](https://fuel.gazebosim.org/1.0/accurrent/models/MBARI%20Tethys%20LRAUV).
It is licensed **CC-BY-4.0**; [license](https://creativecommons.org/licenses/by/4.0/).
Original `model.config`, SDF, Collada meshes, textures, materials and thumbnails
are retained unchanged under `source/`, along with actual Fuel metadata.
The pinned archive SHA-256 is
`81cded5384f86c77afa1afa56050f024463379db912c33a60d938d27d28f4a15`;
original SDF SHA-256 is
`a9e6b39850139ce93cbe7b3198cc5e094541a28b81b14553b4db62ccca98ae16`.

`source_physics/auv_controls_fortress.sdf` retains the Apache-2.0 Gazebo
Fortress example supplied with `libignition-gazebo6 6.18.0-1~jammy`:
[upstream example](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/auv_controls.sdf).
See [Gazebo license](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/COPYING).
This supplies native thruster, fin force PID, LiftDrag and hydrodynamics laws.

Robot Lab's **derived adapter changes** are explicit:

- Original hull/link mass, inertia, collision volume, fin axis/ranges and
  geometry remain. Original complete mass is 148.3571 kg and collision
  displacement is 0.1483571000002 m³. At the installed worlds' 1025 kg/m³,
  an internal 3.7089275 kg sealed trim ballast is added beneath the source
  center of buoyancy. It has no collision/displacement volume. Source mass
  is not silently rescaled; total derived mass is 152.0660275 kg.
- The bundled example's negative added-mass derivatives predate the installed
  force convention. Fortress 6.18 applies **−Ma × acceleration**. Derived mass
  diagonals are positive; signed drag stays negative and coefficients scale
  with fluid density. Added mass and Coriolis remain enabled. The original
  example remains unchanged. [Native implementation](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/hydrodynamics/Hydrodynamics.cc).
- Propeller force is bounded to ±35 N. The originally unlimited rotor gets
  declared lab limits of 2 Nm / 100 rad/s for native velocity control.
  Original fin ranges ±0.261799 rad remain; fin joint force PID is bounded
  to ±2 Nm. Perfect-position mode is disabled.
- Native body/joint/IMU observations are bridged. The display URDF extracts
  the four named visual submeshes with source node transforms, units and
  joint poses; it does not repeat a complete hull for each fin. Its material
  is simplified. Native Gazebo retains the source textured visual model.
- Original front is −X; whole-vehicle spawn yaw and guidance convert that
  convention. Source link/joint axes are retained. Long-body tank/reef spawns
  are explicit per-map settings. Other water worlds retain their selected
  submerged spawn, with source/body clearance qualification pending.

The authored controller is experimental cruise guidance on this simulation
model, not an MBARI hardware autopilot. It commands propeller/fins and uses
actual simulator poses plus real joints. Ideal pressure is labeled emulation.
Fin depth/heading guidance requires forward flow; there is no strafe, pivot
or depth hold at zero speed. Stop zeros propulsion and neutralizes fins;
physical coasting continues. No independent underwater estimation, DVL/sonar,
SLAM/navigation, other backend or complete mission acceptance is claimed.
