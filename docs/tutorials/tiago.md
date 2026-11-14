# PAL Robotics TIAGo

The normal installed robot selector includes **PAL Robotics TIAGo**, profile
`asset_pal_tiago`. This is the complete official ROS 2 Humble PMB2 assembly:
differential base, torso lift, seven-joint arm, two independent parallel
fingers and pan/tilt head. Five pinned official PAL repositories supply the
description and dependencies; all five root licenses are Apache-2.0.
The [official public source manifest](https://github.com/pal-robotics/tiago_tutorials/blob/humble-devel/tiago_public.repos)
and [pinned TIAGo description](https://github.com/pal-robotics/tiago_robot/tree/f1c33c92bdde7c1dd79f0c3e739e98a233dbd30b)
explain the upstream package structure.

## Start from the GUI

1. Select **Mobile manipulators**, **PAL Robotics TIAGo**, its official
   PMB2/arm/gripper variant, **MuJoCo**, **Display** and `nav_empty`.
2. Run the automatically filled command. Let the base settle.
3. Use **Drive** for base motion and Stop. Use **Mobile arm** for all twelve
   position channels, including the torso, fingers and head.
4. Copy measured values before jogging. Enter or drag a target, then explicitly
   press Apply or −/+. Editing alone sends nothing. Finger values are metres;
   torso values are metres; arm/head values are radians.
5. Stop / hold, leaving the control page, changing selection or losing the
   operator heartbeat holds measured positions. Base motion inhibits joint
   commands, and moving joints inhibit Drive.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_pal_tiago simulator:=mujoco \
  mode:=display map_name:=nav_empty
```

PyBullet and Isaac have portable adapter entries too. Their individual
measurements and limitations are in the [receipt](../status/evidence/tiago-2026-11-14/README.md).
Display is the currently exposed workflow. The sensor topics alone do not
qualify SLAM, localization, Nav2, collision-planned arm motion or object tasks;
those remain under R5.9. Gazebo is unavailable for this variant: the official
modern Gazebo controller plugin needs a separate Fortress port.

## What stays original, and what the lab supplies

| Part | Contract |
|---|---|
| Source mechanics | Original link/joint transforms, axes, collision geometry, URDF inertials, public-simulation fixed casters and their low friction |
| Base | Original 0.0985 m wheel radius, 0.4044 m separation and 6 Nm limit; lab caps 0.2 m/s and 0.4 rad/s |
| Arm / head / fingers / torso | Original position ranges and effort limits; lower lab joint rates, bounded position PD and plant gravity support; opt-in native PhysX generalized compensation on Isaac |
| Fingers | Independent source joints; no invented mimic or coupling |
| Home | Official motion's final arm/torso endpoint, head zero and lab 0.04 m finger opening; explicit operator command |
| LiDAR | Declared panoramic lab scan at `base_laser_link`, not the source SICK field of view |
| RGB-D | Source head camera mount and horizontal field of view; lab 320 × 240 / 5 Hz, 0.3–8 m clipping |
| IMU | Ideal simulated root observer, explicitly labelled `base_footprint` |
| Not exposed | Source sonars, wrist force/torque sensor and physical robot drivers |

**Home is an endpoint target, not the vendor's complete timed waypoint path
or a collision-checked plan.** Manual joint controls do not provide obstacle
avoidance. R5.9 must add source-specific planning, grasp/custody and payload
footprints before autonomous manipulation/transport is advertised.

The source description totals **84.66799184112 kg**; this is a model sum,
not a claim about hardware weight. Its two antenna inertia tensors are
indefinite and two tool frames have zero tensors. The source URDF is retained,
but existing backend importers regularize these and may bound tiny inertias.
MuJoCo compiled 84.668099800 kg and excluded three overlapping rest-pose
contact pairs. These are declared import approximations. Do not call this
calibrated or identical dynamics. In the short MuJoCo drive screen, a
0.12 m/s request reached about 0.085 m/s; source joint friction/damping and
the original effort envelope remain intact.

Isaac explicitly binds the source scalar link-friction values to original
colliders, including fixed caster frames and instanced collision meshes.
The adapter uses minimum friction combination so the 0.01 caster friction
can remain low against an ordinary floor; a separately authored floor with
maximum combination can override this and needs its own contact check. This
is a declared scalar contact port, not Gazebo solver equivalence.

The wheel-joint port also preserves the source **2 Nm** Coulomb friction and
**1 Nm/(rad/s)** damping using native axis friction efforts. The importer had
treated the URDF friction number as a dimensionless bearing coefficient.
Static/dynamic effort are both set to the source's single friction value;
viscous damping converts to the SDK's angular degree units. The original
6 Nm motor envelope remains unchanged. See the
[native axis friction units](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/dev_guide/joints/physx_joint_schema.html).

Isaac's previous fixed-slide gravity estimate cannot support the horizontal
arm. This profile explicitly opts into native PhysX actual-pose generalized
gravity compensation each physics step. Feedforward replaces the old slide
estimate and reserves its force from the same original effort envelope; it
does not add an unlimited force. Missing/non-finite native feedback stops the
SDK control path. Other robot profiles retain their existing default. The
failed startup-interlock and slide-only-gravity trials are retained in the
receipt.

The native tensor's floating-base layout starts with six root-wrench entries,
then joint forces. The adapter handles that layout explicitly; the host's
deprecated wrapper truncates/indexes it as a joint-only vector. Fixed and
floating layouts have separate checks. See the
[PhysX gravity-compensation contract](https://nvidia-omniverse.github.io/PhysX/physx/latest/_api_build/classPxArticulationReducedCoordinate.html).

The camera rides the moving head: MuJoCo attaches it to the nearest retained
URDF ancestor, while Isaac selects only rigid ancestors after fixed-link
fusion. Missing movable carriers disable/fail the camera path instead of
pretending a fixed base camera matches moving-head TF.

## Provision or extend the source adapter

```bash
source scripts/ssd_env.sh
python3 scripts/provision_tiago.py --download
```

This downloads immutable pins to SSD, expands Xacro through a private
source-backed ament prefix, validates all fourteen command interfaces and
twelve position envelopes, resolves and hashes every mesh, copies license
notices and incrementally registers one complete robot. It does not install
vendor hardware drivers or modify their checkouts. Re-running preserves other
robots and worlds. The separate `integration.json` records the exact source
pins, resource hashes, source Home endpoint, degenerate inertials and lab
adapters.

Entry points are `scripts/provision_tiago.py`,
`robot_lab_utils/tiago_source.py`, the existing `urdf_articulation` controller
and GUI `urdf_arm_controls.py`. Follow the contributor course for new robot
and map adapters; keep any future Fortress or manipulation variant's support
and evidence separate from this Display profile.
