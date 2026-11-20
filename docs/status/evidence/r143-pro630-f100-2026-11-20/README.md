# Pro630 with F100 gripper in MuJoCo

One normal Robot Lab GUI run on the Jetson used
`asset_elephant_myagv_pro_pro630_f100`, MuJoCo, `nav_empty` and Display.
The **Mobile arm → Gripper** Open and Close buttons drove the actual shaft
and five observed followers. The operator used **End session** and the owned
launch exited cleanly. The raw values and exact generated launch command are
in [the GUI report](gui-gripper-result.json). [Runtime hashes](runtime-sha256.txt).

| Command | Target | Measured leader | Largest settled follower error | First within 0.03 rad |
|---|---:|---:|---:|---:|
| Open | 1.000 rad | 0.999997 rad | 0.0000068 rad | 2.21 s |
| Close | 0.000 rad | 0.000003 rad | 0.0000071 rad | 2.23 s |

## Live flange planning and CAD pads

Both variants completed one **Plan pose → Execute plan** move in the normal
MuJoCo GUI on `nav_empty`. The tests used **Position only** and judged the
measured flange endpoint.

| Variant | Measured move | Endpoint error | Receipt |
|---|---:|---:|---|
| Bare Pro630 | 8.34 mm | 0.115 mm | [GUI report](gui-flange-bare-result.json) |
| Pro630 + F100 | 8.42 mm | 0.032 mm | [GUI report](gui-flange-f100-result.json) |

The collision planner now records the deepest resting contact for each
collider pair. It continues to reject new or deeper penetration. These
profiles use 20% of the generic planned slew limit. An earlier F100 move
was interrupted by the snapshot guard. [Interrupted run](gui-flange-interruption.json).

The current F100 derives the mount and finger hulls and separate pads from
the pinned jaw CAD. The calibration samples the inner pad separation at 41
shaft angles. The nominal grasp frame is centred between the pads at
0.35 rad, where the calculated gap is 40.6 mm. **Set opening (mm)** uses the
same interpolation as the estimated jaw readout. Grasp, transport and
other F100 backends remain open. In the live `nav_empty` GUI the 54 mm
command settled at a shaft angle of 0.4627364 rad, corresponding to an
estimated 54.0000 mm opening. [GUI opening report](gui-calibrated-opening-result.json).

An isolated loaded fixture lost its block on full closure. A metered
closure reached bilateral pad contact in 226 of 250 sampled physics
steps at the end of the close stage. When the fixture rose 44.2 mm,
the loose block stayed on its support. [Full closure](initial-cad-pad-load-bench.jsonl) · [metered closure](metered-pad-closure-bench.jsonl).

The live articulation state now reports pad normal load by free object
name, and the Gripper panel shows the two readings. The force reader passed
a compiled MuJoCo fixture with both pads acting on one free box and a
second free box touching only the floor. These readings support future
custody checks for the selected payload.

## Experimental short-block fetch

The `mobile_fetch_lab_f100` map variant retains the original tables and
floor. The 40 g block is 90 mm tall and sits 60 mm inside the pickup edge.
The private MuJoCo scene finds an open, approach, descend, close and lift
path at the 0.47 m standoff. This establishes kinematic feasibility.
The first GUI attempt refused to start after private model import made
its cached odometry stale. The pick/place node now compiles the private
model during startup before subscribing. The GUI waits for the feedback
the node needs before enabling the fetch button.
[Private planner output](short-block-private-plan.txt).

The next GUI trial docked and reached closure before it was interrupted
when a second simulator qualification started. A separate trial then
docked at `[0.8836, 0.0]` m, descended and closed. The block shifted
about 38 mm sideways during closing. Both measured pad loads were zero
when the controller stopped it before lift. This is a retained failed
fetch. The candidate now meters the jaw reference at 0.10 rad/s during
automated closing. Direct Open and Close use their existing limits.
[Interrupted run](gui-fetch-interrupted.json) · [Closure result](gui-fetch-stopped-at-closure.json).

## Source and retained diagnostics

The bare arm and its wheel/arm measurements use the existing source pin.
The initial F100 mount, collision meshes and linkage came from the vendor Pro630
Gazebo description at `b8176c264e583b073e1c4293c2edb1241b20e1aa`. Three
follower joint coordinates reverse sign with their axes to preserve the
same spatial linkage in the shared positive-ratio contract. One MuJoCo
position actuator drives the five passive equality joints. Lab caps are
0.8 Nm and 0.5 rad/s. The vendor package declares BSD and the mesh package
contains the BSD-2-Clause license. [Vendor source](https://github.com/elephantrobotics/mycobot_ros2/tree/b8176c264e583b073e1c4293c2edb1241b20e1aa/mycobot_pro/mycobotpro630_gazeboros2).

The initial fixed-base jaw bench reached its targets but showed 0.267 rad
maximum follower error with the default soft constraint. With the final
coupling, the bench targets 0.8 and 0.05 rad both settled to the printed
precision, with 0.00552 rad maximum follower error during motion and
0.240 Nm peak motor effort. The first GUI trial reached model import and
was stopped after the launch rejected the optional planner channel list.
The later launch gate accepts that list and validates it in the planner.
GUI cleanup also keeps ownership when publishing neutral fails after ROS
context shutdown.

79 focused source and GUI checks and two targeted shutdown checks passed.
The contact, aperture, planner and GUI follow-up passed 36 focused checks
and built successfully. Provisioning logs, the bench probes, screenshots and
failed and passing launch outputs are on SSD under
`$ROBOT_LAB_RUNTIME_ROOT/pro630-gripper-20261120/`.

## Reproduce the GUI gripper screen

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
export ROS_DOMAIN_ID=143
export IGN_PARTITION=robot_lab_pro630_f100_gui_2 GZ_PARTITION=robot_lab_pro630_f100_gui_2
xvfb-run -a python3 scripts/qualify_urdf_arm_gui.py \
  --robot asset_elephant_myagv_pro_pro630_f100 --simulator mujoco \
  --map nav_empty --gripper-presets --output "$ROBOT_LAB_RUNTIME_ROOT/pro630-f100-repeat"
```
