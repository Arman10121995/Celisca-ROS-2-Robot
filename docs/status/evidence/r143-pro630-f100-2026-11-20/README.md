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

The metered configuration reached the close stage through the normal GUI.
Another qualification started on the Jetson, so this trial ended during
closure. The owned launch exited with code 0. The metered configuration's
loaded lift and transport were measured in the later trials below.
[Metered trial](gui-metered-fetch-interrupted.json).

## Loaded custody and transport, November 21

Three subsequent runs used the normal GUI in MuJoCo Display. The first
metered run kept the block near its starting pose, but the jaw could not
meet the angular settling threshold. The next run used three fresh
bilateral force samples and a 3 mm block stability window to admit the
lift. The block still stayed on the table.

An isolated top-down fixture with the same CAD linkage tested effective
transmission inertia as a MuJoCo tuning parameter. Adding 0.003 kg·m² at
the shaft and 0.00015 kg·m² at each follower lifted the 40 g block by
43.54 mm with both pads in contact for 250 of 250 lift-hold samples.
Peak shaft effort was 0.293 Nm within the existing 0.8 Nm limit. The values
are Lab simulation tuning. [Bench sweep](cad-topdown-inertia-sweep.jsonl).

The full robot with that tuning lifted the block 46.3 mm unsupported.
Drift during the lift hold was 0.4 mm, with roughly 2.43 N on each pad.
It completed the straight carry leg to the drop table with a maximum
19.1 mm payload drift in the base frame. The place planner then rejected
the measured arrival pose with `no IK above the support`. The owned
launch exited cleanly. This run establishes loaded pickup and carry on
this fixture. Supported release remains open.
[GUI measurements, exact commands and terminal samples](gui-fetch-custody-2026-11-21.json).

The next GUI run shortened the place standoff from 470 to 445 mm. The
same block lifted 46.3 mm and reached that dock. The solver found IK over
the table, then rejected a contact with the rigid jaw behind the right pad.
Replaying its observed arm, jaw and object poses reproduced the collision.
The new planner uses the measured jaw geometry and preserves the initial
contact depth only for that held object against the pad-bearing links. Any
new or deeper contact still rejects a path. The private replay then planned
all four place stages, including retreat. This establishes placement
feasibility from that sample. [Planner replay](loaded-place-replay.txt).

A fixed-base MuJoCo physics replay initialized the loaded arm at that
recorded dock with a free 40 g block, then executed the planned path with
the same bounded actuators and transmission inertia. The pads retained
load above the support, unloaded on opening, and the block stayed fully
on the table after retreat. Final measured support height error was
0.11 mm. The initial loaded state came from run 8. This test qualifies
the place motion from that fixture and leaves the continuous GUI mission
as a separate check.
[Physical placement samples](loaded-place-physical-fixture.jsonl).

PyBullet now has a matching normal-force reader for named URDF pads. It
transforms Bullet's link-level contact points into the source collision
geometry so a contact on the same link outside the pad stays excluded.
This implementation has a focused regression test. In a fixed-base Bullet
fixture, the source linkage reached 1.0 rad open, 0.0 rad closed, and the
54 mm aperture reference within 0.000017 rad of each target. Maximum
follower skew during travel was 0.000399 rad. PyBullet Display is exposed
with experimental workflows enabled. Loaded Bullet missions remain open.
[Fixed-base travel samples](bullet-f100-fixed-base-travel.jsonl).

A normal PyBullet Display launch reached the arm page and accepted the
Open button request. A separate simulator qualification started during the
screen, so the owned launch was stopped. The qualifier reported an open
timeout and the launch exited cleanly. The fixed-base results are the
current Bullet travel evidence. [Interrupted GUI screen](bullet-gui-interrupted.json).

### Complete GUI fetch

Run 11 used the normal GUI on `mobile_fetch_lab_f100` with MuJoCo Display
and experimental workflows enabled. The F100 picked up the free 40 g block,
carried it to `drop_table`, released it on the support, withdrew the arm
and backed the base away. The owned launch exited with code 0.

| Measurement | Run 11 |
|---|---:|
| Unsupported lift | 46.3 mm |
| Payload drift during the lift hold | 0.5 mm |
| Maximum payload drift during straight carry | 19.1 mm |
| Final supported height error | −0.1 mm |
| Block movement during final rest check | 0.0 mm at reported precision |
| Base clearance after release | 0.572 m |
| Total task wall time | 170.24 s |

The result covers the short-block map with straight transport. The exact
GUI command, model hash, stage timings, pad forces, final poses and
termination are in [run 11](gui-fetch-custody-2026-11-21.json). The full
trace and screenshots are on SSD under `gui-fetch-11/` in the run directory.
The GUI records this Display measurement when the installed model hash
matches the receipt and its measured lift, custody, release and clearance
pass the recorded screen checks.

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
