# Pro630 with F100 gripper

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
timeout and the launch exited cleanly. The fixed-base run had already measured full travel. [Interrupted GUI screen](bullet-gui-interrupted.json).

An uninterrupted repeat quantified the failure. The jaw held partway
open after a GUI feedback gap, and the control lease expired. PyBullet now
renders RGB-D only when an image consumer is present. Calibration-only
subscribers still get CameraInfo. With this change, a normal GUI run on
`nav_empty` reached both presets with all five followers within 0.000084
rad of the shaft. The measured 54 mm request yielded 53.997 mm. The owned
launch exited cleanly. The badge checks these measurements against the
selected model identity and applies to the unloaded Display control screen.
[Feedback-hold trial](bullet-gui-feedback-hold.json) · [Measured PyBullet GUI presets](bullet-gui-gripper-confirmed.json).

The normal PyBullet short-block fetch docked and descended to the pickup.
The block moved 18.8 mm during closure. Across 51 close-stage samples, the
left pad stayed at zero, the right pad peaked at 1.14 N, and the largest
follower error reached 0.089 rad. The task timed out before attempting
lift. Friction anchors now apply to the links carrying the actual pads.
Timeouts report the measured pad loads and follower error. The solver
iteration setting remains a private candidate for the loaded retry.
[Loaded Bullet trial](bullet-gui-loaded-closure-timeout.json).

In the last measured close sample, the source CAD replay showed the
left pad 7.2 mm away from the shifted block and the right pad 3.4 mm away.
The experimental recovery opens the hand, checks a vertical retreat and
plans again from the latest collision pose. It can do this once when the
object is still supported and has shifted 5–50 mm. Camera-only grasps keep
their original perception binding. A replay of the failed Bullet state
found an open, retract, approach and lift plan. Live recovery remains open.
[Measured-pose planning replay](bullet-grasp-reapproach-plan.json).

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

### Navigation follow-up

The first uninterrupted Navigation trial reached the pickup, lifted the
block 42.9 mm, and got the loaded footprint acknowledged by both local and
global costmaps. It stopped just after entering Nav2 transit when the held
arm's motion registered in the base interlock. The prior controller treated
that transient as an unconditional failure. The new path keeps the exact
Nav2 goal owned through a held-arm pause and leaves velocity inhibition to
the simulator. Active arm commands, coupling disagreement and lost payload
or footprint feedback still stop the task. The F100 Nav2 overlay uses the
source base outline, with inflation above the measured loaded inscribed
radius and DWB rates bounded for the payload workflow. The DWB footprint
critic checks each sampled pose against the active polygon.
[First Navigation trial](nav-gui-held-interlock.json) · [Nav2 footprint critic](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/trajectory_critics/obstacle_footprint/).

The next Navigation trial retained both pad contacts through its 240 s
carry timeout. The loaded base reached `(0.625, 0.627)` m and 1,377 of 2,238
drive ticks were held by the interlock. The running DWB server reported
generic limits of 0.30 m/s for planar and lateral speed. The launch now
passes explicitly declared mecanum DWB limits as ROS parameter overrides.
An isolated Nav2 launch with static frame transforms returned the intended
0.15 m/s planar cap, 0.10 m/s lateral cap, and 0.20 m/s² lateral acceleration.
A normal GUI retry observed those same limits in the running controller
and a 0.10 m/s lateral command during loaded transit. The operator stopped
the retry when another qualification started. Nav2 arrival and release
remain unmeasured with the corrected limits.
[Carry timeout](nav-gui-transit-timeout.json) · [Effective DWB limits](f100-dwb-effective-limits.json) · [Live loaded transit observation](nav-gui-live-limits-interrupted.json).

The uninterrupted run with the declared limits lifted the block 42.7 mm.
Both costmaps acknowledged the footprint. Both pads still measured about
2.43 N after 240 s of carry, at base position `(0.389, 0.644)` m. The
interlock held 1,554 of 2,403 drive ticks. In the sampled held-arm
intervals, shoulder yaw moved up to 0.0131 rad within 0.25 wall seconds.
Every other joint stayed below 0.0019 rad in the same sample windows.
At the observed carry pose, the compiled inertia gives estimated critical
damping of 11.1–17.9 N·m·s/rad under free-to-locked joint assumptions.
The MuJoCo Navigation profile now uses 12 N·m·s/rad at the existing
shoulder servo, within its original 5 N·m effort cap.
[Uninterrupted GUI timeout](nav-gui-declared-limits-timeout.json) · [Joint trace summary and inertia calculation](f100-shoulder-motion.json).

### Complete GUI Navigation fetch

The normal MuJoCo Navigation GUI completed the 40 g short-block fetch in
218.17 s on `mobile_fetch_lab_f100`. The live plant reported the revised
shoulder damping and the 5 N·m effort cap. The block rose 46.4 mm, held
within 0.3 mm during the lift check, and reached both costmaps with about
2.44 N on each pad. The loaded Nav2 action ran for 142.49 s. AMCL reached
the handoff radius and the task waited for that action's terminal
cancellation and a measured base stop. The remaining docking leg used
simulator odometry. Truth was 0.170 m from the run-up target at handoff.
Maximum payload drift during carry was 21.9 mm. The arm released the block
fully on the table, 0.1 mm from the expected support height. The base backed
away 0.576 m and the launch exited with code 0.
[Measured GUI Navigation receipt](nav-gui-fetch-confirmed.json).

### Isaac joint authoring

The Isaac bridge forwards passive mechanical couplings separately from
powered joints. The runtime authors a PhysX mimic constraint on each
follower's imported rotary axis, disables the imported follower drive and
excludes followers from driven targets and effort commands. The installed
Isaac 6.0.1 USD schema fixture verified five relationships across mixed
joint axes, a single 0.8 Nm shaft drive and zero follower drive effort.
The pad observer matches each imported collider against the source pad's
jaw-relative bounds. It reports the normal impulse from that collider
against each named free object, divided by the simulation step. The USD
fixture identified both pads among unrelated hull colliders and the
receiver test separated pad, object and tangential force contributions.
Physical jaw travel and contacts on the imported robot remain open, so
the Isaac profile stays unavailable in the GUI. The fixtures and exported
stage are on SSD in the F100 run directory. [PhysX mimic joints](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/articulations.html#articulation-mimic-joints) · [PhysX contact reports](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/contact_reports.html).

A private normal GUI launch on `nav_empty` imported all five passive
followers and published measured joint feedback for the matching F100.
The bridge sent an empty pad-contact object with no sensor sample and the
GUI rejected it as incomplete. The bridge now omits unavailable samples.
The loaded workflow still waits for measured pad feedback before motion.
Normal Isaac gripper travel remains pending.
[GUI readiness observation](isaac-gui-unavailable-pad-feedback.json).

The exported stage from that actual F100 launch contains two distinct,
matching collision pads. Running the observer's pad lookup against the
export matched both source names. The lookup checks collider identity.
The normal-force measurements still require a loaded run.
[Imported pad identities](isaac-imported-pad-identities.json).

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
