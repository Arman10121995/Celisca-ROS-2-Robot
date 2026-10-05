# Panda controls and TurtleBot 4 imports, October 5

Source baseline `36f38b5`; the containing commit captures the implementation.
Jetson AGX Orin, ROS Humble; all downloads, screenshots and raw traces remain
on the mounted workspace SSD. `manifest.json` records implementation hashes,
original resources and raw trace/screenshot hashes. This checkpoint does not
complete manipulation, TurtleBot 4 missions or the wider platform.

## Actual native Panda control

`menagerie_franka_emika_panda` / MuJoCo / `dataset_room2`: the real GUI Run
command selects `arm_control:=panda`; the Arm tab sends position trajectories
to the pinned model's actual seven PD actuators. The observer measures
`/joint_states` independently of the controller status messages. No probe or
controller writes qpos to execute motion.

[panda-report.json](panda-report.json) records all seven +0.1 rad GUI jogs,
Home, invalid-limit/velocity/joint rejection, action cancellation, GUI Stop,
heartbeat loss and owned cleanup. The final Home maximum error is 0.008143 rad;
all jog errors stay below 0.009 rad. Heartbeat loss aborts after 0.8291 wall
seconds (0.8 s expiry plus observation/transport delay), then holds with
measured velocity below 0.03 rad/s and drift below 0.02 rad. Native shutdown
returns zero. The earlier pre-camera successful repeat remains on SSD.

A first producer clicked a still-disabled next jog before the GUI refreshed,
then timed out waiting for motion. Its failed report/log are preserved. The
corrected producer waits for the actual button to be enabled before clicking;
it does not relax the motion, error or stop limits. The controller code did
not change for that producer correction. A separate interrupted GUI test run
is preserved on SSD; the final 44-test GUI suite passes.

The original model's actuator gains/force limits and coupled open fingers are
retained. Targets have model-limit margins, bounded rest-to-rest cubic speed,
measured action feedback and one active goal. Environment contacts stop motion
reactively. MoveIt/Cartesian planning, predictive/self-collision validation,
grasp, different maps and the other three arm backends remain R5.7/R5.8.

## Actual TurtleBot 4 display/state

Both official Humble Standard/Lite descriptions and original Create 3
resources are downloaded and source-pinned. `turtlebot4-import-checks.json`
records actual PyBullet imports, resolved mesh hashes and dependency/license
hashes. Standard has 49 links; Lite has 39; both retain four movable wheel/
wheel-drop joints. Xacro uses an SSD source-backed package index. Original
vendor sources remain unchanged; Display derivatives omit vendor control and
simulator plugins. Converted visual materials are not full source parity.

Eight named GUI reports cover Standard/Lite on Gazebo Fortress, MuJoCo,
PyBullet and native Isaac Sim in `dataset_room2`: actual clock advancement,
finite joint state, description/TF and owned Run/Stop. These are display/state
screens, not Drive/SLAM/navigation, docking, hazards, sensor parity or stable
long-duration missions. The Arm/Drive controls follow implemented support;
TurtleBot 4 Drive inputs are disabled until its base controller is connected.

Screenshots exposed an initial MuJoCo camera aimed at the world origin. It now
fits the selected robot's actual geometry subtree. Native Panda rendering uses
its robot extent rather than the attached world's extent, and RViz fixes its
frame at `native_body_1` so map spawn offsets do not hide the robot. Both
TurtleBot 4 MuJoCo screens and the Panda control screen repeat after those view
changes. The prior screenshots/reports remain on SSD. The other six backend
screens use unchanged backend plant paths; they predate the final Drive-input
availability UI adjustment.

## Validation and remaining work

Four-package builds; 13 numerical trajectory counterexamples; final **786 fast
checks, one skip, one integration deselection**, **7 physics**, **123 integration**
and **44 Tk command/Drive checks** pass. Exact remote CI `36f38b5` passes the
previous import stage after the Trimesh/Ubuntu NumPy repair; control-stage CI
is separate and recorded after publication. Test totals do not establish robot
missions. Commands/producer code, failed probes and scoped measured results
are retained here. Source trees, images and longer traces are on SSD.

Follow the [Panda guide](../../../tutorials/panda_arm.md) and
[TurtleBot 4 guide](../../../tutorials/turtlebot4.md). Continue actual
TurtleBot 4 drive/sensing and R5.7 Cartesian planning/grasp/backend qualification;
R5.8/R5.9 hands/mobile manipulation remain. Respect R6.7's separately recorded
terrain owner, then resume the older R5.6 mission matrix.
