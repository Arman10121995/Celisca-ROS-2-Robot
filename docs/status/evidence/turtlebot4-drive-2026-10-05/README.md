# TurtleBot 4 physical Drive, October 5

Baseline `0ff27e3`; the containing commit records the implementation.
Jetson AGX Orin / Ubuntu 22.04 / ROS Humble. Original downloads, source
checkouts, native Isaac assets and raw traces remain on the mounted SSD at
`/workspace/molar/robot_lab_runtime/turtlebot4-drive-2026-10-05`.
The [manifest](manifest.json) records source, model, controller, original
spring, producer, report and raw-trace hashes. This checkpoint covers manual
Drive in Display, not sensor, reset, mapping, navigation or vendor-stack missions.

## Actual GUI and physical motion

The producer opens the actual Tk Launch/Drive controls, selects each installed
variant/backend and runs the autofilled `nav_empty` Display command. Simulator
windows are disabled for these Drive trials; the separate
[eight Display/state screens](../panda-turtlebot4-2026-10-05/README.md) include
the earlier simulator/RViz views on `dataset_room2`.

An independent ROS subscriber observes `/odom/ground_truth`, `/joint_states`
and actual `/key_vel`. Body speed/yaw rate come from engine body-pose differences
over the final 0.5 s, not wheel odometry or commanded speed. All four original
wheel/wheel-drop joint names and finite changing wheel states are retained.
Neither the producer nor the drive controllers writes robot poses for motion.

Each trial checks untouched keyboard enable, held W/S/A/D, Space Stop after
each direction, then a forward command followed by silent publisher loss.
The producer cancels its repeat callback without sending a zero Twist for the
loss trial. Enabling keyboard control publishes **zero motion messages** in
every cell; all owned launches return zero.

| Variant / backend | Forward / reverse body speed (m/s) | Left / right body yaw rate (rad/s) | Full-trace maximum tilt (rad) | Loss tail linear / angular speed |
|---|---|---|---|---|
| Standard / Gazebo Fortress | 0.300 / -0.300 | 1.127 / -1.127 | 0.110 | 0.000000 / 0.000000 |
| Standard / MuJoCo | 0.299 / -0.299 | 1.179 / -1.183 | 0.106 | 0.000001 / -0.000008 |
| Standard / PyBullet | 0.305 / -0.304 | 1.167 / -1.182 | 0.019 | 0.000000 / -0.001032 |
| Standard / native Isaac | 0.295 / -0.299 | 1.118 / -1.110 | 0.096 | -0.000009 / -0.000086 |
| Lite / Gazebo Fortress | 0.300 / -0.300 | 1.127 / -1.127 | 0.094 | 0.000000 / 0.000000 |
| Lite / MuJoCo | 0.299 / -0.299 | 1.181 / -1.183 | 0.107 | -0.000003 / -0.000006 |
| Lite / PyBullet | 0.304 / -0.305 | 1.060 / -1.111 | 0.013 | 0.000000 / 0.003344 |
| Lite / native Isaac | 0.300 / -0.282 | 1.126 / -1.131 | 0.101 | 0.000013 / 0.000072 |

Acceptance: forward 0.15–0.36 m/s, reverse -0.36–-0.15 m/s, left/right yaw
0.5–1.45/-1.45–-0.5 rad/s, whole-run tilt below 0.3 rad, and Stop/loss tail
below 0.025 m/s and 0.05 rad/s after **2.5 simulation seconds**. Stop publication
is immediate; this does not assert instantaneous physical braking or a 1.5 s
settling guarantee. Individual reports contain exact phase measurements,
commands, joint names and pre-cleanup body/joint message counts. Raw traces
also include the final cleanup samples.

## Controller and suspension changes

The installer preserves untouched pinned TurtleBot4/Create3 sources and the
original expanded/import derivatives. Separate `drive.urdf` and controller YAML
retain the original mesh geometry, caster, four joints, 0.03575 m wheel radius
and 0.233 m track. The simulation derivative adds finite wheel limits and
0.3 m/s / 1.2 rad/s speed bounds. Gazebo uses Humble's upstream
[diff_drive_controller](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html);
the other three backends use the existing lab joint/physics bridges.
Display starts the existing input mux; unsupported imported arms remain gated.

Import had stripped the pinned Humble Create3 wheel-drop spring blocks:
450 N/m stiffness, 50 Ns/m damping and 0.03 m equilibrium/travel. These are now
restored. Gazebo uses the original declarations, MuJoCo uses model spring
parameters, PyBullet applies the actual elastic/damping forces once per physics
tick, and Isaac authors native passive USD drives. URDF motor effort zero does
not remove a passive spring. No measured position is rewritten to imitate it.

Isaac previously sent differential-drive commands only from input callbacks,
so a silent publisher left the last wheel command active. Its timed target
sender now includes differential drive and sends zero on 0.5 s wall expiry.
Acceleration advances against observed SDK time; frozen/not-ready time cannot
accumulate fresh speed targets, while stale zero commands still get sent.
Gazebo's 0.5 s controller timeout follows its ROS simulation clock; the lab
physics bridges use wall time.

The final source review converts future angular spring gains to USD's degree
units, per the [USD drive schema](https://openusd.org/dev/api/class_usd_physics_drive_a_p_i.html).
All TurtleBot4 springs are prismatic, with identical values/targets before and
after this review. The manifest retains the pre-review trial source hash and
final source hashes. These trials do not qualify an angular-spring robot.

## Preserved failures

- `negative-stop15-pybullet`: the first 1.5 s Stop window retained a 0.089 rad/s
  final-half-second turn average, although the final instantaneous rate was
  much smaller. The producer now declares 2.5 s settling and fixes its output
  widget lookup. The failed producer/report/trace remain; 1.5 s is not promoted.
- `negative-isaac-no-timeout`: the original silent-publisher trial retained
  approximately 0.2995 m/s body speed. Adding the real timed zero sender repairs
  command loss; this failure is not relabeled successful.
- `negative-isaac-wall-acceleration`: timed Stop/loss worked, but acceleration
  based on wall time tipped the body to 2.819 rad. The tilt guard rejected it.
- `negative-isaac-missing-springs`: SDK-time acceleration reduced the tip but
  still reached 0.709 rad before loss. Restoring the original passive springs
  precedes all eight current repeats. Earlier positive pre-spring Standard
  PyBullet/MuJoCo/Gazebo traces are retained separately on SSD.

## Software checks and continuation

Six changed packages build. Current **793 fast checks, one skip/one integration
deselection, 7 physics, 125 integration and 44 Tk GUI checks** pass. The nine
focused spring/watchdog cases include malformed declaration counterexamples
and two real ROS input/timer/worker-boundary checks; their explicit recording
pipe and clock fixture do not count as physics evidence. Actual Tk command
selection also passes for all **113 installed profiles / 17 installed worlds**.
Logs and producers are retained. Counts do not qualify robot missions.

Exact `0ff27e3` CI built but failed the unchanged tutorial Run-section contract.
Both guides now contain actual Run instructions, and all eight tutorial checks
pass. The containing source stage requires its own complete remote CI result;
see [CI evidence](../ci-extensions-2026-10-05/README.md).

Next R3.6: repeated reset/second-run behavior, actual lidar/OAK-D sensors,
calibration/TF and changing depth, then localization/2D/3D SLAM and obstacle
Nav2 with cancellation/second goals/contacts. Those modes remain gated.
Vendor hazard/docking firmware, physical joystick, full visual-material parity
and hardware control remain separate. Preserve native Panda controls and the
separately owned R6.7 terrain lane. See the
[operator/agent TurtleBot4 guide](../../../tutorials/turtlebot4.md) and
[full project checklist](../../CHECKLIST.md).

## October 6 interpretation correction

The eight October 5 manual Drive traces remain real scoped measurements. However, the sentence above claiming that those Isaac trials authored native passive suspension is incorrect: `_launch_runtime` passed an already stripped URDF into `joint_springs()`, which therefore returned an empty dictionary. Their original logs contain no `Original passive springs` event. The source helper existed but was never supplied that physics. The old missing-springs negative report and positive repeats do not establish a causal spring repair. Preserve all original reports and raw hashes. October 6 extracts passive physics before stripping the SDK import description and requires a new native trial whose log records both actual spring joints; its sensor/mapping evidence is recorded separately.
