# Native Panda grasp and interruption, October 6

The real GUI selects `menagerie_franka_emika_panda`, MuJoCo, `nav_empty` and
Display, then enables the physical grasp fixture. The source is Menagerie
`4d038b3feae26ec82b46a4d586379114012a8ac7`; the platform baseline is `531c542`.
The containing commit and source/installed hashes identify this implementation.
This is one measured coupled-finger grasp, not complete R5.7/R5.8/R5.9 acceptance.

[producer.py](producer.py) drives the actual Tk Launch, Hand and Arm widgets.
An independent ROS observer records joint positions, velocities, actuator
efforts and the free object's engine pose. [report.json](report.json) retains
the actual measurements and [source-manifest.json](source-manifest.json) records
source and executed installed-module hashes. The source robot's position
actuators, finger tendon, joint equality and contact geometry remain active.
Only actuator targets/force limits implement motion; no pose writes or welded
attachment perform the grasp.

The 3 cm, 50 g cube starts at z = 0.521494674 m on a pedestal authored from
the source home finger-pad transforms. Close stalls at 29.94 mm joint opening
with cube contacts on both fingers and 0.5 N actuator effort per finger. A real
Joint 2 minus-0.15 rad GUI jog lifts the cube to z = 0.597198521 m, a 7.57 cm
lift, followed by two simulation seconds of hold. Open reaches 80.00 mm and
releases the object; it falls to z = 0.014892245 m under gravity.

The same owned run rejects an 81 mm opening, 21 N force and NaN command;
measures action cancellation and GUI Stop holding the measured opening;
and aborts active closing after **0.808679 wall seconds** without GUI hand
heartbeats. Both a service reset and **GUI Reset Robot during active closing**
restore the object and source home opening while preserving simulation time.
The owned launch, native plant and RViz finish with return code zero.

The GripperCommand opening denotes the sum of slide-joint positions. Force
limits bound actuator effort per finger, not every contact force under an
external load. Contact-stall success requires stillness and reaching that
force bound. Arm and hand actions cannot run simultaneously. Other moving-arm
environment contacts abort reactively; there is no predictive/self-collision
planner or Cartesian MoveIt interface yet.

[manifest.json](manifest.json) identifies raw SSD artifacts, including the
2.5 MB independent trace, by SHA-256. The earlier successful contact/lift
screen is preserved at
`/workspace/molar/robot_lab_runtime/extensions-2026-10-06/panda-gripper-initial-contact-proof`.
A failed stale-installed-module launch is retained in the sibling
`negative/panda-stale-installed-module` directory, including the actual executed
file. MuJoCo's setuptools install copies modules even with symlink-install;
agents must rebuild and compare installed bytes before attributing a trial
to edited source. A zero colcon result alone did not catch that earlier
byte-compilation error.

The installer exposes Hand controls from checksum-matched measurements and
the exact native controller/model contract in
[asset-runtime-support.yaml](../../asset-runtime-support.yaml). Counterexamples
reject a PASS label with no lift, missing contact, excessive effort or a late
watchdog. The [Panda guide](../../../tutorials/panda_arm.md) describes Run and
the action interface. Other objects, repeated pick/place, Cartesian planning,+dexterous hands, mobile manipulation and other backends remain open.
