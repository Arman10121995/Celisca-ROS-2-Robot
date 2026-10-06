# Extension continuation — October 6, 2026

The platform remains **partial**. These results extend baseline `61335fe` on
`master`; each trial retains source, installed-module and asset hashes. The
[October 2 audit](audit-2026-10-02.md) and
[October 5 robot/flight work](continuation-2026-10-05.md) remain valid within
their recorded scopes. Downloaded assets and software checks do not establish
missions.

## TurtleBot 4: available through normal Launch

Standard and Lite now expose Display, Localization, 2D SLAM, 3D SLAM and
Navigation on Gazebo Fortress, MuJoCo, PyBullet and native Isaac. Algorithms
select automatically and the run command fills in. Forty actual mode trials
cover localization/reset/resume, real 2D/3D mapping and GUI Save Map, and clear
and obstacle Nav2 goals. Navigation uses independent body truth, the unchanged
0.15 m / 5° terminal limits and swept obstacle clearance. This covers
`nav_empty` and, for navigation, `nav_obstacle`; other maps remain experiments.

Eight final sensor/Drive trials measure neutral WASD enable, forward/reverse,
both turns, Space Stop and publisher loss, plus the source RPLidar/OAK-D
frames, 640-ray lidar, changing finite depth, calibration and movable-joint
feedback. The normal GUI additionally passed all 40 robot/backend/mode
selection/default/autofill checks. See the [mode evidence](evidence/turtlebot4-modes-2026-10-06/README.md),
[sensor evidence](evidence/turtlebot4-sensors-2026-10-06/README.md) and
[TurtleBot 4 guide](../tutorials/turtlebot4.md).

Gazebo retains the selected world but caps coarse physics steps to 2 ms for
Create 3 suspension. Isaac headless rendering now runs at sensor cadence;
ideal odometry derives velocity from measured physical body poses, removing
stationary SDK contact-velocity bias. Its final mapping trials saved real
RTAB-Map databases and PCDs and exited cleanly. GUI output processing is bounded,
launch completion is independent of the console queue, and PCD export uses a
wall-time deadline and normal process cleanup. Live Monitor now owns its ROS
executor and schedules Tk updates on the GUI thread.

## Native Panda: joint, hand and planned motion

The existing native MuJoCo Arm and Hand controls retain their source actuators,
tendon/equality and physical cube lift/release proof. Cartesian Plan/Execute
reuses upstream MoveIt 2 KDL, OMPL and FCL against collision geometry derived
from the same pinned MJCF and the selected static world. A measured-joint IK
seed avoids unnecessary distant elbow solutions. Planned positions execute
through the existing bounded native trajectory action; the 15 s, speed,
Cancel/Stop and heartbeat contracts remain.

The [Cartesian evidence](evidence/panda-cartesian-2026-10-06/README.md) records
physical TCP motion, collision and unreachable rejection, plan invalidation,
reset, Live Monitor and child-process shutdown. See the
[Panda guide](../tutorials/panda_arm.md) for actual GUI steps. Cartesian planning
with an attached object or the optional grasp fixture, Servo, arbitrary
pick/place, other hands and other backends remain separate tasks.

## Remaining extension work

- R3.6: qualify other installed models' rest poses, materials, licenses,
  controllers and missions. TurtleBot 4 vendor hazards/docking and additional
  maps/routes remain; lab sensor/Drive support does not implement vendor firmware.
- R6.5/R6.6: review connected occupancy previews before navigation registration;
  qualify imported worlds' resources, collision/visual alignment, spawn and
  robot missions. Fourteen dataset worlds and three examples are installed;
  97 other Gazebo SDFs are fixtures requiring behavior/resource review.
- R6.7: terrain GUI/provider/attribution workflow, deterministic regeneration,
  Isaac terrain contacts and class-specific missions. The separate recorded
  owner retains this lane; shared heightfield conversion alone does not close it.
- R5.7/R5.8: repeat planned arm/object missions, add Servo and model-specific
  grippers/hands and cross-backend controllers. Preserve native Panda qualification.
- R5.9: implement and measure mobile base/arm arbitration and an actual
  navigate/reach/grasp/transport/release mission; installed Fetch/PR2 assets
  remain Display imports.
- R5.10: qualify native PX4 clearance/spawn/flight for additional worlds and add
  aerial obstacle planning/mapping; retain measured `nav_empty`/`nav_obstacle` flight.

Then resume the original wheeled/legged map matrix, algorithm comparisons,
concurrency, license review and clean-host reproduction. MuJoCo TurtleBot 4
sensor trials ran at approximately 0.26–0.37 real time; performance remains P4.
Hardware joystick trials remain P2. The [checklist](CHECKLIST.md),
[ledger](platform-status.yaml), [extension execution guide](../ASSET_EXTENSION_GUIDE.md)
and [patch guide](../PATCH_EXECUTION_GUIDE.md) define the outstanding acceptance.
