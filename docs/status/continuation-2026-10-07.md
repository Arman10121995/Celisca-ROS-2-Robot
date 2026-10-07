# Extension continuation — October 7, 2026

The platform remains **partial**. Work continues directly on `master` from
`18ecdd2`, with exact source/installed/asset stages in the trial manifests.
The [October 2 audit](audit-2026-10-02.md) and [October 6 work](continuation-2026-10-06.md)
remain within their recorded scopes. Extensions have priority over the older
roadmap; imports and tests do not establish robot missions.

## Launch controls in a separate column

The latest user correction places Drive/Arm/Hand/Drone and their limits in a
**right-hand column**, with setup and command review beside it. Category/type
filters and reviewed structural tags cover wheels, legs, arm arrangements,
mobile manipulators, hands and drones. Sidebar navigation, Logs and independent
Registry filters retain exact profiles, commands and runtime gates. A compact
Set up / Command & algorithms view keeps controls visible at 1024 pixels and
restores both left columns on resize. Existing widgets/state are retained.
Read [the guide](../tutorials/gui-workspace.md) and
[actual interface/software evidence](evidence/gui-controls-column-2026-10-07/README.md).

The final normal physical Panda repeat measures 0.00649/0.00639 m and
0.780/0.765° TCP errors, with all original negatives, interruption/reset/monitor
and clean child exits. Its source certificate includes the new workspace
layout. [Measured report](evidence/panda-cartesian-controls-column-2026-10-07/README.md)
is separate from screenshots and software tests. A live Drive sampling failure
is retained. The unchanged repeat passes actual neutral enable, W/S/A/D, Stop,
publisher loss and source LDS/joint/TF checks; preview emits zero movement
commands and x drift is 0.00019 m. Both plant/producer exit zero. See
[the final measured Drive report](evidence/gui-controls-column-2026-10-07/live-drive-final/report-measured.json).

## Complete-model catalog and native Registry preview

Launch, Worlds and Installed Extensions now use reviewed parent families with
exact source/variant selectors. Registry retains original IDs, full revisions
and nested components. Six R2/Valkyrie forearm/gripper/leg/upper-body source
topologies are contained in actual complete upstream assemblies. Isolated
parts remain inspectable; controllers and support are not transferred between
source alternatives. Unattached grippers stay in their own component library.

**Registry → Preview 3D** renders natively inside the GUI with orbit, pan,
zoom, Fit and axis views. Seven actual scenes pass rendered-framebuffer,
mouse-input, unchanged selection/command and clean-close checks: Bumperbot,
furnished Celisca floor 2, native Panda, R2 forearm/full R2, X500 and the
two-floor hospital. A normal live Burger/PyBullet preview publishes zero
movement commands, drifts 0.00020 m in x and then passes physical Drive/Stop/
watchdog and original LDS/joint/TF checks. Read the
[operator guide](../tutorials/registry-3d.md) and
[render/live-plant evidence](evidence/registry-3d-2026-10-07/README.md).

Two additional unique Room 2 variants bring the inspected source set to nine
assets. The final redesigned Registry repeat checks furnished Celisca and both
Room 2 variants through its actual Preview button, rendered pixels, mouse
orbit/pan/zoom/Fit, unchanged launch selection and clean close.

URDF frames, native compiled nominal/keyframe poses and SDF include/visual
geometry retain source metres. Full meshes/buffers remain on SSD. Embedded
textures, animated actors and hardware-GPU performance remain open. Software
GL renders the 7.94-million-triangle Celisca mesh, but upload/deletion can pause
the UI. Six hospital fixtures are around z=-754,989,000 m in the source. Fit
flags their remote bounds and frames the building; Whole scene reveals the
raw extent. All poses/geometry remain unchanged. This is an R6.6 source-world
defect, not a repaired collision/spawn/navigation mission.

## TurtleBot3: physical control and source lidar

Official Burger, Waffle and Waffle Pi now have source-backed physical wheel
control on Gazebo Fortress, MuJoCo, PyBullet and native Isaac. Twelve final
normal GUI Display/Drive trials pass neutral keyboard enable, W/S/A/D,
Space Stop, publisher loss, changing finite wheel state and actual mounted
LDS/IMU frames. Lidar has the source 360-ray, 0.12–3.5 m contract at 5 Hz.
See the [Drive evidence](evidence/turtlebot3-sensors-2026-10-07/README.md)
and [operator guide](../tutorials/turtlebot3.md).

The separate control derivative preserves source meshes, authored inertias
and camera geometry. Undefined frame inertias are explicitly regularized to
avoid PyBullet's invented 1 kg defaults. Profile-local small-wheel MuJoCo
servo settings and ramped watchdog deceleration pass the unchanged tilt/stop
limits. Negative inertia/braking stages remain archived. These are lab
controllers, not OpenCR firmware. Source simulation cameras are RGB-only;
rendering and 3D SLAM remain pending.

`nav_empty` uses the robot-specific (-4,-4,0 yaw) default, where the original
3.5 m LDS can see geometry. GUI command preview and estimator initialization
agree; other maps retain their spawns and explicit launch overrides win.
Forty-eight real localization/reset/resume, 2D SLAM/GUI-export and clear/
obstacle Nav2 screens pass across all four engines. Twelve actual map saves
and all owned producers/plants exit cleanly. Initial Waffle Pi MuJoCo/Isaac
costmap failures are retained; enlarging Pi's inflation to 0.65 m and rerunning
all eight Pi navigation screens passes the unchanged 0.15 m / 5° terminal and
0.02 m swept-clearance limits. Read the
[mode evidence](evidence/turtlebot3-modes-2026-10-07/README.md).

Normal Launch now enables the four measured modes with compatible defaults
and autofilled commands. Forty-eight normal Tk selection checks and a fresh
normal grouped-GUI Waffle Pi MuJoCo obstacle route pass independently.
Runtime mode guards cover matching geometry/control/sensor/spawn/estimation/
algorithm contracts; per-trial GUI fingerprints are retained separately from
presentation-only grouping and preview changes. Other maps, longer/repeated
routes and noisy hardware localization need their own qualification.

## Native Panda: fresh normal GUI regression

The shared named-map command change was followed by an actual normal-profile
MuJoCo/Display/`nav_empty` Plan/Execute repeat, followed by a second normal
grouped-GUI regression after the shared SDF visual-reader change. The latest
two physical TCP errors are 0.00656 m / 0.771° and 0.00607 m / 0.762°.
Floor/self-collision and unreachable
rejection, joint/finger invalidation, Cancel/Stop/heartbeat loss, monotonic
Reset/Home, Live Monitor and all four clean child exits pass. The exact new
source/SDK/native proof refreshes the planning certificate; the original
arm, hand and physical cube lift/release contracts remain.
See [the grouped-GUI regression](evidence/panda-cartesian-grouped-gui-2026-10-07/README.md)
and [earlier retained repeat](evidence/panda-cartesian-2026-10-07/README.md).
Servo, moving/attached-object scenes, planned grasp fixtures, other hands,
mobile manipulation and cross-backend control remain open.

## CI and verification boundaries

Exact `18ecdd2` [remote CI](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/37586958626)
passes the full build, fast, physics, integration and registry checks. The
previous `dc39922` ROS-import failures are retained; the unchanged contracts
now run in the sourced integration tier. ROS's canonical Trimesh dependency
key resolves. Later TurtleBot3 source changes require their own exact CI.
See [CI evidence](evidence/ci-extensions-2026-10-07/README.md).

All large source checkouts, trials, traces, maps and logs remain on the
workspace SSD. The completed PX4 storage relocation is preserved. Individual
test tiers, actual GUI selection checks and robot missions have separate
artifacts; none is substituted for another.

## Remaining extension work, before the older roadmap

- **R3.6:** qualify remaining models' rest poses, materials/licenses, physical
  controller interfaces and missions. Preserve the completed named TurtleBot3 mode screens;
  retain TurtleBot4's measured five modes, sensors and map exports. Longer
  routes, imported/furnished maps, RGB camera rendering and vendor firmware,
  hazards or docking need separate measurements.
- **R6.5/R6.6:** review occupancy origin/scale/height/seed, disconnected free
  regions and rotated geometry before navigation registration. The installed
  14 dataset worlds and three environment examples need actual visual,
  collision, spawn and class-specific missions. Fix the hospital's remote
  source fixtures using reviewed placement. The other 97 example SDFs
  include fixtures and plugins requiring review.
- **R6.7:** the separately recorded owner retains terrain work. Provider/data
  attribution, deterministic GUI generation, Isaac terrain contacts and actual
  class-specific traversal remain beyond the shared heightfield converter.
- **R5.10:** retain real PX4 X500 flight/altitude checks in `nav_empty` and
  `nav_obstacle`; qualify the other installed worlds' spawn and ceiling
  clearance, flight missions, aerial planning and mapping.
- **R5.7/R5.8/R5.9:** preserve native Panda joint/Hand/Cartesian controls and
  physical grasp proof. Add Servo, attached-payload transitions, repeatable
  object missions, other models/backends and mobile base/arm arbitration.

Then resume the original wheel/legged map matrix, contact/cancellation and
repeated goals, Go2/BHL terrain and recovery, algorithm comparisons, concurrency,
licenses and clean-host reproduction. MuJoCo performance remains P4;
physical joystick testing remains P2. Use the [checklist](CHECKLIST.md),
[ledger](platform-status.yaml), [extension guide](../ASSET_EXTENSION_GUIDE.md)
and [patch guide](../PATCH_EXECUTION_GUIDE.md) for exact acceptance.
