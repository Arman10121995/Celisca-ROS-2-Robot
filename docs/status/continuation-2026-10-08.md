# October 8 extension and robot readiness checkpoint

The user's order is physical **motion → saved SLAM → navigation** for each
complete source variant and compatible map/backend. The project remains partial.
This continuation follows the October 7 runtime and the published CI repair
`73fcc47`. Exact trial revisions, pre-trial working-tree hashes, executed models,
commands, original failures and measured artifacts are retained in
[the evidence archive](evidence/extensions-finish-2026-10-08/README.md).
Final publication/software checks belong to that archive; predecessor CI does
not qualify the new runtime source.

## Husky delivered through normal Launch

The official source pin is `729f8aa45ccd86fa33a05e07ef698c52c451cd9c`. The lab
derivative retains the source mass/geometry and four independent wheel motors.
A skid-drive controller supplies bounded targets; declared tire, actuator and
body-feedback settings compensate physical slip without writing the base pose.
A named lidar/RGB-D/IMU kit supplies the sensing required by the higher modes.
Read [the operator guide](../tutorials/husky.md).

Four physical Drive screens pass neutral keyboard enable, forward/reverse,
both turns, Stop, publisher loss, changing wheel joints, source-frame sensors
and owned Stop/Run relaunch. The stronger turn gate requires translation below
0.35 m and settled forward speed below 0.12 m/s. An earlier Isaac report retains
its producer PASS verbatim but fails this gate because it crawled 0.69–0.88 m.
The corrected native-pose-rate feedback run passes; no instantaneous braking,
physical joystick or robot-only reset is inferred from these Drive trials.

Twenty higher-mode trials pass: localization, 2D SLAM/export, 3D SLAM/export,
clear Nav2 and obstacle Nav2 on each of Gazebo, MuJoCo, PyBullet and Isaac.
Mapping/localization traces include command loss, robot-only reset, estimator
reseed, monotonic time and resumed movement. Navigation checks independent
body endpoints within 0.15 m / 5°, fresh truth and a one-simulation-second settle.
The obstacle route requires a measured detour and clearance for a conservative
0.62 m circle containing the actual collision envelope.

| Obstacle backend | Body position error | Body heading error | Minimum static-box clearance |
|---|---:|---:|---:|
| Gazebo | 0.102 m | 0.49° | 0.109 m |
| MuJoCo | 0.045 m | 1.06° | 0.109 m |
| PyBullet | 0.113 m | -1.24° | 0.138 m |
| Isaac | 0.123 m | -0.07° | 0.100 m |

These are short named `nav_empty` / `nav_obstacle` screens, not all-map, contact
or payload qualification. Gazebo's Husky-specific EKF uses encoder translation
and IMU yaw rate rather than the conflicting skid encoder yaw. The other
backends use reference body twist; this does not benchmark noisy wheel
localization. Isaac's obstacle endpoint estimate differs by 0.208 m / 7.61°
while the independent physical endpoint passes, so broad estimator accuracy
remains open. Slower PyBullet/Isaac routes retain their actual wall times.

The normal GUI passes all twenty backend/mode selections, compatible algorithm
defaults and autofilled commands. A normal production Gazebo clear-map repeat
passes at 0.093 m / 0.92°. Mode availability is backed by generated-model,
configuration and measured-report checksums, not catalog flags.

## Current manual controls

Drone now contains Forward, Reverse, Strafe L/R, Turn Left/Right and Stop
directly beside Takeoff/Hold/Land and altitude controls. Its input state stays
synchronized with Drive. The actual owned `px4_x500`/Gazebo/`nav_empty` screen
passes neutral enablement, six motion directions and Stop, altitude, release,
publisher loss and landing/disarming. Two-second yaw inputs turn the physical
body +0.601/−0.609 rad, with commands capped at 0.3 rad/s. Publisher-loss hold
drift is 0.041 m; the final landing-height error is below 1 µm in this run.
These are measured open-map controls, not aerial obstacle planning or all-map
flight. [Original reports and source hashes](evidence/extensions-finish-2026-10-08/controls/).

Panda Arm adds **Use current tool pose**, measured XYZ/target readouts and
small X/Y/Z target buttons in metres in `native_world`, holding the tool's
orientation. Target selection does not execute motion. Native gravity/Coriolis
bias feedforward is bounded to ±0.03 rad through unchanged position-actuator
gains/control/force limits. A previous 1 cm Z target moved only 2.9 mm under
gravity; that failed precision screen is retained. The normal GUI repeat now
moves 9.73/10.00/10.47 mm on X/Y/Z with 0.366/0.490/0.640 mm endpoint errors.
Full collision/unreachable/invalidation/Cancel/Stop/watchdog/reset and planner
cleanup pass. The actual Hand repeat lifts the 50 g cube 78.1 mm and releases
it, with 0.845 s heartbeat loss. An older probe's per-contact-point threshold
rejected sixteen aligned pad contacts of about 0.031 N each; the repeat uses
the existing certificate's total-per-finger contact-force criterion, preserving
the 0.5 N actuator bound and physical lift requirement. Other arms/backends,
Servo, attached payloads and repeated pick/place remain open.

## World geometry and occupancy repairs

Hospital source poses around -754,989,000 m were verified defects. Exactly six
SHA-pinned fixtures are corrected in derived single-/two-floor worlds to their
authored 0/3 m floors; original coordinates/rotation/bytes stay preserved.
Collada derivatives retain Gazebo node frames, the default XML namespace and
texture provenance. Native Gazebo MeshManager loads all 22 original/derived
mesh pairs with equal counts and bounding boxes; the shared-reader maximum
bound difference is 7.11e-15 m. This is not full vertex/texture visual parity.

Furniture is explicitly fixed in these imported static snapshots across all
four backends. The hospital spawn is (0.55, 12.45) on reviewed ground-floor
support rather than an isolated 306-cell pocket. Actual body height is checked:
Nav2 success while the robot falls through a missing floor remains a failure.
Current single-floor lobby routes pass for Labbot on Gazebo/PyBullet/Isaac and
Bumperbot on Gazebo/PyBullet. MuJoCo's thin elevator mesh failed volume-inertia
compilation; static collision meshes now use shell inertia. A real open-surface
floor regression supports a falling body. The Labbot/MuJoCo route then exposed
weak small-yaw response: −0.053 rad/s commands produced about −0.0017 rad/s
physical yaw. The core bases now use a profile-local wheel velocity gain of
10 with the same 5 N·m cap. Labbot's repeat passes at 0.107 m / 4.11° in 107 s.
Bumperbot/MuJoCo still fails despite Nav2 success: the actual final heading is
−110.79°, while the pose estimate reports −5.26°. That localization/navigation
failure is preserved. A later normal Display regression exposed stronger-servo
numerical braking/idle creep. Matching Bumperbot's wheel gain 10 with lab rotor
inertia 0.02 kg·m² (unchanged 5 N·m cap) passes full normal Drive/Stop/loss/relaunch.
The same hospital route then holds physical heading within 1.42°, agreeing
with its 1.23° estimate, but times out after 360 s at 0.174 m position error.
The extended-budget repeat reaches Nav2 success at 0.82° heading, but its physical endpoint is 0.175 m from the goal and fails the unchanged 0.15 m body gate. The tighter 0.03 m estimator goal experiment times out after 900 s at 0.233 m / 152.21°. It is retained as a negative result, and the normal profile is restored to 0.07 m. Position/map-estimator alignment remains open. Current
Labbot and Bumperbot Display now route GUI input through the existing mux.
Remaining base/backend/two-floor routes are pending; upper-floor travel and
actors remain unqualified. Failures and original traces are preserved.

Worlds exposes editable **Seed X/Y**, filled from the selected map. Actual
Generate and Stop checks pass with owned child cleanup and unchanged Launch
selection. A custom (-6.75, -6.75) seed exports a real grid with 22,948 free,
1,626 occupied and 37,926 unknown cells. A rotated two-hill terrain fixture
originally produced only unknown cells; separate transformed triangle slices
now produce 24,734 free / 350 occupied / 1,844 unknown cells. Geometry/dependency,
world/seed/recipe and output-byte guards invalidate stale occupancy caches.
Fourteen earlier dataset grids were refreshed at v3; the new v4 repair requires regeneration/review because open wall chains were omitted. A seed-connected slice does not cover an
entire multilevel building or qualify robot missions. The separate R6.7 terrain
conversion/provider lane remains with its existing owner.

## Preserve measured shared controls

Twelve current TurtleBot3 physical Drive/source-LDS regressions, both TurtleBot4
Isaac Drive/lidar/RGB-D regressions and native Panda Hand/MoveIt physical
regressions pass before shared-source guards are refreshed. The original T3
48/T4 40 higher-mode matrices keep their historical source stages; they were
not rerun wholesale at this checkpoint. Native Panda's cube contact/lift/release,
physical Cartesian targets, rejection, cancellation, watchdog and reset are
measured separately. Owned root exits and physical outcomes do not establish
that every Gazebo/RViz/joystick child exits cleanly; raw shutdown errors remain.

The official [Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym/tree/276801e46c5d433564f24658bac64f254b7d2d4b)
source and G1/H1/H1_2 TorchScript checkpoints/deployment models are downloaded
and hashed on SSD. This snapshot lacks its configured Go2 checkpoint. These
are research inputs; no new gait, simulator mode or motion claim follows from
downloads. Match the exact plant/joints/observations/gains before adapting them.

## Remaining work and execution order

1. Finish remaining complete-model motion controllers and exact policy/plant
   contracts, including robust BHL turning, Go2 terrain/recovery, other humanoids/
   quadrupeds, further arms/hands and mobile-manipulator bases.
2. Qualify mounted sensors and actual saved/reloaded mapping after motion passes;
   preserve RGB-only and class-specific capability gates.
3. Extend physical navigation from named screens to remaining compatible worlds,
   longer/furniture-adjacent routes, cancellation, second goals and reset/contact
   measurements. Qualify PX4 world floors/ceilings and real flight separately.
4. Complete textures/licenses/example-fixture review, separately owned terrain
   provider/Isaac contacts, Servo/payload/grasp and mobile manipulation.
5. Resume original 4WS/mecanum full patterns/modes/maps, performance and physical
   joystick checks; then measured algorithm breadth/comparisons, real concurrent
   runs and clean-host reproduction. Full release remains blocked.

Follow [the detailed AI execution guide](../AI_ROBOT_READINESS_GUIDE.md),
[ledger](platform-status.yaml), [checklist](CHECKLIST.md) and
[roadmap](../../ROADMAP.md). Keep large outputs on SSD, preserve original
sources/negative artifacts and work directly on `master`.

## Current CI and occupancy follow-up

`4fe611a` passes required and scheduled remote CI after MuJoCo 3.15 removed
the old flex-contact attribute. Both runs build 25 packages and pass 27 physics,
134 integration (13 skips) and registry; fast results retain their exact
1002/four-skip and 1003/three-skip stages.

A real stationary hospital lidar/map comparison exposes discarded open mesh
wall chains. Occupancy v4 keeps all triangle/plane segments, rejects v3 caches
and passes 24 sourced regression checks. The same 6,462 measured endpoints
improve median grid distance 0.804 → 0.071 m with the actual regenerated map.
The corrected hospital grid is installed and its unchanged normal GUI navigation
repeat is active; other generated grids need regeneration/review. Original
failures and byte/source hashes are in [the archive](evidence/extensions-finish-2026-10-08/README.md).
