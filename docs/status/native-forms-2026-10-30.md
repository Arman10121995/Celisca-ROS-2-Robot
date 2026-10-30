# Native snake, surface vessel and articulated tracks

October 30 continuation on `master` from `db14385`, with the implementation
changes in this delivery. The platform and full R10 acceptance remain partial.
[Actual receipts and source boundaries](evidence/native-forms-2026-10-30/README.md)
record named runs, requested goals, thresholds, source identities and cleanup.

The installed GUI exposes **151 profiles / 81 selectable robot families** and
**52 world profiles / 40 families**. These are availability counts. The new
articulated tracks remain a variant inside the existing tracked family.

| Workflow | Implementation and actual result | Remaining scope |
|---|---|---|
| `lab_snake`, MuJoCo, `nav_empty` | Nine bounded spine motors and twenty passive wheels; actual rolling-contact/encoder/IMU odometry. 2D acquisition: 0.268 m physical travel, 47,073 known / 937 occupied cells. RGB-D acquisition: 0.259 m, 5 database nodes / 517 points. Installed-map and acquired-map Nav2 goals physically settle, with 0.113 / 0.106 m position errors | Terrain/slip/recovery, longer curved/reverse routes, other maps/backends and 3D saved-map missions |
| `lab_usv`, Fortress 6.18, `surface_harbor` | Complete twin-hull/two-propeller source, native buoyancy/drag/thrust and wall-time watchdog. Acquisition: 0.456 m, 6 database nodes / 3,438 points. Two requested XY goals settle within 0.2 m with translation and yaw-rate checks | Saved-map routes, docking, currents/waves/loss, other water worlds and fluid backends |
| `lab_tracked_articulated`, Fortress 6.18/private DART, `nav_empty` | Four native force PIDs, explicit limits and target slew; Tracks control column with measured angles. Targets −0.3/+0.3 rad reach approximately −0.270/+0.270 rad and settle; independent body forward travel 0.3005 m | Terrain/contact/slip, flipper-aware navigation and additional modes/backends. Original fixed-flipper Nav2 receipt does not qualify this variant |
| `tethys_auv`, Fortress, `marine_tank` | Automatic physical transect acquisition is already delivered: 2.833 m body travel, 14 database nodes / 19,515 points, signed-thrust braking and measured settling | Acquired-map route/relocalization, obstacles/current/loss and other worlds |

The snake's steering, encoder kinematics, navigation frames and arrival handling
are corrected. Wheel velocity uses the instantaneous source Jacobian and
current rolling contacts; lateral scrub is not a fictitious planted-foot
constraint. Navigation uses the mean segment tangent, measured head transforms
and scan-angle reprojection into a fixed virtual laser frame. Both static
sensor transforms are sent together so late subscribers receive them.
Nav2 goals use the configured world/map prior. Arrival freezes the current
body wave instead of straightening the robot after the goal succeeds.
Missing travel is bounded, stale estimation stops autonomous motion, and
real AMCL corrections have explicit freshness/covariance requirements.

Surface turning commands yaw rate, then holds the attained heading when the
operator releases it. Arrival checks native IMU yaw rate as well as speed.
The authored low ballast centre stabilizes the free hull; no attitude lock,
body pose or body velocity writes are used. The earlier high-CG tipping and
turn-settlement failures are retained.

The articulated track source keeps original link geometry, mass and belts.
Only the declared derivative unlocks the flipper joints and adds force PIDs.
Missing ROS-to-Gazebo target bridges and gravity-induced proportional-only
servo error were found in actual runs and corrected. The original source
variant is preserved. Connecting the Tracks GUI sends no target or Drive input.

The actual installed Tk check confirms source selection, family grouping and
generated commands for snake SLAM, surface mapping and articulated-track
navigation. This is a GUI wiring check; the physical runs above have separate
receipts. Eight packages build; 94 focused checks pass. These are not a new
whole-platform CI or release qualification.

Continue initial implementation with amphibious vehicles, real RMF
doors/lifts/docking/energy adapters, and the remaining model-specific workflows.
Then complete terrain/recovery, wider missions and the older roadmap.
R10.3 aerial work retains its recorded owner and its successful and failed
flight receipts; these new terrestrial/marine cases do not broaden flight claims.
