# Measured continuation — October 5, 2026

> Historical source-stage record. Use [current status](CURRENT_STATUS.md),
> [the ledger](platform-status.yaml) and [the checklist](CHECKLIST.md) for the
> published October 7 work and remaining qualification. Measurements and
> original claims below retain their dates; later audited corrections apply.

Baseline `700b94e`, followed by the source changes in the containing commit,
on Jetson AGX Orin / Ubuntu 22.04 / ROS Humble. The project remains **partial**.
The [October 2 audit](audit-2026-10-02.md) still applies to demonstration reports;
this continuation records real simulator measurements separately.
The [done / remaining checklist](CHECKLIST.md) summarizes the full project scope.
The corrected direct fast runner passes 755 tests with two explicit skips and
one marked xacro case moved to integration; that real model check passes
separately. Earlier wrapper-run 756/2 totals are retained. The GitHub import
failure was a missing utility source path, masked by the local wrapper.

## Operator-visible changes

- GUI Drive adds **Altitude Up / Down** for PX4 Flight. They ramp vertical
  velocity, release to a hold and clear on Stop, Hold, Land or selection change.
  Enabling keyboard/joystick input alone stays neutral.
- PX4 selection preserves every installed world and autofills its command.
  Native Flight remains Gazebo Harmonic; selecting a world does not qualify
  indoor clearance, obstacle avoidance or flight in another simulator.
- GUI **Worlds** generates actual PGM/YAML occupancy files using the pinned
  Fortress map plugin and real collision-mesh slices. It preserves installed
  maps and stores exports, source hashes and logs on SSD.
- **Installed Extensions** opens downloaded robot-assets, URDFHub equivalents,
  Menagerie and world-library assets in normal Launch selectors with filled
  commands. Current installation has 113 profiles and 17 worlds; import and
  mission status remain separate. The GUI has no operator download step.
- **TurtleBot 4 Standard/Lite** now have physical Drive/WASD in Display on
  all four simulators, with neutral enable, body feedback, Stop and command-loss
  checks. Sensors, reset/obstacle/mapping/Nav2 and vendor docking remain open;
  see [Drive evidence](evidence/turtlebot4-drive-2026-10-05/README.md).
- **Arm** controls the native Panda's actual MuJoCo joints through trajectory
  actions, with measured Home/Stop/cancel/heartbeat loss. Cartesian planning,
  grasp and other arm backends remain open; see
  [Panda guide](../tutorials/panda_arm.md).
- **Reset Robot** now resets wheel motion and the estimator/mapping history,
  while keeping the simulation clock monotonic. Navigation-goal resets and
  drone/legged reset workflows are not covered by this wheel control.
- **Save Map** in RGB-D SLAM flushes RTAB-Map through its backup service,
  copies a stable SQLite database and exports a real point cloud. It does not
  manufacture a simulator world or silently copy unflushed live state.

## Measured robot results

Each row is a bounded screen, not the full robot/map/backend mission matrix.
The [evidence directory](evidence/continuation-2026-10-05/README.md) retains
compact reports, failed runs, source snapshots and raw SSD paths.

| Trial | Independent measurement | Boundary |
|---|---|---|
| PX4 / Gazebo Harmonic / `nav_empty`, actual GUI altitude buttons | +1.116 m / −1.149 m; released hold drift 6.1 / 5.5 cm; waypoint, Drive, land/disarm pass | Named open-map flight screen; earlier probe version |
| PX4 / Gazebo Harmonic / `nav_obstacle`, actual GUI altitude buttons | +1.031 m / −1.018 m; hold drift 4.4 / 2.0 cm; takeoff, waypoint, Drive, land/disarm pass; 1,295 body messages | One additional world; no obstacle planner or all-world flight claim |
| 4WS crab / `nav_empty`, MuJoCo / Gazebo / PyBullet / native Isaac | Position errors 0.082 / 0.114 / 0.039 / 0.019 m; heading errors −2.52 / −1.60 / −2.26 / −4.32° | Earlier continuous-distance critic stage; tighter 0.15 m / 5° body limits, one goal per backend |
| 4WS crab / `nav_obstacle`, MuJoCo / Gazebo / PyBullet | Position errors 0.029 / 0.052 / 0.045 m; headings within 5°; swept clearance 0.047 / 0.045 / 0.052 m | Footprint critic with 0.4 m inflation; static boxes and conservative 0.34 m circular footprint |
| 4WS crab / `nav_obstacle`, native Isaac with 0.65 m inflation and verified drive configuration | 0.049 m / −5.63°; swept clearance 0.287 m; 14,091 truth samples | Nav2 succeeded but the independent 5° heading check failed; final tighter-controller repeat recorded below |
| Final 4WS crab / `nav_obstacle` / native Isaac, 0.015 rad controller yaw tolerance | 0.074 m / −4.21°; swept clearance 0.255 m; 14,487 truth samples; independent acceptance passes | One verified current-setting repeat; 0.15 m / 5° body limits unchanged; no contact telemetry |
| Localization Drive, publisher loss and reset | MuJoCo/Gazebo/Isaac 4WS crab and PyBullet mecanum screens pass; Isaac measures both turn and strafe directions | Named `nav_empty` cells only; no inference to every pattern or world |
| 4WS crab RGB-D SLAM, MuJoCo / Gazebo | Finite reconstructed clouds grow 1,306→15,823 / 1,842→18,851 points; measured vertical geometry and reset/resume | Short real RTAB-Map mapping screens; full map coverage remains |
| Current 4WS crab / MuJoCo / RGB-D SLAM and actual GUI Save Map | All 21 workflow checks and five save checks pass; cloud 1,306→15,880; reset/resume pose error 0.00042 m; nine SQLite keyframes and 7,255 finite PCD points | Running steering mode queried; installed exporter executable; source database preserved; one short `nav_empty` screen |
| Mecanum / PyBullet / 2D SLAM | 23 accepted SLAM poses; up to 56,496 known / 506 occupied cells; reset and estimator resume within 0.0095 m | Upstream SLAM Toolbox graph restarted and acknowledged; initial failed reset runs retained |

The critic now resolves the cell-distance plateau only within the final
0.5 m approach. Away from the goal, path critics choose obstacle detours;
translation and pivot are admissible separately. Nav2 uses the full obstacle
footprint for these bases. A full-run Euclidean goal attraction previously
cut a corner, and the initial Isaac route became trapped near lethal cells.
Those failures are not hidden by the subsequent successful screens.
One manual repeat used the undeclared `four_wheel_steer_mode` argument and
therefore drove the default Ackermann configuration. Its successful 0.007 m /
−4.62° route is retained but **does not establish crab support**. The corrected
wrappers reject that argument and query the running steering configuration;
the evidence index excludes the misattributed report. The first GUI save
attempt also exposed a non-executable installed exporter; that defect is fixed
and the actual corrected Save Map trial above passes.

Actual reset defects included a backwards MuJoCo clock, stale estimator pose,
an unacknowledged Isaac reset and an unchanged Humble SLAM Toolbox graph.
Reset now waits for fresh simulator odometry, reseeds the estimator and
restarts/resets the relevant mapper. Gazebo resets the selected robot pose
without resetting the whole world's clock and controllers.

## World generation and requested expansion

The primitive `nav_obstacle` export contains 1,608 occupied cells. Furnished
Celisca floors 1 and 2 generate 711 / 630 occupied and 4,781 / 4,626 free
cells at 0.1 m resolution. Neither marks a mesh-mask obstacle free. The plugin
flood-fills the seed-connected region; disconnected areas remain unknown.
These files are previews and have not replaced the existing navigation maps.
The actual installed Worlds tab also passes Generate/Stop: cancellation ends
its ROS CLI, generator, Gazebo and service processes before an export completes;
a subsequent Generate creates 23,170 free and 1,608 occupied cells. The first
observer could not see descendants through `/proc/.../children`; its failed
reports are retained. The corrected observer records the real `ps` parent tree.

[R3.6, R5.7–R5.10 and R6.5–R6.7](../../ROADMAP.md) record the new scope.
The [extension guide](../ASSET_EXTENSION_GUIDE.md) gives source pins,
dependency paths, implementation order and acceptance checks. In particular,
the external libraries and Gazebo examples are **not all installed**; terrain
generation/conversion and arm, hand and mobile-manipulator control are queued.
The catalog contains 50 robot-assets URDF variants, 59 Menagerie scene
directories, eight URDFHub featured sources and 19 world/model/archive entries;
these are source listings, with overlaps and partial models, not working robots.

## Continue from here: new extensions first

The latest October 5 user instruction changes the execution order: finish the
new extensions before returning to the older roadmap. Preserve existing fixes
and their remaining acceptance rather than deleting them from the queue.

1. Import licensed external robot/world assets in verified batches; complete
   occupancy review and GUI generation, and terrain integration/conversion.
   Preserve the other agent's claimed shared heightfield work in R6.7.
2. Check flight spawn, floor, ceiling and resources for each selected world,
   then implement arm, hand and mobile-manipulator controls and real missions.
3. Return to obstacle/final-pose repeats for every 4WS pattern and mecanum
   backend, contact telemetry, cancellation and second goals.
4. Complete localization, 2D/3D mapping, reset/history and command-loss matrices,
   including longer furnished-map missions and both differential bases.
5. Preserve the open legged turning/terrain/recovery, algorithm comparisons,
   concurrency and clean-host gates. Physical joystick and hardware acceptance
   remain untested. Source tests do not close these runtime tasks.

The corrected support generator requires exact hashed measurements and leaves
unindexed combinations untested. It derives blocked full-release gates from
the ledger, including the newly requested tasks. Historical October 1 demo
reports remain preserved and do not establish release readiness.
