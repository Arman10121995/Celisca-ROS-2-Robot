# Extension workflow delivery — October 27, 2026

The seven missing registrations are restored. Native ground fleet, aerial swarm,
marine and multi-leg workflows are implemented, with the scoped measurements
below. The platform and full R10 acceptance remain **partial**. Initial
implementation, a measured mission and qualification across maps/backends are
separate states. Read the [receipt](evidence/workflows-2026-10-27/README.md)
and [remaining-work guide](../REMAINING_WORK_GUIDE.md).

Work began at `f75dc60` on `master`; `8e411cb` subsequently recorded the first
implementation checkpoint. The receipt identifies subsequent source hashes,
commands, raw SSD artifacts and failures. No branch or worktree was created.

| Area | Delivered | Actual measured scope | Still open |
|---|---|---|---|
| Catalog | TurtleBot3 Burger/Waffle/Waffle Pi, TurtleBot4 Standard/Lite, Husky and historical Panda; preservation on full/scoped reprovision; explicit source-recheck candidates | Existing resources, grouped GUI variants, defaults/command checks | Qualification of changed controller sources; remaining upstream imports |
| Ground fleet | Queues, actual Nav2 task lifecycle, traffic retreat/resume, fresh physical-peer barriers, source-compatible optional RMF, coverage and native sidebar access | Two opposing robots complete a position swap; actual operator goals and RMF waypoint progress; new isolated batch independently measures both endpoints/settling and cleanup | Complete RMF conflict/infrastructure missions, coverage/contact/loss sweeps, larger fleets and backend ports |
| Aerial swarm | Shared-world launch, two to four native PX4 instances, distinct IDs/ports/frames/leases, native editor and latched flight controls; takeoff checks, FCU-clock integration, settling and owned cleanup | Two actual FCUs take off, respond to manual/group movement, and land/disarm | Altitude-estimate drift/formation accuracy remains a named gap; other behaviors/worlds, loss flights and larger groups need measured acceptance |
| Marine | Independent pressure/DVL/IMU estimation, heading-prior alignment, real RGB-D/depth registration, flushed map export, protected reload/localization, bounded ROV/AUV mission controls and ahead waypoint | ROV tank acquisition/save/reload/localization and a completed estimated-pose route with actual body trace | AUV transect completion, additional obstacle/current/loss missions and other backends remain separately qualified |
| Multi-leg | Retained KDL chains, continuous-support odometry, 2D/3D SLAM, acquired map export/reload, AMCL/Nav2, native goal/save and real spine phase/frequency controls | Hexapod walks/maps/saves/reloads and receives Nav2 success; octopod acquires/saves a real 3D database | Per-model saved-map missions, terrain/recovery/slip and backend ports remain separate acceptance |
| Experiments | Actual fleet workload producer, source/trace/specification-bound results, independent body settling, resource/cleanup capture and native Fleet point-goal queue | Two actual fleet goals pass the declared endpoint contract and owned cleanup | Batch-wide fleet reset and automatic aerial/marine/gait producers are separate remaining integrations |

The catalog contains **147 robot profiles / 78 selectable families** and
**51 world profiles / 39 selectable families**. Component-only group records
can make an internal group count larger. These counts describe availability.

## Use the native GUI

- **Launch:** restored models return to their existing families. Profiles whose
  historical certificates no longer match current code remain source-recheck
  candidates. Enable **experimental robot workflows** to expose those candidate
  controllers/modes; the command includes `allow_experimental_assets:=true`.
  Geometry/control mismatches cannot acquire support through that switch.
- **Ground Fleet:** configure complete robots/map, Run, Connect, and use Tasks,
  ground swarm, coverage or optional RMF. Traffic recovery retains the original
  task identity while a lower-priority robot moves to a checked holding pose.
- **Aerial Swarm:** configure two to four X500s and a selected installed world.
  Connect is neutral. Wait for FCU settling/readiness, Check clearance, then
  Takeoff. Manual buttons latch; altitude buttons, reduced yaw, keyboard and
  joystick controls remain explicit. Hold clears input; Land requests PX4
  landing. Read [the aerial guide](../tutorials/aerial-swarm.md).
- **Marine / Marine missions:** Arm native control, enable RGB-D mapping on the
  next Run, acquire the environment and Save. The saved database fills the
  launch controls. Reload a protected copy with localization enabled. Use the
  estimated position or **Waypoint ahead** to fill XYZ, then Plan and Execute.
  AUV ahead uses its source forward axis and requires a clear cruise/coast
  corridor; it cannot stop-and-pivot like a ROV.
- **Gait:** select the source hexapod, octopod or centipede and MuJoCo. Choose
  Drive, SLAM, 3D SLAM, localization or navigation through the Gait task controls.
  Save the acquired map, select its YAML/database for the next Run, then send
  an explicit Nav2 goal. Gait changes stop motion and require re-arm.
- **Experiments:** choose **Fleet point goals**. Enter, for example,
  `alpha 0.3 -0.5 0; beta 0.3 -1.5 0`. The queue snapshots the Ground Fleet editor
  and launches its own isolated shared world; it does not control the live GUI
  session. Cancel/relaunch resets a batch. Live per-member Reset stays in
  Ground Fleet.

## Sensor and source boundaries

The marine motion observer again drives DVL measurements; sonar only publishes
its directional range. The estimator receives sensors plus configured initial
position/heading, never a simulator pose correction. Fortress IMU heading is
aligned once to that spawn prior. The AUV camera kit adds fixed sensor frames
without changing source mass, collisions, thrusters or fins. Earlier AUV maps
without the corrected heading contract must be reacquired.

RTAB-Map projects actual calibrated depth and uses visual/ICP registration.
The tank now has an authored visible reference board; it supplies image
features, not ground-truth pose or fiducial localization messages. Its
generator uses OpenCV's predefined ArUco dictionary. ROV RGB-D runs at 5 Hz;
other OAK-D users retain the 30 Hz default.

Map saving waits for the real RTAB-Map backup service to flush working memory,
checks SQLite contents and saves actual occupied points. Reload copies the
database read-only and checks known world/estimator provenance. Unknown or
uncertain localization does not become ready from a received message alone.

Multi-leg odometry uses complete source-chain FK, native IMU and actual
continuous supporting contacts. Simulator body pose is only for visualization,
tilt and independent measurement. Gait map topics/costmaps are explicitly
namespaced; saved-map priors preserve the acquired map frame.

The legacy fleet producer's simulated completions and random “coverage” were
removed. The replacement dispatches real owned routes and requires actual
terminal tasks plus independent stamped body endpoints, speed and settling.
Contacts, communication loss and swept clearance stay unassessed unless
actually measured. A completed process alone is never a mission PASS.
