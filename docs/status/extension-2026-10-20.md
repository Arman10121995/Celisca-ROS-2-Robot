# Extension checkpoint — October 20, 2026

Baseline: `b8ea60e`, existing `master`. Codex owns the active implementation
continuation requested October 20. Keep older source receipts unchanged.

## Current inventory and usable workflows

Actual installed manifests and resource existence checks give 31 core plus 116
external robot profiles (147 total), grouped into 78 selectable robot families.
There are 33 core plus 17 external world profiles (50 total), grouped into 38
selectable map families. Components/source variants stay nested; a profile count
does not establish motion, SLAM or navigation support.

- Fleet & Swarm: one actual Gazebo world, separate Bumperbot/Labbot identities,
  independent/group Drive, AMCL/Nav2 dispatch/allocation, basic per-member queues,
  four ground kernels, heartbeat Stop and scoped reset. Two-body diagnostics are
  recorded in the original fleet receipt; queue implementation is newer.
- Marine: complete eight-thruster ROV and source Tethys propeller/fin AUV, five
  real water/collision worlds and native controls. Actual tank/reef diagnostics
  are separate from independent underwater estimation and autonomous missions.
- Multi-leg: complete source hexapod, authored octopod and segmented twelve-leg
  centipede, bounded MuJoCo motors, source IK, contact/tilt guards and native Gait.
  Short forward/Stop diagnostics do not qualify terrain or higher modes.
- Original extension: installed TurtleBot3/4, Husky and source robot families,
  native arm/hand/mobile-manipulator workflows, PX4 X500 flight, embedded registry
  previews, occupancy export and terrain tooling. Exact model/backend/map gates
  remain authoritative.
- MOLAR / ProBOT-L: named drive/lift/telescope/roller GUI screens pass; sensors,
  saved SLAM/navigation, payload transfer and multi-floor work remain separate.
- Policy training: first Go2 run finished/exported at 2,000 iterations and
  24,576,000 steps (`training/go2_walk_r1/progress.jsonl` and `policy/source.json`
  on SSD). The ledger records evaluation and launch diagnostics; no-crash
  controller launches are not independent measured transfer qualification.

## Completion work

1. Fleet: robust queue lifecycle, actual settled task results, traffic/resource
   arbitration, cooperative coverage and a source-compatible optional Open-RMF
   adapter. Aerial fleets require genuinely separate FCUs/identities/leases.
2. Marine: pressure/IMU/velocity sensor contracts, estimation independent from
   controller truth, saved 3D maps and bounded volumetric missions; optional
   backends require actual forces, not scene conversion.
3. Multi-leg and additional forms: sensor/odometry integration, saved mapping
   and navigation, terrain/recovery and complete additional robot workflows.
4. GUI and experiments: native commands/controls/status and real extension
   workload producers with source hashes, timestamps and measured outcomes.
5. Maps: additional complete imports, attribution/resource repairs, reviewed
   occupancy regions and multi-floor/dynamic/terrain missions. The complete
   upstream internet libraries are not all installed or qualified.

Broad physical qualification follows initial implementation. R10 stays partial
until its acceptance evidence exists; older car parking, perception calibration,
manipulation, aerial mapping, algorithm breadth and release tasks remain open.

## Preserve newer negative results

Round 57 records MuJoCo slalom gates reached but parking yaw 14.95° against
10°; Gazebo stops at gate 1; Isaac's course startup is inconclusive. Startup
without errors, catalog entries and unit tests cannot replace physical results.
The October 19 16:15 status summaries are historical and superseded here.
