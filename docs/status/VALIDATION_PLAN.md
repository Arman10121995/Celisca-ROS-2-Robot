# Remaining implementation and validation plan — November 2, 2026

The platform and full extension remain **partial**. Follow the user's order:
finish usable extension workflows first, then the older roadmap. Implement
missing behavior before broad validation. Neutral enablement, bounds, Stop,
watchdogs and truthful terminal results belong in the implementation.
[The roadmap](../../ROADMAP.md) owns acceptance; [the ledger](platform-status.yaml)
owns current task state and ownership. The [agent guide](../REMAINING_WORK_GUIDE.md)
provides code locations and continuation steps.

This plan supersedes the earlier checklist that displayed unmeasured goals as
checkmarked success, named nonexistent launch files and treated PX4 as a
prerequisite for ground/marine work. Historical receipts retain their exact scope.
No full mission is established by a verification flag, catalog count or unit test.

## Environment and reproducibility

System Python/SSD ROS imports and actual Fortress fleet missions work. The old
GLIBCXX report is not a reproduced global blocker; do not replace system
libraries as a default remedy. Diagnose each backend's actual failure. PX4
issues affect its own aerial lane, not every simulator workflow.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
git status --short --branch
ros2 launch robot_lab_bringup shared_world.launch.py --show-args
ros2 launch robot_lab_bringup aerial_swarm.launch.py --show-args
ros2 launch robot_lab_bringup marine_simulator.launch.py --show-args
```

Work on `master`, claim scope before edits, keep artifacts on SSD and use isolated
ROS domains/transport partitions. Ground/marine use Fortress; native PX4 uses
Harmonic. Follow [STORAGE](../STORAGE.md), including PX4 `-d`, bounded logs and
owned process cleanup. The one-off relocation is complete.

## Current measured fleet baseline

| Scope | Actual result | Receipt |
|---|---|---|
| Native Bumperbot/Labbot effort and rate | Complete source/runtime-bound samples while moving about 0.232 m | [Energy/readback](evidence/fleet-energy-2026-11-02/README.md) |
| Bumperbot authored berth charging | Increase while independently settled; departure/discharge and actual broadcaster loss/resumption; SOC remains unknown after coverage loss | Same receipt; authored model, not battery/connector telemetry |
| Native RMF physical door/dock | Actual SDK route, physical door crossing and independently settled dock callback | Same receipt; earlier single-level graph |
| Bumperbot floor placement/map switches | Actual L0-L1-L0 and exclusive floor/AMCL frame tree | [Earlier ride](evidence/fleet-floor-map-transition-2026-11-02/README.md) |
| Automatic native Facility transport | Real approach, boarding, ride, member map/costmap changes and settled exits L0-L1-L0 in 319.913 wall seconds | [Automatic transport](evidence/automatic-floor-transport-2026-11-02/README.md) |

Wheel power comes from each member's actual `/<member>/joint_states` complete
actuator effort/rate samples and named source/runtime certificates. The global
`effort_observation_verified` flag is not qualification. Nominal RMF energy
cost objects remain disabled. The new official SDK multi-floor graph and explicit
lease epochs now have a separate actual SDK L0-L1-L0 and active process-loss/
resumption receipt in the automatic transport evidence. These remain single-member cases.

## Ordered remaining work

| Order | Lane | Next implementation / focused case | Broader acceptance still required |
|---|---|---|---|
| 1 | R10.2 fleet/RMF | Preserve measured SDK L0-L1-L0, commanded-height hold and actual ownership-loss latch; extend to other bases/concurrent conflicts | Other base/floors, concurrent conflicts/passengers, coverage/contact/larger fleets; calibrated energy/RMF energy scheduling |
| 2 | R10.3 aerial | Preserve measured two-X500 formation/loss landing; resolve the exact current FCU issue and implement remaining multi-member/map behaviors | Formation/altitude convergence, member-loss/contact and larger groups on additional maps |
| 3 | R10.4–R10.5 marine | Continue AUV/surface saved-map routes and obstacle/current/loss handling using the existing sensors/map export/estimation | Per-plant/world acquisition, reload, settled routes, contacts and loss outcomes |
| 4 | R10.6–R10.7 multi-leg | Complete per-model higher-mode endpoints and terrain/slip/recovery behavior | Hexapod/octopod/centipede terrain, localization/SLAM/navigation and source-appropriate backends |
| 5 | R10.7 additional forms | Continue flipper-aware navigation and snake/amphibious terrain/slip handling | Articulated tracks, snake and shore missions across additional maps/backends |
| 6 | R10.8 / R4 / R8.3 | Extend existing real workload producers, applied seeds and contact/clearance observations | Larger resource campaigns and source-matched repeated trials |
| 7 | R6 / R3.6 maps | Finish reviewed imports, actor/multi-floor/terrain adapters | Real selected-world, occupancy and robot missions per backend |
| 8 | Older R5 / R7–R9 / R11–R12 | Continue car/perception/manipulation/policy/comparison/release gaps named in the ledger | Exact task acceptance; physical joystick and optional HIL remain separate |

Use the existing GUI workflows and their autofilled commands. For launch
arguments inspect `--show-args` or copy the current GUI command; do not invent
`fleet_facility.launch.py`, `fleet_rmf.launch.py`, `marine.launch.py`, per-gait
launch files or a `members:=N` aerial API.

Reproduction guides: [fleet](../tutorials/fleet-and-swarm.md),
[facility](../tutorials/fleet-infrastructure.md),
[aerial](../tutorials/aerial-swarm.md), [ROV](../tutorials/marine-rov.md),
[AUV](../tutorials/tethys-auv.md), [multi-leg](../tutorials/multi-leg-gaits.md),
[snake](../tutorials/wheeled-snake.md), [amphibious](../tutorials/amphibious-rover.md),
[tracks](../tutorials/tracked-vehicle.md), [cars](../tutorials/bicycle-control.md).

## Required receipt for each mission

Record the exact robot/profile, world/floor, simulator, mode, algorithm and
source/config/asset/runtime hashes; command, domain/partition, seed and SSD
artifact paths; actual independent body/sensor/actuator observations; fixed
acceptance and retained failures; terminal position/heading/rates, Stop/loss
outcomes and owned cleanup. Keep the estimated pose separate from truth used
for measurement. Do not write truth poses or synthetic PASS metrics into the
plant/estimator to make a task complete.

A unit test is a source regression. A native graph parse verifies SDK structure.
A Nav2 success is an action result. A mission requires the named independent
measured endpoint and applicable contact/loss criteria. Wider acceptance needs
its own cases; historical certificates cannot be refreshed by changed-source
metadata or the October 1 demonstration reports.

## Build and publication checks

Build changed packages and run focused checks that address the implementation.
After a coherent delivery, use `scripts/test_tiers.sh fast`, `physics` and
`integration`, plus registry validation, in the documented ROS environment.
Preserve explicit unavailable-engine skips and actual failures. Inspect the
GitHub Actions logs on the exact published revision; a green earlier revision
does not establish the current one. Current CI dependency/cache repair evidence:
[November 2 CI receipt](evidence/ci-onnxruntime-2026-11-02.md).

Update README, roadmap, live ledger, current status/checklist, tutorials and GUI
status in the same delivery. Keep unrelated work and all named failed/successful
receipts. Broad campaigns can follow initial implementation; there is no
fabricated seven-day completion schedule or prechecked all-project success.
