# Scoped workflow continuation receipt

Baseline `8b958e97`, Jetson AGX Orin, ROS 2 Humble. Source changes are recorded
by the adjacent manifest. This receipt does not certify all robots/maps/backends.

## Actual empty-car recovery

An owned Fortress 6.18 shared-world session used `fleet_facility`, one Bumperbot
parked outside the cabin, `navigation:=false`, `gui:=false`, `rviz:=false`,
ROS domain 191 and partition `robotlab-facility-recovery-20261102`. The driver
maintained a real operator heartbeat, requested L1, sent ALL STOP during
ascent, observed a settled mid-shaft hold, then explicitly requested recovery
to L0. No robot/car pose was assigned by the driver.

The measured case completed in **35.949093582 wall seconds**. There were 116
held-height observations. The last five spanned **2.737349009 × 10⁻⁸ m**.
Final observed lift position was approximately **−1.55 × 10⁻¹⁶ m** with a 0 m
command, completed recovery state and physically observed open car gate.
This was an **empty-car** case: occupied recovery, boarding, queue arbitration
and multi-member SDK missions are not qualified by it. Prior single-member
round-trip receipts retain their original revisions.

SSD artifacts:
`/workspace/molar/robot_lab_runtime/facility-queue-recovery-20261102/`:
`recovery_probe.py`, `session.yaml`, `plant.log`, `probe.log`, `trace.jsonl`,
`result.json`. The driver bounded duration/log growth and stopped only its own
launch descendants. The previously running unrelated car-course plant was
left intact.

## Actual queued round-trip

The corrected, separately owned Fortress facility case admitted **one command
containing two stops** for Bumperbot: L1, then L0. The dispatcher autonomously
reserved, approached, boarded, rode, switched maps and exited on both stops.
The external observer checked native body position/yaw/speed and active map
against each completed stop. It completed in **324.258982806 wall seconds**.

| Floor | Observed body XYZ (m) | Yaw (rad) | Linear / angular rate | Active map |
|---|---|---|---|---|
| L1 | 2.218757, 0.010915, 3.000002 | 0.005271 | 0 m/s, 0 rad/s | map_L1.yaml |
| L0 | 2.401653, 0.007862, 0.000002 | 0.012566 | 0 m/s, 0 rad/s | map.yaml |

The existing endpoint bounds were retained: 0.15 m position, 0.15 rad yaw,
0.03 m/s linear and 0.05 rad/s angular, plus the transport's one-simulation-second
settling barrier. No threshold was weakened. This is **one member with two
queued stops**, not concurrent multi-member transport or occupied recovery.

Artifacts: `queued_roundtrip_probe.py`, `queued-roundtrip-corrected.log`, and
`queued-roundtrip/{trace.jsonl,result.json,plant.log,runtime-source/}` under
the SSD root above. Domain 192, partition
`robotlab-queued-roundtrip-corrected-20261102`, navigation enabled, no GUI/RViz.
The source snapshot was taken after startup from unchanged loaded fleet files;
it is not an immutable full-build capture. The later Stop-service forwarding
change is covered by software checks, not this run. Both owned plants exited;
the unrelated car-course plant was preserved.

## Final software / UI checks

- `delivery-source-tests.log`: **127 passed**, 1.58 s: fleet queue/recovery/session ownership,
  existing floor/map and lift transport contracts, marine sensing/replanning,
  aerial group/route models, controller and architecture/source-link checks. These are software fixtures.
- `delivery-gui-tests.log`: **4 passed**, 8.34 s under Xvfb: neutral construction,
  queue order, native aerial route editor, separate Plan/Execute, malformed
  coordinates, preserved latched manual controls, scrollable facility status and no automatic ROS connection.
- `build-delivery.log`: utility, adapter and GUI packages built in 10.4 s.
- `installed-ui.json` / `installed-ui.log`: the installed full application stayed
  neutral; aerial itinerary/manual views remained visible at 1600×1050 and
  1024×768. `installed-parity.json` records matching installed/source module hashes.

Earlier `features.log` (103 passed) and `gui-sourced.log` (three passed) retain
their earlier source scope; final checks above include the later readiness,
service forwarding, stale-planner and scrolling changes.

The first UI invocation lacked the SSD `jsonschema` prefix: one test passed,
two setup errors. Its `gui-final.log` is retained; the corrected sourced
environment passed. No test, threshold or support gate was weakened. The
earlier GUI commit's full CI passed; this delivery needs its own CI result.

The first queued round-trip invocation exposed asynchronous Nav2 status
admission racing: its admission was rejected and no route ran. Artifacts are
retained under `queued-roundtrip-admission-race/`. Admission now reserves an
included idle member and waits for readiness; actual dispatch still checks
active Nav2. The Stop service forwarding regression is a software check,
separate from the empty-car trial's actual topic ALL STOP.

No full matrix, new aerial route flight, marine obstacle/current mission,
calibrated energy model or hardware trial was run for this receipt. The live
ledger, checklist and roadmap keep those boundaries explicit.
