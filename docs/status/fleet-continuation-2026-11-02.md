# Fleet continuation — November 2, 2026

Robot Lab remains **partial**. Current availability is 152 robot profiles / 82
selectable families and 54 worlds / 42 families. Existing amphibious, marine,
aerial, gait, tracked, wheeled and manipulation measurements keep their exact
source/map/backend scope.

This continuation delivers observation-bound fleet energy, exclusive floor/AMCL
frame ownership, and source-declared RMF door/dock graph events. The GUI's
**Fleet & Swarm → Facility** shows observed load, authored charging and battery
state with explicit unknown reasons. **Health → Platform Status** and its latest
delivery link read the repaired, current ledger.

[Actual evidence and reproduction](evidence/fleet-energy-2026-11-02/README.md):
both differential bases produced complete native effort/rate samples while
moving about 23 cm; Bumperbot physically docked and its modeled energy increased,
departure stopped charging, and broadcaster loss/resumption correctly retained
unknown energy coverage. A real RMF patrol crossed the physical door and invoked
the independently settled dock callback. The SDK exit-event serializer crash
was repaired with pinned Apache-2.0 upstream code scoped to the RMF process.

[The earlier floor receipt](evidence/fleet-floor-map-transition-2026-11-02/README.md)
retains its actual Bumperbot L0-L1-L0 body/map observations. Floor placement now
uses `map -> member/map`; AMCL alone owns `member/map -> member/odom`. Truth XY/yaw
never override AMCL. A repeat with this frame tree physically rode L0-L1-L0 and loaded both floor maps in 70.13 wall seconds. Existing worlds without declared floor maps retain their original frame tree.

Implemented source/runtime-bound readback is separate from motor/charging
calibration. Charging is an authored pose-berth model, not connector or physical
battery telemetry. The old global verification flag cannot qualify sources.
RMF nominal energy costs remain disabled.

Next implementation: automatic RMF lift-session graph tasks and level-aware
mission dispatch; additional fleet conflict/loss/contact workflows; remaining
marine saved-map/obstacle/current workflows and per-model terrain/recovery,
then the original car/perception/manipulation/policy/comparison/release queue.
The aerial R10.3 owner is unchanged. Broader robot/map/backend acceptance,
physical joystick and optional HIL remain separate. Do not reopen completed
catalog repair, source workflows or SSD relocation.

Use [the facility guide](../tutorials/fleet-infrastructure.md),
[remaining agent tasks](../REMAINING_WORK_GUIDE.md), [checklist](CHECKLIST.md),
[roadmap](../../ROADMAP.md) and [live ledger](platform-status.yaml).
