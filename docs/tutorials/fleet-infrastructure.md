# Physical fleet infrastructure

## Run

Open **Fleet & Swarm**, choose **fleet_facility**, enable per-member AMCL/Nav2,
review members and Run. Connect is neutral. Use the **Facility** subpage for
physical doors, lift-only requests, automatic selected-member transport, docking, observed power and
modeled battery status. Manual requests require the GUI operator lease; disable
RMF ownership before taking manual control.

The source contains force-driven sliding doors, a three-metre lift/car door,
a shaft/upper landing, L0/L1 occupancy and a pose-berthing dock. Native PIDs
retain authored travel, force and speed limits. Close waits for fresh, clear
fleet-body sweeps. The car door and both fixed landing gates close before lift motion. A landing
opens only with the car stopped within 3 mm of that floor. Wheels are held in a transitioning car, during floor-map
loading, and when declared infrastructure feedback is stale. ALL STOP cancels
fixture intent/docking and holds measured joint targets.

Each robot's fresh body observation classifies its floor; lift position alone
cannot move a robot or change its floor. Its namespaced Nav2 map server loads
`map.yaml` or `map_L1.yaml`, with commands held until map load and both costmap
resets acknowledge completion. AMCL scans pause during vertical movement and
map transitions; raw obstacle scans remain live. The upper grid contains actual
landing/car walls and unknown space outside physical support. The frame tree is
`map -> <member>/map -> <member>/odom -> <member>/base_footprint`:
infrastructure places the floor frame vertically; **AMCL alone** supplies the
planar localization transform. Truth XY/yaw never replace AMCL. Existing worlds
without declared floors keep their original `map -> <member>/odom` tree.
**Transport selected robot** reserves one idle Nav2 member and the lift,
approaches, boards, rides, changes maps and exits. Stop cancels the task.
A full automatic Bumperbot L0-L1-L0 is [measured](../status/evidence/automatic-floor-transport-2026-11-02/README.md);
other robots and concurrent transport remain separate acceptance.

**Navigate selected member to dock** sends actual Nav2 and requires independent
position/heading and low linear/angular velocity for one simulated second.
Unique identities prevent an old dock result from finishing a later request.
This is pose berthing. There is no connector/contact sensor or hardware battery.

With the optional Humble RMF SDK, source-declared main-door approaches and
clearance-tested dock connections become native RMF lane events. The official
door and lift supervisors aggregate requests. The official native building
graph parser creates LiftSessionBegin/Move/DoorOpen/SessionEnd events between
L0 and L1; inside-car vertices are hidden from operator destinations. Choose
`L1_landing`, `L0_landing`, `door_main_1` or `west_charger` from the generated
RMF waypoint selector. Give RMF control explicitly; ALL STOP or feedback/lease
loss invalidates its epoch, requiring explicit re-enablement. Multi-floor SDK
mission qualification is recorded separately from native transport; one real
Bumperbot SDK L0-L1-L0 is measured. **RMF floor round-trip** requests the other
landing and return in one SDK patrol (the macro has no separate mission receipt). Graphs reject blocked approaches;
no door/lift mission is inferred from asset presence.

Provisioning includes a process-scoped correction for Humble's upstream graph
serializer exit-event null dereference. For an existing installation run:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/rmf_env.sh
scripts/build_rmf_graph_fix.sh
```

The extracted Apache-2.0 upstream source/pin is in
`src/robot_lab_utils/vendor/rmf_graph/`. Only the Robot Lab RMF process preloads
the correction. System/SDK libraries remain intact. If it is missing, facility
RMF launch gives the build command; an exited RMF owner shuts down its launch.

Battery status is an **authored simulation model** with configured capacity,
initial charge, hotel load, efficiency, no regeneration and pose-berth charging.
Complete sampled wheel effort/rate supplies the motor load. Source certificates
bind the generated robot description and actual readback library SHA-256;
the old global `effort_observation_verified` flag cannot qualify a model.
A changed description/runtime stays unknown until its own measurement.

Bumperbot/Labbot effort/rate readback was measured with actual body motion.
Bumperbot modeled docking/charging, departure, broadcaster loss and resumption
were exercised. A missed observation interval makes SOC unknown for the rest
of that session: later samples cannot recover unobserved energy. Restart gives
a fresh, explicitly authored initial estimate. Current observed load can still
be displayed. Charging also needs fresh, settled body/dock feedback. RMF's
nominal energy cost objects remain disabled; these are not battery chemistry
or real charging measurements. The car retains its last commanded height
after requests/session completion; measured position is used as a new hold
target only on explicit Stop/loss.

Reproduce source assets with `scripts/generate_fleet_facility.py`; it preserves
source certificates, whose runtime/source hashes are checked on every session.
Other backends currently have static facility geometry, with moving fixture
adapters pending. See [current continuation and limits](../status/fleet-continuation-2026-11-02.md)
and [the earlier floor ride](../status/evidence/fleet-floor-map-transition-2026-11-02/README.md).
