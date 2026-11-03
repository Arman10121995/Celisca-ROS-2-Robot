# Physical fleet infrastructure

November 3: [concurrent routes and post-exit parking](curves-terrain-concurrency.md) add fresh floor-aware traffic and normal owned Nav2 parking tasks. Two overlapping single-floor missions have an actual settled-body receipt; the larger multi-member floor/parking campaign remains separate. Facility wall-time budgets are explicitly declared, while native force, observation, ownership and arrival guards remain.

## Queued transport and explicit recovery

In **Fleet & Swarm → Facility**, select idle Nav2 members and a different
destination floor. **Queue selected members** admits a bounded FIFO itinerary;
**Queue round trip** adds each selected member's outbound and return stops
together. The physical car serves one stop at a time. Queue admission is atomic,
and each stop must change that member's predicted floor. At most 32 stops wait.
The status lists active/queued stops and the reason for any dispatch delay.
The Facility subpage scrolls vertically; narrow columns stack paired buttons,
so long live member/map status cannot hide the remaining controls.

Waiting members are reserved against other fleet commands. The dispatcher
requires fresh independent observations of every member, an empty car or its
own passenger, idle active Nav2 and clear origin/destination landings. A parked
peer can block a landing: use Drive to move it clear. The queue does not invent
a parking maneuver. Success retains the existing actual Nav2 acknowledgement,
observed floor/map/costmap and independently settled exit requirements.

**Cancel transport queue** cancels pending stops and the active transport.
ALL STOP, ownership loss or an active transport failure cancels dependent stops;
feedback resumption does not rearm them. A live RMF lift session retains its
identity until SessionEnd, Stop or lease loss, including when it stops sending
repeated requests. A five-second idle request interval cannot transfer ownership.

After a stopped ride, select a floor and press **Recover held lift to floor**.
This is an explicit new request, not automatic recovery. It requires fresh
joint/clock/all-member body feedback, a settled car, no competing requester,
idle passenger controls, and the entire passenger footprint inside the cabin.
The declared single-member car refuses multiple or straddling passengers.
Clear landing/car gates must close before movement; only a leveled stopped
car opens at the requested floor. Passenger/ownership loss or a 100-second
deadline relatches measured hold. Recovery never assigns a robot pose.

On completion, use manual Drive to leave the cabin or request a new floor
transport. Recovery itself does not autonomously evacuate the passenger.
[The new receipt](../status/evidence/workflow-continuation-2026-11-02/README.md)
measures an **empty-car** mid-shaft Stop and return; occupied/concurrent missions
remain separate. Non-Gazebo facility adapters remain static geometry.

For agent integrations, `/fleet/infrastructure/command` accepts `transport_queue`
with `requests: [{robot: alpha, floor: L1}, {robot: alpha, floor: L0}]`, plus the
current session/operator IDs. `cancel_transports` cancels it; `recover_lift`
requires `floor`. `/fleet/infrastructure/state` contains `transport_queue`,
`reservations`, `lift_recovery` and `last_rejection`. Use actual fleet session
IDs/leases, not copied examples from another run.

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
The original flat 2 × 2 m moving deck is retained. Initial door admission now
requires 0.1 mm actual alignment; the authored force PID's integral term
settles the residual after a ride. Once admitted, the original 3 mm continuous
floor/load tracking bound handles small payload compliance without pausing
localization. Motion, floor changes, loss or Stop clear the admission. Mass, force, speed and joint
limits are retained. Failed bevel candidates were withdrawn. Wheel odometry
and a Nav2 success still require independent settled body arrival; see the retained Labbot boarding failure in the
[November 3 receipt](../status/evidence/curves-terrain-concurrency-2026-11-03/README.md).
Panels have a 10 mm lower clearance so they slide across adjacent floors
without rubbing; their upper edges, link mass/inertia and actuator limits
are retained. Physical open-door feedback remains required before boarding.
A panel beyond its open target remains physically open; actuator upper limits
and the existing minimum clear-opening threshold still apply.
The facility declares 1800 wall seconds of waiting for each queued stop to
start; other configurations default to 600 s. Each stop records its expiry
when admitted, with a validated 3600 s maximum. The Facility feedback shows
the budget. Completion of another stop does not renew an old request, and
Stop or operator/feedback loss still cancels dependent stops.
Other backends currently have static facility geometry, with moving fixture
adapters pending. See [current continuation and limits](../status/fleet-continuation-2026-11-02.md)
and [the earlier floor ride](../status/evidence/fleet-floor-map-transition-2026-11-02/README.md).
