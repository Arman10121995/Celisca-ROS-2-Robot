# Facility, marine and aerial continuation — November 2

Continuation on `master` from `8b958e97`. Platform and full R10 acceptance
remain **partial**. This delivery adds initial workflows and targeted repairs;
it preserves existing robot/policy/physics certificates without rewriting them.

| Area | Implemented in this delivery | Measured / checked | Still open |
|---|---|---|---|
| Facility | Atomic bounded FIFO transport itineraries, multi-member reservations, round trips, clear landing/peer checks, queue status/cancel controls | Actual one-command two-stop Bumperbot L0-L1-L0 in 324.259 s; prior transport receipts retained | Concurrent multi-member passenger missions, automated landing parking/evacuation, wider SDK conflicts/contact, other bases/floors/backends |
| Lift recovery | Explicit held-car recovery with fresh bodies, stationary passengers, whole-footprint containment, clear-door closure, leveling and owner-loss hold | Actual empty-car ascent, mid-shaft Stop, held settling, explicit L0 return and observed door opening in 35.949 s | Occupied-car recovery and independent passenger/contact screens; automatic evacuation is not implemented |
| RMF ownership | A live SDK lift session cannot be stolen merely because five seconds passed | SessionEnd/competing-request regression | Wider live concurrent SDK sessions, calibrated charging/automatic energy scheduling |
| Marine | Neutral input on a blocked segment, cancelable generation-bound replanning, fresh ownership/start/deadline checks, amphibious shore planner reuse | Worker cancellation, stale-result, owner/sensor/deadline/start-drift checks | Physical obstacle/current/loss campaigns, AUV/surface saved-map routes and curvature-aware AUV paths |
| Aerial | Native world-XYZ formation itinerary editor, background source/separation checks, explicit Plan/Execute, bounded execution and FCU-epoch settling | Geometry, cancellation, stale feedback and real native editor interactions | Actual route flight/landing, three/four-member and additional-map/behavior/contact/loss missions |

The marine current field is explicitly an **authored command**, not an observed
water-current sensor. Native force control consumes it; route costs do not
claim current-aware optimization. A blocked underactuated AUV stops propulsion
and requires an explicit forward-aligned replan. It cannot enter an unrestricted
cornered route through the automatic replanning path.

The fleet Stop service now forwards the same session-scoped ALL STOP intent
to facility ownership as the GUI topic path; its response says Stop is
requested, not that a physical car has already stopped. Queued admission can
wait for Nav2 lifecycle readiness. Actual dispatch still requires active,
included, idle Nav2 and fresh all-member observations.

The GUI opens neutral. Fleet & Swarm → Facility contains queue/round-trip/cancel
and recovery controls. Aerial Swarm's formation itinerary switches into the
existing control column. The learning explorer links the new source and guides.
The previous GUI commit `8b958e97` passed GitHub CI run `41520955830`; that is
distinct from CI for this delivery.

See [the scoped receipt](evidence/workflow-continuation-2026-11-02/README.md),
[facility guide](../tutorials/fleet-infrastructure.md),
[formation route guide](../tutorials/formation-routes.md), and
[the remaining-work instructions](../REMAINING_WORK_GUIDE.md).

Next implementation priorities remain source-appropriate AUV curved routes,
terrain/slip/recovery and flipper-aware navigation, remaining reviewed worlds/
actors/backend adapters, and the original perception/manipulation/policy/
comparison/release queue. Full roadmap acceptance also requires actual broader
missions and clean-host reproduction. Hardware-only checks remain separate.
