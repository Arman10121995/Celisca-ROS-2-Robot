# Actual amphibious and facility cases

Recorded on October 30, 2026, from `09034dc` on `master` with the implementation
tree. [Delivery and limits](../../amphibious-facility-2026-10-30.md),
[manifest and raw SSD directories](manifest.json).

| Case | Robot / map / plant | Actual result |
|---|---|---|
| [Sprung shore round trip](sprung-shore-roundtrip/extension-mission-result.json) | `lab_amphibious` / `amphibious_shore` / Fortress 6.18 | Requests [6.3, 0, 0.134] and [−5.8, 0, 0.74]; actual [6.400351, −0.026534, 0.133078] and [−5.759569, −0.076106, 0.727807]. Both physically settled for 1 simulated second; final speeds 0.00235 / 0 m/s and yaw rates 0.00298 / 0 rad/s |
| [Sprung RGB-D acquisition](sprung-rgbd-acquisition/extension-mission-result.json) | Same | Actual body travel with protected 6-node / 1,930-point acquired map; current hybrid estimator contract and database hash in result |
| [Physical facility/cabin/dock](facility-cabin-dock/facility-result.json) | Bumperbot `alpha` / `fleet_facility` / Fortress 6.18 | Neutral startup; physical main-door open/close, moving lift/car door L0 → L1 → L0, then real Nav2 and independently observed dock settlement. Exact actual joint positions/body observation in result |
| [Historical rigid manual traversal](historical-rigid-manual-transition/transition-result.json) | Earlier rigid `lab_amphibious` / shore / Fortress | Actual neutral bank → water → bank Drive/Stop diagnostic. This is the rigid source, not current sprung-model qualification |

The built-in navigation producer records requested XYZ and independent stamped
native body observations. The round-trip result requires its recorded 0.2 m
position, 0.05 m/s speed and 1 simulated second settlement contract. It also
records yaw rates; that archived producer did not require the later
surface angular-rate gate for this new robot. The current producer applies
the surface contract (including ≤0.05 rad/s) to the amphibious rover as well.
No body truth is fed to the mission planner; the inner actuator controller
retains its explicitly labeled simulator observer.

The facility producer is an external physical diagnostic, so its parent
`mission_result` is null. It reads actual native joint positions and the
adapter's independently measured body docking result. A sent goal, typed
request, source map count or library availability never sets success. This
does not qualify a robot riding the lift, floor-map switching, RMF traffic
negotiation, a physical connector/contact, charging or measured battery power.
The diagnostic explicitly requires unqualified battery fractions to stay null.

All copied parents report owned cleanup complete. Raw traces, actual RTAB-Map
databases/clouds, generated native sessions, resource logs and physics/plugin
archives stay on the SSD. Interrupt-time exit errors can exist despite no
surviving owned descendants. Requested seed 42 is not an applied simulator
seed; contacts and swept clearance remain unmeasured.

Failures remain in the manifest's raw directories: a direct-executable relative
import, the underwater sensor origin losing ground lock near the bank, the
rigid chassis unloading two wheels uphill, and a full-route collision recheck
exceeding its control budget. The source/plugin archive preserves the rigid
model before passive suspension edits. A stale Physics 5.4.0 CMake path was
refreshed against the installed 5.4.1 SDK, without replacing system libraries
or the private tracked engine.

Source hashes are recorded **after** the runs; this is not an immutable
whole-build qualification. Built-in producer/spec/trace identities and frozen
producer copies are retained in each case. External diagnostics record their
startup producer hash. Earlier source/plant scopes remain distinct.

[GUI check](gui-current.json), [probe](gui_check.py),
[actual round-trip driver](run_shore_roundtrip.py),
[facility driver](run_facility.py) and [producer](facility_trial.py),
[focused regression checks](tests-workflows-fixed.log). These software checks
are separate from the actual physical results above. No new full CI or
clean-host acceptance is claimed.

Native force implementation uses the documented
[Gazebo 6.18 Link API](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1Link.html)
and the installed SDK joint-effort interfaces. Fixture motion uses Gazebo's
native force position controllers. Geometry, suspension, drag and energy
parameters are authored research models, not hardware calibration.
