# Curves, terrain and concurrent missions — November 3, 2026

Continuation on `master` from `43473387`. The three requested initial
workflows are implemented. Runtime qualification remains scoped to the
specific cases below; full R10/platform acceptance stays **partial**.

| Area | Delivered implementation | Physical evidence / limits |
|---|---|---|
| Curved Tethys routes | Pinned MIT Dubins solver; source −X/course conversion; flow run-up, depth/clearance and braking checks; sideslip-aware guidance; GUI course/radius fields | Actual 20° tank curve and settled arrival, 34.046 s. Tight-turn failures retained. Other curves/maps/current/saved-map cases remain separate |
| Terrain/backend adapters | Source 6/8/12-leg adapters for MuJoCo, Bullet, Fortress and PhysX; actual force feedback, explicit bounded low-relief adaptation and selected-backend GUI autofill | Octopod low-relief travel/hold measured on MuJoCo, Bullet and Fortress. Isaac runtime check and per-model/full task matrix are recorded in the receipt; no universal walking/navigation claim |
| Concurrent fleet missions | Existing concurrent member queues connected to floor-aware traffic, exit parking, batch outbound/return order, preloaded parking maps and finite slow-host budgets; optional 5 Hz RGB-D | Two independent Nav2 routes in one three-base facility world completed with 45.774 s overlap and independent settled bodies. Full multi-member floor parking remains separate |

Use [the GUI tutorial](../tutorials/curves-terrain-concurrency.md). Its native
architecture node links source, range/contact/odometry topics and the current
claim boundary. [Receipt, failures and reproduction](evidence/curves-terrain-concurrency-2026-11-03/README.md).

## Source and control boundaries

- AUV curves reuse PythonRobotics at
  `8c392897c7f41a6e4d9aab9cb2e9737a7a6fa031`; license/provenance ship with the
  package. Six Dubins families plus an exact tangent-arc roundoff correction
  share the same source collision checks. No source actuator force was raised.
- Fortress's stock contact component serializes positions but omits actual
  force/normal/depth. The pinned upstream Physics system at
  `728569bc66e2f64f8c1ac36758a276a10f9ba6e6` has a private Gait-world serializer
  patch, preserving the integration/contact/actuation laws. Missing force data
  is invalid, not an invented zero. Complete empty observed frames are zero.
- Generated Fortress models change the native joint reference to source
  stance, with equivalent transforms, shifted limits and restored source
  encoder reporting. Bullet virtual fixed-frame masses are corrected to the
  source description. Native SDK gains remain engine-specific adapters.
- Terrain rays describe ground relative to the **level source stance**.
  Measured body pitch must not become a command to shorten the low-side legs.
  Mean heave is removed; reach/slew, missing rays, tilt/body, live ownership,
  timestamps and Stop are guarded. Continuous insufficient support or
  inconsistent support velocity beyond 80 ms of simulation time interrupts
  terrain walking. Invalid kinematic samples never integrate odometry.
- Native terrain observations use bounded wall-time publishing cadence while
  carrying actual simulation timestamps. This avoids making ordinary slow
  simulation look like a missing sensor, without relaxing freshness guards.
- The small source heightfield keeps its triangle support in Isaac; the
  duplicate-floor optimization must not delete millimetre-scale terrain.
- Simulator pose places native/static-model range probes and supplies an
  independent tilt/endpoint observer. Contact/FK odometry receives no simulator
  XY. Initial/reset floor alignment does not run during motion.

## Remaining acceptance, in order

1. Finish the exact Isaac terrain mission if its receipt is unsuccessful;
   inspect native contact tensors/rays and source geometry, preserving actual
   error logs and the body/force/feedback gates.
2. Complete the two-member L0→L1→L0 parking wave in the receipt producer using
   the declared 900/300/180 s facility wall budgets. The earlier 100 s phase
   timeout reached physical L1 but expired before observed door/map completion.
   An occupied car, source-clear parked peer and independently settled exit
   must be observed; two single-floor goals do not prove this campaign.
3. Record hexapod and centipede travel/reverse/turn/Stop/loss/reset and actual
   terrain/higher-mode missions on each backend candidate. Source tests and
   the octopod case do not transfer those certificates.
4. Extend AUV curve direction/radius/depth, obstruction/current/loss and
   acquired-map missions within the existing force and tracking envelopes.
5. Resume the separate aerial route-flight/landing, occupied recovery, broader
   SDK conflict/contact/loss, calibrated energy and original roadmap queue.

The implementations above do not close these broader acceptance tasks from
metadata, compile results or catalog counts. Historical certificates retain
their exact source/revision/robot/map/backend/task scope.
