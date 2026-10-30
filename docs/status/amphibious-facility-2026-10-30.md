# Amphibious and fleet facility delivery

Continuation on `master` from `09034dc`. Initial source/control integration is
delivered with the exact physical cases below. Full platform/R10 acceptance
remains partial. [Raw-case references, commands and measurements](evidence/amphibious-facility-2026-10-30/README.md)
separate earlier rigid-wheel diagnostics from the current sprung model.

Installed GUI availability is **152 robot profiles / 82 selectable families**,
**54 worlds / 42 families**. Counts describe installed choices, not missions.

| Workflow | Delivered and measured | Remaining |
|---|---|---|
| Amphibious rover | Complete four-wheel/two-propeller source, passive suspension, native force limits/leases, latched controls, bank and water spawns. Autonomous bank → water → bank goals physically settle about 0.104 / 0.087 m from their requests | Other terrain/world/backend missions, longer loss/slip cases and calibrated dynamics |
| Amphibious mapping | Hybrid downward-motion/range emulation plus native IMU; actual RGB-D database export. Sprung-model acquisition produces 6 nodes / 1,930 points with the current estimator contract | Acquired-map relocalization/route missions and sensor calibration; no image-matching or hardware DVL claim |
| Fleet doors/lift | Complete facility world, real sliding doors and moving lift car/door, typed RMF protocol, explicit source bounds, peer sweep/clock guards and neutral startup. Door open/close and L0 → L1 → L0 motion measured | Robot-in-car transport, floor-map/TF switching, graph annotations and full RMF negotiation/loss cases |
| Fleet docking | Facility GUI dispatches actual Nav2; independent body position/heading/linear/angular settling acknowledges a unique dock request. Bumperbot reaches its L0 dock | Additional bases/sites, actual connector/contact and charging acceptance |
| Energy | Explicit estimate/charging hooks use configured model parameters and reported actuator data; unqualified or stale motor observations remain **unknown** | Implement/measure actual backend motor-power readback before enabling SOC/energy planning; nominal RMF cost objects remain disabled |

The rover uses 8 Nm wheel motors and 35 N native propellers. The rigid source
could reverse uphill but unloaded two wheels after turning around, stalling an
autonomous return. Its source/plugin and failed trace are preserved. Four
passive 10,000 N/m springs and 200 Ns/m dampers maintain contact without
raising motor limits. The current 54.5 kg model's wheel/body poses remain free
in native physics. Water immersion gates thrust; no pose or body-velocity
writes produce motion.

Shore planning projects source support geometry into a 2.5D route, checks
non-terrain obstacles and follows the physical bank/ramp/water transition.
Execution rechecks the next 1.5 m; re-tracing the complete 12 m route every
tick exceeded the 50 ms budget under concurrent load. A missed check now
reports its deadline separately from a geometric obstruction. Acquired cloud
returns are ground only within 2 cm of declared support; unknown regions and
other returns remain obstacles. Saved maps use
`amphibious_downward_flow_range_v1` and protected copies.

The shared world starts the infrastructure adapter automatically for a map
declaring `fleet_infrastructure`. It publishes actual joint-derived door/lift
states, retains lift ownership, keeps separate door requesters and uses actual
Nav2 corridor paths to request the main door. Closing waits for clear, fresh
body sweeps at the car's current height. Missing feedback holds measured
targets; ALL STOP invalidates docking and lift intent. Lift interlocks hold
wheel commands for members in the transitioning car. RMF docking callbacks
require matching request identity and actual settled body feedback.

The **Facility** subpage fits in the Fleet control notebook. The amphibious
rover uses Launch's existing **Marine** column with an appropriate arm label,
native wheel/thrust feedback, disabled depth/strafe and command autofill.
[Amphibious guide](../tutorials/amphibious-rover.md),
[facility guide](../tutorials/fleet-infrastructure.md).

Earlier import, sensor-origin, rigid traction and recheck-budget failures are
retained. A stale CMake reference to Physics 5.4.0 was corrected by refreshing
the build cache against the installed 5.4.1 SDK; system libraries and the
private tracked-physics prefix were not replaced. The new native runs use
Fortress 6.18. Software checks and these named measurements are not whole
backend, clean-host, release or hardware qualification.

Next initial work is actual motor-power/charging integration, multi-floor
RMF/map transitions and remaining robot-specific adapters; then wider native
missions and the older roadmap. Preserve aerial R10.3 ownership and its exact
successful/failed flight scopes. Do not restart completed catalog repair,
storage relocation, map exporters or the delivered amphibious controls.
