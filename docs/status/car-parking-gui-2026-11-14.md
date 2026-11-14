# Car parking GUI and runtime continuation — November 14, 2026

Codex owns R5.5 after the user confirmed Claude stopped. The normal
**Launch → Drive & limits** panel now has **Park at a map pose** for the
Ackermann, rear-steer and anti-Ackermann profiles in Navigation. It offers
Connect, estimated-pose copy, X/Y/heading, a whole-task budget, explicit
Park/Cancel and estimated errors. Editing or connecting sends no movement.
[Operator guide](../tutorials/car-parking.md).

The existing Nav2 → heading-shuffle → straight-trim runtime is retained and
bounded before exposing it:

- Requests match the current runtime instance and the GUI's exact owned
  robot/backend/Navigation launch. Runtime topics respect ROS namespaces.
- The GUI waits for the navigator's actual active lifecycle state; advertised
  action endpoints alone do not make it ready. Lifecycle feedback also gates
  the active correction.
- A 0.8 s default operator lease and 5–600 s whole-operation deadline cover
  discovery, action acceptance, navigation, alignment and trim. A late goal
  accepted after timeout/cancel is explicitly canceled. A competing navigation
  goal interrupts correction ownership.
- Correction requires a fresh ROS-clock map→base estimate, a fresh local grid
  and pose in that grid's frame. The authored selected-robot costmap footprint
  is swept for 0.8 s with ≤1 cm corner increments and a 2 cm margin. Convex
  polygon/cell-square intersections include occupied, inscribed, unknown and
  outside-grid space; they are not a vertex-only clearance test.
- Manual Drive, Stop/Space, GUI close and loss of the owned launch cancel
  parking. Source wheel/steering actuators and their limits remain unchanged.
- `parked` requires estimated position ≤0.10 m and heading within the default
  3° tolerance. A completed shuffle with lateral error reports failed.

Checks: **85 focused runtime/geometry/Tk/document tests pass**, including the
actual control widget, source front/rear footprints, rotated grids, reverse
sweeps, unknown/outside space, late goal acceptance, heartbeat/discovery
deadlines, ROS-clock transform age and terminal error gates. Utils, GUI and
navigation packages build. The numerical car fixture is explicitly separate
from the live simulator mission.

An installed normal-GUI Ackermann / `nav_empty` / PyBullet screen, ROS Humble,
Jetson, domain 114, selected its normal algorithms, invoked Run, copied the
estimate and parked 0.6 m ahead with a 10° requested heading:

| Observation | Final screen |
|---|---|
| Independent simulator body position error | 0.048043 m |
| Independent body heading error | −0.477703° |
| Operator-loss case | Second real goal interrupted with heartbeat-lost state |
| Full three-axis observed body speed after interruption | 0.000123412 m/s |
| Loss → settled observation, including 0.5 s below 0.05 m/s | 1.748607 wall seconds |
| Independent final position-difference speed | 0.000123405 m/s |
| Owned launch exit | 0; parking process exits cleanly |

Every failure/earlier scope is retained on SSD. The first probe referred to a
nonexistent `rviz_var` and never launched. The first real attempt exposed the
Nav2 lifecycle startup race and was rejected; the ready gate fixes it. An
initial numerical fixture had previously called >10 cm lateral error parked;
the stricter result correctly fails it, and the old case remains a negative
test. Earlier operator-loss telemetry checked only longitudinal velocity;
its independent terminal 3D position-difference speed was 0.0533 m/s. That
screen is **not** the final full-vector stop result above. The final producer
checks the whole velocity vector and sustained settling; both its original
trace and the independent terminal calculation are retained.

The source base is `31db2d73` plus the recorded working-tree SHA256s, identical
to the installed code measured by the final producer. The report includes the
resolved command and installed hashes. [Artifacts](evidence/car-parking-gui-2026-11-14/README.md).

This is one short empty-world GUI parking/operator-loss screen. The original
10/12 car-course cases retain their old source scope. The two slalom failures,
other robot/map/backend parking missions, obstacle-route repeats and whole
R5.5 acceptance remain open in ROADMAP. Process crash and all fault scenarios
are not inferred from this operator-loss case.

Published CI at `3e40d84a` failed the new guide's missing required Run section;
`31db2d73` fixes that and eight tutorial checks pass. Its
[published run 43562150247](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/43562150247)
is green: 25 packages, fast 1,344 passed/44 skipped/four deselected, physics
137 passed/13 skipped, integration 314 passed/29 skipped and registry passes.
That revision contains the takeover batch; this new parking revision needs
its own published result.
