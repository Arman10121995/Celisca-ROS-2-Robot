# Park a car at a map pose

## Run

Select **Ackermann car**, **rear-steer car** or **anti-Ackermann car**, choose
a mapped world, and select **Navigation**. Run the normal autofilled command.
Open **Launch → Drive & limits** and scroll to **Park at a map pose**.
The panel appears only for these car profiles in Navigation.

1. **Connect parking state** reads the owned launch's controller. It sends no
   movement. Wait for localization and the active Nav2 navigator.
2. **Use estimated pose** copies the current map X/Y/heading into the fields.
   Edit X/Y in metres and heading in degrees. Editing sends no motion.
3. Set a whole-task budget, 5–600 seconds; the default is 180 seconds.
4. Press **Park at target**. It stops manual Drive and sends one explicit
   request. Nav2 first approaches the target. The car then corrects heading
   with forward/reverse arcs and trims its position along the final heading.
5. **Cancel parking**, Drive Stop, Space, the header's Stop motion, closing
   the GUI, changing the owned launch, or starting manual driving cancels the
   operation. The controller also stops if its operator heartbeat disappears.

This uses the existing `car_alignment` law and original robot controller;
it does not rotate a car in place or write its physical pose. The rear-steer
car steers the rear axle. Anti-Ackermann steers its outer front wheel more
than the inner one. Their physical geometry remains distinct.

## What the state means

States are **navigating → aligning → trimming → parked**, or **failed** /
**canceled**. The panel shows estimated final position and heading errors.
`parked` requires both position within 0.10 m and heading within the configured
3° default. A successful Nav2 action or completed shuffle alone is insufficient.
Lateral error can remain after a straight trim; that correctly reports failed.
The observer is map→base localization, separate from simulator truth.

Before each nonzero correction, the controller requires fresh localization,
an active navigator, a fresh local costmap and the operator lease. It sweeps
the selected robot's actual Nav2 footprint over 0.8 seconds at ≤1 cm corner
increments with a 2 cm margin. Occupied, inscribed, unknown and outside-grid
space block correction. Missing/stale information stops it. This may refuse
a tight bay that needs a different manoeuvre. Nav2 still plans the approach.
An external navigation goal during correction interrupts parking ownership.
The whole budget also covers goal acceptance, alignment and trim.

## ROS interface

Root-session topics are `/car_parking/state`, `/car_parking/command` and
`/car_parking/heartbeat` (`std_msgs/String`, `String`, `Empty`). The runtime
uses relative names so fleet namespaces do not share a parking command bus.
State includes the current `instance_id`, robot model, estimated pose and
`navigation_ready`. Every park/cancel request must echo that instance ID:

```json
{"op":"park","instance_id":"from-current-state","x":1.0,"y":0.0,"yaw":0.0,"timeout":180.0}
```

Yaw is radians on ROS, degrees in the GUI. Publish a heartbeat more frequently
than the default 0.8 second lease for the entire operation. Stale instance IDs
and invalid budgets are refused. These topics command the owned navigation
session; do not also run a second parking controller in the same namespace.

The original course measured 10/12 named robot/backend cases before these
new GUI and guard changes. That result retains its old source scope. Current
checks and any new live screen are recorded in
[the implementation receipt](../status/car-parking-gui-2026-11-14.md).
The two slalom failures and full map/backend/course qualification remain open.
