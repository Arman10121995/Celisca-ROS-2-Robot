# Physical Panda after the control-column redesign

The normal installed native Panda repeats actual MoveIt KDL/OMPL/FCL planning
and physical MuJoCo actuator execution in Display / `nav_empty`. Both the
initial column stage and [final responsive stage](final-responsive/report.json)
retain exact pre-trial source/installed/native/SDK hashes and raw-trace SHA
manifests. Neither uses an experimental planning override.

| Final actual GUI target | Independent physical TCP error | Orientation error |
|---|---:|---:|
| +0.05 m z | 0.00649 m | 0.780° |
| +0.04 m y | 0.00639 m | 0.765° |

Floor/self-collision and unreachable rejection, physical joint/finger plan
invalidation, invalid offset, Cancel/Stop/heartbeat loss, monotonic Reset/Home,
Live Monitor and all four clean child exits pass the unchanged criteria.
Root and producer exit zero. No position writes produce motion.

The strict planning certificate now includes `workspace_ui.py` alongside the
original runtime contracts and points to the final measured report. The
original arm/hand controller and physical cube grasp proof remain unchanged.
The actual selected-world acknowledgement supplies its digest; the original
producer omitted map files from its source sweep, so unchanged source/installed
world parity is checked separately at collection and explicitly labeled.

Servo, moving/attached payload scenes, planned grasp fixtures, repeated arbitrary
pick/place and other maps/models/backends remain open. The saved screenshots
in [GUI evidence](../gui-controls-column-2026-10-07/README.md) are interface proof,
separate from these measured robot reports.
