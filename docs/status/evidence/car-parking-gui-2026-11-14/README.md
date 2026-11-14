# Car parking control artifacts — November 14

[Scope and interpretation](../../car-parking-gui-2026-11-14.md).

`live-report.json` / `live_probe.py` record the actual normal-GUI target,
installed hashes, independent body endpoint and full-vector operator-loss
settling. The probe suppresses only the parking heartbeat to inject loss;
robot motion and all ROS/simulator feedback remain real. `terminal-body-speed.json`
independently derives speed from the final stamped body positions.

`final-checks.log` contains 85 passed/six deselected checks; `runtime-initial.log`
retains the original false-parking fixture failure. Startup/setup negative
reports and build output are also retained. `manifest.json` lists base revision,
changed source SHA256s and hashes/sizes of every raw artifact. Full body/command
traces, GUI output, first passing screen, longitudinal-only screen and startup
failure logs stay on SSD at
`/workspace/molar/robot_lab_runtime/car-parking-gui-20261114/`.

`ci-followup.json` identifies the preceding published documentation failure
and fix; no current full-CI pass or full car-course/backend certificate follows.
