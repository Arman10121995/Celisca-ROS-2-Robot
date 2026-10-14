# Scoped shared-world evidence

Read [the implementation status](../../fleet-2026-10-15.md) and
[receipt.json](receipt.json). The compact [events.jsonl](events.jsonl) contains
actual session/command/Nav2/heartbeat/reset events, not generated PASS flags.

Full raw artifacts remain on the SSD at:

`/workspace/molar/robot_lab_runtime/r10-extension-2026-10-14/`

- `shared-world-nav-2/report.json`: 334 GUI-observed states and ten measured
  milestones; `gui-console.log` includes the actual command and process output.
- `source-snapshot-live/`: five exact live source files, each checked against
  the producer's SHA-256. A subsequent comment clarification does not refresh
  the recorded runtime source identity.
- `shared-world-gui-1` through `-6`: earlier failed and passing diagnostic
  stages, including the camera self-return and neighbor-separation blocks.
- `shared-world-nav-1`: missing common-map remap; no localized readiness was
  reached. Its unsuccessful startup and shutdown output are preserved.
- Build logs and `focused-checks-final.log`: implementation checks; 13 tests,
  separate from physical measurement.
- The resolved live world, descriptions, source/generated hashes, per-instance
  controller/Nav2 parameters and requested preset remain at
  `/workspace/molar/robot_lab_runtime/fleet_sessions/session-51gw9kr8/`.

Baseline `59661cb`; domain 188; partition `robot_lab_r10_nav_20261014_b`;
normal GUI-generated command, aarch64 Humble/Fortress installation. Receipt
artifacts name exact model/map/backend/task/source scope. The Nav2 success
snapshot still had nonzero yaw velocity: settled arrival was not assessed.
Contact, broader reset/loss, additional maps/members/backends and fleet/swarm
mission qualification remain open.
