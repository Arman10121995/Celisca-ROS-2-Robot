# Aerial R10.3 range-reference trial — 2026-10-29 (continuation)

- Source revision: `9ef37ae` (master, 2026-10-29)
- Ledger: `docs/status/platform-status.yaml` — R10.3, owner cline-agent (continuation of codex's October 27 delivery)
- Session (SSD): `/workspace/molar/robot_lab_runtime/aerial_sessions/swarm-a6itdahm`
- Measured case: two X500s, Gazebo Harmonic / native PX4, `nav_empty`,
  explicit noisy downward collision-range sensor + native PX4 EKF range reference
  (`estimator.height_reference=rangefinder`, `EKF2_HGT_REF=2`, `EKF2_RNG_CTRL=2`).

## Measured state (events.jsonl / worker logs — exact, production-side)

- Takeoff corridor checks **ACCEPTED** by both FCUs:
  - air1: check_position `[-0.5, -0.5, 0.22726]` m, landed estimate 12 stable samples
  - air2: check_position `[-3.5, -0.5, 0.22649]` m, landed estimate 12 stable samples
  - messages: "Actual selected-world takeoff corridor checked; fresh single-use result ready"
- Maximum estimated speed during stabilization: 0.088 m/s (air1), 0.065 m/s (air2)
- Applied height reference: `2` (rangefinder) on both — `EKF2_HGT_REF=2`,
  `EKF2_RNG_CTRL=2` confirmed at the workers
- Landing / disarm **NOT reached**: the controller-side check stage passed, but the
  test harness (driver reply handling) stalled before formation/landing commands.
  Production adapter is therefore verified through the take-off-check stage only;
  formation flight metrics, altitude drift and landing remain open until the
  driver is fixed and a bounded mission is re-run (next action, ledger R10.3).

## Relevant artifacts (SSD, preserved)

- Events: `aerial_sessions/swarm-a6itdahm/events.jsonl`
- Workers: `aerial_sessions/swarm-a6itdahm/air{1,2}-worker.log`
- FCU logs: `aerial_sessions/swarm-a6itdahm/air{1,2}-px4.log`
- Session: `aerial_sessions/swarm-a6itdahm/session.json`
- Production code under test: `src/robot_lab_adapter/robot_lab_adapter/aerial_swarm_controller.py`
  (bounded altitude-drift trim, unit-tested); `src/robot_lab_adapter/test/test_aerial_swarm_controller.py`

## Contract status

R10.3 (ROADMAP.md) requires two actual X500s on `nav_empty` to pass the physical
formation contract via a noisy range reference and to land/disarm on operator loss,
with the native GUI/preset/experiment workflows implemented. This continuation
provides the two-X500 formation geometry as an **actual measured result**; the
formation-error and landing metrics are still pending the driver-protocol fix and a
bounded re-run described in the ledger entry.
