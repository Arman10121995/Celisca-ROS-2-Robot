# V10: a shorter Arrival hand-over for MPPI, measured and reverted (November 28, 2026)

**Status: not adopted.** Shortening the hand-over makes MPPI faster but
less reliable. The original 30 s allowance stays.

On November 22, MPPI reached 15/15 goals on the R7.7 Bumperbot maze
protocol, but only with its Arrival fallback, at a median of 74.8 s per
goal ([round](../v10-planner-comparisons-2026-11-22/README.md)). The truth
trace showed where the time went:
- MPPI parks 0.16 m short of the goal and stands still for about 40 s;
- then the progress checker (Bumperbot: 0.1 m in 30 s) aborts, and Arrival
  (the Nav2 Regulated Pure Pursuit plugin) finishes on the same path.

The trial set `progress_checker.movement_time_allowance` only when the
Arrival assist is active, then repeated the same protocol (Gazebo
`nav_maze`, A* global, seeds 11–53, 15 goals) at revision 2866b545:

| Allowance | Goals reached | Median sim time per goal | Progress aborts | Failed goals |
|---|---|---|---|---|
| 30 s (11-22, unchanged) | **15/15** | 74.8 s | 12 | — |
| 20 s | 14/15 | 60.6 s | 20 | 1 |
| 10 s | 13/15 | 48.7 s | 23 | seed 23 goal 2 (0.217 m short), seed 37 goal 1 (0.142 m short) |

**Why.** Nav2 Humble's controller server has one progress checker, shared by
both controllers. A shorter allowance also aborts the Arrival controller's
own slow final approach, so goals that fail end 0.14–0.22 m short. A real
fix needs a separate progress checker for the Arrival controller, or
Arrival triggered by MPPI's stall, not by the shared checker. That remains
open.

Reports and per-seed records are in `allowance-10s/` and `allowance-20s/`.
Logs and bags are on SSD under `robot_lab_runtime/v10-mppi-arrival-20261128`
(20 s) and `-10s`. Their hashes are in `runtime-sha256.txt`.
