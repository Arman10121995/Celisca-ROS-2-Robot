# V10: Gazebo control comparison repeated at one revision (November 22, 2026)

**Status: 30 of 30 trials ran; all six controllers reached 15 of 15
goals, as on October 13.** `run_round.sh` repeats the
[October 13 Gazebo protocol](../r78-control-benchmark-2026-10-13/README.md)
from 06:20 to 07:14, one shared lock hold per trial. The trials ran at
revisions b803693 and 9278e12. The second differs only in docs: `git diff`
shows no change under `src/` or `scripts/`.

| Setting | Value |
|---|---|
| Robot, map, simulator | Bumperbot, nav_maze, Gazebo |
| Seeds | 11, 23, 37, 41, 53; three goals each, the same 15 goals for every controller |
| Fixed | `global_planning:=a_star_planner local_planning:=pure_pursuit` |
| Under test | the `control:=` FollowPath tracker (robot_lab `tracking_control`) |
| Measurement | Gazebo truth |

## Results ([report.md](report.md), [tracking.md](tracking.md))

| Controller | Goals | Sim time median | Final error median (Oct → now) | Cross-track RMS (Oct → now) | Worst clearance |
|---|---|---|---|---|---|
| pid_controller | **15/15** | 26.7 s | 0.067 → 0.066 m | 6.27 → 6.49 cm | 0.280 m |
| lqr_controller | **15/15** | 27.1 s | 0.075 → 0.073 m | 5.60 → 6.13 cm | 0.304 m |
| mpc_controller | **15/15** | 27.1 s | 0.076 → 0.077 m | 5.72 → 6.48 cm | 0.304 m |
| nonlinear_mpc | **15/15** | 27.0 s | 0.066 → 0.071 m | 6.30 → 6.79 cm | 0.304 m |
| feedback_linearization | **15/15** | 26.7 s | 0.067 → 0.063 m | 6.10 → 6.46 cm | 0.261 m |
| backstepping_controller | **15/15** | 27.1 s | 0.076 → 0.086 m | 5.25 → 5.96 cm | 0.304 m |

- Success and median times match October within 0.4 s.
- Cross-track error to the latest plan, measured by
  `scripts/path_tracking_error.py` on the bags, is 3–14 % higher for
  every controller. The ranking is almost unchanged: backstepping is
  lowest, then LQR, and NMPC is highest in both rounds. MPC moved from
  third to fourth, behind feedback linearization.
- The offline benchmark of the same controllers is numerically identical
  to October's
  ([offline round](../v10-offline-benchmarks-2026-11-22/README.md)). So
  the small rise comes from the closed loop in Gazebo, not from the
  control laws. Its source (plans, timing, simulation) is not separated
  here.
- The worst final error was 0.183 m (PID). The worst clearance was
  0.261 m, and no trial froze (`scripts/tf_freeze_goals.py`).

## Limits

- One robot, one map, one simulator and 15 goals. The trackers all follow
  the same A* plans, so the paths differ little between controllers.
- The cross-track rise is reported, not explained.

## Reproduce

```bash
docs/status/evidence/v10-control-comparison-2026-11-22/run_round.sh
python3 scripts/planner_comparison_report.py <run> --markdown report.md > report.json
python3 scripts/path_tracking_error.py <run> --markdown tracking.md
```

Per-seed records are copied here. Bags, launch logs and traces are on SSD
under `robot_lab_runtime/v10-control-cmp-20261122`. Their hashes are in
`runtime-sha256.txt`.
