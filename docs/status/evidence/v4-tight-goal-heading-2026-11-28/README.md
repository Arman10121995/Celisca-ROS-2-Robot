# Goal B heading for robots whose heading is not scored (November 28, 2026)

**Status: implemented, experimental.** Every rerun now plans goal B; 4 of
7 failed cells pass.

`scripts/sim_nav_mission.py` aimed goal B along the straight line from the
robot to the goal. Beside an obstacle, that heading often points into the
obstacle. A car or forward-arc footprint does not fit there, so Hybrid-A*
found no path. This caused these earlier failures:
- the Isaac Ackermann car ([V3](../v3-car-matrix-2026-11-21/README.md));
- Go2 and B2 forward-arc cells ([V4](../v4-legged-nav-2026-11-22/README.md)).

**Change.** When the final heading is not scored
(`--max-yaw-error-deg 0`), goal B's heading now runs parallel to the
nearest obstacle edge, on the side closer to the approach bearing.
Robots whose heading is scored are unchanged. A unit test is in
`scripts/test_sim_nav_mission_heading.py`.

**Reruns** at revision 7d3fe5dd plus the change, with each cell's original
arguments:

| Cell | Before | Now | Resume A / goal B (truth) |
|---|---|---|---|
| Isaac Ackermann car | goal B: no path, aborted 7.8 m short | **passed** | 0.112 / 0.060 m |
| MuJoCo Go2 seed 2 | goal B: no path | **passed** | 0.014 / 0.212 m |
| Isaac Go2 seed 3 | goal B: no path | **passed** | 0.061 / 0.150 m |
| MuJoCo B2 seed 3 | goal B: no path | **passed** | 0.263 / 0.165 m |
| MuJoCo B2 seed 1 | goal B: no path | goal B reached 0.106 m; resume A 0.346 m (limit 0.30) | — |
| MuJoCo B2 seed 2 | goal B: no path | both reached; goal B 0.346 m (limit 0.30) | 0.137 / 0.346 m |
| Gazebo Go2 seed 1 | lethal start after the cancel | resume A 0.151 m; goal B aborted 0.296 m short | — |

No cell failed on "no valid path" any more. The remaining failures are B2
end accuracy just over the limit and one Gazebo Go2 abort near the goal.

## Reproduce

```bash
docs/status/evidence/v4-tight-goal-heading-2026-11-28/run_round.sh
```

Reports are copied here. Logs, bags and traces are on SSD under
`robot_lab_runtime/v4-tight-goal-heading-20261128`. Their hashes are in
`runtime-sha256.txt`.
