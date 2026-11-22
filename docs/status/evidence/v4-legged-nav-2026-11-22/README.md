# V4: legged Nav2 missions repeated on current code (November 22, 2026)

**Status: 13 of 24 cells passed at the default goal radius; A1 kept
6/6, while G1 and B2 regressed.** With the radius that October's goals
imply, B2 passed 2/3, as in October.

`run_round.sh` repeats the [October 13–16 P7 mission](../p7-legged-navigation-2026-10-13/README.md):
- `scripts/sim_nav_mission.sh <sim> <robot> nav_obstacle` in nav mode;
- a detour goal, a cancel after 1 m, a resume, and a goal near an
  obstacle;
- each goal must end within 0.30 m of simulator truth, with the heading
  not checked;
- measured from the trunk (`--base base`; `pelvis` for the G1);
- seeds 1–3.

The runs used revision ed128652, from 08:13 to 09:14, one shared lock
hold per run.

| Robot | MuJoCo | Isaac | Gazebo | October |
|---|---|---|---|---|
| Go2 | 2/3 | 2/3 | 2/3 | MuJoCo 3/3, Isaac 2/3, Gazebo 3/3 |
| A1 | **3/3** | **3/3** | — | 3/3, 3/3 |
| G1 | 0/3 | 1/3 | — | 3/3, 2/3 |
| B2, default radius 0.25 m | 0/3 | — | — | — |
| B2, `--robot-radius 0.6` | 2/3 | — | — | 2/3 |

## Failures

| Cell | What happened |
|---|---|
| Gazebo Go2 seed 1 | After the cancel, the forward-arc Go2 stood at (−6.84, −8.02), facing away from goal A and in lethal cost beside the wall. "Starting point in lethal space"; resume A and goal B both aborted. |
| MuJoCo Go2 seed 2, Isaac Go2 seed 3 | Resume A passed (0.03 m, 0.07 m). Hybrid-A*/GridBased found no path to the near-obstacle goal B. |
| MuJoCo G1 seeds 1–3, Isaac G1 seeds 1 and 3 | Nav2 reported success, but truth was 0.30–0.43 m from the goal. The estimate ended up to 0.55 m off truth (range 0.02–0.55 m over all G1 goals), against 0.07–0.19 m in the October seed-2 run. MuJoCo seed 1 and Isaac seed 3 also aborted goal B near the obstacle. |
| MuJoCo B2, default radius, seeds 1–3 | Hybrid-A* found no path to goal B. These goals sit 0.43–0.50 m from obstacles. |
| MuJoCo B2, r 0.6, seed 1 | The planner aborted resume A 1.0 m short, then goal B. |

## Why the goals are not October's for B2

`sim_nav_mission.py` (unchanged since October) chooses goal B with a
clearance just above `--robot-radius`. The October receipt does not record
the radius used.
- October's Go2 and G1 goal-B clearances (0.35–0.55 m) fit the default
  0.25 m.
- October's B2 clearances (0.65–0.89 m) fit only a larger radius.
- An offline replay of the goal choice matched the October B2 seed 1
  goals at 0.6. However, the same replay did not reproduce today's
  default-radius goals, so the replay is not exact.

The B2 r0.6 runs are therefore labelled "inferred". Their goals are
0.70–0.80 m from obstacles, not October's exact goals.

## What changed since October

These deliberate, separately measured changes touch these cells:
- Go2 global inflation 0.7 m at cost scaling 2.0 (26b961e4);
- Go2 back to forward arcs (56d3edc8, ec6210ab);
- G1 inflation 0.8 m (f276b839) and a 0.12 m G1 goal tolerance (51c75430);
- MuJoCo articulation integrated implicitly (ec6210ab);
- an Isaac PD armature and floor material (51c75430).

The G1's larger localization error appears on both MuJoCo and Isaac. Its
leg odometry and localization configs are unchanged, so one of the
physics changes is a candidate. That is not yet tested.

## G1 leg-odometry screen (diagnosis)

`scripts/leg_odometry_check.sh <sim> unitree_g1_29dof nav_obstacle` runs
the October screen in slam mode. The G1 walks forward, arcs both ways,
slows and stops, and the screen reports where the leg odometry and the
EKF end relative to truth:

| Backend | Walked | Leg odometry drift | EKF drift | October (MuJoCo, v2) |
|---|---|---|---|---|
| MuJoCo | 6.61 m | 0.574 m (8.7 %) | 0.694 m (10.5 %) | 9.94 m walked, 0.302 m (3.0 %), EKF 2.5 % |
| Isaac | 7.19 m | 4.648 m (65 %) | 4.305 m (60 %) | not screened |

So the G1's odometry itself is worse, and on MuJoCo the same commanded
screen covers a third less ground. The gait has changed.
- The G1 has no articulation contract.
- Its PD actuators were already integrated implicitly on October 12.
- So the October 16 articulation change is not the cause.

A bisect needs older revisions of the spawner and robot configs:
- 71 commits since October's last G1 round (80b52f0) touch these files;
- the Python install is a copy, so each step needs a rebuild;
- the other agent is active, and there is no separate worktree on this
  host.

It is left open.

## Limits

- One map, three seeds per cell.
- The B2 r0.6 goals are inferred, not October's exact goals.
- The causes above are not bisected. The failures are kept as measured.

## Reproduce

```bash
docs/status/evidence/v4-legged-nav-2026-11-22/run_round.sh          # 24 cells
docs/status/evidence/v4-legged-nav-2026-11-22/run_round.sh b2_r06   # B2 at radius 0.6
```

Reports are copied here, with their trace paths made relative to the
runtime root. Launch logs, TF bags and traces are on SSD under
`robot_lab_runtime/v4-legged-nav-20261122`. Their hashes are in
`runtime-sha256.txt`.
