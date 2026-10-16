# R5.5/R5.6: wheeled nav_obstacle missions over goal seeds 2 and 3 (2026-10-15)

The October 13 rounds ran one goal seed per cell
([4WS/mecanum](../r56-4ws-missions-2026-10-13/README.md),
[cars](../r55-car-missions-2026-10-13/README.md)). Round 35 (`371f724`,
21:53-00:11) repeated every cell with `scripts/sim_nav_mission.sh` and
`--seed 2` and `--seed 3`. That is 64 missions in MuJoCo, PyBullet, Gazebo
and Isaac:

- the 4WS car in its ackermann, in_phase, crab and pivot patterns;
- the mecanum car;
- the Ackermann, rear-steer and anti-Ackermann cars.

Each seed draws a different detour goal A and near-obstacle goal B. The
mission is: detour to A, cancel and stop on the way, resume, then go to B.
It passes on Nav2 success at both goals, with final errors against truth
within 0.30 m.

Per-cell results: [report.md](report.md) and [report.json](report.json).
Raw reports: `<sim>_<cell>_seed<N>_nav_obstacle.json`.

## Results: 37 of 64 pass (round 35; crab, in_phase and the cars since fixed)

| Cell | MuJoCo s2 / s3 | PyBullet s2 / s3 | Gazebo s2 / s3 | Isaac s2 / s3 | Passed |
|---|---|---|---|---|---|
| 4WS ackermann | pass / pass | pass / pass | pass / pass | pass / pass | **8/8** |
| 4WS pivot | pass / pass | pass / pass | pass / pass | pass / pass | **8/8** |
| mecanum | pass / pass | pass / pass | pass / pass | pass / pass | **8/8** |
| 4WS crab | B aborted / B aborted | B aborted / B aborted | B aborted / B aborted | B aborted / B aborted | **0/8** |
| 4WS in_phase | B aborted / B aborted | B aborted / B aborted | B aborted / B aborted | B aborted / B aborted | **0/8** |
| ackermann_car | B 0.34 m / pass | pass / B 0.31 m | pass / B 0.33 m | B 0.36 m / pass | 4/8 |
| rear_steer_car | B aborted (1.90 m) / B 0.33 m | pass / pass | pass / pass | B 0.33 m / pass | 5/8 |
| anti_ackermann_car | pass / B 0.33 m | B 0.30 m / pass | pass / pass | B aborted (1.99 m) / B 0.32 m | 4/8 |

**4WS ackermann, pivot and the mecanum car pass all 24 runs.** Both goals
end 0.01-0.20 m from truth.

**Crab and in_phase fail at goal B in all 16 runs.** Goal A always
succeeded (0.01-0.09 m). On the way to B:

- the drive stopped 0.55-1.1 m from the nearest obstacle, in the open,
  2-4 m short of the goal;
- the controller (DWB) then commanded zero velocity for about 30 s, until
  the progress checker aborted ("Failed to make progress");
- the body held the heading it had at A (about -150°). In these patterns
  the wheels stay parallel, so the base cannot turn while it translates.

Pivot and ackermann reach the same goals from the same A. Seed 1's goal B
was east of A, and crab and in_phase passed it on October 13.

So the October 13 summary "every 4WS pattern passes in every backend" held
for seed 1 only. The cause and fix follow below.

**The cars miss goal B by a few centimetres.** Their near-obstacle errors
are 0.26-0.36 m in every backend, straddling the 0.30 m limit.

- Two runs aborted B outright: rear-steer in MuJoCo, seed 2, and
  anti-Ackermann in Isaac, seed 2.
- Goal A also ends 0.11-0.22 m off, with 26-33° heading errors at the
  Gazebo detour.
- A car cannot turn on the spot, and its Hybrid-A* and Regulated Pure
  Pursuit stack settles its arrivals that coarsely.

The support matrix accepts at most 0.25 m, so every car cell fails its
screen (see [support-matrix-2026-10-15](../support-matrix-2026-10-15/README.md)).

Since fixed: the cars had kept Nav2's 0.25 m goal tolerance. With 0.1 m
and a wider planner inflation they pass 35 of 36 over seeds 1-3
([car-missions-2026-10-16](../car-missions-2026-10-16/README.md)).

## Crab and in_phase: cause and fix (rounds 38-41, 2026-10-16)

**A suspected cause that was not it (round 38, `2445af7`,
[crab-resteer-only/](crab-resteer-only/)).** Reversing a sideways command
swung every wheel through 180° with the wheels stopped, which was suspected
to cause the stall.

- Fixed in `d99c5ea`: the drive keeps the equivalent wheel angle and
  reverses the spin instead, with a test.
- The four MuJoCo reruns and the first Gazebo rerun still aborted goal B,
  1.66-3.98 m short. The queue was stopped there.

**The cause (round 39, [crab-diagnosis/](crab-diagnosis/)).** MuJoCo crab,
seed 2, ran again with DWB's per-cycle evaluations recorded
(`EXTRA_BAG_TOPICS="/evaluation ..."`, now accepted by
`scripts/sim_nav_mission.sh`). During the stall
([summary](crab-diagnosis/dwb-evaluation-at-stall.json)):

- DWB scored 2939 trajectories. The parallel-steering critic made 2760
  illegal, because they translate and turn at once.
- The 138 legal translations all moved forward or sideways: DWB sampled
  `vx` from 0 to 0.3 m/s only. Goal B lay behind the base.
- The 40 legal pivots were penalised by the parallel-steering critic, and
  none shortened the goal distance within the 1.7 s horizon.
- Standing still scored best (20.07 against 20.15 and 20.59), every cycle,
  until the progress checker aborted.
- Seed 1's goal B lay in front of the base, so forward translation was
  enough there.

**The fix (`ae2989c`).** Parallel wheels can drive backwards, so the
`omni_parallel` motion model (crab, in_phase) now samples `vx` from
-0.3 to 0.3 m/s, with a launch test.

**Result (rounds 40-41, `ae2989c`,
[crab-backward-sampling/](crab-backward-sampling/)): 24 of 24 pass.** That
covers crab and in_phase, seeds 1-3, in MuJoCo, PyBullet, Gazebo and Isaac.
Goal A ends 0.006-0.106 m and goal B 0.056-0.159 m from truth.

## Limits

- Two seeds per cell in round 35, one world; three seeds for crab and
  in_phase since the fix.
- DWB's scores were recorded for one stalled run (MuJoCo crab, seed 2).
  The other 15 round-35 stops are attributed to the same cause from their
  identical traces, not from their own recordings.
