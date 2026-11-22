# V9: Nav2 missions under seeded scan faults (November 22, 2026)

**Status: 15 of 15 passed, including a harder stratum. No fault cost a
goal; a 15 s scan blackout cost up to 36 s and pushed one goal to 0.28 m.** `run_round.sh` runs the Bumperbot `nav_obstacle` mission
(`scripts/sim_nav_mission.sh`) on MuJoCo for goal seeds 1–3. The steps
are a detour goal, a cancel after 1 m, a resume, and a goal near an
obstacle; each goal must end within 0.30 m of truth.

The three conditions are:
- **clean**;
- **noise-dropout** (`noise-dropout.yaml`): Gaussian range noise of
  0.05 m and 20 % of scans dropped, for the whole run;
- **occlusion-window** (`occlusion-window.yaml`): half the scan
  occluded from 20 s to 50 s of simulation time.

The seeded `sensor_fault_relay` sits between the simulator's
`/scan_clean` and `/scan`. AMCL, Nav2 and the costmaps read the faulted
scan; truth is untouched. The runs used revision c9d499df, one shared lock
hold per run.

Each cell is wall time / truth error. Resume A and goal B are shown; cancel
A passed in every run.

| Seed | Clean: resume A | Clean: goal B | Noise-dropout: resume A | Noise-dropout: goal B | Occlusion: resume A | Occlusion: goal B |
|---|---|---|---|---|---|---|
| 1 | 21.3 s / 0.128 m | 22.8 s / 0.089 m | 26.2 s / 0.063 m | 26.8 s / 0.126 m | 25.1 s / 0.072 m | 23.2 s / 0.084 m |
| 2 | 26.9 s / 0.040 m | 61.9 s / 0.106 m | 27.8 s / 0.042 m | 52.9 s / 0.118 m | 29.2 s / 0.048 m | 51.4 s / 0.110 m |
| 3 | 26.5 s / 0.065 m | 43.7 s / 0.086 m | 29.3 s / 0.035 m | 34.0 s / 0.128 m | 26.9 s / 0.011 m | 33.3 s / 0.128 m |

- **No goal was lost.** AMCL's beam model and the costmaps absorbed 5 cm
  noise with 20 % dropout, and a 30 s half-scan occlusion.
- **Time cost.** Resume A was 1–5 s slower under faults in every seed. Goal
  B did not show a fault cost: the clean runs were slower on seeds 2–3
  (61.9 and 43.7 s), which reflects run-to-run variation in Nav2's
  near-obstacle approach. Three seeds do not separate a small fault cost
  from that variation.
- **Not a recovery-time measurement yet.** No condition caused a failure or
  a recovery behaviour, so there was nothing to recover from (see the
  harder stratum below).

## Harder stratum (same protocol, revision 8c8e8599)

- **bias-window** (`bias-window.yaml`): every range reads 0.3 m long from
  20 s to 60 s.
- **blackout-window** (`blackout-window.yaml`): the whole scan is occluded
  (no returns) from 20 s to 35 s.

| Seed | Bias: resume A | Bias: goal B | Blackout: resume A | Blackout: goal B |
|---|---|---|---|---|
| 1 | 25.6 s / 0.157 m | 29.6 s / 0.073 m | 23.8 s / **0.281 m** | 27.2 s / 0.054 m |
| 2 | 27.7 s / 0.099 m | 51.2 s / 0.132 m | **62.5 s** / 0.133 m | 59.1 s / 0.107 m |
| 3 | 24.2 s / 0.155 m | 43.0 s / 0.100 m | 23.0 s / 0.116 m | 43.3 s / 0.119 m |

- **All 6 passed.** None of the launch logs shows a progress failure, a
  spin/back-up/wait recovery or a lethal start (count 0 each).
- **Bias** raised the resume error to 0.10–0.16 m (clean 0.04–0.13 m),
  with times close to clean.
- **Blackout** has the largest effects:
  - seed 2's resume took 62.5 s against 26.9 s clean, 36 s spent
    re-acquiring after the scan came back;
  - seed 1 ended its resume 0.281 m from the goal, near the 0.30 m limit.
  - Nav2 kept the goal through the blackout without any recovery
    behaviour.
- So the measurable cost of these faults is time and end error, not a
  recovery sequence. A fault that forces Nav2's recoveries still has to be
  found.

## Limits

- One robot, one map, one backend; scan faults only.
- The relay's per-run fault digests are on SSD under
  `robot_lab_runtime/sensor_faults/`. Their hashes are in
  `relay-trace-sha256.txt`.

## Reproduce

```bash
docs/status/evidence/v9-sensor-faults-2026-11-22/run_round.sh
```

Reports are copied here, with trace paths relative to the runtime root.
Launch logs, TF bags and traces are on SSD under
`robot_lab_runtime/v9-sensor-faults-20261122`. Their hashes are in
`runtime-sha256.txt`.
