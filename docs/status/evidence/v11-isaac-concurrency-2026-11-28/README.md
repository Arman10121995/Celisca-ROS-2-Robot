# V11: Isaac in the concurrent-experiment coordinator (November 28, 2026)

**Status: 5 of 5 jobs passed once Isaac had a CPU budget that fits it.**
Isaac alone passed, and Isaac + MuJoCo at once passed twice. The
setup is the [November 22 round](../v11-concurrency-2026-11-22/README.md)'s
protocol, unchanged:
- `ros2 run robot_lab_benchmark robot-lab-concurrent`;
- Bumperbot, `mode:=nav`, `nav_obstacle`, wheel odometry, joystick off;
- the built-in Nav2 workload of two goals, with a 0.15 m / 5° truth
  contract.

The runs used revision d91e9465, one shared lock hold per coordinator run.

## First pass: the 800 % CPU budget stopped Isaac (`summary-budget-800.json`)

All three Isaac jobs (alone, and twice beside MuJoCo) ended as
`resource_budget_exceeded`. Their memory stayed within budget (6.6 GB of
8 GB). Isaac's own process tree used 1000–1050 % CPU, and the coordinator
stops a job after three samples over its budget. The MuJoCo jobs beside it
passed (0.041 / 0.028 m and 0.078 / 0.044 m). The budget did what it is for:
it stopped only its own job.

## With `max_cpu_percent: 1200` for Isaac (`summary.json`)

The host has 12 cores. The other settings are unchanged.

| Run | Job | Result | Goal errors (truth) | Wall | Real-time factor | Peak RSS / CPU |
|---|---|---|---|---|---|---|
| I0, alone | Isaac | **pass** | 0.031 m 0.1° / 0.017 m 1.1° | 112 s | 0.90 | 6582 MB / 1076 % |
| I1, together | Isaac | **pass** | 0.052 m 0.2° / 0.071 m 0.4° | 125 s | 0.77 | 6578 MB / 917 % |
| | MuJoCo | **pass** | 0.066 m 1.1° / 0.085 m 2.8° | 83 s | 0.96 | 1952 MB / 897 % |
| I2, together | Isaac | **pass** | 0.037 m 0.3° / 0.034 m 1.3° | 127 s | 0.77 | 6543 MB / 916 % |
| | MuJoCo | **pass** | 0.089 m 1.1° / 0.061 m 0.8° | 82 s | 0.97 | 1901 MB / 916 % |

- MuJoCo kept its solo real-time factor (0.96) beside Isaac. Isaac dropped
  from 0.90 to 0.77.
- Every job reports `cleanup_complete`. The `survivors.txt` lines are the
  round's own runner shell, whose path contains "isaac". No simulator or
  Nav2 process was left behind.

Isaac with Gazebo or PyBullet, and more than two jobs at once, were not run.

## Reproduce

```bash
docs/status/evidence/v11-isaac-concurrency-2026-11-28/run_round.sh
```

Logs, traces and resource samples are on SSD under
`robot_lab_runtime/v11-isaac-concurrency-20261128` (first pass in
`budget-800/`). Their hashes are in `runtime-sha256.txt`.
