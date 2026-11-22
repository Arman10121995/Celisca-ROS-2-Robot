# V11: concurrent experiments repeated, and three simulators at once (November 22, 2026)

**Status: the October 15 protocol passed again, and Gazebo + MuJoCo +
PyBullet now run Nav2 goals at the same time.** The first triple runs
found a launch bug that kept every Gazebo coordinator job from moving.
It is fixed (implemented, experimental) and both triples after the fix
passed, 6/6 jobs.

The setup repeats the [October 15 R8.3 round](../r83-concurrency-2026-10-15/README.md):
- the coordinator `ros2 run robot_lab_benchmark robot-lab-concurrent`;
- Bumperbot, `mode:=nav`, `nav_obstacle`, `odometry_source:=wheels`, with
  the joystick disabled, as the GUI queue does;
- the built-in Nav2 workload: (−3.8, −3.3, 0°), then (−5.5, −7.0, 180°);
- a 0.15 m / 5° truth contract held for one simulated second;
- a 4096 MB / 800 % budget per job (Gazebo 6144 MB).

Each coordinator run took the shared physics lock for its own duration.
`summary.json` comes from `summarize.py`, in October's fields.

| Run | Jobs (parallel) | Result | Goal errors (truth) | Wall | Real-time factor | Peak RSS |
|---|---|---|---|---|---|---|
| A, serial | MuJoCo | **pass** | 0.075 m 1.1° / 0.076 m 2.2° | 84 s | 0.97 | 1798 MB |
| | PyBullet | **pass** | 0.048 m 1.5° / 0.042 m 0.4° | 96 s | 0.87 | 1240 MB |
| B1, both at once | MuJoCo | **pass** | 0.049 m 1.2° / 0.059 m 0.5° | 82 s | 0.96 | 1820 MB |
| | PyBullet | **pass** | 0.058 m 0.8° / 0.035 m 0.3° | 93 s | 0.87 | 1219 MB |
| B2, both at once | MuJoCo | **pass** | 0.072 m 1.0° / 0.077 m 2.8° | 86 s | 0.96 | 1771 MB |
| | PyBullet | **pass** | 0.056 m 1.5° / 0.049 m 1.7° | 93 s | 0.88 | 1205 MB |
| C, MuJoCo budget 300 MB | MuJoCo | **stopped: resource_budget_exceeded** at 395 MB | none | 5 s | | 395 MB |
| | PyBullet | **pass** | 0.064 m 2.3° / 0.064 m 2.3° | 91 s | 0.87 | 1233 MB |
| E1, three at once | Gazebo | **failed**: never moved, Nav2 status 6 | none | 57 s | 0.95 | 2190 MB |
| | MuJoCo, PyBullet | **pass, pass** | ≤ 0.089 m, ≤ 2.1° | 86, 99 s | 0.93, 0.81 | 1774, 1215 MB |
| E2, three at once | Gazebo | **failed**: the same | none | 57 s | 0.95 | 2164 MB |
| | MuJoCo, PyBullet | **pass, pass** | ≤ 0.071 m, ≤ 2.8° | 85, 99 s | 0.94, 0.80 | 1815, 1215 MB |
| F, Gazebo alone | Gazebo | **failed**: the same | none | 56 s | 0.98 | 2162 MB |
| F2, Gazebo alone, fixed | Gazebo | **pass** | 0.062 m 1.2° / 0.028 m 0.6° | 82 s | 0.98 | 2276 MB |
| E3, three at once, fixed | Gazebo | **pass** | 0.066 m 0.0° / 0.018 m 1.2° | 85 s | 0.96 | 2288 MB |
| | MuJoCo | **pass** | 0.087 m 1.4° / 0.085 m 0.4° | 88 s | 0.92 | 1967 MB |
| | PyBullet | **pass** | 0.067 m 2.7° / 0.061 m 0.1° | 102 s | 0.77 | 1329 MB |
| E4, three at once, fixed | Gazebo | **pass** | 0.021 m 1.0° / 0.026 m 1.6° | 88 s | 0.96 | 2278 MB |
| | MuJoCo | **pass** | 0.087 m 1.2° / 0.079 m 1.1° | 87 s | 0.92 | 1987 MB |
| | PyBullet | **pass** | 0.072 m 1.1° / 0.029 m 0.8° | 107 s | 0.77 | 1315 MB |

- **Compared with October.** Every A–C outcome repeats, and the goal
  errors are similar. PyBullet now runs at a real-time factor of 0.87,
  against 0.36, so its jobs take about 95 s instead of 205 s. The budget
  stop again came 5 s after start and stopped only its own job.
- **Three simulators at once.** Each job kept its own ROS domain (20–22)
  and Gazebo partition. The three ran together in about the time of the
  slowest. MuJoCo and Gazebo kept their solo real-time factors
  (0.92–0.98). PyBullet dropped from 0.87 to 0.77–0.81 when three ran.
- **Cleanup.** Every job reports `cleanup_complete`, and no process was
  left behind ([survivors.txt](survivors.txt)). The one listed process is
  a `/molar` Nav2 lifecycle manager that started on November 20, before
  this round, and is not from these jobs.
- Each job's Nav2 rejected the first goal one or two times while the
  stack was becoming active, as in October.

## The Gazebo bug: `enable_joystick:=false` removed the velocity mux

The Gazebo job failed the same way three times (E1, E2, and F alone),
and its body moved 7 µm. Its controllers were active, and the progress
checker aborted the first goal. October never paired Gazebo, and the V2
Gazebo missions launch with the joystick on, so neither had seen this.

The cause was in `simulated_robot.launch.py`: `enable_joystick:=false`
skipped the whole `joystick_teleop.launch.py`. That file also starts
`twist_mux` and `twist_relay`, which carry Nav2 and GUI velocity to
`robot_lab_controller` on Gazebo. MuJoCo and PyBullet still moved, which
shows they have their own route for the commands. The GUI's
concurrent-experiment queue forces `enable_joystick:=false`, so every
Gazebo job queued from the GUI was affected.

The fix: `joystick_teleop.launch.py` takes `enable_joystick` and
conditions only `joy_teleop` and `joy_node` on it, and the bringup always
includes it. A test, `src/robot_lab_controller/test/test_joystick_teleop_launch.py`,
checks that the mux and relay start with the joystick off. F and F2 ran at
the same revision, 6cdebdd2, and differ only by this fix.

## Revisions

A–C ran at be0cadc6. The other agent committed during the round, so E1
ran at 65224e21 and E2–E4 at 6cdebdd2. Those commits touched arm,
MuJoCo-world and X500 launch code, not the Nav2 or Bumperbot path; each
`run.json` records its revision and tree state. The fix was uncommitted
for F2, E3 and E4, and is in the commit with this receipt.

## Limits

- Seeds are passed to the jobs, not applied by the simulators.
- Contacts and swept clearance are not assessed.
- Isaac was not paired, and no run had more than three jobs.
- Resetting during a workload and cancel were not repeated (October D2
  stands).

## Reproduce

```bash
docs/status/evidence/v11-concurrency-2026-11-22/run_round.sh            # A-E2
docs/status/evidence/v11-concurrency-2026-11-22/run_round.sh after_fix  # F2, E3, E4
python3 docs/status/evidence/v11-concurrency-2026-11-22/summarize.py <run> <name>... > summary.json
```

Logs, traces and resource samples are on SSD under
`robot_lab_runtime/v11-concurrency-20261122`. Their hashes are in
`runtime-sha256.txt`.
