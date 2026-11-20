# R10.6: push-disturbance recovery mission for the octopod (November 20, 2026)

**Status: implemented, experimental.** The gait's bounded slip/support-loss
recovery (`gait_recovery.SlipRecovery`) had been unit-tested only. On
11-19 the octopod crossed ice without slipping, and slopes steep enough to
make it slip were outside its terrain envelope. A sideways push now
triggers the recovery physically. The octopod pauses in stance, resumes at
half speed, returns to full speed and finishes its walk: in 4 of 4 pushes
that tripped the support check (60–100 N).

## What changed

- **`gait_simulator.launch.py`:** new arguments `perturbation_force_n`,
  `perturbation_start_s`, `perturbation_duration_s` and `perturbation_axis`
  (default 0 N, i.e. off). They pass through to the MuJoCo spawner's
  existing bounded force pulse on the base: world axis, simulation time,
  0.3 s here. Other backends ignore them. A test checks the defaults.
- **`push_probe.py`:** the 11-19 slip probe with `GAIT_PUSH=force,start,
  duration,axis`. It records every slip-recovery state it sees.

The gait controller, its recovery budget (3 per 60 s), the 80 ms
support-loss and slip triggers and the 10 s recovery timeout are
unchanged.

## Measured (`lab_octopod`, 2.24 kg, MuJoCo, `gait_low_relief`, terrain gait, 0.045 m/s for 25 s, push at t = 12 s for 0.3 s along +y)

| Push | Forward progress (x) | Shoved sideways (y) | Recovery | Pause → resume | Max tilt | Result |
|---|---|---|---|---|---|---|
| 10 N | 0.991 m | 0.004 m | not triggered | — | 0.002 rad | walk completed |
| 20 N | 0.991 m | 0.006 m | not triggered | — | 0.004 rad | walk completed |
| 30 N | 0.992 m | 0.011 m | not triggered | — | 0.004 rad | walk completed |
| 60 N | 0.877 m | 0.309 m | support loss for 80 ms | 1.28 s | 0.013 rad | **recovered**, walk completed |
| 60 N (repeat) | 0.996 m | 0.311 m | not triggered (support stayed above the threshold) | — | 0.013 rad | walk completed |
| 80 N | 0.875 m | 0.735 m | support loss | 1.32 s | 0.028 rad | **recovered**, walk completed |
| 100 N | 0.902 m | 1.319 m | support loss | 1.48 s | 0.030 rad | **recovered**, walk completed |
| 100 N (repeat) | 0.847 m | 1.304 m | support loss | 1.58 s | 0.039 rad | **recovered**, walk completed |

Each recovery followed the same sequence:

1. The fault paused the gait (speed scale 0) in stance.
2. After stance, at least half the legs in support and 1 s settled, it
   resumed at half speed.
3. 3 s later it returned to full speed.

No push used more than 1 of the 3 recoveries in the window. After every walk the
held stance drifted under 3 mm. The probe's own "progress" number is the
Euclidean distance, so it includes the shove. The forward column is the x
progress.

## Limits

- The trigger was support loss. The contact-kinematic slip trigger was not
  reached: the feet slid with the body rather than slipping under a
  stance.
- One model (octopod), one world, MuJoCo only. Pushes are lateral and
  timed, not random.
- 60 N is at the trigger threshold: once it recovered, once it slid 0.31 m
  without tripping.

## Reproduce

```bash
GAIT_CASE_DIRECTORY=<new dir> GAIT_MAP=gait_low_relief GAIT_WALK_S=25 GAIT_SPEED=.045 GAIT_PUSH="80,12,0.3,1" \
  python3 docs/status/evidence/r106-gait-push-2026-11-20/push_probe.py
```

Traces are on SSD under `robot_lab_runtime/gait-push-20261120` (hashes in
`runtime-sha256.txt`).
