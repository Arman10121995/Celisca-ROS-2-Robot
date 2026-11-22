# V5: drone flights past moving people, repeated (November 22, 2026)

**Status: 3 of 3 passed.** This repeats the [November 13 R5.11
protocol](../drone-moving-2026-11-13/README.md) on current code, with
`scripts/qualify_drone_moving.py` unchanged.

The setup:
- PX4 X500 with the depth camera (`x500_depth`) in Gazebo, with RTAB-Map
  and the observed-obstacle route layer (`aerial_obstacles:=mapped_depth`).
- World `aerial_moving`: two scripted walkers cross the route at x = 0 and
  x = 5 m.
- The drone checks takeoff clearance, takes off and flies from (−10, 0) to
  (10, 0, 1.2).
- The depth camera bridge is stopped 3 s after the goal is reached, to
  simulate sensor loss.

A run passes when the goal is reached, the drone responds at least once to
the moving people, every true drone–walker distance stays above 0.6 m, and
the drone holds and then lands after the camera loss.

| Run | Result | Closest walker 0 / walker 1 | Responses | Goal reached | Camera loss → hold / land |
|---|---|---|---|---|---|
| 1 | **pass** | 1.75 m / 2.25 m | 2 walkway holds, then overflight at 3.5 m | 163 s | 1.94 s / 17.9 s |
| 2 | **pass** | 1.65 m / 2.03 m | 2 walkway holds, then overflight at 3.45 m | 171 s | 1.93 s / 17.1 s |
| 3 | **pass** | 1.70 m / 1.79 m | 2 walkway holds, then overflight at 3.5 m | 149 s | 1.73 s / 17.9 s |

November 13 runs 32–34 on the code of that day: 2 of 3 passed, with closest
approaches of 1.08–2.32 m. Today every run took the same strategy: two
look-both-ways holds, then flying over each walkway. None replanned or
evaded, so those two responses were not exercised.

Each run restarted PX4's estimator once before takeoff, through the resting
heading gate. Heading drift was then 0.004–0.005 °/s.

## Limits

- One world, Gazebo only, and scripted walkers that do not react to the
  drone.
- The passing distances come from the scripted walker trajectory, because
  Fortress publishes no actor poses (see the November 13 receipt).
- Each report lists `leftover_pids`. These were the launch's own processes
  at report time; none were still running afterwards.

## Reproduce

```bash
docs/status/evidence/v5-drone-moving-2026-11-22/run_repeats.sh <evidence-dir>
```

Launch logs are on SSD under `robot_lab_runtime/v5-drone-moving-20261122`;
their hashes are in `runtime-sha256.txt`.
