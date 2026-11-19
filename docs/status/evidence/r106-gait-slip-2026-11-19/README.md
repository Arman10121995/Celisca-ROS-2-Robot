# R10.6: a slip mission for the gait recovery (November 19, 2026)

**Status: implemented, experimental; recovery not exercised.** The octopod
walked across an icy plate on MuJoCo without a measured slip. Slopes steep
enough to slip at that friction are outside the gait's terrain envelope,
so the gait stops before reaching them. The bounded slip/support-loss
recovery (`gait_recovery.py`) therefore stays unit-tested only.

## What was added

- **World `gait_slip_patch`** (`robot_lab_maps/tools/gait_slip_patch.py`):
  - `gait_low_relief` (same heightfield, spawn and map) with a 0.8 × 1.0 m
    plate of friction 0.05 (about ice on steel), lying 8–12 mm up, just
    above the relief peaks, and starting 0.35 m ahead of the spawn.
  - Registered for Gazebo and MuJoCo.
  - **MuJoCo friction fix in the world:** MuJoCo uses the larger friction
    of two geoms unless one has a higher `priority`. The spawner gives robot
    geoms with authored friction priority 1, so the plate needs priority 2.
    Without it the feet's own friction applied, and friction 0.05 and 0.01
    walked the same distance to the millimetre.
- **`slip_probe.py`:** the November 2 terrain-gait probe with map, walk time
  and speed as parameters. It records `slip_recovery` at every sample.

## Measured (`lab_octopod`, MuJoCo, terrain gait enabled, straight walk)

| Run | Plate | Speed / time | Result |
|---|---|---|---|
| `octopod-1` | flat, μ 0.05, no priority | 0.02 m/s, 40 s | crossed, 0.71 m, no slip (plate friction not applied, see above) |
| `octopod-fast` | same | 0.045 m/s (limit), 25 s | 0.963 m, no slip |
| `octopod-mu001` | μ 0.01, no priority | 0.045 m/s | 0.964 m: same as μ 0.05, which exposed the priority problem |
| `octopod-priority` | μ 0.05, priority 1 | 0.045 m/s | 0.968 m: equal priority still uses the larger friction |
| **`octopod-p2`** | **μ 0.05, priority 2 (the committed world)** | **0.045 m/s, 25 s** | **crossed, 0.996 m, body centre ending 0.25 m past the plate centre; no slip and no recovery; held drift 2.9 mm** |
| `octopod-roll6` | rolled 6° (tan 0.105 > μ) | 0.045 m/s | **stopped before the plate**: "Observed relief/step exceeds source gait reach envelope" (2 cm per foot) |
| `octopod-roll4` | rolled 4°, 0.6 m wide | 0.045 m/s | stopped the same way before the plate |
| `octopod-roll4-flush` | rolled 4°, centre at mid-relief | 0.045 m/s | front feet onto the plate (x −0.29), then the envelope stop |

**Finding:** the octopod's quasi-static gait needs far less friction than
0.05 on level ground, so flat ice does not make it slip. A side slope large
enough to beat that friction (over 2.9°) puts a foot more than the gait's
2 cm terrain envelope off level, so the terrain gait stops safely first. A
physical slip mission inside the envelope would need lower friction still,
or disturbance forces (pushes). The recovery remains unit-tested.

## Reproduce

```bash
python3 src/robot_lab_maps/tools/gait_slip_patch.py
GAIT_CASE_DIRECTORY=<dir> GAIT_MAP=gait_slip_patch GAIT_WALK_S=25 GAIT_SPEED=.045 \
  python3 docs/status/evidence/r106-gait-slip-2026-11-19/slip_probe.py
```

The rolled variants used the same tool with the plate pose and `roll`
changed (see each result's notes). Traces and plant logs are on SSD under
`robot_lab_runtime/gait-slip-20261119`.
