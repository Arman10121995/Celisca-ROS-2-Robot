# R10.7: the tracked vehicle goes up a five-step flight and down the other side (November 20, 2026)

**Status: implemented, experimental.** Under the automatic flipper policy,
`lab_tracked_strong_flippers` now climbs five 0.18 m steps to 0.9 m,
crosses the top tread and descends five steps to the floor, 2 of 2 runs
(about 97 s each). The single-step course of 11-19 and the ledge stop of
the [flight run](../r107-tracked-flight-2026-11-20/README.md) still pass.
Gazebo Fortress with the private native track engine.

## What changed

- **New world `terrain_stairs_over`**
  (`robot_lab_maps/tools/terrain_stairs_over.py`; registered in `sim_maps`
  and the environment registry):
  - The flight is rebuilt from 1.1 × 1.1 m treads made of the source's
    0.18 m slabs, with the same material and friction as `platform_08`.
  - Five steps up from x −5.2, a 1.1 m top tread at 0.9 m, and five steps
    down to the floor at x 4.7, leaving 3.3 m before the arena wall.
- **Next-step depth** (`terrain_ahead`): `drop_m` is now the shallowest
  lower level in the band (90th percentile of the heights below the ground
  line), not their median. At the top, the band also held the treads
  beyond the next one; a median of 0.36 m read as a ledge, and the guard
  stopped a 0.18 m descent (`auto-1`).
- **Step memory** (`FlipperPolicy`):
  - At a stair edge the tread just below is too close for the camera to
    see (nothing nearer than about 1.2 m at that depth). So exactly when the
    vehicle starts down, the drop reads as unseen and the guard stopped it
    (`auto-2`).
  - A stair-depth step measured within the last 3 s now keeps counting as
    the step. Once the hull tips, the ledge check (level only) is off
    anyway.
  - On the flight world the 0.9 m end still stops the vehicle. The last
    step there was seen far more than 3 s earlier.

## Measured (`terrain_stairs_over`, spawn (−7.3, −4.0) heading +x, auto flippers, 0.15 m/s; the drive stops at x 6.0)

| Run | Policy | Result | Top reached | Max nose up / down | Notes |
|---|---|---|---|---|---|
| `auto-1` | flight version (median depth) | **not passed**: 5 up, then `ledge` at the top edge (x 0.28) | 49.4 s | | median depth 0.36 m |
| `auto-2` | shallowest-level depth | **not passed**: same stop at x 0.28 | 49.4 s | | the tread below left the camera's view at the edge |
| `auto-3` | + step memory | **not passed**: stalled climbing the *first* riser (nose up 0.20 rad at x −5.46) for 166 s | — | | flipper state matched the passing runs; the host was loaded by another Robot Lab GUI (load average 5.8) |
| **`auto-4`** | + step memory | **passed**: 5 up, 5 down, level on the floor at x 6.0 in 97.0 s | 49.9 s | 0.324 / 0.297 rad | roll ≤ 0.001 rad |
| **`auto-5`** | same | **passed** in 96.8 s | 49.7 s | 0.324 / 0.295 rad | roll ≤ 0.002 rad |
| `single-step-regression` | final | the 11-19 course: up the box side, down the far side, ends at y 1.539 (11-19: 1.539) | — | | no ledge reading now |
| `flight-ledge-regression` | final | `terrain_stairs_flight`: 5 up, stops 1.05 m before the 0.9 m end (x 2.55), as before | 49.3 s | | |

Each step on the descent goes `descend` (front flippers pressed, rear at
rest), nose down to about 0.3 rad, then level.

## Limits

- Straight approaches only: angled approaches to a riser are not run.
- One vehicle, Gazebo only (the native track engine).
- The step memory can carry the vehicle over an unseen drop that comes
  within 3 s (0.45 m at 0.15 m/s) of a measured stair step.
- `auto-3`'s first-riser stall is a single failure. It was not reproduced,
  and its cause is not established.

## Reproduce

```bash
python3 ../flipper-stairs-2026-11-14/launch_tracked.py <dir>/launch.log robot_model:=lab_tracked_strong_flippers \
  map_name:=terrain_stairs_over spawn_x:=-7.3 spawn_y:=-4.0 spawn_yaw:=0.0
CASE_OUTPUT=<dir>/auto CASE_FLIPPERS=auto CASE_DURATION=180 CASE_STOP_X=6.0 python3 stairs_probe.py
```

Traces are on SSD under `robot_lab_runtime/tracked-over-20261120` (hashes in
`runtime-sha256.txt`).
