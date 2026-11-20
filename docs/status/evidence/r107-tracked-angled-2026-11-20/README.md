# R10.7: angled step approaches for the tracked vehicle (November 20, 2026)

**Status: implemented, experimental.** On a new wide single step,
`lab_tracked_strong_flippers` climbs a 0.18 m face it meets at 15°, 30° or
45° off square, under the automatic flipper policy. At 15° it also comes
down the far edge to the floor (2 of 2). Descending at 30° or 45° rolls
past the 0.35 rad roll hold, so the vehicle stops tilted on the edge; that
is the measured limit. Gazebo with the private native track engine.

## What changed

- **New world `terrain_step_wide`**
  (`robot_lab_maps/tools/terrain_step_wide.py`; registered): one slab of
  the source 0.18 m rise, 3 m deep and 6 m wide (x 0–3, y −7 to −1). The
  terrain_stairs boxes are 1.1 m wide or have other boxes next to them, so
  angled runs on the 11-19 course hit a corner or a double face (kept:
  `angle-left-017` reared at a 0.36 m face, `angle-right-017` met a corner
  and held on roll).
- **The lower surface beyond an edge is measured out to 2.0 m**
  (`terrain_ahead`), no longer only inside the 1.4 m band:
  - From the camera's 0.41 m height, the floor 0.18 m below an edge is
    first visible 1.16 m ahead. The band held too few of those points, so a
    lone curb read as an unseen ledge, and the vehicle stopped 1 m before
    it (the `wide-15deg` stop).
  - A 0.9 m cliff's floor first shows at about 2.5 m and stays unseen. The
    ledge depth threshold is now 0.25 m, giving a clear margin over 0.18 m
    steps.
- **The 3 s step memory also covers deeper treads in view.** At the head
  of a descending flight the tread just below is too close to see, and the
  window then shows the deeper treads beyond. Before this change that read
  as a 0.36 m ledge (`reg-over`).

## Measured (`terrain_step_wide`, auto flippers, 0.15 m/s; each approach aimed to meet the face near y −4)

| Run | Approach (off square) | Climb | Descent off the far edge | Max roll | Result |
|---|---|---|---|---|---|
| `wide-15deg` | 15° | onto the step | — (stops at x 2.0) | 0.14 rad | on the step, level; a `ledge` stop before the change above |
| `wide-30deg` | 30° | onto the step | — | 0.23 rad | **on the step, level** |
| `wide-45deg` | 45° | onto the step | — | 0.29 rad | **on the step, level** |
| `over-15deg`, `reg2-over-15deg` | 15° | onto the step | **down to the floor** | 0.23 rad | **passed 2/2** (to x 4.5 in 52.5 s) |
| `over-30deg` | 30° | onto the step | rolled to 0.352 rad at the edge, then `hold`; stayed on the edge (roll 0.38) | 0.38 rad | **not passed** |
| `over-45deg` | 45° | onto the step | rolled to −0.367 rad, then `hold` (roll −0.40) | 0.42 rad | **not passed** |

### Regressions on the final policy

| Run | Course | Result |
|---|---|---|
| `reg2-over` | `terrain_stairs_over`: up five steps and down five | passed, 96.4 s, level on the floor |
| `reg2-flight-ledge` | `terrain_stairs_flight`: five steps up, then the 0.9 m end | stops at x 2.54, as before |
| `reg2-single-step` | 11-19 single-step course | passed, ends at y 1.540 (11-19: 1.539) |
| `reg-over` | up-and-over, before the memory change | not passed: `ledge` at the top edge (deeper tread in view) |

## Limits

- Angled *descents* beyond 15° are not handled. One track drops first, and
  the roll hold (0.35 rad) stops the vehicle on the edge. The policy does
  not steer, so squaring up to the edge is up to the operator or a planner.
- One vehicle, one step height, Gazebo only.
- The step memory (3 s) can carry the vehicle over a deeper drop that comes
  within 0.45 m of a measured stair step.

## Reproduce

```bash
python3 ../flipper-stairs-2026-11-14/launch_tracked.py <dir>/launch.log robot_model:=lab_tracked_strong_flippers \
  map_name:=terrain_step_wide spawn_x:=-3.0 spawn_y:=-4.8 spawn_yaw:=0.2618
CASE_OUTPUT=<dir>/auto CASE_FLIPPERS=auto CASE_DURATION=90 CASE_STOP_X=4.5 \
  python3 ../r107-tracked-over-2026-11-20/stairs_probe.py
```

Traces are on SSD under `robot_lab_runtime/tracked-angled-20261120` (hashes
in `runtime-sha256.txt`).
