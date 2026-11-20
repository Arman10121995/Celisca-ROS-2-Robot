# R10.7: the tracked vehicle climbs a five-step flight and stops at a ledge (November 20, 2026)

**Status: implemented, experimental.** Under the automatic flipper policy,
`lab_tracked_strong_flippers` (40 Nm, full-press climb) now climbs a whole
flight: five 0.18 m risers with 1.1 m treads, 0.9 m up. It climbed all five
in 3 of 3 runs, reaching the top tread in about 50 s at 0.15 m/s. On the top
tread, a new ledge guard stopped forward drive 1.05 m before the flight's
0.9 m drop. Without the guard, the vehicle had driven off that drop and
flipped. Gazebo Fortress with the private native track engine.

## What changed

- **New world `terrain_stairs_flight`**
  (`robot_lab_maps/tools/terrain_stairs_flight.py`; registered in
  `sim_maps` and the environment registry). It is `terrain_stairs` with its
  five-step flight moved 2 m along +x. In the source the flight starts
  0.8 m from the arena wall, too close for the 0.5 m hull and its flippers
  to meet the first riser head on. The 11-19 runs had climbed the first
  box from its side. The original world is unchanged.
- **The climb ends on a level tread with a riser ahead**
  (`tracked_flippers.FlipperPolicy`):
  - The climb used to end only when the hull was level and no riser was
    ahead. On a flight one always is, so in `auto-1` the vehicle stayed in
    `climb` on the first tread, propped on fully pressed flippers with its
    tracks off the tread, for 70 s.
  - Now 1 s level ends the climb. The next state is `approach` if a riser
    is ahead, `travel` if not (the existing next line already had both
    branches).
- **Ledge guard** (`terrain_ahead`, `FlipperPolicy`, `tracked_control`):
  - `terrain_ahead` now reports whether a lower tread is seen in the depth
    band (`step_down_seen`) and its depth (`drop_m`).
  - A drop with no lower tread in view, or one deeper than 0.3 m, puts the
    level vehicle in a new `ledge` state. It blocks forward drive only;
    reverse and turning stay free, and the state clears when the drop is
    out of view.
  - A 0.18 m stair shows its lower tread about 1.1 m ahead and still
    descends. The floor beyond the flight's 0.9 m end is out of the band
    until about 2.5 m ahead.
  - The unit fixture's 0.3 m drop with an unseen floor, previously
    expected to descend, is now a ledge.

## Measured (`terrain_stairs_flight`, spawn (−7.3, −4.0) heading +x, auto flippers, 0.15 m/s)

| Run | Policy | Result | Max z | Climb pitch | Notes |
|---|---|---|---|---|---|
| `auto-1` | before the fix | **not passed**: stuck on tread 1 (1 step) | 0.29 | −0.32 rad | `climb` never ended with the next riser in view |
| `auto-2` | climb fix | **5 steps** (top at ~50 s); then drove off the 0.9 m end in `descend` and flipped (roll π) | 1.01 | −0.32 rad | the probe drives for 100 s |
| `auto-3` | climb fix + ledge on measured depth only | **5 steps**; drove off the end again (the drop showed only as missing floor) | 1.01 | −0.32 rad | |
| **`auto-4`** | climb fix + ledge guard | **5 steps**; `ledge` at x 2.52, stopped at 2.55, 1.05 m before the edge, and stayed there to t = 100 s | 1.01 | −0.32 rad | roll 0 throughout |
| `single-step-regression` | final | the 11-19 course: climbs the first box from its side, descends its far 0.18 m side, ends at y 1.48 (11-19: 1.54) | 0.29 | | a 0.1 s `ledge` reading while mounting the box (y −4.39) did not stop it |

Every riser on the flight follows the same pattern: `approach` (front
flippers raised to the edge), `climb` (both pairs at the joint limit, nose
up to 0.32 rad), then level, then `approach` for the next riser.

## Limits

- Ascent only. A multi-step descent of a flight is not run: this flight
  has no way down, and the arena's descending flight starts at the wall.
- Angled approaches to the riser are not run, so R10.7 keeps them.
- One vehicle, Gazebo only (the native track engine).
- The ledge guard relies on the forward depth camera. Missing depth data
  falls back to the earlier behaviour (no terrain reading → travel).

## Reproduce

```bash
python3 ../flipper-stairs-2026-11-14/launch_tracked.py <dir>/launch.log robot_model:=lab_tracked_strong_flippers \
  map_name:=terrain_stairs_flight spawn_x:=-7.3 spawn_y:=-4.0 spawn_yaw:=0.0
CASE_OUTPUT=<dir>/auto CASE_FLIPPERS=auto CASE_DURATION=100 python3 ../flipper-stairs-2026-11-14/stairs_probe.py
```

Traces are on SSD under `robot_lab_runtime/tracked-flight-20261120` (hashes
in `runtime-sha256.txt`).
