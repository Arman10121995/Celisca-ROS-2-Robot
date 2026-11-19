# R10.7: a 40 Nm flipper variant crosses the 0.18 m step (November 19, 2026)

**Status: implemented, experimental.** `lab_tracked_strong_flippers` climbs
the `terrain_stairs` 0.18 m step face under the automatic flipper policy,
drives along its tread and comes down the far side. 2 of 2 runs pass. Gazebo
Fortress with the private native track engine; the host was also training
a Go2 policy.

## What changed

- **New robot `lab_tracked_strong_flippers`.**
  - `scripts/generate_articulated_tracks.py` now writes two variants of the
    retained tracked source.
  - The existing `lab_tracked_articulated` (4 Nm) is reproduced byte for
    byte.
  - The new one has a 40 Nm flipper force PID: cap, joint effort limit and
    gains are all 10×.
  - Links, other joint fields, belts and sensors are identical. A test
    checks this.
- **New policy option `full_press`.**
  - `tracked_flippers.FlipperPolicy(full_press=True)` changes the climb
    state only: both flipper pairs press to the ±0.8 rad joint limit
    (elevation −0.3 rad). By default the front pair stays up and the rear
    pair presses to −0.15 rad.
  - `tracked_control` takes it from the parameter `flipper_full_press`,
    which the launch sets from the robot profile. Only the new variant sets
    it, so the 4 Nm vehicle is unchanged.
- **Registered** in the robot profiles, the registry and the tracked asset
  group.

## Results: same course as [flipper-stairs-2026-11-14](../flipper-stairs-2026-11-14/README.md)

The vehicle spawns at (−6.65, −5.9) heading +y. It drives at 0.15 m/s
toward the step's side face at y = −4.55. The tread runs to y = −3.45.

| Run | Outcome | Max z | Max nose-up | Notes |
|---|---|---|---|---|
| Fixed flippers (`fixed-result.json`) | Reared on the edge; tilt guard | 0.229 | 0.658 rad | Same as the 4 Nm vehicle, so the actuator alone does not climb |
| Auto, default climb (`auto-rear-only-result.json`) | Stalled at y −4.81, nose up 0.27 rad, for 32 s | 0.183 | 0.304 rad | The strong rear flippers now hold (−0.56 rad; the 4 Nm ones were back-driven to −0.03), but the tracks cannot pull over the edge |
| Manual, from that stall (`manual_flip.py 0.65,-0.65,12 0.8,-0.8,12`) | Front at 0.65 / rear at −0.65: still stalled. Both at the joint limit: up and onto the tread | 0.258 | — | Output recorded below |
| **Auto, full press, run 1** (`auto-full-press-1-result.json`) | **Crossed**: up the face, level on the tread from y −4.39 to −3.49 (z 0.252), down the far side, then travel to y 1.54 | 0.288 | 0.324 rad | States: approach → climb → descend → travel; roll < 0.001 rad |
| **Auto, full press, run 2** (`auto-full-press-2-result.json`) | **Crossed**, the same within millimetres (fresh launch) | 0.288 | 0.324 rad | 484 level samples on the tread |

In both full-press runs the end position was y 1.54 after 50 s, the same
distance as the 0.15 m/s command, so the crossing did not slow the drive.
`steps_climbed` and `on_tread` in the result files compare the end height
with the start. They read 0 and false because the vehicle finishes back on
the floor past the step; the trace shows the climb.

Manual experiment output (truth y, z and pitch, with ROS pitch negative
when nose-up):

```text
f=+0.65 r=-0.65 y=-4.810 z=0.183 p=-0.268 pos=[0.65, -0.56, 0.65, -0.56]   (12 s, no motion)
f=+0.80 r=-0.80 y=-4.611 z=0.238 p=-0.266 pos=[0.8, -0.75, 0.8, -0.75]
f=+0.80 r=-0.80 y=-4.386 z=0.258 p=-0.035 pos=[0.78, -0.8, 0.78, -0.8]
f=+0.80 r=-0.80 y=-3.950 z=0.252 p=-0.000 pos=[0.69, -0.69, 0.69, -0.69]
```

**Finding:** two things are needed together to climb. One is the stronger
actuator; the other is a climb state that presses with both pairs. The
actuator alone, with fixed flippers or the default climb, does not climb.

## Limits

- One step height (0.18 m) and one approach, square to the face, on one
  backend (Gazebo). A multi-step flight, angled approaches and descent-only
  runs are not tested.
- 40 Nm is a lab choice, not a vendor actuator rating; the source vehicle
  has fixed flippers.
- The state stays in `climb` for about 6 s on the level tread before
  `descend`; it does not return to `travel` there. It is harmless here but
  not tuned.

## Reproduce

```bash
python3 ../flipper-stairs-2026-11-14/launch_tracked.py <dir>/launch.log robot_model:=lab_tracked_strong_flippers \
  map_name:=terrain_stairs spawn_x:=-6.65 spawn_y:=-5.9 spawn_yaw:=1.5708
CASE_OUTPUT=<dir>/auto CASE_FLIPPERS=auto CASE_DURATION=50 python3 ../flipper-stairs-2026-11-14/stairs_probe.py
```

Launch logs and traces are in
`/workspace/molar/robot_lab_runtime/flipper-strong-20261119/`, with hashes
in `runtime-sha256.txt`. The `auto-1` launch ran the policy without
`full_press`. The full-press launches ran after it was added.
