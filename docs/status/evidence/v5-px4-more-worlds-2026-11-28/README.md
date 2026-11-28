# V5: PX4 manual flight in twelve more installed worlds (November 28, 2026)

**Status: 11 of 12 worlds passed, two of them after adding a flight spawn.
`small_warehouse` did not fly, and its cause is open.**
`run_worlds.sh` repeats the November 21 GUI screen (`scripts/qualify_px4_drive_gui.py`,
unchanged, under Xvfb) on worlds not flown before. The setup:
- the native X500 on PX4 SITL in Gazebo Harmonic;
- Run, the takeoff check, Takeoff, the seven directions and altitude,
  Stop, publisher loss and Land;
- every motion measured by the Gazebo body (`/px4/odometry_truth`).

The runs used revision d91e9465, plus the flight spawns below.

| World | Result | Checked rise | Takeoff height | Publisher-loss drift |
|---|---|---|---|---|
| nav_empty | **passed** | 3.0 m | 3.10 m | 0.02 m |
| nav_obstacle | **passed** | 3.0 m | 2.95 m | 0.03 m |
| nav_dynamic | **passed** | 3.0 m | 3.02 m | 0.04 m |
| nav_narrow_passage | **passed** | 3.0 m | 2.97 m | 0.02 m |
| nav_sensor_degraded | **passed** | 3.0 m | 2.95 m | 0.05 m |
| simple_office | **passed** | 3.0 m | 2.96 m | 0.05 m |
| bigger_warehouse | **passed** | 3.0 m | 2.98 m | 0.05 m |
| warehouse_demo | **passed** | 3.0 m | 2.92 m | 0.04 m |
| celisca_floor_2 | **passed** | 3.0 m | 2.91 m | 0.13 m |
| residential_demo | **passed** with a flight spawn | 3.0 m | 2.99 m | 0.04 m |
| terrain_stairs | **passed** with a flight spawn | 3.0 m | 2.97 m | 0.04 m |
| small_warehouse | **not passed**: stayed at rest after Takeoff | (3.0 m) | — | — |

The first runs are kept under `before-fix/`.

## Two spawns were blocked; the takeoff check refused them correctly

| World | Ground spawn | What blocks it | X500 `flight_spawn` (checked clear for 3 m offline and live) |
|---|---|---|---|
| residential_demo | (0, 0) | Inside the living-room table (x −0.66 to 0.67, height 0.32 m) | (−1.5, −1.5) |
| terrain_stairs | (−7, −5) | The rotor envelope overlaps `platform_04`'s 18 cm edge by about 5 cm | (−7.0, −5.6) |

`flight_spawn` (`sim_maps.yaml`) applies only to the X500. The ground
robots keep their spawns, as on `terrain_heightfield`.

## small_warehouse

In three runs the X500 armed and PX4 reported "ascending", but the Gazebo
body stayed at its resting height for the whole 55 s stage (z 0.24 →
0.25 m):
- twice from the origin;
- once from a second clear interior spot, (−3, −5).

Two causes were tested and **ruled out**; both changes were reverted:
- **A wrong spawn spot**: the second spot failed the same way, so the
  blockage is building-wide.
- **The world's 0.001 s / 1 kHz physics step**: PX4's own 0.004 s / 250 Hz
  physics was substituted in the prepared flight world, and confirmed
  there, with no change.

The takeoff check finds 3 m clear. Its scene covers the building shell
`aws_robomaker_warehouse_WallB_01`, whose collision mesh spans the whole
warehouse (14 × 21 × 9 m). How PX4's Gazebo Harmonic handles that shell is
the leading suspect and is not yet tested. `bigger_warehouse` and
`warehouse_demo`, with other shells, fly.

## Reproduce

```bash
docs/status/evidence/v5-px4-more-worlds-2026-11-28/run_worlds.sh <run-dir>              # the twelve worlds
WORLDS="residential_demo terrain_stairs" docs/status/evidence/v5-px4-more-worlds-2026-11-28/run_worlds.sh <run-dir>
```

Reports are copied here. Traces and GUI logs are on SSD under
`robot_lab_runtime/v5-px4-more-20261128`. Their hashes are in
`runtime-sha256.txt`.
