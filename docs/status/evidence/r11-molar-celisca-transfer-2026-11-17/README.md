# R11 MOLAR payload transfer on CELISCA floor 1 (Gazebo, November 17, 2026)

**Status:** implemented, experimental. **R11's done-when is met.** It asks
for a measured route plus a payload transfer on a CELISCA floor:

- route A was measured on `celisca_floor_1_detailed` on 2026-11-15
  ([receipt](../r11-molar-nav-2026-11-15/README.md));
- the transfer below runs on the same world.

## Run

`scripts/molar_station_transfer_check.py --simulator gazebo --map
celisca_floor_1_detailed --time-scale 4`. The `--map` and `--time-scale`
options are new.

**Setup.** MOLAR (`molar_hri`) starts at the map's spawn (0.975, 0.425,
yaw 0). The occupancy grid shows 1.6 × 0.8 m of free floor ahead. Through the
normal articulation interface, the checker:

1. spawns a 0.5 kg box on the deck;
2. lifts the deck 0.10 m;
3. extends both telescope stages;
4. spawns a 0.05-friction station end to end with the deck;
5. runs the roller at 8 rad/s, then retracts and lowers.

**Result (`run3.json`): passed.**

- **Lift:** 0.0998 m. **Extension:** deck end measured, station placed
  1 cm past it.
- **Belt:** roller measured 8.0 rad/s; the box left the deck (rear overlap
  −1.4 mm).
- **Retract:** the box rests on the station (`on_station`), 0.669 m ahead
  of the base and centred across, top at 0.845 m. It was not dragged back
  (0.0 m).

## Failures on the way

- **Run 1:** the lift did not settle within 30 s wall time. This world runs
  Gazebo well below real time, hence `--time-scale`, which scales the
  checker's waits.
- **Run 2:** the articulation never became ready. `joint_state_broadcaster`
  failed to activate within the spawner's default 5 s switch timeout.
  Display-mode spawners now use `--switch-timeout 60`, as MOLAR's nav-mode
  controllers have since 2026-11-15.

## Limits

- One run, Gazebo only, floor 1.
- The station is spawned in front of the robot, not a CELISCA fixture.
- Repeats are V8.
- PyBullet/Isaac belts and multi-floor operation are separate
  (R11, R10.2).
