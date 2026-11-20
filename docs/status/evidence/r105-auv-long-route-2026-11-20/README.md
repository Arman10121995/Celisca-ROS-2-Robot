# R10.5: long AUV routes from the sensor estimate (November 20, 2026)

**Status: implemented, experimental.** The Tethys AUV now flies long
routes from its IMU, pressure and DVL estimate, with RTAB-Map mapping
online, on Gazebo:

- 20 m straight, 2/2;
- a curved 23 m Dubins route;
- a 43 m two-leg mission.

Each settles 0.10–0.18 m from its goal by truth. The tank routes improved
too: 6.5 m now ends 0.05 m from the goal (the 6.5 m route diverged on
11-19), and 3.5 m 0.08 m (was 0.20).

## Why the 6.5 m route diverged, and what changed

A 4 Hz attitude trace (`auv_mission_probe.py`) showed three controller
causes. Roll stayed under 0.01 rad, so roll coupling was ruled out.

1. **Sideslip feedback (path follower, `auv_routes.py`).**
   - The cause: the heading command subtracted the *measured* sideslip.
     The source hull's added masses (Y_v̇ 129 vs X_u̇ 5) give a destabilizing
     Munk moment with sideslip. Pointing the nose further off by that
     sideslip fed it back: 0.2 rad of sideslip built up in a tank with no
     current.
   - The change: the follower now subtracts the crab angle the authored
     current requires (zero here), plus an integral line-of-sight term (gain
     0.05, bounded at 2 m·s) for unmodelled drift. Fixing this alone made
     the 6.5 m route pass (0.40 / 0.42 m). The 20 m route still failed after
     13–16 m.
2. **A second integral on depth (follower → `auv_controller.py`).**
   - The cause: the follower added 0.3 /s of path-depth correction to the
     vertical rate, and the controller integrates that rate into its depth
     target. On 20 m the target wound up 0.4 m off, and depth swung 0.8 m
     in a growing 30 s cycle.
   - The change: the correction is now 0.05 /s, with the path slope fed
     forward.
   - The cause, part 2: the inner depth hold commanded up to ±0.2 rad of
     pitch at 0.6 /s, and the hull's slow pitch response then overshot. The
     change: 0.3 /s with stronger vertical-speed damping, limited to
     ±0.05 m/s and ±0.1 rad. Depth now stays within ±0.08 m on 20 m.
3. **Line-of-sight lookahead.**
   - The cause: 1 m of lookahead (1 rad per metre of cross-track) drove a
     growing heading oscillation, for a hull limited to 0.12 rad/s. The
     change: 4 m, about two hull lengths. Heading then stays within
     0.013 rad on 20 m.

Also found: the source LiftDrag fins stall at an angle of attack of 0.17
rad, and the fin joint limit is 0.26 rad. Clamping the command to 0.15 alone
did not stop the divergence (run `fin-015`), and the change was reverted. The
fins still saturate briefly in the final runs, without loss of control.

**New world `marine_open_water`:** a 60 × 40 m bay with the tank's water,
seafloor depth, buoyancy and physics, and no walls
(`robot_lab_maps/tools/marine_open_water.py`; registered in `sim_maps`, the
environment registry, and the Tethys `spawn_by_map` at (−20, −10)). The
tank holds at most a 6.5 m straight route, because of the planner's 1.3 m
radius and 3 m braking corridor. Tethys' 10–30 m Dubins turn cannot turn
in it: an 8 m curved goal and a 9.4 m goal were refused at planning.

## Measured (Gazebo, `slam:=true`, depth 3 m, judged by truth)

| Run | World | Route | Result | Truth to goal | Estimate error (max) |
|---|---|---|---|---|---|
| `baseline` | tank | 6.5 m (before any change) | canceled, course envelope | 0.92 m | 0.09 m |
| `fin-015` | tank | 6.5 m, fins ≤ 0.15 rad only | canceled | 0.86 m | 0.09 m |
| `ilos-1`, `ilos-2` | tank | 6.5 m, follower fix | completed | 0.40 / 0.42 m | 0.12 m |
| `open-straight-20m` | open water | 20 m, follower fix | canceled after 16 m | 3.40 m | 0.21 m |
| `open-straight-20m-depth` | open water | + weak outer depth correction | canceled | 3.81 m | 0.21 m |
| `open-20m-gentle-depth` | open water | + gentle inner depth hold | canceled (heading oscillation) | 4.26 m | 0.20 m |
| **`open-20m-los4`, `open-20m-2`** | open water | 20 m straight, all changes | **completed** (76.5 / 74.0 s) | **0.178 / 0.176 m** | 0.24 m |
| **`open-curve-23m`** | open water | 23.4 m curved Dubins route from (0, −10) | **completed** (84.2 s) | **0.139 m** | 0.25 m |
| **`open-two-leg-5`** | open water | 20 m, then a 23.3 m curved leg (43 m) | **completed**; leg 1 0.187 m | **0.103 m** | 0.38 m |
| **`tank-6m-final`** | tank | 6.5 m | **completed** | **0.051 m** | 0.12 m |
| **`tank-3m-final`** | tank | 3.5 m (11-19 run 3 route) | **completed** | **0.080 m** | 0.10 m |

Planning refusals, kept: `long-9m` and `curve-8m-1` in the tank (no
clear braking corridor or turn), and `two-leg-1` (no U-turn in the tank).
Probe-sequencing failures, kept: `open-two-leg-2` to `-4`. After leg 1 the
probe planned leg 2 before the completed mission's Stop had disarmed the
vehicle, so Execute was refused. The probe now waits for that Stop and then
re-arms.

### Backend probe regression (`r104` `auv_probe.py`, tank, 1.5 m deep)

| Phase | Gazebo 11-19 → now | MuJoCo 11-19 → now |
|---|---|---|
| Cruise 10 s | 2.452 → 2.439 m | 2.432 → 2.427 m |
| Cruising turn | 1.193 → 1.188 rad; depth drift 0.236 → 0.000 m | 0.863 → 0.865 rad |
| Climb command | −0.087 → −0.013 m (gentler depth loop) | 0.596 → 0.588 m |
| Operator loss | disarms | disarms |

## Limits

- Gazebo only for the missions. MuJoCo is regression-probed; PyBullet and
  Isaac share the controller but were not re-run.
- No current and no obstacles on these routes. The crab model uses the
  operator-authored current, unsensed.
- The idle estimate still drifts (1.0 m over a 6 min wait in
  `open-two-leg-2`), so each leg must start soon after the previous one.
- The commanded climb rate is now slower, at most 0.05 m/s.

## Reproduce

```bash
ros2 launch robot_lab_bringup auv_simulator.launch.py map_name:=marine_open_water slam:=true spawn_z:=-3.0 gui:=false rviz:=false
CASE_OUTPUT=<new dir> CASE_TARGET=0.0,-10.0,-3.0 CASE_TARGETS="15.0,5.0,-3.0" \
  python3 docs/status/evidence/r105-auv-long-route-2026-11-20/auv_mission_probe.py
```

Traces are on SSD under `robot_lab_runtime/auv-long-20261120` (hashes in
`runtime-sha256.txt`).
