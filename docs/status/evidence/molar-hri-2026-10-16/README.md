# MOLAR / ProBOT-L on the detailed CELISCA floors, 2026-10-16 to 10-19

Normal-GUI runs of the MOLAR family (R11, `molar_hri` and `probot_l`) in
Display mode on `nav_empty`, `celisca_floor_1_detailed` and
`celisca_floor_2_detailed`, one simulator at a time.

**Producer.** [`scripts/qualify_molar_hri_gui.py`](../../../../scripts/qualify_molar_hri_gui.py)
drives the real Tk GUI under Xvfb. It selects the robot, backend, map and
Display mode, presses Run, and then uses the GUI's own Drive keys and MOLAR
page buttons. It checks the following against simulator truth and the
measured joints:

1. The base stands still at start.
2. It drives forward (W), then forward and left together (W+A).
3. It stops.
4. The lift goes to 0.06 m, then both telescope stages to 0.06 m.
5. With the telescope out, a forward key leaves the base still.
6. The lift goes to its 0.68 m top, then the telescope to its 0.14 m stage
   limit, a 0.28 m tip extension (added today).
7. The roller turns at 2 rad/s.
8. Stop / hold holds the lift and both stages where they are.
9. Leaving the MOLAR page drops the heartbeat, and the roller stops.
10. The telescope retracts, then the lift lowers, before the run ends.

The reports are in [reports/](reports/), named
`<backend>_<robot>_<map>.json`, and indexed for the R9.3 matrix as
`articulation_screen` cells ([molar-batch.yaml](molar-batch.yaml)).
Traces, GUI logs and screenshots stay on the SSD under
`/workspace/molar/robot_lab_runtime/claude-molar/`, beside a copy of each
run's producer.

## Fixes found by these runs

An earlier agent's MuJoCo run never moved. Its PyBullet run passed on
`nav_empty`; Gazebo and Isaac had no runs. Each fix below came from a
failing run.

1. **MuJoCo base locked** (`ec6210a`).
   - **Cause:** the robot MJCF's implicit integrator was not carried into
     the merged world. Under explicit Euler the transfer roller
     (1.4e-5 kg m² against kd 0.2) read −82 rad/s at a fixed angle. That
     counted as articulation activity, so the base stayed inhibited.
   - **Fix:** the spawner sets `implicitfast` for any robot with an
     articulation.
2. **Lift sagged under its own weight** (`7917d64`).
   - **Cause:** a pure PD (kp 20 000 N/m) held the 14.2 kg carriage, two
     stages and roller 7 mm low (0.053 m for 0.06 m, 0.673 m for 0.68 m).
     Each Stop holds the measured position, so it lowered the lift another
     7 mm.
   - **Fix:** MuJoCo adds the slides' generalized gravity force
     (`qfrc_bias`) as a joint force. Isaac adds the subtree weight along
     each slide axis as a joint effort
     (`robot_lab_utils.urdf_articulation.gravity_loads` /
     `gravity_feedforward`). In `b5ae10d` the other agent made the servo
     and the feedforward share one source effort limit.
   - **Result:** the lift reaches 0.0600 m and 0.6800 m and holds through
     Stop.
3. **Isaac base locked** (`c7fedb2`, refined in `b5ae10d`).
   - **Cause:** Isaac reported the resting lift at −0.044 m/s (std 0.018)
     with its position unchanged within 1.5 mm. Its wheels and roller
     jittered at 0.3 and 0.05 rad/s.
   - **Fix:** "articulation active" now comes from each joint's position
     change over 0.25 s of simulated time, not from the reported velocity.
     It also counts a motion that returns to its start.
4. **Isaac lift held as if the base were driving** (`84673d6`).
   - **Cause:** the same jitter in Isaac's body velocity rejected the lift
     command: the lift reached 0.0009 m for a 0.06 m target.
   - **Fix:** base motion now comes from the root pose change over 0.25 s.
5. **Gazebo floors loaded without contact** (`b5ae10d`, the other agent).
   - **Cause:** Fortress/DART ignored the source OBJ's normal-less
     collision submeshes.
   - **Fix:** the floors' collision is an ASCII STL with explicit normals
     and the source's exact float64 triangles.
6. **Gazebo base would not turn** (`26f1039`).
   - **Cause:** at the default friction of 1, DART never swivelled the
     casters. W+A turned the base 0.004 rad in 4 s for 0.4 rad/s while the
     drive wheels slipped.
   - **Fix:** caster tyre friction 0.1 in Gazebo, as on Bumperbot's
     casters. The casters now swivel and the base turns at about
     0.38 rad/s.
7. **Isaac base thrown on the detailed floors** (`26f1039`).
   - **Cause:** the floors' own triangles at z = 0 are long duplicated
     slivers (12 m by 4 cm), coincident with the ground plane Isaac always
     adds. In its first second the base was thrown 0.3–1.6 m and tilted up
     to 0.38 rad.
   - **Fix:** the Isaac runtime leaves a world mesh's triangles lying on
     that plane to the plane: 4684 of 8448 on floor 1, 3992 of 7430 on
     floor 2.
8. **Gazebo lift never rose** (`811841d`).
   - **Cause:** DART's velocity servo, which `gz_ros2_control` uses for
     position commands, could not leave the lower stop the loaded lift
     rested on. The unloaded telescope moved.
   - **Fix:** the derivatives' lift and telescope hard stops sit 5 mm below
     the controlled range. The contract still bounds commands to 0–0.68 m
     and 0–0.14 m.

9. **MuJoCo spawn faulted the articulation** (`3af4313`, test `ca218dd`).
   - **Cause:** with the stop 5 mm lower, the spawn drop faulted the
     articulation. `qfrc_bias` holds the slides against gravity, not
     against the falling base, so it raised the falling lift 10 mm. The
     landing then drove it 9–11 mm below 0, past the contract's 10 mm
     measurement allowance.
   - **Fix:** the feedforward ramps in over the first second after a
     (re)spawn, and the slides' stops get a 5 ms limit time constant. The
     landing now dips to −0.0055 m.

Earlier runs stay on the SSD under their round prefixes:

- `pre_`: before the full-range step;
- `noff_`: without the gravity term;
- `m1b_`: before the activity fix;
- `m1c_`: before the Isaac pose-based base motion;
- `m1d_`: 2026-10-16 at `84673d6`;
- `m3a_`: 2026-10-19 before the MuJoCo spawn fix (Gazebo and Isaac
  passed there at `811841d`).

## Results (round M3b, `ca218dd`, 2026-10-19)

All 16 runs pass, at one revision. Distances come from simulator truth and
joint values from the measured `/joint_states`. "Stop drift" is the largest
lift or stage move in the 1.5 s after Stop / hold. The last column is the
roller 2 s after leaving the MOLAR page, which drops the heartbeat. The GUI
selection (family, variant, Display, map and world variant, backend) is in
[isaac_f2_gui.jpg](isaac_f2_gui.jpg); RViz opens over the panel.

| Robot | Backend | Map | Result | Forward m / W+A turn rad | Drive with telescope out, m | Lift at 0.06 m | Lift at 0.68 m / tip at 0.28 m | Coupling error, m | Roller at 2 rad/s | Stop drift, m | Roller after heartbeat loss, rad/s |
|---|---|---|---|---|---|---|---|---|---|---|---|
| molar_hri | gazebo | nav_empty | pass | 0.516 / 0.456 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 0.0e+00 | 2.00 | 1.7e-06 | -0.000 |
| molar_hri | gazebo | CELISCA F1 | pass | 0.552 / 0.658 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 5.6e-17 | 2.00 | 3.1e-07 | 0.000 |
| molar_hri | gazebo | CELISCA F2 | pass | 0.534 / 0.601 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 5.6e-17 | 2.00 | 9.4e-13 | 0.000 |
| molar_hri | mujoco | nav_empty | pass | 0.534 / 0.384 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 6.8e-09 | 1.90 | 1.9e-09 | -0.000 |
| molar_hri | mujoco | CELISCA F1 | pass | 0.575 / 0.605 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 5.9e-09 | 1.90 | 6.7e-07 | 0.000 |
| molar_hri | mujoco | CELISCA F2 | pass | 0.576 / 0.567 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 8.7e-09 | 1.90 | 4.9e-10 | 0.000 |
| molar_hri | pybullet | nav_empty | pass | 0.519 / 0.493 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 2.8e-07 | 2.00 | 1.5e-07 | -0.000 |
| molar_hri | pybullet | CELISCA F1 | pass | 0.494 / 0.495 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 9.2e-07 | 2.00 | 5.7e-07 | -0.000 |
| molar_hri | pybullet | CELISCA F2 | pass | 0.502 / 0.472 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 3.6e-07 | 2.00 | 6.7e-07 | -0.000 |
| molar_hri | isaac | nav_empty | pass | 0.558 / 0.282 | 0.001 | 0.0599 | 0.6799 / 0.2796 | 5.0e-05 | 2.01 | 2.9e-05 | -0.031 |
| molar_hri | isaac | CELISCA F1 | pass | 0.563 / 0.452 | 0.001 | 0.0599 | 0.6800 / 0.2800 | 1.2e-05 | 2.00 | 1.4e-04 | -0.043 |
| molar_hri | isaac | CELISCA F2 | pass | 0.576 / 0.459 | 0.001 | 0.0600 | 0.6800 / 0.2800 | 5.5e-06 | 1.86 | 7.1e-05 | 0.089 |
| probot_l | gazebo | nav_empty | pass | 0.514 / 0.449 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 2.8e-17 | 2.00 | 1.7e-06 | -0.000 |
| probot_l | mujoco | nav_empty | pass | 0.517 / 0.399 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 8.1e-12 | 1.90 | 2.4e-11 | -0.000 |
| probot_l | pybullet | nav_empty | pass | 0.514 / 0.486 | 0.000 | 0.0600 | 0.6800 / 0.2800 | 3.5e-07 | 2.00 | 2.5e-08 | 0.000 |
| probot_l | isaac | nav_empty | pass | 0.551 / 0.360 | 0.001 | 0.0600 | 0.6800 / 0.2794 | 2.0e-04 | 2.01 | 3.7e-04 | -0.027 |

Simulation only: no physical robot, FMS, door, building lift or PLC path is
touched. Masses, inertias, caster parameters, servo gains and friction are
estimates, and payload transfer is not modelled.
