# R10.4: Tethys AUV on MuJoCo (November 19, 2026)

**Status: implemented, experimental.** The pinned Tethys AUV (propeller
and fins) runs on MuJoCo behind the unchanged `auv_controller`
(`auv_simulator.launch.py simulator:=mujoco`). Cruise matches Gazebo; a
cruising turn is about 30 % slower.

## What was ported (`marine_tethys_plant.py`)

The forces come from the source `model.sdf` and the world's buoyancy:

- **Buoyancy:** each collision box's volume at its centre. The source trim
  makes the vehicle neutral: 0.14836 m³ of water against 152.07 kg.
- **Hydrodynamics:** the Tethys Fortress block (quadratic damping on the
  velocity relative to the current, added mass), parsed from the SDF.
- **Thruster:** under velocity control, as in Fortress Thruster: the
  commanded thrust along the propeller axis at the propeller link, with no
  reaction torque. The joint turns at sign(T)·√(|T| / (ρ Ct D⁴)).
- **Fins:** the source JointPositionController (P gain 0.1, ±2 N m, no
  damping), integrated with each fin's inertia about its axis, within
  ±0.2618 rad.
- **LiftDrag on each fin:** a line-by-line port of Fortress LiftDrag:
  - velocity at the centre of pressure, sweep and angle of attack;
  - the stall branches of cl and cd, scaled by 1 − sin²(sweep) as the
    source does;
  - zero moment.

The hull, ballast, fins and propeller form one rigid body (summed source
mass and inertia); fin angles change only the LiftDrag geometry. The world
is a seabed plane. The plant publishes `/marine/odometry`, `/joint_states`
and `/clock`, as the Gazebo session's bridge does.

## The added-mass Coriolis term

Fortress 6.18 applies an added-mass Coriolis wrench unless
`disable_coriolis` is set; its library contains that option. Its signs pass
through helpers that are not in the inspected source, so all three forms
were run with the same controller and probe:

| Turn phase (25 s, 0.3 m/s, 0.1 rad/s) | Distance | Yaw | Depth | Armed at end |
|---|---|---|---|---|
| **Gazebo (Fortress 6.18)** | 6.93 m | +1.19 rad | rose 0.24 m | yes |
| MuJoCo, Fossen −C_A(ν)ν | 4.53 m | +0.97 rad | rose 0.56 m | **no**: pitch rose to 0.5 rad with the elevator saturated |
| MuJoCo, opposite sign | 7.43 m | **−0.24 rad** (wrong way) | sank 0.38 m | yes |
| **MuJoCo, no Coriolis (default)** | 7.17 m | +0.86 rad | rose 0.32 m | yes |

The plant defaults to no Coriolis term, the closest to Gazebo; `--coriolis`
(launch: `tethys_coriolis:=`) selects the others.

## Measured (`auv_probe.py`, `marine_tank`, spawn depth 1.5 m)

The phases are: neutral 5 s; cruise at 0.3 m/s for 10 s; turn at
0.1 rad/s while cruising for 25 s; climb at 0.1 m/s while cruising for
10 s; operator heartbeat loss for 6 s.

| Phase | Gazebo | MuJoCo (default) |
|---|---|---|
| Neutral | 0.0 m drift | 0.0 m |
| Cruise | 2.45 m, mean surge 0.242 m/s, depth +4.9 cm | 2.43 m, 0.241 m/s, +0.3 cm |
| Turn | 6.93 m, yaw +1.19 rad, rose 0.24 m | 7.17 m, yaw +0.86 rad, rose 0.32 m |
| Climb | 0.32 m (by then near the tank wall) | 2.92 m, rose 0.59 m |
| Operator loss | disarmed, propulsion zero, fins neutral | disarmed |

The climb phases are not comparable: by then the Gazebo vehicle is near
the tank wall, and the MuJoCo plant has only a seabed.

Results are in `*-result.json`. Launch logs are on SSD under
`robot_lab_runtime/tethys-mujoco-20261119` (hashes here). The runs shared
the host with Go2 policy training.

## Limits

- One run per configuration. The turn is about 30 % slower than on Gazebo,
  and the reason is not found.
- No walls, sensors or mapping pipeline on MuJoCo.
- The fins are kinematic in the hull's dynamics (their reaction torque on
  the hull is not applied).
- Tethys on PyBullet and Isaac, and an Isaac ROV adapter, remain.
