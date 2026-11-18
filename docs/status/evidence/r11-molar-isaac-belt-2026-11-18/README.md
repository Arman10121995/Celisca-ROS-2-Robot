# R11 MOLAR payload transfer on Isaac: native deck belt (November 18, 2026)

**Status: implemented, experimental.** With this, the MOLAR payload transfer
passes on all four backends (Gazebo, MuJoCo, PyBullet, Isaac) on
`nav_empty`.

## Deck belt on Isaac

The Isaac runtime (`isaac_runtime._deck_belt`) applies PhysX
`PhysxSurfaceVelocityAPI` to MOLAR's deck link (`telescope_stage2`). This
is how Isaac's own conveyor (`isaacsim.asset.gen.conveyor`) works:

- contact friction makes loose bodies on the deck follow its surface;
- the surface speed is the measured roller rate times the 12 mm roller
  radius, along the deck's local x;
- it is written only when it changes.

The spawner enables the belt when the world has loose bodies and the robot
has `transfer_roller_joint` and `telescope_stage2`, the same condition as
the PyBullet belt.

**Sign, as measured on Isaac Sim 6.0.1:** a surface velocity of +x drove
the payload towards the deck's −x, and it fell off the rear (run 1, kept).
The belt is therefore authored as −x.

**Checker:** `molar_station_transfer_check.py --simulator isaac` uses the
same SDF world as PyBullet (loose payload, static station clear of the deck
frame).

## Measured (`--time-scale 3`)

| Run | Result | Payload ahead of base | On station / off deck | Retract drag | Base peak at roller start |
|---|---|---|---|---|---|
| 1 (belt +x) | **not passed**: payload driven backwards off the deck | −0.144 m | no / no | 44.9 mm | 0.053 m/s |
| 2 | passed | 0.671 m | yes / yes | 0.0 mm | 0.028 m/s |
| 3 | passed | 0.673 m | yes / yes | 0.0 mm | 0.026 m/s |

Runs were measured while Go2 stairs training loaded the CPU. Launch logs
are on SSD under `robot_lab_runtime/molar-isaac-belt-20261118`.

## Limits

- Two passing runs, one world.
- The direction sign is an empirical convention for this Isaac build.
- The station is static, without rollers, as on the other backends.
