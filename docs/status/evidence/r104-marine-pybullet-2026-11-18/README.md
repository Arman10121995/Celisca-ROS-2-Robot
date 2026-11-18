# R10.4: the lab ROV on PyBullet (November 18, 2026)

**Status: implemented, experimental.** `lab_rov` runs on PyBullet behind
the unchanged marine controller, with the same water forces as on MuJoCo.

## What changed

- **`marine_mujoco_plant.WaterForces`:** the ported Fortress buoyancy,
  hydrodynamics (linear and quadratic damping on the velocity relative to
  the current, added mass) and thruster wrench now sit apart from the
  physics engine. The MuJoCo plant (`MarinePlant`) keeps its behaviour, and
  its five tests pass unchanged. The ROS node loop is shared (`run_plant`).
- **`marine_pybullet_plant.py`:**
  - one Bullet rigid body with the source mass and inertia, its hull box
    and a seabed plane;
  - each 2 ms step applies the same wrench about the centre of mass;
  - the topics are the MuJoCo plant's;
  - `marine_simulator.launch.py simulator:=pybullet` selects it, for
    `lab_rov` only.
- **MuJoCo spawn yaw fixed.** The plant's MJCF wrote the spawn yaw into
  `euler`, which MJCF reads in degrees, so a 0.3 rad spawn came out as
  0.005 rad. It now sets `angle="radian"`. `lab_rov` spawns at yaw 0 in the
  measured tank runs, so those results are unaffected.
- **Parity test:** both plants, given the same commands for 10 s (surge
  plus yaw, spawn yaw 0.3 rad), end within 0.15 m and 0.05 rad of each
  other. An offline comparison over 15 s gave surge 7.52 m on both, and yaw
  0.221 rad (MuJoCo) against 0.218 rad (PyBullet).

## Measured (`rov_probe.py` from `marine-mujoco-currents-2026-11-14`, `marine_tank`)

The phases are: arm and hold for 12 s; surge at 0.2 m/s for 10 s; settle
for 6 s; yaw at 0.3 rad/s for 6 s; hold for 10 s; hold in a 0.1 m/s +y
current for 15 s; then operator heartbeat loss for 6 s.

| Phase | PyBullet (this run) | MuJoCo (2026-11-14) | Gazebo (2026-11-14) |
|---|---|---|---|
| Hold while arming | 7.9 cm drift | 5.7 cm | 2 cm |
| Surge (10 s) | 1.041 m | 1.030 m | 1.074 m |
| Yaw (6 s) | 1.439 rad | 1.437 rad | 1.495 rad |
| Depth deviation while manoeuvring | ≤ 4 mm | ≤ 4 mm | ≤ 1 mm |
| 15 s in 0.1 m/s current | 3.5 cm | 0.9 cm | 1.0 cm |
| Operator loss | disarmed, thrust zero, sank 0.23 m in 6 s | disarmed, sank 0.23 m | disarmed, sank 0.29 m |

The result is `pybullet-1-result.json`. The launch log is on SSD under
`robot_lab_runtime/marine-pybullet-20261118` (hash here). The run shared
the host with Go2 policy training.

## Limits

- One run. The world is the seabed plane only (as on MuJoCo); tank walls and
  other geometry are not converted.
- Isaac, and a Tethys (fins) plant on MuJoCo or PyBullet, remain.
- No propeller dynamics; the added-mass Coriolis terms are omitted, as in
  the MuJoCo port.
