# R10.7: the amphibious rover on MuJoCo and PyBullet (November 21, 2026)

**Status: implemented, experimental; the shore round trip is not passed.**
`lab_amphibious` now runs on MuJoCo and PyBullet through the normal launch
(`simulated_robot.launch.py simulator:=mujoco|pybullet mode:=marine`). It
uses the same amphibious controller, simulated pressure/DVL/sonar sensors,
estimator and shore navigation as on Gazebo. Until now it ran on Gazebo
only.
- On both backends it rests on the bank at the Gazebo height and floats at
  the Gazebo draft.
- It drives down the ramp and reaches the water goal on every run.
- Climbing back out onto the bank failed in 7 of 8 settled attempts. One
  MuJoCo run passed. Gazebo passes the same round trip 2/2 today and 1/1
  on October 30.

## What changed

- **`robot_lab_utils/amphibious_drive.py`:** a backend-neutral port of what
  Gazebo applies.
  - From `robot_lab::AmphibiousDrive`: wheel speed servos (12 Nm per rad/s,
    ±8 Nm), passive suspension springs (10,000 N/m, 200 N s/m, ±600 N),
    immersion-gated propeller thrust (±35 N), and hull drag scaled by the
    wetted fraction.
  - From Fortress graded `Buoyancy`: buoyancy on every collision box and
    cylinder. A fixed-joint shape, the OAK-D housing, belongs to its moving
    ancestor.
  - Leases run on simulation time (0.4 s). The `/amphibious/plant` state
    message has the same fields as Gazebo's.
- **`robot_lab_mujoco/amphibious_plant.py` and
  `robot_lab_pybullet/amphibious_plant.py`:** apply those efforts and
  forces. The PyBullet hull drag torque is applied as a force couple.
- **Spawners:** an `amphibious_config` parameter. The spawners take the
  controller's `/marine/thrusters/<joint>/force` and `/amphibious/wheels`,
  and publish `/marine/odometry` (map → base_link, body-frame twist),
  `/marine/imu` (base_link) and `/amphibious/plant`.
- **Launch:** `marine_simulator.launch.py` runs the robot spawner with the
  shore world's collision conversion, the controller and the marine
  pipeline. `simulated_robot.launch.py` lets lab_rov, lab_usv and
  lab_amphibious pass a ported simulator through; the marine launch still
  checks the exact pair. The rover profile lists Gazebo, MuJoCo and
  PyBullet.
- **PyBullet water worlds:**
  - A world with a Buoyancy system no longer gets the default ground plane
    at z 0. That plane was a solid lid on the shore world's water surface:
    the rover sat at z 0.33 on top of the water in runs 1–4.
  - Visual-only models there (the water surface) are not given collisions,
    as `gen_mjcf_worlds.py` already does for MuJoCo.

## Measured

The workload is the October 30 protocol (`run_shore_roundtrip.py`):
- spawn on the bank at (−6, 0);
- goal 1 in the water at (6.3, 0, 0.134);
- goal 2 back on the bank at (−5.8, 0, 0.74);
- 260 s budget, each goal settled for 1 s.

| Run | Water goal | Bank goal | Result |
|---|---|---|---|
| Gazebo 10-30 | 0.104 m | 0.087 m | passed |
| Gazebo today, 1 and 2 | 0.106, 0.107 m (z 0.133) | 0.086, 0.088 m (z 0.728) | **passed 2/2**, 145 s |
| MuJoCo 1 | 0.151 m (z 0.141) | 0.122 m (z 0.727) | **passed**, 156 s |
| MuJoCo 2 | 0.141 m | — | failed: bounced at the waterline (to 1.8 m/s), attitude envelope stop |
| MuJoCo 3, 4 (1 ms step) | 0.151, 0.154 m | ended 0.36 m away, unsettled | failed: time budget; bounced, then slow |
| MuJoCo 5, 6 (trial heave drag 600) | 0.152, 0.150 m | — | failed: bounced (2.2 / 2.0 m/s) |
| PyBullet 1–4 | — | — | failed: drove on the default plane at z 0.33 (fixed) |
| PyBullet 5, 6 | 0.122, 0.127 m | — | failed: 5 bounced at the waterline; 6 stalled on the ramp at x −1.46 |
| PyBullet 7, 8 (trial heave drag 600) | 0.126, 0.128 m | — | failed: stalled on the ramp at x −1.83 and −1.53 |

Before any run, a stationary spawn on the bank rests at z 0.727 (MuJoCo)
and 0.728 (PyBullet); Gazebo gives 0.728. Afloat at x 8 PyBullet gives
z 0.14.

## What goes wrong at the waterline

An offline MuJoCo harness held the rover where the ramp leaves the water
(x ≈ −0.3), with one actuator at a time:

| Actuators | Result |
|---|---|
| Propellers only | **bounces**: the height swings 0.51 m, the vertical speed reaches 2.1 m/s |
| Wheels only (in deeper water) | no motion; the wheels do not reach the ramp |
| Wheels and propellers | climbs onto the bank |
| Propellers only, heave drag 600 N s/m | no bounce (0.035 m swing) |

There the hull floats with its bottom at the surface while the wheels
barely touch the ramp. The propellers shove the bow up the ramp; the hull
drops back and rebounds at about 10 % of critical damping, and thrust
gating as the stern leaves the water adds energy. Stronger heave drag
removed the bounce offline but not in the missions (MuJoCo 5, 6), so it
was reverted. The missions also stall: climbing the 13° ramp takes about
6.8 Nm per wheel against the 8 Nm cap, and PyBullet stops short of the
bank. Gazebo's implicit contact climbs it reliably. The ported contact,
traction and thrust-gating behaviour at the transition remains the open
item.

## Limits

- Return to the bank not passed on either ported backend. No RGB-D
  mapping there (the camera is off); no Isaac version.
- One world (`amphibious_shore`).

## Reproduce

```bash
python3 docs/status/evidence/r107-amphibious-backends-2026-11-21/run_shore_roundtrip.py mujoco <dir>
```

Traces are on SSD under `robot_lab_runtime/amphibious-backends-20261121`
(hashes in `runtime-sha256.txt`).
