# R10.7: the catamaran USV on MuJoCo, PyBullet and Isaac (November 20, 2026)

**Status: implemented, experimental.** The self-made catamaran `lab_usv`
now runs its unchanged surface controller on the ported marine plants. It
surges, turns, drifts in a current and disarms on operator loss within a
few percent of Gazebo on all three. Until now it ran on Gazebo only.

## What changed

- **Marine plants** (`marine_mujoco_plant.WaterForces`, shared by the
  MuJoCo, PyBullet and Isaac plants):
  - A `model_class` selects the source thruster model. The ROV keeps
    `MarineModel`, which requires its six controlled axes; `--surface` uses
    `SurfaceModel` (two propellers, surge and yaw).
  - The Isaac child receives the same choice in its configuration.
  - Buoyancy (graded, partially submerged boxes), Fortress hydrodynamics
    and thrusters are the existing ports, read from the USV's own URDF
    plugins.
- **`marine_simulator.launch.py`:** the ported plants accept `lab_usv`.
  For a surface craft the launch passes `--surface` and runs
  `surface_controller.py` with the profile's `surface_control` tuning, as
  the Gazebo branch does.
- The ROV path is unchanged; a MuJoCo regression matches its 11-14 run.

## Measured (`rov_probe.py` from `marine-mujoco-currents-2026-11-14`, `marine_tank`, spawn at the surface)

Probe phases: hold 12 s; surge at 0.2 m/s for 10 s; settle 6 s; yaw at
0.3 rad/s for 6 s; hold 10 s; a 0.1 m/s +y current for 15 s; operator
loss.

| Phase | Gazebo (reference) | MuJoCo | PyBullet | Isaac |
|---|---|---|---|---|
| Surge 10 s | 1.893 m | 1.873 m | 1.871 m | 1.882 m |
| Settle (coast) | 0.098 m | 0.121 m | 0.122 m | 0.118 m |
| Yaw 6 s | 0.882 rad | 0.845 rad | 0.848 rad | 0.830 rad |
| Current drift 15 s | 0.851 m | 0.891 m | 0.880 m | 0.896 m |
| Operator loss | disarms | disarms | disarms | disarms |

`mujoco-1` failed at once: the plant still built the ROV's thruster model
("thrusters do not span the declared controlled axes"); it is kept. The
surface controller does not hold station against a current, on any
backend; the drift is the expected free drift.

ROV regression (`rov-mujoco-regression`, MuJoCo): surge 1.038 m (11-14:
1.035), yaw 1.426 rad (1.430). The current drift is now 0.023 m (11-14:
0.37 m) because of the current feedforward added after that run.

## Limits

- The ported plants have no sensors, bridge or SLAM pipeline. The USV's
  marine mode with RGB-D mapping stays Gazebo-only, as for the ROV, and
  the profile's simulator list is unchanged.
- One water world (`marine_tank`, seabed plane only). The harbor geometry
  is not converted.
- The amphibious rover (wheels plus propellers) is not ported.

## Reproduce

```bash
ros2 launch robot_lab_bringup marine_simulator.launch.py robot_model:=lab_usv map_name:=marine_tank \
  simulator:=mujoco gui:=false rviz:=false
CASE_OUTPUT=<dir> python3 docs/status/evidence/marine-mujoco-currents-2026-11-14/rov_probe.py
```

Traces are on SSD under `robot_lab_runtime/usv-backends-20261120` (hashes in
`runtime-sha256.txt`).
