# R11: MOLAR rides the fleet lift between floors (November 19, 2026)

**Status: implemented, experimental.** MOLAR (`molar_hri`, 158 kg) is now
a shared-world fleet member. In `fleet_facility` on Gazebo it boards the
physical lift, rides L0 → L1 → L0, and the infrastructure switches its
floor and namespaced Nav2 map at both landings (2 of 2 runs).

## What changed

- **MOLAR fleet member** (`fleet_session.py`):
  - `molar_hri` joins the member models with a 0.49 m planning radius (its
    0.70 × 0.62 m base).
  - Its own diff-drive base controller is loaded under the fleet's
    controller name, with the same wheels, geometry and limits
    (`molar_fleet_controllers`). The lift, telescope and roller controllers
    are not loaded in a fleet.
  - The fleet derivative adds a `base_footprint` root at `base_link`, which
    already sits on the ground plane.
  - MOLAR's measured AMCL overlay (from goal B) is merged for MOLAR only.
    Bumperbot and Labbot keep the shared `amcl.yaml`.
  - Its URDF scan is already the ±105° front field.
- **Lift capacity** (`scripts/generate_fleet_facility.py`, regenerated
  `fleet_facility.world`; the hand-edited `infrastructure.yaml` is
  untouched):
  - The car drive was capped at 1600 N, but the 50 kg car with MOLAR needs
    about 2040 N. The cap is now 4000 N with a ±3000 N integral.
  - The integral gain rose from 1000 to 8000, so the extra load is taken up
    in about 5 s instead of about 40 s. The integral zero (0.2 rad/s) stays
    well below the lift's natural frequency, for both light and MOLAR
    loads.
  - Speed, P/D gains and the 3 mm landing tolerance are unchanged.
- **Trial driver** `lift_trial.py`: the November 2 floor-map trial with
  member, spawn, boarding line and speed as parameters. MOLAR spawns at
  x 2.0, because at 2.5 its nose would overlap the closed landing gate. It
  boards until the base is at x ≥ 3.95, so the whole body is on the 2 m car.

## Measured (`fleet_facility`, Gazebo; `lift_trial.py`)

| Run | Member | Result | L1 arrival (body z) | L0 return (body z) |
|---|---|---|---|---|
| `run-1` | MOLAR, old integral gain 1000 | **not passed**: the lift stopped at 2.9906 m (9.4 mm under L1, tolerance 3 mm), so the floor never switched; the ride phase timed out | 2.991 | — |
| `run-2` | MOLAR | passed in 78.5 s: floor L1, map `map_L1.yaml`, then L0, `map.yaml` ready | 3.000 | −0.0001 |
| `run-3` | MOLAR | passed in 78.6 s | 2.999 | 0.000 |
| `bumperbot-regression` | Bumperbot (`floor-map-session.yaml` of 2026-11-02) | passed in 68.7 s, as before (70.2 s) | 3.000 | 0.000 |

Boarding used the fleet's manual mode at 0.12 m/s under its watchdog; there
were no pose writes.

## Limits

- Lift ride and floor/map switch only. A MOLAR Nav2 goal on L1 and a full
  fleet transport task (queue, parking, return) with MOLAR are not run yet.
- One MOLAR at a time; Gazebo only. The deck lift, telescope and roller are
  not driven in a fleet.

## Reproduce

```bash
ros2 launch robot_lab_bringup shared_world.launch.py \
  session:=docs/status/evidence/r11-molar-lift-2026-11-19/molar-lift-session.yaml navigation:=true gui:=false rviz:=false
ROBOT_LAB_EXPERIMENT_OUTPUT=<dir> CASE_MEMBER=molar CASE_SPAWN_X=2.0 CASE_BOARD_X=3.95 \
  python3 docs/status/evidence/r11-molar-lift-2026-11-19/lift_trial.py
```

Logs and traces are on SSD under `robot_lab_runtime/molar-lift-20261119`
(hashes in `runtime-sha256.txt`).
