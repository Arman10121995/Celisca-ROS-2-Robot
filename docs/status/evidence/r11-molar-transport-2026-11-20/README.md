# R11: MOLAR fleet lift transport L0 → L1 → L0 (November 20, 2026)

**Status: implemented, experimental.** MOLAR (`molar_hri`) completes the
fleet's native lift transport between floors in `fleet_facility` on Gazebo.
This is **Fleet & Swarm → Facility → Transport selected robot**, the workflow
Bumperbot passed on November 2. The operator only sends two Facility
`transport` requests, one per floor. The infrastructure's `LiftTransport` then
drives each step through Nav2: the approach, the landing gate, boarding, the
physical ride, the floor and map switch, and a settled exit. Each step is
judged by independent body truth. There are no pose writes and no manual
driving.

## What changed

- **Per-model transport goals** (`fleet_infrastructure._transport_config`): a
  `transport.models.<model>` entry in the world's infrastructure config
  replaces the world's boarding, landing and exit goals for members of that
  model. Other members keep the world's goals unchanged.
- **Exit goals** (`LiftTransport.landing`): an optional `exits` map gives the
  goal for leaving the car, separate from the approach goal on the same
  floor. Without it the landing is used for both, as before. Parking
  clearance is measured from the exit goal.
- **MOLAR's goals** (`fleet_facility/infrastructure.yaml`, generator in step):

  | Goal | Bumperbot / Labbot (world) | MOLAR |
  |---|---|---|
  | L0 approach | (2.3, 0, 0) | (1.75, 0, 0) |
  | Boarding | (3.65, 0, 0) | (3.95, 0, 0) |
  | L1 approach | (2.2, 0, 0) | (2.1, 0, π) |
  | L0 / L1 exit | same as the approach | (2.1, 0, π) |

  Each goal heading is the heading MOLAR arrives with on a straight route.
  So MOLAR never makes a final-heading turn next to a rail, the gate or the
  car walls. Turns happen at route starts, where the controller's
  rotate-to-heading turns in place with zero translation.

The scan guard, Nav2 tolerances, lift and the transport's settle gate
(< 0.15 m, < 0.15 rad, < 0.03 m/s, < 0.05 rad/s for 1 s) are unchanged.

## Measured (`fleet_facility`, Gazebo, `transport_trial.py`)

| Run | Goals | Result |
|---|---|---|
| `run-1` | L0 approach (2.1, 0, 0) | **not passed**: the approach stalled 0.13 m short. MOLAR's lidar is 0.34 m ahead of its base, so at x 2.1 the closed gate is 0.41 m from it. The guard needs 0.61 m + speed terms for MOLAR's 0.49 m radius ("scan stopping corridor"), and Nav2 aborted after 141 s |
| `run-2` | L0 approach 1.75, boarding (3.95, 0, π) | **not passed**: approach and gate opening passed, then the 180° final turn in the car failed. The RPP controller turns in place only once its estimate is inside the 2 cm goal tolerance. AMCL drifted ~0.1 m while MOLAR turned, so RPP kept adding small translations, and the guard stopped them against the car walls. Nav2 aborted after 188 s |
| `run-3` | as in the table above | **passed** in 182.9 s |
| `run-4` | same | **passed** in 188.6 s |
| `bumperbot-regression` | Bumperbot, world goals (2026-11-02 session) | **passed** in 174.2 s (319.9 s on 2026-11-02, before the stronger lift); exits L1 (2.215, 0.008) and L0 (2.398, 0.023), yaw ≈ 0 |

Landings by truth (`run-3`; `run-4` within 9 mm and 0.02 rad):

| Exit | x / y / z (m) | yaw (rad) | Rates | Active map |
|---|---|---|---|---|
| L1 (93.9 s) | 2.143 / −0.007 / 3.000 | −3.107 | 0 / 0 | `map_L1.yaml` |
| L0 (182.9 s) | 2.173 / −0.005 / 0.000 | −3.116 | 0 / 0 | `map.yaml` |

## Limits

- One MOLAR, alone, on Gazebo. MOLAR with a peer in the facility, the
  parking extension and the RMF graph (`fleet_building_graph` still reads the
  world goals) are not run.
- The deck lift, telescope and roller are not driven in a fleet, so no
  payload rides along.
- The L0 approach stops 0.72 m short of the gate, because the guard's
  corridor is measured from the forward lidar.

## Reproduce

```bash
ros2 launch robot_lab_bringup shared_world.launch.py \
  session:=docs/status/evidence/r11-molar-transport-2026-11-20/molar-transport-session.yaml \
  navigation:=true gui:=false rviz:=false
CASE_OUTPUT=<new dir> CASE_ROBOT=molar \
  python3 docs/status/evidence/r11-molar-transport-2026-11-20/transport_trial.py
```

Logs and traces are on SSD under `robot_lab_runtime/molar-transport-20261120`
(hashes in `runtime-sha256.txt`).
