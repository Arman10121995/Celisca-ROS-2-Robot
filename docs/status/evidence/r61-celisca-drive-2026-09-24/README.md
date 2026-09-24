# Celisca scale and GUI drive qualification (2026-09-24)

Source baseline: `c6e50f9` plus this working-tree change. Tests ran on the
Jetson AGX Orin with ROS 2 Humble. Each live launch used its own ROS domain,
headless simulator and `start_rviz:=false`; navigation goals were nevertheless
published on `/goal_pose`, the same path as RViz's 2D Goal Pose tool.

## Geometry and spawn

The Celisca floor STL spans about 34 × 15.5 m before SDF scaling. Its former
`3 3 3` scale made a roughly 102 × 46.5 m building. The furniture STLs
already span about 59 × 27 m before SDF scaling, so the same multiplier made
them roughly 176 × 80 m. The floor worlds now use scale 1, the furniture
worlds 0.575, and floor-2 furniture is translated `(0.9, -2.7)` m to align
its footprint. The 100 × 100 m visible physics floor is now 40 × 25 m;
MuJoCo's generated plane has the same visible footprint. The six Celisca
occupancy YAMLs scale their resolution and origin by one third, so their
pixel geometry stays registered with the smaller worlds. MJCF was regenerated
from SDF and `gen_mjcf_worlds.py --check` passed.

At `(0, 0)` the scaled occupancy maps offer only 0.16–0.22 m of wall
clearance. A live floor-2 furniture/Labbot Nav2 goal aborted with "Starting
point in lethal space". All six Celisca spawn and initial poses now use
`(0, 1)`, which has 0.97–1.15 m of occupancy-map clearance. A regression
test reads the actual PGM and checks 0.35 m of free area around every spawn.
After this change, the same furnished goal succeeded.

## GUI drive path

The GUI sends `/key_vel` into `twist_mux`, which publishes
`/robot_lab_controller/cmd_vel_unstamped`. Gazebo consumes the relayed stamped
command; the other spawners formerly listened only to raw `/cmd_vel`. They
now consume the mux output when present, while retaining raw `/cmd_vel` for
standalone/display runs. The mux input remains authoritative over competing
raw navigation commands. `sim_drive_check.py --cmd-topic /key_vel --quick`
exercises this exact GUI path at its 10 Hz repeat rate.

| Mode, robot, map | Backend | `/key_vel` result |
|---|---|---|
| localization, Bumperbot, nav_obstacle | PyBullet | 0.84 m displacement in an 8 s manual probe |
| SLAM, Labbot, nav_obstacle | MuJoCo | 0.82 m displacement in a 7 s manual probe |
| 3D SLAM, Bumperbot, nav_obstacle | PyBullet | 0.40 m in the quick straight phase; stopped afterward |
| localization, Bumperbot, Celisca floor-1 furniture | PyBullet | 0.23 m in the quick straight phase; stopped afterward |
| localization, Bumperbot, nav_obstacle | Gazebo | 1.19 m in the quick straight phase; stopped afterward |
| localization, Bumperbot, nav_obstacle | Isaac Sim | 0.084 m in the quick straight phase at slow host real-time factor; stopped afterward |

## Resized-map navigation

The command was `python3 scripts/sim_nav_check.py --via-topic --distance
1.5 --clearance 0.25 --timeout 120` (distance 2 m in the first PyBullet
floor-1 run; timeout 180 s for Isaac). Every reported result below was
`succeeded`:

| Backend | Robot, map | Goal | Final map-frame error |
|---|---|---|---|
| PyBullet | Bumperbot, Celisca floor 1 (before spawn moved from `(0,0)` to `(0,1)`) | `(1.85, 0.77)` | 0.256 m |
| PyBullet | Labbot, Celisca floor-2 furniture | `(1.5, 1.0)` | 0.326 m |
| MuJoCo | Bumperbot, Celisca floor 1 | `(1.5, 1.0)` | 0.257 m |
| Gazebo | Labbot, Celisca floor 1 | `(1.5, 1.0)` | 0.252 m |
| Isaac Sim | Bumperbot, Celisca floor 1 | `(1.5, 1.0)` | 0.256 m |

The final combined GUI, map, bringup and backend selection run passed 320 tests
with two integration tests skipped. The GUI command/autofill suite contributed
24 passes after fixing its Tk teardown fixture. The GUI still correctly disables walking,
mapping and navigation for Berkeley Humanoid Lite and other robots without a
verified `/cmd_vel` locomotion controller. Its supported localization and
display selections continue to autofill a runnable command.
