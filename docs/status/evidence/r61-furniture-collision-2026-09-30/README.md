# Celisca 20% resize and furnished-map collision check (2026-09-30)

The 20% resize in `dc55512` enlarged all six map rasters and plain-floor
worlds, but it applied the plain-floor scale (1.386) to the differently sized
furniture STLs. The furnished 3D footprint was roughly 1.8 times its 2D map.
The corrected furniture scale is 0.79695; floor 2's furniture model offset is
also scaled to `(1.188, -3.564)`. `celisca_extents.py --align` now reports
furnished width ratios 1.02 and height ratios 1.07 (floor 1) / 1.05 (floor 2),
with centre offsets 0.13 m / 0.37 m. The MJCF files were regenerated from SDF.

The raw furniture STLs contain 3,307,716 and 3,968,826 triangles. Gazebo ODE
repeatedly reported `Trimesh-trimesh contach hash table bucket overflow` with
these as collision geometry. In a Bumperbot floor-2 furniture baseline, stack
readiness took 117.8 wall seconds and no goal was accepted within 160 seconds.
Reducing to ~50,000 triangles *alone* still triggered the overflow because the
mesh floor overlapped the world's existing `physics_floor` box. The checked-in
collision proxies retain ~30,000 above-floor wall/furniture facets, while the
original high-resolution STLs remain the Gazebo visuals. They can be regenerated
with `tools/simplify_celisca_furniture_collision.py` and `pymeshlab`.

All post-fix checks used `scripts/sim_nav_check.sh` with `/goal_pose`, a 0.8 m
forward goal, and 0.5 m clearance. The JSON files here are the direct checker
outputs. All starts were `(0, 1.32)` in the 2D map.

| Simulator | Robot | Map | Stack ready | Result | Measured forward motion |
| --- | --- | --- | ---: | --- | ---: |
| Gazebo | Bumperbot | floor 1 furniture | 15.7 s | succeeded, 5.5 s | odom forward 0.542 m |
| Gazebo | Bumperbot | floor 2 furniture | 14.0 s | succeeded, 4.3 s | odom forward 0.555 m |
| Gazebo | Labbot | floor 2 furniture | 12.7 s | succeeded, 4.3 s | odom forward 0.561 m |
| PyBullet | Bumperbot | floor 2 furniture | 8.9 s | succeeded, 7.7 s | 0.553 m |
| MuJoCo | Bumperbot | floor 2 furniture | 8.1 s | succeeded, 6.2 s | 0.545 m |
| Isaac | Bumperbot | floor 2 furniture | 41.3 s | succeeded, 23.6 s | 0.558 m |

This qualifies startup and a short clear-corridor goal, not furniture obstacle
avoidance or the complete robot/map/mode matrix. The furnished 2D PGM is
currently byte-identical to its plain-floor counterpart on each floor, so
furniture is absent from the *static* 2D occupancy map. Local sensor costmaps
may observe it, but a route through the furniture remains to be tested.

Source checks: resize `--factor 1.2 --check`, MJCF generator `--check`, and
the focused map tests passed. The full maps test directory passed 36 tests.
