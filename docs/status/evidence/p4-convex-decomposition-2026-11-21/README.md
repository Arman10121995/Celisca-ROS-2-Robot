# P4: convex decomposition of a large world is not a usable replacement for rigid flexes (November 21, 2026)

**Status: measured negative result; nothing changed in the MuJoCo plant.**

P4's remaining cost is that MuJoCo updates every static flex vertex and
element each step (3.7 of 4 ms per tick on `celisca_floor_1`). A search
for alternatives found that MuJoCo collides only convex meshes, and that
other MuJoCo bridges handle concave static meshes with CoACD convex
decomposition (or SDF geoms, or heightfields for terrain). Static convex
geoms would cost nothing per step away from the robot.

## Tried

- CoACD 1.0.14 (MIT, aarch64 wheel from PyPI, installed into the SSD
  geometry dependencies) on `celisca_floor_1`. The mesh (70,029 triangles
  after the existing retriangulation, 47 × 21 × 1.9 m) is split into the
  same compact spatial pieces as the flexes (3,000 triangles), so the voxel
  preprocessing works at piece scale.
- Threshold 0.05 (CoACD's default concavity): 530 hulls in 437 s (cached by
  mesh hash and parameters).
- `world_convex.py` (the decomposition, cache and checks) and
  `convex_check.py` (the run) are kept here.

## Measured against the reviewed occupancy map (cell centres at 0.1 and 0.3 m)

| Cells | Count | Inside a hull |
|---|---|---|
| free | 1,683,181 | **17.5 %**: false obstacles, rooms and door gaps partly filled |
| occupied | 34,447 | 68.1 %: walls covered |

Each piece mixes floor slab and walls, so hulls at 5 % concavity span room
corners. A finer threshold multiplies hulls and decomposition time and still
only approximates the triangles. The flexes keep every triangle. So the
P4 options that remain are unchanged: fewer collision triangles, or SDF
static collision, each with its own contact and mission checks.

## Reproduce

```bash
python3 docs/status/evidence/p4-convex-decomposition-2026-11-21/convex_check.py \
  src/robot_lab_maps/maps/celisca_floor_1/meshes/celisca_floor_1.stl 1.386 \
  src/robot_lab_maps/maps/celisca_floor_1/maps/map.yaml 0.05
```
