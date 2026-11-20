# R6.5: the repo `small_house` world now projects to an occupancy grid (November 20, 2026)

**Status: implemented.** `generate_occupancy_map.py` could not project the
repo's `small_house` world: its `model://aws_robomaker_residential_*`
furniture and walls were unresolved. It now produces a 0.1 m grid from all
their collision meshes. That grid agrees with the grid of the installed
`dataset_small_house` snapshot of the same house: occupied-cell IoU 0.846,
free-space IoU 0.949, same origin and 10 free regions.

## What changed

- **`generate_occupancy_map.py`:** after the world's own folders, model
  names are also looked up in the installed external dataset model sets
  (`robot_lab_runtime/external_assets/extracted/*/*/*/models`). The AWS
  RoboMaker models are already on the SSD there. The search list is
  recorded in the generation report (`model_search_dirs`).
- **`sdf_world.uri_resolver`:** as in Gazebo's resource-path search, a
  relative file that is not found next to its model is also looked up
  under the root that holds a model directory. The AWS models write their
  meshes as `file://models/<model>/meshes/...`. Paths that resolved before
  are unchanged.

## Measured

| Grid | Cells (0.1 m) | Occupied | Collision meshes sliced | Regions (enclosed) |
|---|---|---|---|---|
| repo `small_house` (new) | 230 × 155 | 1510 | 942 slice paths, 4137 mask cells | 10 (9) |
| `dataset_small_house` (same generator) | 230 × 155 | 1547 | — | 10 (9) |

The datasets differ slightly: the installed snapshot fixes its furniture
consistently across engines. Before this change the run failed with
"World geometry cannot be projected: unresolved mesh file://models/...". The
other world, SDF and collision tests still pass (30).

## Limits

- The repo's checked-in `small_house/maps/map.pgm` (0.05 m, 500 × 500,
  origin −12.5 m) is not replaced automatically. Use the GUI's
  **Register reviewed grid…** step after review.
- The repo world itself still depends on the SSD dataset's models to load
  in Gazebo.

## Reproduce

```bash
python3 src/robot_lab_maps/tools/generate_occupancy_map.py --world src/robot_lab_maps/maps/small_house/worlds/small_house.world \
  --output-dir /workspace/molar/robot_lab_runtime/<dir> --seed-x 0 --seed-y 0
```
