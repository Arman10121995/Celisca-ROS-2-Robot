# R10.5: Tethys relocalizes on a geometry-only saved map (November 20, 2026)

**Status: implemented, experimental.** The Tethys AUV now relocalizes on a
saved map built in its default geometry-only mode, with no visual words,
and flies its mission from that map, 2 of 2 runs in `marine_tank_textured`
on Gazebo. On 11-19 only the opt-in `visual_features:=true` relocalized. In
the plain `marine_tank` geometry-only relocalization still fails: only the
seabed plane is in view, which gives ICP nothing to match.

## Why it failed, and what changed

- **The cause.** Without visual words RTAB-Map has no appearance-based loop
  closure, and its default (`RGBD/StartAtOrigin=false`) puts a localizing
  robot at the mapping session's *last* pose, here the end of the route.
  ICP proximity matching only searches near that assumed pose. So a fresh
  launch at the spawn never matched (`reloc-geom`: no localization in
  300 s; the idle estimate drifted 0.76 m).
- **The change** (`marine_pipeline.py`): in geometry-only Tethys
  localization, `RGBD/StartAtOrigin=true`. The marine estimator starts at
  the spawn (`marine_spawn_imu_v1`), as did the mapping run, so the map
  origin is where the new session starts. ICP proximity against the nearby
  map nodes then confirms and corrects the pose.
  - RTAB-Map logged "last localization pose is ignored (RGBD/StartAtOrigin=true),
    assuming we start at the first node", then "Localization was good".
  - Visual-feature mode and mapping are unchanged. A test pins the
    parameter's condition.

## Measured (Gazebo, spawn (−4, −3, −3) yaw 0.5, goal (−0.93, −1.32, −3), geometry-only, `slam:=true`)

| Run | World | Step | Result |
|---|---|---|---|
| `map-geom` | textured tank | map and save (35 nodes) | completed 0.113 m from the goal; map saved |
| `reloc-geom` | textured tank | relocalize, RTAB-Map default start | **not passed**: never localized (300 s); estimate drift 0.76 m |
| **`reloc-geom-origin`** | textured tank | relocalize, start at map origin | **localized; mission completed 0.084 m from the goal**, estimate error ≤ 0.098 m, 16.2 s |
| **`reloc-geom-origin-2`** | textured tank | repeat | **localized; completed 0.092 m**, ≤ 0.098 m, 15.9 s |
| `map-plain` | plain tank | map and save | completed 0.101 m; map saved |
| `reloc-plain` | plain tank | relocalize, start at map origin | **not passed**: every ICP match rejected for "too low complexity" (seabed plane only); never localized |

The panels' 0.8 m boxes and walls in the textured tank give ICP the 3D
structure it needs. The markers themselves are not used in this mode.

## Limits

- The session must start at the mapped spawn. A different start pose needs
  visual words (`visual_features:=true`) or an operator initial pose.
- 3D structure must be in the camera's view at the start, as in the
  textured tank. Featureless water does not relocalize.
- One short route, one world, Gazebo only.

## Reproduce

```bash
python3 ../r105-auv-estimated-mission-2026-11-19/launch_auv.py <dir>/map.log map_name:=marine_tank_textured slam:=true spawn_z:=-3.0
CASE_OUTPUT=<dir>/map CASE_TARGET=-0.93,-1.32,-3.0 CASE_SAVE_MAP=1 python3 ../r105-auv-long-route-2026-11-20/auv_mission_probe.py
python3 ../r105-auv-estimated-mission-2026-11-19/launch_auv.py <dir>/reloc.log map_name:=marine_tank_textured slam:=true \
  localization:=true database:=<saved rtabmap.db> spawn_z:=-3.0
CASE_OUTPUT=<dir>/reloc CASE_TARGET=-0.93,-1.32,-3.0 python3 ../r105-auv-long-route-2026-11-20/auv_mission_probe.py
```

Logs are on SSD under `robot_lab_runtime/auv-geom-reloc-20261120` (hashes in
`runtime-sha256.txt`).
