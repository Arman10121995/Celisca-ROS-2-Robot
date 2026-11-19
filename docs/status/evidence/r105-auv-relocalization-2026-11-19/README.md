# R10.5: the Tethys AUV relocalizes on a saved map and flies from it (November 19, 2026)

**Status: implemented, experimental.** In a new textured tank, Tethys maps
its route and saves the map. In fresh launches it relocalizes on that map
and flies the same mission from it, 2 of 2 runs, on Gazebo. The host was
also training a Go2 policy.

## What changed

- **Why the tank never relocalized (run 5 of
  [the estimated mission](../r105-auv-estimated-mission-2026-11-19/README.md)):**
  - The world was featureless, and the Tethys RTAB-Map pipeline is
    geometry-only by design (`Kp/MaxFeatures -1`, ICP registration only),
    so repeating panels cannot pull a cruising AUV sideways.
  - A saved map therefore held no visual words to relocalize against.
  - ICP alone saw only the seabed plane: "structural complexity too low".
- **`visual_features:=true`** (`auv_simulator.launch.py`, opt-in) turns on
  RTAB-Map's visual words and visual+ICP registration
  (`marine_pipeline.pipeline(visual_features=)`). It applies to mapping
  and to localization. The default stays geometry-only, as measured.
- **New world `marine_tank_textured`**
  (`robot_lab_maps/tools/marine_textured_tank.py`). It is `marine_tank`
  plus two 2 × 3 m panels of distinct ArUco markers, each with a
  marker-textured 0.8 m box 0.5 m in front of it.
  - The panels stand 6 m from the measured start, at ±30° of the heading.
  - The Tethys RGB-D camera looks along the heading (72° field of view,
    8 m depth clip), so both are in view at the start, where
    relocalization has to happen.
  - Both are more than 2.1 m from the route line and its 3 m terminal
    braking corridor, beyond the planner's 1.3 m radius.
  - No texture repeats.
  - Registered in `sim_maps.yaml` and the registry environments. The
    Tethys spawn is the same as in `marine_tank`.

## Measured (spawn (−4, −3, −3), yaw 0.5; goal (−0.93, −1.32, −3); `auv_mission_probe.py`)

| Run | World layout / mode | Result |
|---|---|---|
| `map-1` | boards and pillars behind the start (first layout), geometry-only, map and save | completed 0.24 m from the goal; map saved |
| `reloc-1` | relocalize on `map-1` | **not passed**: never localized. The camera faces forward, so the structure was out of view, and the map had no visual words |
| `map-2` | one panel with boxes 3.9 m past the goal, visual features | **not planned**: the mapped panel lay inside the 3 m terminal braking corridor |
| `map-3` | final layout (side panels), visual features, map and save | completed 0.31 m from the goal; map saved |
| **`reloc-2`** | **relocalize on `map-3`**, fresh launch | **localized; mission completed 0.18 m from the goal** (truth), estimate error ≤ 0.14 m, 18.5 s |
| **`reloc-3`** | repeat of `reloc-2` | **localized; completed 0.31 m from the goal**, estimate error ≤ 0.14 m, 17.9 s |

RTAB-Map logged "Localization was good" on the saved map. The controller
accepted the plan only once it reported `localized`.

## Limits

- One short route in one purpose-built world, Gazebo only. Relocalization
  depends on visual words; the default geometry-only pipeline still does
  not relocalize in this tank.
- Opting into visual features for cruising brings back the repeating-panel
  risk that geometry-only mode avoids. Here every marker is unique.
- Ideal DVL and pressure emulation; the longer route from the earlier
  receipt still diverges.

## Reproduce

```bash
cd ../r105-auv-estimated-mission-2026-11-19
python3 launch_auv.py <dir>/map.log map_name:=marine_tank_textured slam:=true visual_features:=true spawn_z:=-3.0
CASE_OUTPUT=<dir>/map CASE_TARGET=-0.93,-1.32,-3.0 CASE_SAVE_MAP=1 python3 auv_mission_probe.py
python3 launch_auv.py <dir>/reloc.log map_name:=marine_tank_textured slam:=true visual_features:=true \
  localization:=true database:=<saved rtabmap.db> spawn_z:=-3.0
CASE_OUTPUT=<dir>/reloc CASE_TARGET=-0.93,-1.32,-3.0 python3 auv_mission_probe.py
```

Logs and traces are on SSD under `robot_lab_runtime/auv-reloc-20261119`
(hashes in `runtime-sha256.txt`). The saved map is
`robot_lab_runtime/marine_missions/8c91c94dc5794c629ff0d4abbc643918/map-639a310959`.
