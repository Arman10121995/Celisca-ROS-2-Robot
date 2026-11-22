# V5: PX4 manual flight in the installed worlds (November 21–22, 2026)

**Status: 7 of 8 worlds passed; terrain_heightfield was refused at
takeoff.** On November 5, none of these worlds flew.

`scripts/qualify_px4_drive_gui.py` drives the normal Drone page of the GUI
with the native X500 on PX4 SITL in Gazebo Harmonic. Each run:
1. presses Run;
2. checks takeoff clearance, then presses Takeoff;
3. presses the seven direction buttons and the altitude controls, then
   release and Stop;
4. tests publisher loss, then presses Land.

An independent Gazebo body (`/px4/odometry_truth`) measures every motion.
The support matrix re-checks each report as `flight_manual_screen`.

| World | Result | Checked rise | Takeoff height | Publisher-loss drift |
|---|---|---|---|---|
| aerial_course | **passed** | 3.0 m | 2.94 m | 0.04 m |
| aerial_indoor | **passed** | 3.0 m | 2.98 m | 0.03 m |
| nav_maze | **passed** | 3.0 m | 3.00 m | 0.02 m |
| nav_warehouse | **passed** | 3.0 m | 2.96 m | 0.05 m |
| celisca_floor_1 | **passed** (second run) | 3.0 m | 3.08 m | 0.06 m |
| outdoor_terrain | **passed** | 3.0 m | 2.98 m | 0.04 m |
| small_house | **passed** | 2.04 m (ceiling limited) | 1.98 m | 0.01 m |
| terrain_heightfield | **refused at takeoff** | — | — | — |

## What had to change

Each problem below was found by a failed run; every failed run is kept
under `failed-before-fix/` and `first-rerun/`.

1. **Readiness before the check.** Since November 13, Takeoff waits for
   the resting heading to settle (`takeoff_ready`). The qualifier pressed
   Check earlier, so the check was refused and no run armed. It now waits
   for the published readiness. The gate itself is unchanged.
2. **A refusal was invisible.** The Drone page's planner status poll
   overwrote the check's service reply, so a refused or unanswered check
   left the planner on "unchecked" with no reason shown. The page now keeps
   the reply on the takeoff line. The qualifier presses Check again, as an
   operator would, up to five times, and records each reply. In nav_maze
   and nav_warehouse the first request got no reply within 10 s and the
   second was accepted.
3. **Teardown hang.** After writing its reports, the GUI/rclpy shutdown
   hung until the 900 s run timeout. A 30 s hard exit now follows the
   report.
4. **Display.** Tk's `update()` blocked on the desktop display `:0` for
   three runs. Runs now use Xvfb, as the other agent's GUI runs do, and
   take the shared lock before the run budget starts.
5. **outdoor_terrain** launched as `map_name:=terrain_rough`, the world's
   folder name, which `sim_maps.yaml` does not list. The resolver now maps
   that folder to its canonical id, `outdoor_terrain` (unit test added).
6. **small_house** includes AWS RoboMaker furniture from
   `robot_lab_models`. That directory was missing from two places:
   - PX4's Gazebo resource path, so Gazebo never loaded the world and PX4
     waited for it;
   - the takeoff check's collision scene, which then correctly refused an
     incomplete scene.

   Both now search it, as the PyBullet and Isaac world loaders already did.
7. **Ceiling-limited climbs** are now accepted by the matrix. Indoors, the
   check limits the climb to the clear height. The matrix's flight verdict
   now expects that checked rise instead of always 3 m (test added).

The first celisca_floor_1 flight under Xvfb drifted 0.204 m after Stop,
against a 0.20 m limit. It is kept, and the repeat passed.

## terrain_heightfield

The takeoff gate refused the spawn: "Selected world blocks the minimum
0.8 m takeoff corridor". The refusal is correct:
- The authored spawn (−6, −6, 0.02) lies 1.45 m below the heightmap
  surface. The PNG, the visual mesh and the measured convention all agree
  on that height.
- The PX4 Gazebo Harmonic body rested at z 0.24 there. It was inside the
  hill, so that simulation did not collide with the heightmap.

The spawn is shared with the measured legged runs on this world, so it is
not moved here. Flight on this world needs a spawn on the surface, and
heightmap contact in PX4's Gazebo.

## Reproduce

```bash
docs/status/evidence/v5-px4-worlds-2026-11-21/run_worlds.sh <evidence-dir>
python3 docs/status/evidence/v5-px4-worlds-2026-11-21/derive_measured.py <evidence-dir> <ssd-dir> <world>...
python3 scripts/r9_3_index_missions.py docs/status/evidence/v5-px4-worlds-2026-11-21/px4-worlds-batch.yaml
```

`report-measured.json` adds the Gazebo body summary from each run's trace,
as the November 10 round did. Traces and GUI logs are on SSD under
`robot_lab_runtime/v5-px4-20261121`; their hashes are in `runtime-sha256.txt`.
The mapped-obstacle half of V5 is in
[v5-drone-moving-2026-11-22](../v5-drone-moving-2026-11-22/README.md).
