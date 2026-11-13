# Generated terrain on four simulators — 2026-11-13 (ROADMAP R6.7)

## What was implemented

- `robot_lab_utils/terrain_generator.py`: one elevation field becomes an SDF
  `<heightmap>` world, a MuJoCo hfield scene, a slope/step occupancy grid, a
  preview and a manifest. Sources: **procedural** (spectral fractal terrain
  from a seed: smoothness exponent, smallest feature, height range) or an
  **elevation raster** (GeoTIFF / `.npy` / 16-bit PNG in metres, with required
  attribution and license; centre-cropped and resampled). A flat spawn pad is
  blended in with a smoothstep. The raster follows the measured Gazebo layout
  ([October 5 probe](../r6-7-heightfield-probe-2026-10-05/README.md)); heights
  are 8-bit, quantised to relief/255, and every derived output uses the
  quantised field.
- `manifest.json`: inputs, provider/attribution/license/source sha256, datum,
  polygon, metres per pixel, elevation range, spawn, occupancy statistics and
  the sha256 of every output. `regenerate()` rebuilds from it and compares
  every file.
- Registration in `$ROBOT_LAB_RUNTIME_ROOT/terrain/maps.yaml`, merged by
  `installed_assets.installed_profiles('maps')` (a generated name can never
  shadow an existing world id).
- GUI: **Worlds & mapping → Terrain** (`robot_lab_gui/terrain_controls.py`):
  source and area, robot use, Generate (off the Tk thread), terrain/drivable
  previews, Use in Launch, Preview 3D, and saved terrains (load settings,
  check regeneration, remove from list).
- `scripts/terrain_support_probe.py`: support height through the real launch.

## Support height on the four simulators

Method: `bumperbot`, Display mode, launched once on flat `nav_empty` and then
at each probe point on the terrain; settled body height from
`/odom/ground_truth` (spread < 2 mm over 1 s of simulation after ≥ 4 s).
Measured support = terrain height − flat height (the robot's own offset
cancels). Expected = the generated, quantised surface (bilinear). Points: the
spawn pad plus the highest and lowest near-flat (< 3° within ~1.5 m) points
inside the central 70 %. Tolerance 3 cm.

`generated_ridges` (seed 6, 40 m, 129 samples, 3.0 m relief, smoothness 4,
smallest feature 14 m) — [report](support-generated_ridges.json),
[manifest](manifest-generated_ridges.json):

| Point | Expected | Gazebo | MuJoCo | PyBullet | Isaac |
|---|---|---|---|---|---|
| spawn pad (0, 0) | 1.918 m | 1.917 | 1.918 | 1.918 | 1.918 |
| summit | 3.000 m | 2.999 | 3.000 | 3.000 | 3.000 |
| lowest ground | 0.000 m | 0.002 | 0.002 | 0.002 | 0.002 |

`generated_hills` (seed 11, 40 m, 129 samples, 2.0 m relief, smoothness 3,
smallest feature 4 m) — [report](support-generated_hills.json):

| Point | Expected | Gazebo | MuJoCo | PyBullet | Isaac |
|---|---|---|---|---|---|
| spawn pad (0, 0) | 0.949 m | 0.948 | 0.949 | 0.949 | 0.949 |
| high ground (9.9, 14.0) | 1.129 m | 1.129 | 1.130 | 1.129 | 1.130 |
| low ground (−11.2, 14.0) | 0.933 m | 0.934 | 0.933 | 0.934 | 0.934 |

All 24 points pass; the largest error is 2.1 mm. This is the first measured
**Isaac terrain contact** (the October 5 work verified Isaac at the geometry
level only). Both terrains regenerate byte for byte from their manifests
(7 files each). The bodies dropped 0.1 m from their spawn height onto the
surface on every simulator, so the heights are contact, not a held pose.

After the probe the generator gained the original module's named styles
(rolling hills, ridges, rough, crater), preview overlays and a CLI, and the
visual mesh's comment header was reworded. Both terrains were regenerated:
world, raster, MuJoCo scene and occupancy files are byte-identical to the
probed ones (the probe report lists their hashes); only `preview.png` and the
mesh's first comment line changed (mesh geometry identical, checked by
re-hashing with the old header). The manifests here are the current ones.

## Retained failure and repair

[First Gazebo run](retained-gazebo-heightmap-render-crash.json): every terrain
launch aborted. Gazebo Fortress (ign-gazebo 6, ogre-next 2.2.5 GL3+) draws
`<heightmap>` visuals with its Terra terrain shaders, and on this Jetson the
fragment shader fails to compile as soon as the robot's depth camera renders
it. The flat reference was fine; MuJoCo and Isaac passed in the same run. The
repository's own `terrain_heightfield` world crashed the same way with a
camera robot (checked the same day).

Repair: generated worlds, `terrain_heightfield` and `gait_low_relief` keep
`<heightmap>` for collision (the measured path every backend converts) and
draw the same grid as an OBJ mesh from `heightfield.py`. Afterwards Gazebo
passed every point above, and `terrain_heightfield` streamed camera images
with no crash. The collision rasters, occupancy maps and MuJoCo scenes of the
two repository worlds are unchanged.

## Scope

Support height and regeneration only. Driving, walking and flight on
generated terrain (V9), imported-raster projections/vertical datums, and
web tile providers (optional, token-gated) are not covered.
