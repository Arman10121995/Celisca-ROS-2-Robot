# R6.7 heightfield convention probe (2026-10-05)

Every non-Gazebo backend silently dropped SDF `<heightmap>` terrain: the shared
reader listed it under `unsupported geometry`, so MuJoCo, PyBullet and Isaac
each rendered a world with the ground missing. This directory holds the
measurements that fixed the replacement converter's layout.

**Why a probe was needed.** The SDF spec says only that `<size>` is "the size
of the heightmap in world units". It does not say how pixel intensity becomes
metres, which image row is `+y`, or whether the raster spans `size` or
`(n-1)/n * size`. Each of those is a plausible-looking guess that produces
terrain which is the right *shape* and the wrong *place* or *height*.

## Method

Deliberately asymmetric rasters were dropped into the **installed** Gazebo
(gz-sim 8 / Harmonic, ODE, `/usr/bin/gz`) and the settled sphere poses were
read back from `/world/<name>/dynamic_pose/info`. 0.2 m spheres were released
12 m above known `(x, y)` points; `surface_z = pose_z - 0.2`. Rasters, world
files, server logs and raw pose dumps are kept here so every number below can
be re-derived. `parse.py` extracts the poses from a dump.

## Results

| Probe | Raster | Measurement (Gazebo) | Convention |
|---|---|---|---|
| `n_a100.png` | uniform 100, `size` z = 4 | surface **4.0 m** | elevation is normalised by the raster **maximum**, not 255 |
| `n_50_150.png` | bands 50 / 150 | **1.333 m** and **4.0 m** | `v / max * size_z` (50/150·4 = 1.333) |
| `t_yband.png` | top half 0, bottom half 255 | `y=+8` → **0.0**, `y=-8` → **4.0** | image **row 0 is maximum +y** |
| `t_xband.png` | left half 0, right half 255 | `x=-8` → **0.0**, `x=+8` → **4.0** | column 0 is minimum x |
| `g33/g65/g17.png` | uniform 255, 20 m extent | server AABB **±9.69697 / ±9.84615 / ±9.41177** | cell-centred grid spanning `(n-1)/2 * size/n` |

The `/255` alternative is ruled out numerically: it predicts 1.568 m and
2.353 m for the 100 and 50/150 probes, against measured 4.0 m and 1.333/4.0 m.
The `size/2` span alternative predicts ±10.0 m for every resolution, against
the three distinct measured AABBs.

Gazebo's own heightfield AABB is quoted verbatim in the retained `p*.log`
server logs (`ODE Heightfield AABB: min = {-9.69697, ...} max = {9.69697, ...}`).

Two discarded trials are retained rather than hidden: `ramp.*` used a sloped
raster on which the probe spheres **rolled downhill and off the terrain**
(reported `surface_z` of −6.1 to −727 m), and the first attempt to read poses
used `ign topic -e` against a gz-msgs server, which returned nothing. Flat
plateaus plus `gz` CLI services were used instead.

## What the four backends now do

| Backend | Mechanism | Verified |
|---|---|---|
| Gazebo | native `<heightmap>` | reference, measured above |
| MuJoCo | native `<asset><hfield>` (PNG re-encode; `size` = radius, radius, elev, elev) | ball rest 0.4996 / 4.4996 m with the world pose at z=0.5, i.e. 0.0 / 4.0 m of terrain |
| PyBullet | concave trimesh from the shared converter | ball rest **−0.0000 / +4.0000 m** |
| Isaac | triangle mesh staged to binary STL | 2048 faces, z bounds 0.000–4.000, unit normals |

MuJoCo and PyBullet therefore reproduce the Gazebo surface exactly at the
probe points. **Isaac is verified at the geometry level only** — the STL is
correct and loadable, but no Isaac runtime contact trial was executed in this
session, so Isaac terrain *contact* remains unqualified.

`robot_lab_utils/heightfield.py` implements this layout, and
`src/robot_lab_bringup/test/test_sdf_heightfield.py` pins each measured
number as a regression test.

## Scope

This closes the shared conversion and the three non-Gazebo backends. It does
**not** close R6.7: the pinned terrain-generator web workflow, provider
attribution/manifest, deterministic regeneration and class-specific
driving/walking/flight missions on terrain are still outstanding.