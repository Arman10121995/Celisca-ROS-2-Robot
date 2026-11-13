# Real-world area import on four simulators — 2026-11-13 (ROADMAP R6.8)

## Survey before building

Reviewed first (the lab's "import before building" principle):

| Tool | Fit | Decision |
|---|---|---|
| [saiaravind19/gazebo_terrain_generator](https://github.com/saiaravind19/gazebo_terrain_generator) (BSD-3, active) | map polygon → heightmap, satellite texture, OSM buildings → Gazebo world | needs a Mapbox token, Gazebo only, no roads/trees/traffic, Python ≥ 3.12; recorded as an optional route for satellite texture |
| [OSM2World](https://github.com/tordanik/OSM2World), [blosm](https://github.com/vvoovv/blosm), [gazebo_osm](https://github.com/osrf/gazebo_osm) | OSM → 3D models / Blender / old Gazebo roads | not used: no four-backend conversion; the building/road geometry needed here is simple |
| CesiumJS (Apache-2.0) | globe with OSM imagery, no token | **used** for the picker |
| Terrain Tiles, AWS Open Data (Terrarium) | global elevation tiles, no account | **used** for the ground |
| OpenStreetMap via Overpass (ODbL) | buildings, heights/levels, roads, trees, forests, water | **used** for features |
| Eclipse SUMO 1.28 (EPL-2.0, `pip install eclipse-sumo`, aarch64) | road network with sidewalks, vehicles and pedestrians from OSM | **used** for people and traffic |

The lab's R6.7 terrain generator provides the four-backend heightmap conversion, the manifest, the
registry and the support-height probe.

## Implemented

`robot_lab_utils/real_world_import.py`, `real_world_traffic.py`, `real_world_server.py` +
`web/globe.html`, GUI **Worlds & mapping → Real place** (`robot_lab_gui/real_world_controls.py`), CLI,
`scripts/terrain_support_probe.py` (real-world mode: spawn, two open points, two largest roofs). See the
[guide](../../../tutorials/real-world-import.md).

## Rostock centre (54.0874–54.0901 N, 12.1382–12.1428 E, 308 × 300 m)

116 buildings, 155 roads, 57 trees; relief 11.95 m (8-bit steps of 4.7 cm); 257 × 257 terrain samples;
spawn chosen at (0.04, −0.04) with 1.5 m clearance. [Manifest](manifest-rostock_centre.json),
[preview](preview-rostock_centre.png).

Support height through the real launch (`bumperbot`, Display, settled `/odom/ground_truth` minus the same
robot on `nav_empty`) — [report](support-rostock_centre.json):

| Point | Expected | Gazebo | MuJoCo | PyBullet | Isaac |
|---|---|---|---|---|---|
| spawn (0.0, 0.0) | 8.388 m | 8.387 | 8.388 | 8.371 | 8.388 |
| open ground (89.8, −72.3) | 8.813 m | 8.812 | 8.815 | 8.812 | 8.813 |
| open ground (−75.1, 75.2) | 6.978 m | 6.978 | 6.989 | 6.975 | 6.974 |
| roof (−112.8, 82.8) | 19.956 m | 19.955 | 19.945 | 19.956 | 19.956 |
| roof (43.3, 0.6) | 27.097 m | 27.096 | 27.086 | 27.097 | 27.097 |

All 20 points pass (3 cm tolerance; largest error 1.7 cm, PyBullet at the spawn). A robot resting on a
roof at the building's height on every simulator confirms the building collisions (MuJoCo uses each
building's convex hull: 1.1 cm lower on these roofs).

People and traffic: `rostock_traffic` (same area): SUMO 1.28 built the network with sidewalks and
crossings, 300 s of random trips; 40 people and 30 vehicles became Gazebo actors.
[Manifest](manifest-rostock_traffic.json).

In Gazebo, the world with its 70 actors loads and runs, and the same five support points pass
(largest error 1 mm; [report](support-rostock_traffic.json)). Two repairs came out of this check:

* The Gazebo launch copies the world to a temporary folder and made relative `<uri>`s absolute, but
  not relative `<filename>`s. The vehicle skin was "not found" there. It now rewrites both.
* A plain box mesh is not an actor skin ("Mesh skeleton ... not found"). The vehicle box is now a
  rigged Collada mesh with one bone and a static `walk` animation, and the world was regenerated.

The final run shows no mesh or skeleton warnings. That the people and vehicles render and move was not
checked visually: no camera was in the scene.

## Retained failures and repairs

* Gazebo Fortress segfaulted loading the building meshes (`dart::collision::OdeMesh::fillArrays`, null
  normals): the OBJ files had no vertex normals. Repair: every generated mesh carries normals.
* The step rule flagged ordinary slopes on the 1.2 m grid as kerbs (contour rings of blocked cells).
  Repair (shared with R6.7): a step is a jump larger than the slope limit allows across one cell. The
  procedural `generated_ridges` map changed accordingly (world files unchanged).
* [First probe](retained-road-collision-run.json): the spawn point read 3.3 cm high on MuJoCo, PyBullet
  and Isaac. Roads had no `<collision>`, so the shared SDF reader used their 4 cm ribbons as collision
  there. Repair: a far-underground placeholder collision keeps roads and water visual-only everywhere.
* An earlier check ran while two of my own leftover launches were on the same ROS domain (a `setsid`
  wrapper forked); those readings were discarded and the launches stopped by PID.

## Scope

Support height, roof contact and loading. Robot missions in imported places, people/traffic behaviour,
map projection/vertical datum against surveyed points, and satellite textures are not covered.
