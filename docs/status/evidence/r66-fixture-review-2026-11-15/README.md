# Gazebo example fixtures and Fuel worlds (R6.6, 2026-11-15)

Claude. Implemented, experimental.

## Review of the 100 Fortress example SDFs

`scripts/review_gazebo_examples.py` reads every example world of the pinned
gazebo_examples checkout (`b90cd2c`) and classifies it from its own content.

- **Parsing.** Comments containing `--` and `ignition:` attribute prefixes
  are parsed the way Gazebo's own parser accepts them.
- **Fuel includes.** Models a world includes from Fuel are opened from
  Gazebo's cache and reviewed with the world. Plugins written on the
  `<include>` are attributed to the included model.
- **Load test.** Every world is loaded headless in its own transport
  partition: `ign gazebo -s -r --iterations 200`, 40 s limit. The Jetson's
  harmless EGL driver-probe warnings are not counted as errors.

Report: [review.md](review.md), [review.json](review.json).

| Class | Worlds | Meaning |
|---|---:|---|
| robot_demo | 33 | a model with joints driven by a controller plugin |
| sensor_demo | 23 | demonstrates a sensor |
| environment | 23 | a scene: installable as a world |
| plugin_demo | 16 | demonstrates a system plugin (buoyancy, comms, wind, …) |
| actor_demo | 4 | scripted actors |
| empty | 1 | `actors_population.sdf` is an empty file upstream |

Of the 99 non-empty worlds, 95 load without errors. The 4 that fail:

- `3k_shapes`: exceeds the 40 s budget;
- `nested_model`: a DART "physics entity ptr" error;
- `triggered_publisher`: its `Twist` data string does not parse;
- `tunnel`: cannot resolve `Phone.png`.

## Fuel models and licences

`scripts/fuel_models.py` reads each Fuel model's licence from the Fuel REST
API (`User-Agent: RobotLab-world-import/1.0`), caches it on the SSD, and
clears a world only when every model it uses has an allowed licence
([fuel_licences.md](fuel_licences.md)).

The 32 example worlds that use Fuel reference 80 models. All 80 are CC0 1.0
(33) or CC-BY 4.0 (the rest), so all 32 worlds are licence-clear. The files
come from Gazebo's own Fuel cache, which this host keeps on the SSD.

Two integration changes were needed:

- `sdf_world.fuel_cache_path` resolves `https://fuel.../models/...` URIs to
  that cache, including the old `fuel.ignitionrobotics.org` host. The world
  converter uses it too.
- `file://` URIs are now percent-decoded, because Fuel names carry spaces.

## Installed

`integrate_external_worlds.reviewed_example_environments()` installs the
environments that load, have no unreviewed Fuel include, and are
licence-clear. Previously only `empty`, `default` and `shapes` were installed.

Result: 19 of 20 installed, each with a generated occupancy grid. That adds
16 new world profiles (34 installed maps).

- **From Fuel:** `heightmap`, `import_mesh`, `lightmap`, `plot_3d`.
- **Local:** `debug_shapes`, `empty_gui`, `grid`, `lights`, `minimal_scene`,
  `physics_options`, `resource_spawner`, `rolling_shapes`, `shapes_bitmask`,
  `spaces`, `video_record_dbl_pendulum`, `world_joint`.
- **Not installed:** `polylines`, because the static-geometry extraction
  does not support `<polyline>` shapes.

Robot, sensor, plugin and actor demos stay fixtures, not maps, and
`elevator`'s Fuel model was not in the cache to review. The installed worlds
are flat demo scenes. No robot mission has been run in them.
