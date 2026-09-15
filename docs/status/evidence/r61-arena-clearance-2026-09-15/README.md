# R6.1 navigation-arena clearance — 2026-09-15

Base checkout: `2cb2bfe`; implementation changes are in the working tree.
`manifest.json` records the checked source hashes. This is static fixture
qualification, not a live robot mission or a Nav2 planner comparison.

## Defect and repair

The previous validator checked obstacle centers in the occupancy image. It did
not check route segments or robot footprint. The original reference paths in
nav_obstacle, nav_maze, nav_narrow_passage and nav_warehouse intersected geometry,
even though their individual waypoints passed the existing occupancy tests.

`before.json` runs the new validator with navigation metadata from `2cb2bfe`;
all four nonempty arenas fail. `after.json` checks the repaired metadata; all five
navigation arenas pass. Static world and map assets were preserved. The reference
routes in central metadata and the environment registry were updated together.

The validator reads composed SDF collision poses, actual map origin/resolution,
occupancy thresholds and image dimensions. It checks obstacle interior pixels,
clear-space pixels, spawn/goals, route endpoints and full swept circular footprints.
Map clearance conservatively includes the entire occupied/unknown cell and map
boundary. Invalid/truncated PGM input and invalid footprint radii are rejected.

Routes were constructed with `arena_clearance.reference_route`, using each
generator definition's spawn, goal and bounds, a 0.14 m radius and 0.20 m extra
geometry clearance. This deterministic visibility-graph utility generates test
fixtures; it is not a runtime planner implementation. Other robots must use a
conservative radius for their own footprint, and steering robots additionally
need curvature/orientation feasibility checks.

| Arena | Route length (m) | Minimum geometry clearance beyond radius (m) | Conservative map clearance (m) |
|---|---:|---:|---:|
| nav_empty | 0.707 | 5.360 | 5.250 |
| nav_obstacle | 21.292 | 0.207 | 0.087 |
| nav_maze | 29.482 | 0.200 | 0.090 |
| nav_narrow_passage | 21.759 | 0.207 | 0.111 |
| nav_warehouse | 28.353 | 0.201 | 0.091 |

## Verification

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/robot_lab_maps/test \
  src/robot_lab/robot_lab_registry/test/test_p4_2_nav_arenas.py \
  src/robot_lab/robot_lab_registry/test/test_p4_6_navigation_metadata.py \
  src/robot_lab/robot_lab_registry/test/test_p6_benchmarking.py
```

Result: **114 passed in 15.30 s**, including 27 new clearance tests. Regression
cases cover free endpoints with a colliding segment, footprint versus centerline,
rotated/degenerate geometry, a hole away from an obstacle center, wrong map
origin/resolution, truncated image data, oversized robots and missing routes.

Both changed packages built with `colcon build --symlink-install
--packages-select robot_lab_maps robot_lab_registry`. The initial CMake configure
misreported pytest as missing because host pytest plugin autoload failed. A
configure with autoload disabled registered the test correctly:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --symlink-install \
  --packages-select robot_lab_maps --cmake-force-configure \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select robot_lab_maps \
  --ctest-args -R arena_clearance --event-handlers console_direct+
python3 install/robot_lab_maps/share/robot_lab_maps/tools/validate_nav_arenas.py \
  --json /tmp/r61-clearance-installed.json
```

Result: CTest `arena_clearance` **27 passed**, and the installed validator passed
all five arenas. These 27 cases repeat a subset of the 114, not additional unique
tests. Build output included existing underlay/setuptools warnings.

R6.1 remains partial: broader mesh/terrain/aerial worlds, class-specific
footprints, runtime reset and sensor/estimator-history checks remain. R8.1 still
needs obstacle-world live missions using declared spawns and real distinct-planner
comparisons. No live navigation success is inferred from these repaired fixtures.
