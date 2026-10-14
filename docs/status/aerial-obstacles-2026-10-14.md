# Observed aerial obstacle planning implementation — October 14, 2026

Continued published `master` at `0049ee2`; independent child task
`R5.10.mapped_obstacles` preserves Claude's parent ownership and existing
PX4 flight/map work. This batch implements mapped-obstacle planning and live
depth interruption. **No PX4/FCU or simulator was launched by its diagnostics.**

## Implemented

- An opt-in normal-GUI **Drone route: mapped + live depth obstacles** option,
  dependency selection/autofill, saved-profile restoration and observed-layer
  readiness/count/age displays. Display/other robots reject this route option.
- Real RTAB-Map PointCloud2 and registered depth/CameraInfo consumers with
  capture-time transforms into the FCU map, timestamp/frame/layout validation,
  latest-message worker, bounded points/cells and explicit reject/hold behavior.
- Canonical static geometry plus conservative observed voxels in the existing
  asynchronous 3D A* planner. Current observed occupancy is rechecked across
  the route before execution, outside the Offboard setpoint loop.
- A 2 m current-occupancy lookahead interrupts planned routes on blockage or
  stale sensing. Operator-command generations invalidate background results
  after manual/Stop/goal changes. No late automatic execution or recovery resume.
- Declared SciPy and `sensor_msgs_py` runtime dependencies. Existing native
  models, SSD lease/log/`-d` handling, manual/altitude and Takeoff paths remain.

Read the [operator and next-agent guide](../tutorials/mapped-drone-planning.md).
This is a static-plus-observed occupancy candidate, not certified dynamic
flight or future obstacle prediction.

## Focused checks and evidence boundaries

Four affected packages build; the final build took 9.42 s. **23 checks pass in
the actual SSD PX4 Python environment**: voxel inflation/thin obstacles, pixel
projection/clip bounds, a real A* detour around a fixture, runtime route guard,
background Plan/Execute ownership and existing camera/frame checks.

The first host-Python run retained a missing-`pymavlink` failure. That optional
SDK case explicitly requires the existing PX4 venv and passes there. No SDK
dependency was copied onto internal storage to mask the environment boundary.

One real ROS transport/TF diagnostic uses **synthetic** CameraInfo, depth and
PointCloud2: capture transform/projection, a new point obstruction, and actual
message-loss age rejection complete. It launches no vehicle; it cannot prove
camera calibration, flight hold distance or collision clearance. An actual
installed Tk selection confirms RGB-D/RTAB-Map/observed-route command autofill
and exclusion in Display/Bumperbot; it requests no motion.

Source/install hashes and raw diagnostic logs are bound by
[the receipt](aerial-obstacles-receipt-2026-10-14.json). Large logs remain under
`/workspace/molar/robot_lab_runtime/aerial-obstacles-2026-10-14/`.
Prior named flight certificates remain historical; none was refreshed or
broadened by these software/data-fixture checks.

## Remaining

Actual normal GUI mapped flights, physical new-obstacle and sensor-loss
interruption, mapping/TF alignment, setpoint cadence, tracking and contact
clearance; saved maps, repeated starts, remaining worlds and backends.
Automatic dynamic detours and future actor prediction are not implemented.
Manual/Takeoff controls do not gain observation guarding. Unknown/unseen
regions still cannot be certified free from a partial sensor cloud. The full
project remains partial, with coupled/dexterous/object/backend controllers,
terrain/provider work, simulator-applied seeds and release reproduction still
open in the roadmap.
