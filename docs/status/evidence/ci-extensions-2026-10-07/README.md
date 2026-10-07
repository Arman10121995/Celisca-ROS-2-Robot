# Extension CI tier repair, October 7

The exact published extension revision `dc39922fe57071b4184a843c46606677805cc71a`
built successfully in [Actions run 37463485222](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/37463485222),
then failed three tests in the unsourced fast tier. Two Isaac description
checks import `rclpy`; the URDF-root/Nav2 launch check imports `launch`.
Neither ROS module is available to plain Python in that CI step. The full
negative log and exact job report are retained on the SSD and hashed in
`manifest.json`.

Those three unchanged checks now have the integration marker and explicit
node IDs in the sourced integration manifest. Pure spring parsing and
navigation acceptance checks remain in the fast tier. No assertion, required
job, or failure condition was removed. The maps package also uses
`python3-trimesh-pip`, the actual key in the
[official rosdep Python definitions](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml).
The old `python3-trimesh` key produced an unresolved dependency diagnostic.
Local resolution now returns the `trimesh` pip package; the existing CI
Trimesh compatibility pin remains.

Measured local checks on the Jetson:

- Fast tier with ROS/package paths removed and plain `/usr/bin`/`/usr/local/bin`
  Python: **881 passed, 2 skipped, 4 deselected**, 158.58 seconds.
- Full sourced integration tier in isolated domain 231: **134 passed**,
  39.82 seconds, including the three moved checks.
- `robot_lab_maps` rebuild and canonical rosdep resolution pass.

The manifest records exact source hashes, commands and complete SSD output.
These are development checks, separate from the actual TurtleBot4/Panda
mission evidence.

The exact published correction `18ecdd24172fd6adfa0d5e2c55809dd2d882d4ae`
passes [Actions run 37586958626](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/37586958626):
full build, **879 fast checks / four skips / four deselections**, **18 physics
checks**, **134 integration checks**, and registry cross-reference validation.
The report and complete log are retained and hashed. This closes this tier
repair at that source revision; later robot/controller changes need their
own CI observation.

## Published control-column and extension revision

Exact `091d38884ab22b5e42a789fd19030046461e0a2a` passes
[Actions run 37617835178](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/37617835178):
25 core packages build (optional `orbslam3` excluded), **933 fast checks / four
skips / four deselections**, **20 physics checks**, **134 integration checks /
twelve skips**, and registry validation. The [job report](091d388-report.json)
and hashed complete SSD log retain the actual result. Local 94-test real-GUI
checks and source-matched physical Drive/Panda regressions remain separate
from CI. Documentation follow-up keeps the published runtime files unchanged;
a later runtime edit needs new relevant evidence and exact CI.
