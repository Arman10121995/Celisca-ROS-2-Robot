# Final continuation checks — October 6, 2026

Baseline `61335fe`, supplemented by the containing commit's source stage.
Robot missions have their own **pre-trial** source/installed/native/SDK hashes;
these final software checks do not substitute for missions or a clean-host run.
All builds and large runtime output remain on the workspace SSD.

| Check | Actual outcome |
|---|---|
| Changed packages | Built utils/GUI/bringup; preceding six-package TurtleBot4 source build and installed-module hashes retained |
| `scripts/test_tiers.sh fast` | 885 passed, one explicit skip, one integration deselection |
| `scripts/test_tiers.sh integration` | 131 passed |
| `scripts/test_tiers.sh physics` | 18 passed |
| GUI command/process/backend/composition suite under Xvfb | 81 passed, including actual ROS/Tk executor separation |
| Module compilation and registry cross-references | Passed |
| Normal TurtleBot4 selection/default/autofill | All 40 variant/backend/mode combinations passed |
| Normal Panda Plan/Execute | Two physical targets, rejection, invalidation, interruption/reset/monitor and all four clean child exits passed |

Exact logs, commands, source hashes and prior build/normal-GUI reports are
fingerprinted in [verification-manifest.json](verification-manifest.json).
Large files remain at `/workspace/molar/robot_lab_runtime/extensions-2026-10-06`.
[Measured support screens](support-matrix-current.md) derive from exact hashed
reports; all 54 indexed measurement sets pass their bounded numeric checks,
while full-release gates remain blocked by unfinished roadmap acceptance.

The robot reports and failures are described in
[TurtleBot4 modes](../turtlebot4-modes-2026-10-06/README.md),
[final sensors/Drive](../turtlebot4-sensors-2026-10-06/README.md) and
[native Panda Cartesian control](../panda-cartesian-2026-10-06/README.md).
Current upstream CI for this source stage is checked after publishing; older
`531c542` CI applies to the Drive stage only. Runtime asset counts and passing
unit tests do not qualify every robot/map/backend or hardware mission.
