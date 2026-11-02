# CI inference dependency correction — November 2, 2026

Published fleet revision `61ddab2cd1162528a03ba2b45fda294a4823d159` built successfully in
[ROS 2 CI](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/41454462710),
job `125307491277`. The fast tier reported 1 failed, 1119 passed, 42 skipped,
and 4 deselected in 59.29 seconds. Physics/integration did not run after that failure.

The failing G1 recurrent-policy regression loads the actual exported checkpoint;
its environment lacked `onnxruntime`. This is a missing workflow dependency,
not a robot walking qualification. Both required and scheduled workflows now
install `onnxruntime==1.23.2`, matching the locally exercised runtime. The test
is retained, with its module description corrected to describe actual inference.

The exact failing test passes locally (1 passed, 0.42 seconds). No skip or
acceptance threshold was added. The remote result after publishing this fix
must be inspected separately; no successful CI rerun is claimed here.

Raw excerpt: `/workspace/molar/robot_lab_runtime/completion-20261102-QUo52j/ci-61dd-failure.log`.
Local check: `test-ci-inference.log` in the same SSD directory. This correction
does not broaden any policy/model/map/backend motion certificate.

The first correction (`3a51d427`) then exposed an ABI conflict in both required
and scheduled CI: ORT's NumPy requirement upgraded Humble's apt 1.21.5 to
NumPy 2, while the apt SciPy/cv_bridge extensions use the NumPy 1 ABI. Both
workflows now also pin NumPy 1.24.4. An isolated SSD environment exercised
NumPy 1.24.4 + apt SciPy 1.8.0 + ORT 1.23.2 and cv_bridge imports, an actual
compiled distance-transform call, the exact recurrent checkpoint and the
failing collection modules. Host Python/ROS packages were not changed.
Published `4468119e` then passed build and the entire fast tier: **1132 passed,
40 skipped, 4 deselected in 78.97 seconds**, in
[run 41470361223](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/41470361223),
job `125362000772`. Physics exposed a separate optional-cache portability bug:
four MuJoCo collision checks tried to stat the Jetson-only SSD runtime directory.
The helper now retains exact original geometry when that optional mount is absent;
it creates no fallback cache on internal storage. A missing-cache regression is
retained.

Published **`d064eecef72d9a66bb8b033b2a288be3c03ea5f1` passes the complete required
workflow**, [run 41479322708](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/41479322708),
job `125393131123`: build success; fast **1132 passed, 40 skipped, 4 deselected**
(78.16 s); physics **65 passed** (1.60 s); integration **177 passed, 16 skipped**
(26.57 s); registry validation success. The original inference/geometry tests and
all workflow gates remain. Local sourced NumPy-1.24.4 environment also passed
the six affected MuJoCo collision/ray/cache checks in 2.90 seconds.

This green result belongs to `d064eece`. Later fleet implementation commits
need their own remote workflow result; no robot mission is inferred from CI.
