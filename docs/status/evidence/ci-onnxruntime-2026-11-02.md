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
