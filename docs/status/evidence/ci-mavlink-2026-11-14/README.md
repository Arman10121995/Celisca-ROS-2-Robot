# CI MAVLink dependency repair — November 14, 2026

Published CI [run 43533140298](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/43533140298)
at `a2b5e281` passed fast/physics and reached integration. Collection then
failed in `test_px4_heading_gate.py` because `px4_ros_controller.py` imports
`pymavlink`, which the runner had not installed. The earlier world-copy tier
repair reached its intended integration tier; no test is removed or hidden.

Both normal and scheduled workflows now install `pymavlink==2.4.50`, matching
the existing [documented flight runtime](../../../tutorials/px4_x500.md).
The [published package](https://pypi.org/project/pymavlink/2.4.50/) is pinned,
not an unversioned dependency upgrade. Actual local heading/walkway checks
pass **10 tests** with that exact runtime. No FCU, robot or simulator is
started by these fixtures; this is a dependency/control contract check.

Reproduce after sourcing ROS/install:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=114 \
  "$ROBOT_LAB_RUNTIME_ROOT/px4/venv/bin/python3" -m pytest -q \
  src/robot_lab_adapter/test/test_px4_heading_gate.py \
  src/robot_lab_adapter/test/test_px4_walkway_scan.py
```

Full exact-revision published CI remains pending. Raw failed-run output and
local logs remain on SSD at
`/workspace/molar/robot_lab_runtime/continuation-20261114/`.
