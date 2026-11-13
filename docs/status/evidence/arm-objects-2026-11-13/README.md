# R5.7 measured object-stage implementation checks

These logs cover software/source contracts, installed package builds and a
widget-only smoke. They do not certify a physical pick/place mission.

Reproduce the 14 focused checks from the workspace on an idle simulator host:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/robot_lab_utils:$PWD/src/robot_lab_mujoco/python:$PWD/src/robot_lab_gui:${PYTHONPATH:-}"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=114 python3 -m pytest -q \
  src/robot_lab_bringup/test/test_grasp_candidates.py \
  src/robot_lab_bringup/test/test_native_object_catalog.py \
  src/robot_lab_gui/test/test_grasp_pose_request.py \
  src/robot_lab_mujoco/test/test_native_asset_display_grasp_fixture.py
colcon build --packages-select robot_lab_utils robot_lab_bringup robot_lab_mujoco robot_lab_gui
```

The original source-fixture checks explicitly skip when their optional source
assets are absent. Local logs show that they ran. Preserve source semantics;
do not replace scene clients in a real acceptance producer with test doubles.
[Scope and limitations](../../arm-objects-2026-11-13.md).
