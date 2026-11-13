# R5.7 measured support-placement implementation checks

The logs are software/native geometry contracts, real ROS request/ACK message
checks and actual Tk widget checks with planner test doubles. They do not
certify a physical pick/place mission. [Scope](../../support-placement-2026-11-14.md).

Reproduce on an idle simulator host:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/robot_lab_utils:$PWD/src/robot_lab_mujoco/python:$PWD/src/robot_lab_gui:${PYTHONPATH:-}"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=114 xvfb-run -a python3 -m pytest -q \
  src/robot_lab_bringup/test/test_support_placement.py \
  src/robot_lab_bringup/test/test_native_object_catalog.py \
  src/robot_lab_bringup/test/test_grasp_candidates.py \
  src/robot_lab_gui/test/test_support_placement_controls.py \
  src/robot_lab_gui/test/test_grasp_pose_request.py \
  src/robot_lab_mujoco/test/test_native_asset_display_grasp_fixture.py
colcon build --packages-select robot_lab_utils robot_lab_mujoco robot_lab_bringup robot_lab_gui
```

The installed-widget repeat runs `test_support_placement_controls.py` after
sourcing install, without the explicit source PYTHONPATH entries. Its two
checks pass. Source hashes identify the exact implementation. Optional
original-source fixture checks ran locally; other hosts explicitly skip when
their assets are absent.
