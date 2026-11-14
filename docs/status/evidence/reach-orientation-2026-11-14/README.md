# R5.9 source pad-orientation implementation checks

These checks cover private native FK/IK, source constraints, task/schema and
GUI builder contracts. Actual Tk runs use protocol fixtures and issue no robot
commands. They are separate from physical loaded Stretch missions.
[Scope and limits](../../reach-orientation-2026-11-14.md).

Reproduce on an idle simulator host:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/robot_lab_utils:$PWD/src/robot_lab_mujoco/python:$PWD/src/robot_lab_gui:${PYTHONPATH:-}"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=114 xvfb-run -a python3 -m pytest -q \
  src/robot_lab_bringup/test/test_native_control_kinematics.py \
  src/robot_lab_bringup/test/test_mobile_task_and_takeoff.py \
  src/robot_lab_gui/test/test_mobile_reach_orientation.py \
  src/robot_lab_bringup/test/test_bounded_control_planner.py
colcon build --packages-select robot_lab_utils robot_lab_mujoco robot_lab_gui
```

Original source gains, limits and live qpos are checked unchanged. The rotating
wrist fixture explicitly declares radians. Initial setup failures are retained
on SSD in `checks.log`; final logs identify the corrected fixture. Native
geometry tests use MuJoCo; no new installed source-robot mission is certified.
