# R5.9 alternate IK and multi-goal search checks

These are source/control, native synthetic geometry, algorithm and actual Tk
contracts. They do not certify a physically completed installed Stretch
mission. [Scope and retained failures](../../ik-branches-2026-11-14.md).

Reproduce on an idle simulator host:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/robot_lab_utils:$PWD/src/robot_lab_mujoco/python:$PWD/src/robot_lab_gui:${PYTHONPATH:-}"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=114 xvfb-run -a python3 -m pytest -q \
  src/robot_lab_bringup/test/test_bounded_control_planner.py \
  src/robot_lab_bringup/test/test_native_control_kinematics.py \
  src/robot_lab_bringup/test/test_mobile_task_and_takeoff.py \
  src/robot_lab_gui/test/test_mobile_reach_orientation.py
colcon build --packages-select robot_lab_utils robot_lab_mujoco robot_lab_gui
```

The independent native audit samples 101 motor-slew times per returned
segment. Source gains, original live qpos, final target and held grip are
checked separately. The multi-goal closed-form cases use explicit synthetic
obstacles; none are simulator flight/navigation qualification. Raw failing
logs remain on SSD, not replaced by selected green traces.
