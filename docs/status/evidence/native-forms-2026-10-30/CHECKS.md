# Reproduce the scoped implementation checks

From the existing workspace on the SSD:

```bash
source scripts/ssd_env.sh
source install/setup.bash
colcon build --packages-select robot_lab_robots robot_lab_utils robot_lab_maps \
  robot_lab_registry robot_lab_bringup robot_lab_gui robot_lab_mujoco \
  robot_lab_benchmark --symlink-install
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/robot_lab_bringup/test/test_serpentine_contract.py \
  src/robot_lab_bringup/test/test_tracked_flippers.py \
  src/robot_lab_bringup/test/test_tracked_control.py \
  src/robot_lab_bringup/test/test_gait_contract.py \
  src/robot_lab_bringup/test/test_marine_contract.py \
  src/robot_lab_bringup/test/test_marine_sensors.py \
  src/robot_lab_bringup/test/test_robot_taxonomy.py \
  src/robot_lab/robot_lab_benchmark/test/test_extension_result.py \
  src/robot_lab/robot_lab_benchmark/test/test_extension_workload.py
ros2 run robot_lab_registry robot-lab validate \
  --config-dir install/robot_lab_registry/share/robot_lab_registry/config
```

Observed: eight-package build, 94 focused tests, registry validation. The final
four-package rebuild covers the last GUI/profile changes. Read both retained
test logs: the first expanded batch exposed old fixture fields, then passed
after the fixture supplied the real constructor defaults. No full-suite or
remote CI pass is claimed.

`gui_forms_probe.py` constructs actual installed Tk widgets under Xvfb and
checks the exact generated arguments, grouped variants, installed inventory
and current ledger/Health rendering. It sends no motion commands. Its fixed
SSD output directory is the original run; change it for a new diagnostic to
preserve this evidence.

`run_forms.py` uses the real isolated runner and native workload producers.
Its `snake_mapping`, `snake_3d_mapping`, `snake_navigation`,
`surface_mapping` and `surface_navigation` choices launch actual plants.
`LAB_SAVED_MAP` passes an acquired map into snake navigation. Preserve each
new run's actual producer identity and result; do not replay a stored PASS.
The native flipper driver/producer is also retained. It is an external
diagnostic, separate from built-in mission acceptance.

Use the new operator guides to run these through the GUI. Larger maps,
terrain, fluid backends, RMF infrastructure, amphibious transitions and
other named pending tasks require their own implementations and observations.
