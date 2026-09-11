# R3.4 command autofill follow-up

Date: 2026-09-11. Base revision: `726707a`, with the accompanying working-tree
changes. Scope: GUI command generation, presentation, clipboard and execution
dispatch; this is not simulator mission qualification.

The Launch tab shows a wrapped command above its output, with Copy Command,
Run Command and Stop in the same visible area. Every launch selector refreshes
the command, including the previously missing GUI/headless radio handler.
The selected Room vacuum route uses its wrapper launch file while retaining
the resolved arguments. Map profile names such as `simple_office` resolve to
their registry IDs (`small_office`) for validation and algorithm filtering.

Preview and clipboard text use shell quoting; Run passes the same arguments
directly to `Popen`. Invalid selections and resolver errors clear the previous
command and disable Run/Copy. Repeated Ctrl+Enter cannot start duplicate
launches. Process completion refreshes validation before enabling Run.

## Verification

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 xvfb-run -a python3 -m pytest -q src/robot_lab_gui/test
```

Result: **37 passed in 12.21 seconds**, including 10 new real-Tk cases covering
startup, radio/combobox changes, map aliases, planner arguments, vacuum route,
shell quoting, clipboard, exact execution arguments, duplicate-run prevention,
invalid selections, resolver errors and legacy fallback. Launch process
creation is mocked. Unsupported Labbot/MuJoCo selection is correctly rejected
by the existing registry validator; no backend support claims were changed.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_lab_gui --event-handlers console_direct+
```

Result: **1 package finished in 3.14 seconds**. Installed `launcher.py` matches
the source file. A separate Xvfb smoke check opened the installed GUI with all
tabs, changed the headless option, copied its command, asserted that Run passed
the copied arguments to mocked `Popen`, and destroyed the window successfully.
The full window was also visually inspected at 1280×860: the command and
Run/Copy/Stop controls are visible without scrolling the options panel.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_room_vacuum.launch.py --show-args
```

Result: success; nested simulation arguments include `simulator`, `gui`,
`global_planner_plugin` and `local_planner_plugin`.

Initial pytest invocations failed before collection because an installed
AnyIO plugin expected a newer pytest API; disabling plugin autoload follows
the repository test scripts. Initial GUI checks exposed the map alias defect,
fixed here. One full-window probe incorrectly canceled child-owned Tk jobs
through the root before destruction; normal window destruction passed.
No live simulator mission or hardware motion was run.
