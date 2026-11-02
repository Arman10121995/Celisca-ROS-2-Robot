# GUI usability and presentation — November 2, 2026

Publication follow-up: commit `8b958e97` passed [GitHub CI run 41520955830](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/41520955830). The timed-out local fast invocation below remains an incomplete local run.

Baseline `aed999ee`, existing `master` repository. This delivery refreshes the
shared native interface and navigation; it does not qualify robot missions or
refresh historical source certificates. [User quick tour](../../../tutorials/gui-quickstart.md).

## Delivered interface

- Home dashboard with neutral workflow cards, loaded profile/adapter counts,
  current selection and main launch-process status.
- Purpose-based, scrollable navigation across 13 workspaces, automatic/manual
  sidebar collapse, contextual headings/help and a persistent Stop motion.
- Ctrl+K workspace/control search with ranked results and keyboard navigation.
  Search contains movement/launch key presses; key releases remain available.
- Setup → review/launch → control presentation, expandable model/support notes,
  explicit experimental hints, unchanged command preview/copy/launch callbacks.
- A single selector for all nine existing control pages, including Marine
  missions. Native notebooks, page identities and legacy aliases are retained.
- Shared dark surfaces, clearer typography/contrast, readable disabled choices,
  consistent selected tabs, and distinct Stop motion / End session labels.
- Preserved registry/3D, worlds, fleet, aerial, experiments, training, telemetry,
  architecture/tutorial and diagnostic workflows under the shared shell.

## Actual checks

| Check | Result |
|---|---|
| Existing workspace and learning UI | [11 passed](layout-and-learning-tests.log), 67.72 s |
| Latest search/control/Home checks | [3 passed](experience-tests.log), 16.13 s |
| Focused Drive/architecture regressions | [29 passed](focused-fast.log), 0.17 s |
| Package installation | [Changed GUI package built](build.log) |
| Installed whole GUI | [13 workspaces, 9 control pages](installed-gui.log); Home/launch at 1600×1050 and 1024×768; neutral GUI/fleet/aerial controls |
| Final source/install parity | [Four module hashes](source-manifest.json) match |
| Full local fast suite | [Retained log](fast-tier-incomplete.log): reached the 300-second limit around 42%; no completed suite result claimed |

The source interaction checks assert that search does not start/end a launch,
schedule Drive or create a command publisher, even with keyboard Drive enabled.
They check direct Drive ranking, Ctrl+Enter containment, Escape dismissal,
secondary control-page routing, widget/command preservation, model-note toggling
and Home resize behavior. Existing tests exercise Stop callbacks, selection,
command autofill, scrollable controls and architecture/reader interaction.

An initial search test exposed the general Launch workspace ranking above the
specific Drive page. Title matches now rank first; the negative log is retained
on SSD. No test threshold or capability rule was weakened.

The incomplete fast run used the existing isolated NumPy-1.24.4 SSD environment.
It does not replace focused UI results or the published revision's own CI.

## Screenshots

- [Home, wide](home-wide.png) and [Home, small window](home-narrow.png).
- [Launch, wide](launch-wide.png) and [Launch, small window](launch-narrow.png).
- [Workspace search](search-wide.png), [Registry](registry-wide.png),
  [learning workspace](learn-wide.png).

Screenshots capture the installed app with no simulation launched. Inventory
counts are loaded profile metadata, not qualified families or mission scores.
ROS tool availability is distinct from a live connection; a launch-process
badge is distinct from robot/controller readiness.

## Reproduce

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
colcon build --packages-select robot_lab_gui
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 xvfb-run -a python3 -m pytest -q \
  src/robot_lab_gui/test/test_experience.py \
  src/robot_lab_gui/test/test_workspace_layout.py \
  src/robot_lab_gui/test/test_architecture_explorer.py
ros2 run robot_lab_gui robot_lab_gui
```

The host's isolated checks used system Python and the existing SSD-extracted
jsonschema dependency path. Raw logs, probes and screenshots remain under
`/workspace/molar/robot_lab_runtime/gui-usability-20261102/`. Tests run under
isolated Xvfb and do not launch a robot or simulator. The old simulator/mission
matrix and controller limits retain their existing acceptance scope. No
source-bound robot certificate was renewed from these UI checks.
