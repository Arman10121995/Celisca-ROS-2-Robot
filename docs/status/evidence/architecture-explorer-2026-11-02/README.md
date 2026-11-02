# Native architecture explorer — November 2, 2026

The existing **Health** workspace now appears as **Learn & health** in the
sidebar. It contains the native architecture atlas, inline tutorial/source
library and preserved diagnostics/roadmap. [User guide](../../../architecture/interactive-guide.md).
Baseline was `170b0a84` on `master`; this delivery changes documentation and
read-only inspection, not robot controllers or mission qualification.

## Delivered

- Twelve interactive views and 36 source-linked component inspectors covering
  composition, all backend responsibilities, estimation, control, maps, frames,
  clocks, robot classes, fleets/RMF/lifts, aircraft, marine, manipulation,
  experiments, networking and evidence.
- Six guided tours, Next/Previous/Play, search, connected-card drill-down,
  zoom/pan, keyboard selection, layer spotlight and standalone SVG export.
- Inline Markdown reading with headings, search, local links/history, readable
  table records, glossary, linked native flowcharts, bounded source-directory
  browsing and actual maintained package/dependency manifests.
- An illustrative domain/name/type/reliability/durability playground. It changes
  no environment or actual endpoint. Existing diagnostics remain in their subpage.
- Opt-in asynchronous ROS discovery with actual advertised nodes, topics,
  service declarations and bounded offered/requested endpoint QoS. UI status
  reports timestamp/domain/middleware and limits; JSON export preserves data.

## Checks and actual observations

| Check | Actual result / retained artifact |
|---|---|
| Source topology / links / SVG / networking / inventory | [21 passed](model-tests.log), 0.11 s |
| Real Tk tours/reader/cancellation/narrow lesson | [3 passed](gui-tests.log), 29.34 s, isolated Xvfb |
| Changed package | `colcon build --packages-select robot_lab_gui` succeeded |
| Installed-source parity | [Final readback hashes](source-manifest.json) match seven GUI modules |
| Installed whole GUI | [All 12 views / 36 components](installed-gui.log) rendered; 1024 px canvas 459×329, inspector 284 px; controls stayed disconnected |
| Final narrow networking layout | [Installed 1024×768 check](narrow-layout.log): fits horizontally, scrolls vertically, no discovery/control activation |
| Actual isolated ROS introspection | [Probe](ros-probe.log), domain 229: source/sink endpoints and service discovered, no probe service calls or additional probe subscribers; inspection node disappeared and caller context stayed alive |
| Actual GUI discovery worker | [Domain-230 snapshot](actual-ui-snapshot.json) and installed GUI check: worker completed, QoS rendered, independent probe context remained alive, no command/control node created |
| Cancellation during actual discovery | [Cancellation probe](cancel-probe.log): active inspector node observed, worker cancelled, inspector disappeared and caller context stayed alive |
| Full fast regression tier | [1157 passed, 37 skipped, 4 deselected](fast-tier.log), 236.51 s; 58 test files in the sourced NumPy-1.24.4 SSD environment |

The probe nodes use only `/architecture_probe/*` test interfaces. No simulator
or robot mission was launched. [The separate discovery snapshot](actual-ros-snapshot.json)
records actual declarations and QoS, not simulated traffic or PASS metrics.
Discovery itself briefly creates a private rclpy node/context; its normal
parameter-event publisher is distinct from application command publishers.
The worker never calls robot services or subscribes to sensor/control topics.

The installed GUI/worker probe preceded the final source-directory reader and
narrow-wrap adjustments. Those final adjustments have their own source Tk
checks, final package build and parity manifest. Do not infer an immutable
whole-build mission certificate from these UI observations.

## Visuals

[Native diagram/inspector](native-explorer.png),
[small-window networking lesson](native-network.png),
[actual ROS graph/QoS inspector](native-live-graph.png),
[standalone vector overview](architecture-overview.svg).

An early small-window networking screenshot exposed horizontal clipping. The
viewport-based wrap correction and final installed check are retained; the
earlier image remains in the SSD run directory. Font metrics are cached to keep
selection/tour redraws from repeatedly allocating X11 fonts. No timing speedup
or cross-host performance claim is made. The supplemental cancellation probe
initially used a global executor for its private caller context; the corrected
harness uses a private executor. The initial harness log remains on SSD; the
production discovery helper already used its own context/executor.

## Reproduction and limits

All raw builds, probes, screenshots and logs live at
`/workspace/molar/robot_lab_runtime/architecture-explorer-20261102/` on SSD.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
colcon build --packages-select robot_lab_gui
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/robot_lab_gui/test/test_architecture_model.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 xvfb-run -a python3 -m pytest -q \
  src/robot_lab_gui/test/test_architecture_explorer.py
ros2 run robot_lab_gui robot_lab_gui
```

The host checks used system Python with the SSD-extracted jsonschema dependency
path when user packages were excluded. The normal sourced GUI also loads the
workspace dependencies. Pure architecture checks are in the fast tier; real Tk
checks are in the integration tier and explicitly skip without a display.

Teaching diagrams are authored architecture, not live telemetry. Discovery
snapshots are bounded advertised metadata, not message delivery, compatibility,
rate/TF/lifecycle readiness or successful robot missions. The playground models
selected policies, not the full DDS/network configuration. Robot/backend/map
support and all remaining R10/original roadmap acceptance retain their earlier
scope. No source guard or physical certificate was refreshed.
