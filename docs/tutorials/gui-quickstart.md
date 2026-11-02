# Robot Lab: a quick tour of the workspace

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
ros2 run robot_lab_gui robot_lab_gui
```

Reopen the GUI after rebuilding it. The full control center starts on **Home**.
Nothing is launched, connected or armed by visiting Home or another workspace.
Inventory counts describe loaded profiles/adapters, not qualified robot missions.

## Find your way

The sidebar groups work by purpose:

| Group | Workspaces |
|---|---|
| Start here | Home and its workflow cards |
| Operate | Run a robot, Ground fleet, Aerial swarm, Live telemetry |
| Explore | Robot & world registry, Worlds & mapping, Installed assets |
| Experiment | Cleaning missions, Experiments, Policy training, Test suites |
| Understand | Learn & diagnose: architecture, tutorials, source and status |

Use **Find / Ctrl+K** to search workspace names, controls and keywords such as
`Drive`, `Hand`, `mapping`, `RMF` or `network`. Arrow keys choose a result;
Enter opens it. Escape closes search. Search navigates only: it cannot start
or stop a launch, arm input, or send robot commands. Movement/launch key presses
are contained by the search window; releases still reach the main application.

The sidebar collapses automatically below 1250 pixels. Its short labels have
hover descriptions. Use the header's menu button to expand/collapse it manually;
a manual choice is kept for that GUI session. The header keeps the current
selection and **Stop motion** visible. **Help** opens the guide for the current
workspace. **Logs** reveals the shared console.

## Configure, review, then control

Choose **Run a robot** (the underlying Launch workspace).

1. **Set up your experiment.** Filter the robot category/type, choose its complete
   family and source variant, task, world and simulator. Model details/support
   notes are available through the disclosure button. Unsupported modes retain
   their existing explanations and gates. Options contains advanced settings,
   source-specific policies, viewer configuration and saved profiles.
2. **Review & launch.** The configuration hint says whether a command is prepared
   or needs attention; an experimental selection stays explicitly labeled.
   Review **Algorithms & checks**, the generated command and Details.
   **Launch session** executes the same prepared command shown in the preview;
   **Copy command** is for a sourced terminal. A prepared command is not runtime
   readiness, successful localization or a mission result.
3. **Control & observe.** Choose **Drive & limits**, **Arm**, **Hand**, **Drone &
   limits**, **Marine**, **Marine missions**, **Gait**, **Tracks** or **MOLAR** from
   the control selector as available. It selects the existing native page; it
   does not change the robot, task, experimental flags, connection or ownership.
   Each controller retains its own Connect/Enable/Plan/Execute requirements.
   **Telemetry** opens the separate measurement workspace.

On smaller windows, **Set up / Command & algorithms** alternate the left pane;
controls remain on the right. The same widgets and values survive resizing.
Scroll individual panels to reach settings. Save Map and Reset Robot keep their
existing mode/readiness restrictions.

## Stop motion versus end a session

- **Stop motion** invokes the existing controller Stop/cancel operations. It is
  available in the header across workspaces. It does not intentionally close
  the simulator process.
- **End session** uses the existing owned launch shutdown path. It closes the
  launched session and its owned processes.
- **Space** retains the Drive Stop behavior where manual input is enabled and
  focus permits Drive keys. Typing in text fields is separate from driving.
- **Escape** retains launch shutdown outside the search window; inside search
  it dismisses search. **Ctrl+Enter** launches from the normal workspace;
  **Ctrl+R** refreshes catalogs. Search does not forward these key presses.

The status bar distinguishes ROS tool availability from a live robot connection.
The Home badge reports the main launch-process state, not successful robot
startup or active controllers. Inspect actual feedback and readiness in the
relevant controller or telemetry page.

## Explore and learn

Registry retains grouped families, classification filters and native 3D
inspection. Worlds keeps its map-generation tools. Fleet/swarm, aircraft,
experiments, training, diagnostics and learning retain their own controllers
and live-state handling under the shared navigation, palette and typography.

**Learn & diagnose** is the existing Health workspace: use interactive
architecture diagrams/tours, the source/tutorial library, networking lessons,
opt-in ROS discovery, or Diagnostics & roadmap. No discovery starts just by
opening a page. See [the architecture guide](../architecture/interactive-guide.md).

This interface refresh does not qualify more robots, maps, backends or algorithms.
Historical source-bound mission certificates are not refreshed by cosmetic or
navigation checks. [Current status](../status/CURRENT_STATUS.md) and
[the roadmap](../../ROADMAP.md) retain remaining work.
