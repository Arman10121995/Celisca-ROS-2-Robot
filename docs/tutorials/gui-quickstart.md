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

Each sidebar entry has an icon, and the first nine have shortcuts:
**Ctrl+1** (Home) to **Ctrl+9**, in sidebar order. The sidebar collapses to
its icons below 1250 pixels. Hovering an entry shows its description and
shortcut. The header's menu button expands or collapses it manually, and a
manual choice is kept for that GUI session.

The header shows:
- **the session indicator:** "No session running", or "Running ·
  robot · world · simulator · mm:ss" in green while a launch session is
  alive. A blocked launch shows its reason in yellow. Clicking the
  indicator opens Run a robot, and that sidebar entry carries a ● while
  the session runs.
- **the current selection**, robot / world / simulator / task.
- **Help (F1)**, which opens the guide for the current workspace.
- **Logs**, which reveals the shared console.
- **Stop motion.**

## Read a robot's status in the lists

Every row of **Robot family** and **Model / source variant** carries a mark
for the simulator you chose, and the line under the selectors explains it:

| Mark | Meaning |
|---|---|
| ● green | **measured**: at least one recorded screen or mission passed on this simulator |
| ◐ amber | **experimental**: it runs and can be controlled here, but nothing on this simulator is measured yet |
| ○ grey | **display only**: it loads and shows itself; no controller |
| ✕ red | not available on this simulator (pick another simulator to use it) |

The letters after the name are its modes on this simulator: **D** display,
**L** localization, **S** SLAM, **3** 3D SLAM, **N** navigation, **F** flight,
**M** marine, **G** gait. A **capital** letter is measured, a small one runs but
is not measured here, and **(n)** in brackets needs the *Experimental
imported-asset modes* switch. A family shows its best variant. Example:
`● asset_elephant_myagv_pro_rollers   D (l) (s) (3) N` on PyBullet — driving
and navigation measured, the other modes behind the switch.

The marks come from the recorded evidence only: the generated support
matrix, the installed robots' runtime screens and each profile's
`measured_screens` (`robot_lab_utils/robot_status.py`). A measured mark is a
named screen on one setup, not a guarantee for every world.

## Configure, review, then control

Choose **Run a robot** (the underlying Launch workspace).

1. **Set up your experiment.** Narrow the list with **Category** and **Type**
   (side by side), then choose the **Robot family** and its source variant,
   the task, world and simulator. Model details/support
   notes are available through the disclosure button. Unsupported modes retain
   their existing explanations and gates. Options contains advanced settings,
   source-specific policies, viewer configuration and saved profiles.
2. **Review & launch.** A card states the setup in words, for example
   "bumperbot in celisca_floor_1 · Inspect task · Gazebo simulator". Its hint
   says whether a command is prepared or needs attention, and an
   experimental selection stays explicitly labeled.
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

**Docs & tutorials** (inside Learn & diagnose, or **Start the beginner course**
on Home) is the documentation library:
- the [beginner course](../learn/README.md) first, then the guides grouped by
  purpose ([`docs/library.yaml`](../library.yaml));
- figures and animations inline (click to enlarge), and the diagrams as
  images;
- **Find guide / package** filters titles, **On this page** jumps to a
  heading, **Find text** searches the page, and **Back** returns.

This interface refresh does not qualify more robots, maps, backends or algorithms.
Historical source-bound mission certificates are not refreshed by cosmetic or
navigation checks. [Current status](../status/CURRENT_STATUS.md) and
[the roadmap](../../ROADMAP.md) retain remaining work.

## Cleaning missions and test suites

- **Cleaning missions:** a robot-vacuum app. On the left is a live map
  with rooms, the cleaned area, the plan, the dock and the robot, plus
  tools for rooms, zones, no-go areas, virtual walls, spot, dock and room
  split. On the right are the Clean, Rooms, Schedule, History, Care and
  Setup pages. Setup launches the robot with the cleaning system
  ([guide](robot-vacuum.md)).
- **Test suites:** each suite has its own **Run** button, and the console
  opens when a suite starts. **Stop test run** is at the top right.
