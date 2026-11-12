# Robot-vacuum cleaning system (OOMWOO-class)

The Vacuum workspace (**Cleaning missions** in the sidebar) is a cleaning
app for any robot whose profile declares `supports_room_vacuum: true`: the
OOMWOO-class robot vacuum (`oomwoo`) and Bumperbot. It drives the
`robot_lab_vacuum_cleaning` node, which plans and drives the cleaning
itself. Status: **implemented and experimental**. See the bottom of this
page for what has been measured. How it works inside, with figures and
animations: [Inside the robot vacuum](../learn/07-robot-vacuum-inside.md).

## Run the cleaning system

1. **Setup** page: robot `oomwoo`, home map `small_house`, simulator
   Gazebo, map source **Saved map**. Press **Launch robot + cleaning**.
   This runs `simulated_room_vacuum.launch.py controller:=cleaning`
   (`nav` mode: saved map plus localisation, and the cleaning node).
2. The map appears with the rooms coloured and named. Rooms are found
   automatically.
3. **Clean** page: choose **Whole home** and press **Start**. The robot
   cleans each room's edges first, then the floor in overlapping lanes,
   and returns to the dock. The first job puts the dock where the robot
   started.

With **Build a new map (SLAM)**, run a **Mapping run** first. The robot
drives to the edges of the known map until no reachable unknown edge is
left. It then divides the rooms and returns to the dock.

## What you can do

| Feature | Where | How it works |
|---|---|---|
| Whole home | Clean | All rooms in the order on the Rooms page, then any reachable floor outside a room |
| Selected rooms | Map **Rooms** tool + Clean | Click rooms. The click order is the cleaning order (shown as 1 ·, 2 ·) |
| Zones | Map **Zone** tool | Drag up to 8 rectangles. Each zone keeps its number of passes |
| Spot | Map **Spot** tool | A 1.6 m square spiral, twice, around the clicked point (or around the robot) |
| Edges only | Clean | Along every wall and piece of furniture in all rooms |
| Mapping run | Clean | Frontier exploration of a SLAM map, then room division |
| Suction | Clean | Eco / Standard / Turbo / Max: speed, dust pick-up and battery runtime from the robot profile |
| Passes | Clean | 1–3. Each further pass crosses the previous one at right angles |
| Edge cleaning | Clean | Wall-following pass at the closest safe distance before the lanes |
| No-go areas | Map **No-go** tool | The robot body stays out of them |
| Virtual walls | Map **Wall** tool | Never crossed |
| Carpets | Map **Carpet** tool | Drag a rectangle over a rug. *Carpet boost* (Care) runs Max suction there; a mopping job treats carpets as no-go areas |
| Mopping | Clean → *Cleaning* | Vacuum, Vacuum + mop or Mop only, with a *Water flow* (low / medium / high). Robots with a mop module (`water_tank_ml` in the profile). Edge passes are vacuumed only |
| Pause / Resume / Stop / Dock | Clean | Resume continues where the job paused. Dock ends the job and returns |
| Locate | Clean | Rings the robot on the map. An idle robot also wiggles |
| Room names, order, merge | Rooms | Rename, ↑/↓, Merge (two or more adjoining rooms) |
| Split a room | Map **Split** tool | Drag a line across the room |
| Per-room settings | Rooms | Suction, passes and edges per room. Blank means use the job setting |
| Divide again | Rooms | New room division with a doorway width. Names carry over by position |
| Dock | Map **Dock** tool or Setup | Click the dock, then drag the direction the robot faces when docked |
| Schedules | Schedule | Days, time, what to clean (all / selected rooms / edges) and suction. Runs on the robot |
| Do not disturb | Schedule | No automatic resume after charging inside the window |
| History | History | Each run: result, area, coverage, time, recharges, obstacles. **Show on map** draws its cleaned area |
| Maintenance | Care | Main brush, side brush, filter, sensor and mop pad timers with Reset. Dustbin and water tank levels |
| Recharge and resume | Care | Below *Return at* % it docks, charges to *resume at* %, then continues the job |
| Auto-empty dock | Care | When the bin is full it docks, empties for 10 s and continues |
| Refilling dock | Care | When the water tank runs dry it docks, refills (up to 20 s) and continues. Without it, Vacuum + mop goes on vacuuming and Mop only ends the job |
| Obstacles | automatic | LiDAR stop, back-off and a detour around anything not on the map. The obstacle is remembered for the run |
| Floor objects | automatic (front camera) | Cables, socks, shoes and pet mess are recognised and kept away from (15 / 10 / 6 / 35 cm plus 8 cm). They show on the map as C, S, Sh and ! and are remembered across jobs. The **Erase** tool on one cleans over it from the next job. In simulation the recogniser is Gazebo's segmentation camera, an ideal detector |
| Getting free | automatic | Six bumps in 30 s: wedged; it forgets those bump marks and backs out, and the third time asks for help ("Stuck"). Walled in by its own marks, or lost in a closed pocket of the map: it forgets nearby marks, then drives to the nearest open floor |
| Bumper, cliff, stuck | automatic | Contact or a floor drop: back off, mark it and go around. Not moving for 6 s: free-itself manoeuvre, error after 3 tries |
| Drive by hand | Setup → Drive pad | Keyboard and joystick override the cleaning system (twist_mux priority) |

## Pieces

- `robot_lab_vacuum_cleaning/grid.py`, `planner.py`: the planning map
  (map obstacles, unknown cells, keep-outs and run obstacles inflated by
  the robot radius plus a margin), A*, lanes, edge contours, spot spiral
  and frontiers.
- `rooms.py`: room division. Free space is split where it narrows
  relative to the rooms on either side. Cores at increasing clearance
  thresholds are grown back by a watershed, so borders settle in
  doorways. Strips no robot fits into are dropped.
- `mission.py`: jobs and the state machine (idle, cleaning, mapping,
  paused, returning, docking, charging, emptying, docked, error). It
  includes pure pursuit and the battery, dustbin and wear models. No ROS
  imports. Unit tests drive it with a kinematic robot.
- `store.py`: persistence under `$ROBOT_LAB_RUNTIME_ROOT/vacuum` (or
  `~/.ros/robot_lab_vacuum`): per-map layouts (rooms, keep-outs, zones,
  dock), settings, schedule, wear and `history.jsonl` with each run's
  cleaned area.
- `vacuum_cleaner.py` (node). It needs `/map` and the `map →
  base_footprint` transform, so it works in `nav` and `slam` mode. It
  reads `/scan`, and optionally `/bumper` (ros_gz Contacts) and
  `/cliff/*`. It drives `/cmd_vel`. Topics: `/vacuum/command` (JSON in),
  plus `/vacuum/status`, `/layout`, `/history`, `/plan`, `/events`,
  `/coverage`, `/rooms` and `/path` out. Services: `vacuum/{start,pause,
  resume,stop,dock,locate}` (std_srvs/Trigger).

Command-line examples:

```bash
ros2 service call /vacuum/start std_srvs/srv/Trigger
ros2 topic pub --once /vacuum/command std_msgs/String \
  "data: '{\"cmd\": \"start\", \"mode\": \"rooms\", \"rooms\": [2, 4], \"suction\": \"turbo\"}'"
```

A recorded, headless run of one job:

```bash
scripts/sim_vacuum_mission.sh gazebo oomwoo small_house out.json \
  '{"mode": "zones", "zones": [[4.0, -3.5, 7.5, -0.5]]}'
```

## Camera inspection and learned recognition

The **Objects** page shows timestamped RGB crops of observed floor objects. Setup selects
`segmentation` (ideal simulator labels), `learned` (local trained checkpoint) or `none`.
A locally provisioned experimental checkpoint is now prefilled; learned physical
avoidance remains unqualified and the default detector stays ideal segmentation.
See the [camera workflow and agent instructions](vacuum-camera.md) for training, launch arguments
and independent object-world clearance checks.

## The `oomwoo` model

OOMWOO ([makers-pet/oomwoo](https://github.com/makers-pet/oomwoo)) is an
open-source robot vacuum whose hardware is not frozen. Its wiki names only
"an affordable 2D LiDAR" and a Raspberry Pi CM4/CM5, so the lab model uses
values typical of a 2D-LiDAR home vacuum, not OOMWOO measurements:

- 0.33 m body;
- differential drive (wheel radius 0.035 m, separation 0.22 m) and two casters;
- a 360° LiDAR turret (5 Hz, 12 m);
- a ring contact bumper (`/bumper`);
- two downward cliff ranges (`/cliff/front_left`, `/cliff/front_right`);
- an IMU;
- cleaning width 0.26 m;
- an optional mop module with a 200 ml water tank (a lab choice).

Gazebo Fortress has all sensors. The PyBullet, MuJoCo and Isaac bridges
cast a 2D LiDAR for any robot but have no bumper or cliff sensor.

## Models, not measurements

The simulators have no battery, dust, water or brush wear. Battery runtime
per suction mode, charge time, dustbin capacity, water tank and part
lifetimes come from the robot profile (`vacuum:` in `robots.yaml`). Water
use is 1.0 / 1.8 / 2.6 ml per mopped m² (low / medium / high). Carpets are
drawn by the operator: the simulated floors have no carpet the robot could
sense, so there is no carpet sensor. **Battery model speed**
(Care page) accelerates the battery for demonstrations. Schedules use the
computer's local clock.

## Measured so far

Gazebo, `small_house`, `nav` mode, judged by `cleaning_screen` from the
simulator truth (2026-11-10, [evidence](../status/evidence/vacuum-2026-11/README.md)).
Every run below docked within 0.051 m by truth.

| Job | Coverage | Notes |
|---|---|---|
| Office, edges | 95.4 % | |
| Zone + no-go box + virtual wall | 99.6 % | centre ≥ 0.26 m from the box, no crossings |
| Spot beside furniture | 90.7 % | |
| Zone with a hidden 6 cm box | 97.5 % | found by the stall detector |
| Same, with the Gazebo bumper working | 97.0 % | 4 bumps at the box |
| Bedroom, battery model 60× | 98.4 % | 10 recharge-and-resume stops |
| Zone + keep-outs in MuJoCo | 99.6 % | |
| Whole home | 96.3 % | 141.6 m² in 77 min, one auto-empty stop |
| Mapping run (slam mode) | map complete | 6 rooms, 85 m, 6 min |

Not yet measured:
- PyBullet and Isaac;
- repeats and other houses.

The Gazebo bumper publishes contacts since 89b71ebb (world-level Contact
system). Catalog entries, unit tests and screenshots do not qualify a cleaning mission.
