# 7 · Inside the robot vacuum

**In one sentence:** the cleaning system divides the map into rooms, plans
a path that sweeps every reachable bit of floor, follows it while dodging
what the map does not show, and handles the battery, the dustbin and the
dock like a commercial robot vacuum.

Everything here is in `src/robot_lab_vacuum_cleaning/`. The operator guide
(buttons and pages) is [robot-vacuum.md](../tutorials/robot-vacuum.md);
this chapter explains how it works.

![The vacuum cleaning the office (offline kinematic run of the real code)](figures/vacuum_cleaning.gif)

## The parts

```mermaid
flowchart TB
    subgraph Node["vacuum_cleaner (ROS node, vacuum_cleaner.py)"]
        IN["inputs: /map, TF map→base, /scan,<br/>/bumper, /cliff/*, /vacuum/command"]
        OUT["outputs: /cmd_vel, /vacuum/status, layout,<br/>coverage, rooms, plan, events, history"]
    end
    subgraph Core["pure Python core (no ROS: unit-testable)"]
        GRID["grid.py<br/>cells ↔ metres, masks"]
        ROOMS["rooms.py<br/>room division, merge, split"]
        PLAN["planner.py<br/>inflation, A*, lanes, edges, spot, frontiers"]
        MIS["mission.py<br/>jobs, state machine, follower,<br/>battery, dustbin, wear, stall detector"]
        STORE["store.py<br/>layouts, schedule, history on disk"]
    end
    IN --> MIS
    MIS --> OUT
    MIS --> PLAN
    MIS --> ROOMS
    PLAN --> GRID
    ROOMS --> GRID
    MIS --> STORE
    GUI["GUI Vacuum page"] -->|"/vacuum/command JSON"| IN
    OUT -->|status, maps| GUI
```

Keeping the core free of ROS has a big advantage: the unit tests drive the
real mission code with a tiny simulated robot in milliseconds, and the
figures on this page come from the same code.

## 1 · Dividing the map into rooms

People say "clean the kitchen", so the robot must know where the kitchen
is. `rooms.py` finds rooms in four steps:

![Room division in four steps](figures/room_division.png)

1. Take the free space of the map.
2. Compute each cell's distance to the nearest wall. Doorways are narrow,
   so their clearance is small. Room centres are wide.
3. Keep only the wide parts. These **cores** fall apart at the doorways;
   each core is the middle of a room. It tries several widths, because a big
   hall and a small office need different thresholds.
4. Grow every core back over the free space, widest cells first (a
   *watershed*). Two rooms meet where the floor is narrowest: the doorway.

You can then rename, merge or split rooms in the GUI. Names are kept when
the map is divided again.

## 2 · Planning the coverage

For each room in order:

![Coverage plan for one room](figures/coverage_plan.png)

1. **Edges:** follow walls and furniture at the closest safe distance.
   The side brush reaches into the corners.
2. **Lanes:** straight parallel lines 0.24 m apart (the brush is
   0.26 m wide, so they overlap). The direction is chosen to need fewer
   turns. Lanes are ordered greedily, each starting from the end nearest the
   previous one, which gives the classic back-and-forth pattern.
3. A **second pass** (if chosen) crosses the first at right angles.
4. Moving between lanes uses A* (chapter 6), so it never crosses a no-go
   area or a virtual wall.

Zones and spots get an **outline** pass first, so the lane ends leave no
fringe. A spot is a square spiral on open floor; next to furniture the
spiral would be cut into pieces, so it becomes outline + lanes instead:

![Spot clean on open floor and beside furniture](figures/spot_clean.png)

**No-go areas and virtual walls** are added to the planning map as
obstacles with an extra 12 cm berth, so localisation error cannot carry the
body into them.

## 3 · Running the job: the state machine

The mission is always in exactly one **state**, and events move it to the
next:

```mermaid
stateDiagram-v2
    [*] --> idle
    idle --> cleaning: Start
    idle --> mapping: Start (mapping run)
    docked --> cleaning: Start / schedule
    cleaning --> paused: Pause
    paused --> cleaning: Resume
    cleaning --> returning: job done / Dock / battery low / bin full
    mapping --> returning: no unknown edges left
    returning --> docking: near the dock
    docking --> charging: aligned on the dock
    docking --> emptying: bin full and auto-empty
    emptying --> cleaning: resume the job
    charging --> cleaning: battery ≥ resume level and job unfinished
    charging --> docked: full
    cleaning --> error: stuck 3 times / way blocked
    returning --> error: way home blocked 8 times
    error --> cleaning: Resume
```

## 4 · Following the path and dealing with surprises

Each control step (20 Hz) the follower projects the robot onto its path,
aims a lookahead ahead (pure pursuit, chapter 6) and outputs `(v, w)`. Then
the surprises:

```mermaid
flowchart TD
    STEP["control step"] --> Q1{"bumper, cliff,<br/>or stall?"}
    Q1 -- yes --> BACK["back off 8 cm"]
    Q1 -- no --> Q2{"LiDAR: obstacle within 3 cm<br/>ahead, and before the path's end?"}
    Q2 -- yes --> BACK
    Q2 -- no --> Q3{"commanding motion but<br/>not moving for 6 s?"}
    Q3 -- yes --> FREE["free-itself manoeuvre<br/>(back off, turn 90°)"]
    Q3 -- no --> DRIVE["drive (v, w); paint the cleaned cells"]
    BACK --> MARK["remember the obstacle on the planning map"]
    MARK --> REPLAN["continue the same lane after the obstacle<br/>(A* detour to the next free point)"]
    FREE --> MARK
```

### The virtual bumper (stall detector)

A box lower than the LiDAR plane is invisible. When the robot pushes
against it, its wheels keep turning, and odometry and AMCL report that it
is moving. In a measured Gazebo run the estimate "drove" 1.2 m while the
robot stood still. The stall detector compares two LiDAR scans 1.5 s apart
under two hypotheses:

![Stall detector](figures/stall_detector.png)

It only uses surfaces that face the direction of travel. A wall *beside*
the path looks the same whether the robot moved or not (the "aperture
problem"), so it decides nothing. A stall needs "did not move" to fit
clearly better, and "moved" to miss by at least half the claimed distance.

## 5 · Battery, dustbin, consumables

The simulators have no battery, so the battery is a **model**: each suction
mode has a runtime on a full charge (Eco 180 min … Max 80 min) and charging
takes 180 min. Below 20 % the robot goes home, charges to the resume level,
and continues where it stopped:

![Recharge and resume](figures/battery_recharge.png)

The dustbin fills with cleaned area times suction (150 m² at Standard).
When it is full, a robot with an auto-empty dock goes home, empties for
10 s and continues. The brushes, filter and sensors count cleaning hours
against their replacement intervals (main brush 300 h, side brush 200 h,
filter 150 h, sensors 30 h).

## 6 · The mapping run (new houses)

With no map yet (`slam` mode), the **mapping run** finds **frontiers**:
free cells next to unknown ones. It drives to the nearest reachable one,
lets SLAM extend the map, and repeats. When no reachable frontier is left,
the map is complete: it divides the rooms and returns to the dock.

```mermaid
flowchart LR
    F["find frontiers<br/>(free next to unknown)"] --> C{"any reachable?"}
    C -- yes --> G["A* to the nearest one"] --> D["drive; SLAM grows the map"] --> F
    C -- no --> R["divide rooms → return to dock"]
```

## What has been measured

The recorded Gazebo runs (`scripts/sim_vacuum_mission.sh`) are judged
against the simulator's truth: coverage, keep-out compliance, and the final
pose against the dock. See [chapter 8](08-evidence-and-testing.md) for the
method and [current status](../status/CURRENT_STATUS.md) for the numbers.

## Try it

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/robot_lab_vacuum_cleaning/test   # 23 tests, ~20 s
python3 docs/learn/figures/make_figures.py rooms coverage cleaning
scripts/sim_vacuum_mission.sh gazebo oomwoo small_house /tmp/run.json \
  '{"mode": "rooms", "rooms": [4], "edge": true}'          # one recorded Gazebo job (~6 min)
```

Next: [8 · Evidence and testing](08-evidence-and-testing.md).
