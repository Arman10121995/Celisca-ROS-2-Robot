# Learn Robot Lab from zero

This course is for someone who has never used ROS. It explains what this
project does, how the pieces fit together, and how a robot goes from "a
picture of a house" to "a cleaned floor". The robot vacuum runs through it
as the example.

Each chapter has diagrams and figures. The figures are drawn from the
project's own code and maps by
[`figures/make_figures.py`](figures/make_figures.py), so they show what the
software really does. GitHub draws the diagrams (Mermaid) directly in the
page.

## The path

```mermaid
flowchart LR
    A["1 · The big picture<br/>what Robot Lab is"] --> B["2 · ROS 2 concepts<br/>nodes, topics, TF"]
    B --> C["3 · Networking<br/>how programs find each other"]
    C --> D["4 · Robots and sensors<br/>URDF, LiDAR, wheels"]
    D --> E["5 · Maps and localisation<br/>where am I?"]
    E --> F["6 · Planning and control<br/>how do I get there?"]
    F --> G["7 · Inside the robot vacuum<br/>everything together"]
    G --> H["8 · Evidence and testing<br/>how we know it works"]
```

| # | Chapter | You will understand | Key visuals |
|---|---|---|---|
| 1 | [The big picture](01-big-picture.md) | what happens when you press **Launch**, and which program does what | system map, launch sequence |
| 2 | [ROS 2 concepts](02-ros2-concepts.md) | nodes, topics, services, actions, parameters, launch files, QoS, TF, simulated time | message-flow diagrams, TF frames |
| 3 | [Networking](03-networking.md) | DDS discovery, domains, ports, Gazebo transport, bridges, MAVLink, debugging | discovery sequence, isolation diagram, troubleshooting flowchart |
| 4 | [Robots and sensors](04-robots-and-sensors.md) | URDF/xacro, links and joints, LiDAR, cliff, bumper, IMU, differential drive | robot tree, LiDAR scan, wheel kinematics |
| 5 | [Maps and localisation](05-maps-and-localization.md) | occupancy grids, SLAM, the particle filter (AMCL), odometry drift | real map, particle filter in three steps |
| 6 | [Planning and control](06-planning-and-control.md) | inflation, A*, Nav2, pure pursuit, the control loop | A* animation, pure pursuit, Nav2 pipeline |
| 7 | [Inside the robot vacuum](07-robot-vacuum-inside.md) | room division, coverage lanes, edges, spot, the mission state machine, stall detection, battery | room division in four steps, cleaning animation, state machine |
| 8 | [Evidence and testing](08-evidence-and-testing.md) | unit tests vs simulation runs, reports, the support matrix | evidence pipeline, matrix chart |

## Part 2 · Extending the lab

For contributors who want to add a robot, a world or a control to the lab.
It assumes chapters 2, 4 and 5.

```mermaid
flowchart LR
    I["9 · What you need<br/>prerequisites, file map,<br/>integration path"] --> J["10 · Adding a robot<br/>source, model, profile,<br/>adapters, screens"]
    I --> K["11 · Adding a world and map<br/>SDF, MJCF, occupancy,<br/>spawn, checks"]
    J --> L["12 · Putting it in the GUI<br/>families, taxonomy, control pages,<br/>algorithms, docs, evidence"]
    K --> L
```

| # | Chapter | You will be able to |
|---|---|---|
| 9 | [Extending the lab](09-extending-the-lab.md) | set up a host, find every file that matters, follow the seven-stage integration path |
| 10 | [Adding a robot](10-adding-a-robot.md) | pin a source, check its model, write a profile or provisioning script, declare estimates, measure it |
| 11 | [Adding a world and map](11-adding-a-world-and-map.md) | add an SDF world, its MuJoCo scene and occupancy map, set spawn and initial pose, check each simulator |
| 12 | [Putting it in the GUI](12-putting-it-in-the-gui.md) | group and classify it, give it a control page, register algorithms, document it and record evidence |

The worked example is the Elephant Robotics myAGV Pro and its Pro630 arm
version ([tutorial](../tutorials/myagv-pro.md)).

Short on time? Read 1, skim 2, then go to 7. Every chapter starts with a
"**In one sentence**" summary and ends with "**Try it**" commands.

## Before you start

- You need the workspace built (see the root [README](../../README.md)).
  Nothing in chapters 1–6 requires running a simulator; the "Try it" boxes
  that do say so.
- Words in **bold** the first time are in the [glossary](../architecture/glossary.md).
- The more advanced, complete references are the
  [architecture overview](../architecture/overview.md) and
  [networking reference](../architecture/networking.md). The GUI has an
  interactive version under **Learn & diagnose**.

## Regenerating the figures

```bash
python3 docs/learn/figures/make_figures.py          # all (a few minutes)
python3 docs/learn/figures/make_figures.py astar    # one
```

Names: `grid lidar mcl tf inflation astar pursuit diff rooms coverage spot
stall cleaning battery matrix`.

The algorithm tutorials ([planning](../tutorials/planning.md),
[local planning](../tutorials/local_planning.md),
[state estimation](../tutorials/state_estimation.md),
[sensor fusion](../tutorials/sensor_fusion.md),
[perception](../tutorials/perception.md), [control](../tutorials/control.md),
[localisation](../tutorials/localization.md)) open with a visual section whose
figures come from [`figures/algorithm_figures.py`](figures/algorithm_figures.py).
It runs the project's own `robot_lab_algorithms` classes (`planners dwa kalman
imu pid perception`).
