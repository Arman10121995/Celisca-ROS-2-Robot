# 2 · ROS 2 concepts

**In one sentence:** ROS 2 lets many small programs (**nodes**) cooperate
by sending each other typed messages, without knowing where the others run.

A robot's software is not one big program. It is dozens of small ones: one
reads the LiDAR, one estimates the position, one plans, one drives the
wheels. ROS 2 is the "post office" between them.

## Nodes

A **node** is one program (or one part of a program) with one job. While a
robot-vacuum simulation runs, `ros2 node list` shows about 25 of them, for
example:

| Node | Job |
|---|---|
| `/robot_state_publisher` | publishes where each part of the robot is (from the URDF) |
| `/controller_manager` | runs the wheel controller |
| `/map_server` | serves the saved map |
| `/amcl` | estimates where the robot is on the map |
| `/vacuum_cleaner` | plans and drives the cleaning |
| `/twist_mux` | picks which velocity command reaches the wheels |

## Topics: one-way broadcasts

A **topic** is a named channel with a fixed message type. Any number of
nodes may publish on it, and any number may subscribe. Nobody waits for an
answer.

```mermaid
flowchart LR
    LIDAR["Gazebo LiDAR<br/>(via ros_gz_bridge)"] -- "/scan<br/>sensor_msgs/LaserScan, 5 Hz" --> VAC[vacuum_cleaner]
    LIDAR -- "/scan" --> AMCL[amcl]
    MAP[map_server] -- "/map<br/>nav_msgs/OccupancyGrid" --> AMCL
    MAP -- "/map" --> VAC
    VAC -- "/cmd_vel<br/>geometry_msgs/Twist" --> MUX[twist_mux]
    KEY["keyboard / GUI drive pad"] -- "/key_vel" --> MUX
    MUX -- "/robot_lab_controller/cmd_vel_unstamped" --> WHEELS[diff-drive controller]
    VAC -- "/vacuum/status (JSON, 2 Hz)" --> GUI[GUI Vacuum page]
```

A message type is a small fixed structure. `geometry_msgs/Twist`, the
standard "drive like this" command, is:

```text
linear:  {x: 0.25, y: 0.0, z: 0.0}    # forward speed, m/s
angular: {x: 0.0, y: 0.0, z: 0.8}     # turn rate, rad/s (positive = left)
```

### Who wins when two nodes drive? The multiplexer

Several nodes may want to move the robot: you on the keyboard, a joystick,
the cleaning system. `twist_mux` passes on the highest-priority input that
is still active (newer than its 0.5 s timeout):

```mermaid
flowchart LR
    J["/joy_vel joystick<br/>priority 99"] --> M{twist_mux}
    K["/key_vel keyboard<br/>priority 90"] --> M
    N["/cmd_vel navigation, vacuum<br/>priority 80"] --> M
    S["safety_stop lock<br/>priority 255"] -. blocks all .-> M
    M --> W[wheels]
```

That is why pressing a key in the GUI drive pad overrides the vacuum at
once. When you let go, the vacuum takes over again after 0.5 s.

## Services: ask and wait for an answer

A **service** is a request/response call, like a function call to another
program. The cleaning system offers `vacuum/start`, `vacuum/pause`,
`vacuum/resume`, `vacuum/stop`, `vacuum/dock` and `vacuum/locate`
(type `std_srvs/Trigger`: no input; the answer is success plus a message).

```mermaid
sequenceDiagram
    participant You as ros2 service call
    participant V as vacuum_cleaner
    You->>V: /vacuum/start (Trigger)
    V-->>You: success: true, message: "Started whole home cleaning: 232 paths, about 85 min"
```

## Actions: long jobs with progress and cancel

An **action** is for jobs that take time: "drive to this goal". The caller
gets feedback while the job runs and can cancel it. Nav2's
`navigate_to_pose` is an action.

```mermaid
sequenceDiagram
    participant C as Client (GUI, script)
    participant N as Nav2 bt_navigator
    C->>N: goal: x = 3.0, y = 1.0
    N-->>C: accepted
    loop while driving
        N-->>C: feedback: distance remaining 2.1 m
    end
    alt you press Stop
        C->>N: cancel
        N-->>C: canceled
    else arrives
        N-->>C: result: succeeded
    end
```

| | Topic | Service | Action |
|---|---|---|---|
| Direction | one-way | request → response | goal → feedback… → result |
| Waits? | no | yes, briefly | no; you get updates |
| Cancel? | – | – | yes |
| Example here | `/scan`, `/cmd_vel` | `vacuum/dock` | `navigate_to_pose` |

## Parameters

A **parameter** is a node's setting, given at start and sometimes changeable
while it runs. The cleaning node has, for example, `robot_model`
(`oomwoo`), `map_key` (`small_house`) and `use_sim_time` (`true`):

```bash
ros2 run robot_lab_vacuum_cleaning vacuum_cleaner --ros-args -p robot_model:=oomwoo -p use_sim_time:=true
ros2 param get /vacuum_cleaner robot_model
```

## Launch files: starting many nodes at once

A **launch file** (Python, in `launch/` folders) starts and configures a
group of nodes, and can include other launch files. Launch **arguments**
(`name:=value`) choose what to start:

```mermaid
flowchart TB
    RV["simulated_room_vacuum.launch.py<br/>controller:=cleaning"] --> SR["simulated_robot.launch.py<br/>robot_model, map_name, simulator, mode"]
    RV --> VC[vacuum_cleaner node]
    SR --> GZ["gazebo.launch.py<br/>world + robot + bridge"]
    SR --> CT["controller.launch.py<br/>ros2_control, twist_mux"]
    SR --> LO["global_localization.launch.py<br/>map_server + AMCL"]
    SR --> NV["navigation.launch.py<br/>Nav2"]
```

## QoS: how carefully to deliver

**Quality of Service** settings say how a topic behaves when the network is
busy or a subscriber starts late. Publisher and subscriber must be
compatible, or no data flows (a classic beginner's surprise).

| Setting | Options | Used here for |
|---|---|---|
| Reliability | *reliable* (resend lost messages) / *best effort* (drop them) | `/scan` is best effort: a late scan is useless anyway |
| Durability | *volatile* / *transient local* (keep the last message for late joiners) | `/map`, `/vacuum/layout`, `/vacuum/rooms`: a GUI started later still gets them |
| History depth | keep the last N | commands: 10 |

## TF: where is everything?

A robot has many **coordinate frames**: the map, the odometry origin, the
robot's base, the LiDAR, each wheel. **TF** keeps the transforms between
them, and any node can ask "where is the LiDAR in map coordinates, right
now?"

![TF frames](figures/tf_frames.png)

```mermaid
flowchart TB
    map -->|"AMCL or SLAM<br/>(jumps to correct drift)"| odom
    odom -->|"wheel odometry + EKF<br/>(smooth, but drifts)"| base_footprint
    base_footprint --> base_link
    base_link --> laser_link
    base_link --> imu_link
    base_link --> bumper_link
    base_link --> wheel_left_link
    base_link --> wheel_right_link
```

Why two "world" frames? Wheel odometry is smooth but slowly drifts.
Localisation against the map is accurate but jumps. Keeping them as
separate frames gives a controller a smooth pose and a planner an accurate
one (chapter 5).

## Simulated time

A simulator may run slower or faster than real time. Every node started
with `use_sim_time:=true` reads the clock from the `/clock` topic, which
the simulator publishes. Ten seconds of driving are then ten *simulated*
seconds, even if the computer needed fifteen.

## Try it

With a simulation running (chapter 1):

```bash
ros2 topic list                       # all channels
ros2 topic echo /cmd_vel              # watch velocity commands
ros2 topic hz /scan                   # how often the LiDAR publishes
ros2 topic info /map --verbose        # publishers, subscribers, their QoS
ros2 run tf2_tools view_frames        # draws the TF tree to frames.pdf
ros2 service call /vacuum/locate std_srvs/srv/Trigger
rqt_graph                             # the live node/topic graph
```

Next: [3 · Networking](03-networking.md).
