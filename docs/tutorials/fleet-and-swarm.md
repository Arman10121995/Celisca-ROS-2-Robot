# Shared-world fleet and ground swarm controls

The native **Fleet & Swarm** tab starts several complete Bumperbot/Labbot
instances in one selected Gazebo world. Their physics, obstacles and clock
are shared. Each instance has separate controllers, sensors, joint states,
odometry, TF frames and optional AMCL/Nav2 actions. This is an experimental
extension on the current ROS 2 Humble / Gazebo Fortress 6 installation.
Other shared-world backends, Open-RMF traffic scheduling, aerial swarms and
marine/multi-legged adapters remain separate roadmap work.

## Start from the GUI

1. Open **Fleet & Swarm**. Select the map family and its installed variant.
2. Keep the example `alpha` Bumperbot and `beta` Labbot, or add up to eight
   instances of these two complete models. Instance ids must be distinct
   lowercase ROS names. Instances do not add duplicate robot catalog entries.
3. Leave X/Y blank for nearby automatic placement. Placement checks the
   selected occupancy grid, including unknown/exterior cells and member
   footprint separation, and prefers a lateral row. A world without a 2D map
   requires explicit X/Y for every instance. Z/yaw can override the map defaults.
   Occupancy clearance does not establish dynamic or 3D collision clearance.
4. Enable **Per-robot AMCL + Nav2** for localized goals. The GUI selects
   AMCL, Smac 2D and regulated pure pursuit using the existing robot parameters.
   One shared map server publishes the actual selected occupancy map.
5. Choose Gazebo/RViz windows and click **Run shared world**. The run command
   is filled automatically; Copy command reproduces the same session file.
   Connect and keyboard enable start neutral and send no movement input.

The GUI writes presets and resolved launch assets under the workspace SSD.
Save/load preserves map, complete model ids and optional explicit spawn poses.
Editing a preset affects the next launch; it does not move live entities.

## Drive one robot or a group

Choose a live instance, or **All included**, and click **Arm manual · neutral**.
Click a direction once to latch it; click again to release and brake. Forward,
reverse, yaw left/right and forward diagonals use the existing bounded ramp
logic. The displayed Δ values are the change per 0.1 seconds, not actual speed.
The live status separately shows the applied command and measured positions.
Fleet limits are 0.35 m/s and 0.8 rad/s or the lower robot profile limits.

Enable WASD/joystick for W/S forward/reverse and A/D yaw. A real gamepad starts
from its calibrated neutral position; touchscreens and other non-gamepads are
ignored. Keyboard input applies while this tab is selected and a text editor
does not have focus. **Space / ALL STOP** clears group inputs and stops owned
actions. Stop selected affects the selected manual members. Changing the
manual target clears the previous latches. Leaving the application clears
manual inputs; closing/stopping the simulator sends All Stop.

The controller closes the command gate on stale clock, wheel feedback, scan
or simulator pose. Commands expire after 0.35 wall seconds; manual/swarm
operator heartbeat expires after 0.8 wall seconds. The wheel controller also
has its own 0.4 simulation-second command timeout. Source acceleration limits
still apply: motor commands going neutral does not mean an instantaneous
physical stop. Scan and predicted member-separation guards can block an input;
the live member reason explains why. They are conservative guards, not a
qualified collision-avoidance or traffic system.

## Dispatch and allocate actual Nav2 goals

The **Tasks** page takes X/Y/yaw in the common map frame. Dispatch selects one
member; **Nearest idle** allocates the goal from fresh estimated positions to
an idle included member. Goals require free occupancy with body clearance,
fresh localization and a real available namespaced NavigateToPose server.
The default budget is 120 wall seconds, with an explicit runtime range of
5–600 seconds. No demonstration goal runner or synthetic PASS metric is used.

Pause cancels the actual owned action and retains its goal. Resume sends it
again after the old action returns a terminal result. Stop immediately closes
the motor gate and requests cancellation. A cancellation acknowledgement is
not a terminal result: replacement or manual commands wait for the result.
Late goal acknowledgements are canceled and cannot reopen a stopped gate.
Autonomous Nav2 goals retain their own budget; the operator heartbeat applies
to manual and swarm ownership. Battery, charging, doors and lifts are unknown.

Reset first requests Stop/cancel and waits for measured physical settling.
It restores only that entity's recorded settled spawn pose through Gazebo's
SetEntityPose service. World time, other entities and continuous wheel encoder
history remain intact. Display re-anchors odometry; Nav2 receives an initial
pose and waits for a consistent localization transform. The acknowledgement,
subsequent physical pose, member isolation and dynamic clearance are distinct
observations. Broad reset/contact qualification remains open.

## Ground swarm page

Start controls all included members with fresh estimated state and scans.
The initial centralized kernels are:

| Controller | Actual commanded behavior |
|---|---|
| Formation | Seek equally spaced positions on a lateral row about the group centroid or explicit map anchor |
| Rendezvous | Seek a spaced ring about the centroid/anchor; members do not converge into one occupied point |
| Dispersion | Separate close neighbors using a bounded repulsive velocity field |
| Flocking | Combine separation, centroid cohesion, circular heading alignment and an operator heading bias |

Spacing is 0.6–3 m. Differential bases rotate toward a feasible heading before
translation; they are never repositioned to a formation. These are labeled
educational kernels, not the full ROS2swarm implementation, a distributed
protocol, cooperative coverage or Open-RMF scheduling. Member loss stops the
controlled group. Leave group keeps the physical robot in the world and stops
the group; Rejoin requires fresh feedback and starts neutral. Air swarms need
separate FCU instances/leases and are not enabled by this ground interface.

## Source and measurement details

The launch uses one physics server and one sensor bridge clock. Model entity
names and Gazebo sensor topics are distinct; ROS namespace alone is not enough.
Source link/joint names remain local to each entity. Robot state publisher,
feedback controller and sensor frames use `<id>/...` consistently. Wheel
feedback owns `<id>/odom → <id>/base_footprint`; AMCL or a known spawn/reset
anchor owns the map relationship. Simulator truth is separate from estimates.

The first-party Labbot fleet derivative raises its physical lidar mount by
**5 cm**. Its original scan plane intersects the OAK-D visual. This removes
self returns by moving the actual sensor and its TF, rather than inventing
free scan rays. The resolved manifest records the adjustment. Single-robot
descriptions and historical source certificates are unchanged.

Some initial model-odometry diagnostics reported zero twist while their
poses moved. Fleet velocity/settling is calculated from successive measured,
stamped poses; the original reported twist is retained separately. Sensor
samples must advance and match the current world clock. Wheel and localization
estimates are labeled and never replaced with simulator truth for navigation.

## CLI and artifacts

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup shared_world.launch.py \
  map_name:=nav_empty navigation:=true gui:=true rviz:=true
```

The default session is `robot_lab_bringup/config/fleet_demo.yaml`. Example:

```yaml
schema_version: 1
map: nav_empty
members:
  - id: alpha
    model: bumperbot
  - id: beta
    model: labbot
    spawn: {x: -0.5, y: 0.5, z: 0.1, yaw: 0.0}
```

`session:=/absolute/path/session.yaml` selects a saved preset. Other simulator
values are rejected explicitly. The launch prints an SSD `fleet_sessions/
session-.../session.json` path containing the selected world, source/generated
hashes, placements and generated descriptions/configs. `requested-session.yaml`
preserves the request; `events.jsonl` records real commands, rejections, Nav2
results and reset acknowledgements with wall/simulation time, bounded at 16 MiB.
`/fleet/state` publishes actual member counts, ages, estimates, simulator poses,
commands and action state. Runtime artifacts are not automatically a passed
mission; record exact scope and failed attempts with the ledger.

The GUI/controller share a session id and one live operator lease. Direct
clients must publish `/fleet/operator_heartbeat` with `session_id/operator_id`
and `/fleet/command` with those fields, an `op` and target id list. `/fleet/stop`
is a Trigger service for a session-wide neutral gate and cancellation request.
Each `<id>/reset` Trigger acknowledges the reset request; observe state for
completion. ROS domain/transport partition isolation is still required when
running a separate simulation. Isolated experiment jobs remain separate worlds.

## Agent continuation

Read the ledger and claim the next R10 task before edits. The runtime is in
`robot_lab_utils/fleet_controller.py`; session preparation is in `fleet_session.py`,
bounded group kernels in `fleet_motion.py`, launch in `shared_world.launch.py`
and the native editor in `robot_lab_gui/fleet_tab.py`. Keep these contracts:

- Humble's ground launch forces Fortress 6; PX4's Harmonic 8 is separate.
  Keep compatible system plugins and explicit per-instance controller files.
- The control plugin prepends its namespace to `robot_param_node`; use the
  relative `robot_state_publisher` name. Prefixing it twice aborts the server.
- Pass the controller file to the spawner before controller initialization,
  and serialize embedded manager startup. Do not name a callback `_clock`:
  rclpy already owns that field.
- Preserve per-instance actuator/source limits and generation/cancel barriers.
  Do not silently reset the common world clock or relabel odometry as truth.
- Add real task queues/coverage and a Humble-compatible Open-RMF adapter before
  claiming scheduling, resources or infrastructure. Record unknown energy data.
- Additional plants, robot families and marine/gait dynamics need their own
  adapters and GUI capability gates; a copied namespace is not an adapter.
- Keep broad map/member/contact/loss sweeps in the validation lane. Necessary
  launch errors are implementation defects and should be fixed as encountered.

Primary implementation references: [Humble control plugin namespaces](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html),
[feedback differential-drive controller](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html),
[Humble multi-robot Nav2 example](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/cloned_multi_tb3_simulation_launch.py).
