# Robot Lab glossary

Read this alongside the [interactive architecture guide](interactive-guide.md).
Search this page in **Learn & health → Tutorials & source library**. The
**Glossary** button also opens it directly from System diagrams.

## The software pieces

| Term | Plain meaning | Robot Lab example |
|---|---|---|
| Package | A named unit of ROS code/configuration with a manifest | `robot_lab_gui`, declared by its `package.xml` |
| Node | A running ROS participant performing a specific job | A member's AMCL process |
| Launch | A recipe that starts/configures processes and their relationships | `simulated_robot.launch.py` |
| Profile | Named configuration for an exact robot/world/mode | Core robot parameters in `robots.yaml` |
| Resolver | Code that validates selections and produces effective settings | Shared GUI/CLI manifest and command resolution |
| Manifest | A saved, explicit description of what a run requests | Robot/backend/map/algorithm selection and overlays |
| Plugin | A replaceable implementation loaded into a host node/engine | A selected Nav2 planner or Gazebo system |
| Backend / plant | The engine and physical simulated system it advances | A MuJoCo robot or Fortress shared world |
| Bridge / adapter | Code translating one system's interface to another | Native engine measurements to ROS messages |
| C4 diagram | A way to explain context, containers and internal components | Atlas overview followed by focused subsystem views |

## Communication

| Term | Plain meaning | What to inspect |
|---|---|---|
| Topic | A named channel carrying a particular message type | Exact resolved topic and advertised endpoints |
| Publisher / subscriber | The sender / receiver declarations on a topic | Their type and requested/offered QoS |
| Service | A request/response operation | Map load or costmap clear; existence differs from readiness |
| Action | A long-running goal with feedback, result and cancellation | NavigateToPose or a joint trajectory |
| DDS / RMW | ROS's distributed middleware and its abstraction layer | Discovery, domain and middleware implementation |
| ROS domain | A logical DDS discovery boundary | `ROS_DOMAIN_ID` inherited by GUI and nodes |
| Namespace | A prefix for relative ROS names | `/alpha/scan` versus `/beta/scan` |
| QoS | Policies for message delivery/history and endpoint compatibility | Best-effort/reliable and volatile/transient-local |
| Gazebo partition | A separate native simulator transport boundary | `GZ_PARTITION` / `IGN_PARTITION` |
| MAVLink | The telemetry/command protocol used by the current PX4 controller | Per-FCU UDP endpoint and system ID |
| Snapshot | An observation captured at one time | The live graph's timestamped advertised endpoints |

Namespaces, frame IDs, domains, simulator partitions and FCU identities are
different kinds of names/isolation. See [networking](networking.md).

## Motion, perception and planning

| Term | Plain meaning | Common confusion |
|---|---|---|
| Pose | Position and orientation in a specified frame | A coordinate without its frame is ambiguous |
| TF | A time-indexed relationship between coordinate frames | Two publishers for one edge can cause jumps |
| Odometry | A locally continuous estimate of motion | It can drift; it is not automatically global truth |
| Localization | Estimate the robot's pose against a known reference/map | It does not create a gait or a new map |
| SLAM | Build a map while estimating motion | A saved grid, a cloud and a database are different products |
| AMCL | A particle-based planar known-map localization method | It needs suitable scans, map, frames and initial pose |
| RTAB-Map | A mapping/localization system used here for RGB-D workflows | Camera calibration and capture-time transforms matter |
| Nav2 | ROS 2's navigation stack of cooperating servers/plugins | It is more than one path-planner function |
| Costmap | A spatial representation of obstacles/costs for planning/tracking | It includes robot clearance and may contain dynamic observations |
| Footprint | The robot's planar occupied shape/envelope | A point-safe route may still collide with a wide robot |
| Global planner | Chooses a geometric route through a larger map | It does not directly generate motor effort |
| Local controller | Tracks a route with current obstacles/kinematics | Its command still needs ownership and native actuation |
| Actuator | A native source of physical joint/wheel/rotor/thruster force | An input topic alone does not prove force was applied |
| Policy / gait | A robot-specific rule/model for legged joint actions | A loaded checkpoint or standing hold is not qualified walking |
| IK / FK | Inverse / forward mapping between joint angles and end-effector pose | A Cartesian target can be unreachable or collision-constrained |
| Servo / jog | Small incremental articulation or Cartesian motion | Target selection and explicit execution remain distinct |

## Ownership, flight and evidence

| Term | Plain meaning | Robot Lab consequence |
|---|---|---|
| Lifecycle | Configured/active/inactive state of a managed node | An inactive controller cannot accept normal commands |
| Readiness | Verified prerequisites for the selected task | Process existence is insufficient |
| Heartbeat / lease | Evidence that a control owner is still present | Stale ownership stops/holds the relevant workflow |
| Epoch / generation | Identity distinguishing old versus new control/run state | Returning feedback must not revive a stopped task |
| Interlock | A condition preventing an unsafe/invalid transition | Lift gates close before vertical movement |
| FCU | Flight control unit, here the PX4 SITL instance | Its estimator/arming/offboard state is distinct from the GUI |
| Offboard | Flight commands supplied by an external controller | Requires the FCU's own prerequisites and maintained setpoints |
| Ground truth | Independent simulator observation used for evaluation | Keep it separate from the algorithm's estimated input |
| Settled endpoint | Actual pose with sufficiently low measured motion | A sent goal or action success may not establish settling |
| Evidence / receipt | Scoped commands, source identity, measurements and outcomes | One named mission does not qualify every robot/map/backend |
| Experimental | Implemented candidate with limited qualification | It must remain visibly different from measured support |
| CI | Automated build and source/contract regression checks | It does not replace physical simulator missions |
| Real-time factor | Simulated elapsed time divided by wall elapsed time | Slow simulation and stale ownership are different issues |

Continue with [the architecture reference](overview.md),
[the tutorial index](../tutorials/README.md) or
[current status](../status/CURRENT_STATUS.md).
