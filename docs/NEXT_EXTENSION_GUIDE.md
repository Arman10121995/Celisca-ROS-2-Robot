# Next extension: fleets, swarms, marine and multi-legged robotics

October 15 native AUV continuation: [Tethys propeller/fin controls](tutorials/tethys-auv.md)
and [actual reef depth/turn + corrected ROV diagnostic](status/auv-2026-10-15.md)
are implemented. Preserve the installed Fortress 6.18 mass-sign convention,
explicit trim, source notices, forward-flow constraints and coasting Stop.
Independent marine sensing/SLAM/routes, fleet queues/RMF/coverage/multi-FCU
and remaining forms still precede the old roadmap; broad validation is separate.

Requested October 14, 2026. The latest user instruction puts this extension
first, then resumes the previous initial implementation work. R3–R9 and their remaining acceptance stay open.
R10 defines the additional scope; a researched source or planned GUI panel is
not a runnable robot. GUI Health reads the same task ledger. Shared-world
Gazebo Bumperbot/Labbot and first ROV implementation are now partial; remaining
marine/gait integration is tracked separately. Ground robots currently use Humble's Fortress 6 server and
plugins; PX4 uses a separate Harmonic 8 runtime. Do not mix their plugin ABIs.

October 15: [the native Fleet & Swarm workflow](tutorials/fleet-and-swarm.md)
and [actual scoped diagnostic](status/fleet-2026-10-15.md) are implemented.
The current handoff moves to marine/gait initial integration, while remaining
queues/Open-RMF/coverage/multi-FCU and shared backend adapters stay open.
Do not replace these with catalog counts or broaden the one nav_empty result.

October 15 also implements the [first ROV/Marine workflow](tutorials/marine-rov.md)
and five actual water/collision worlds. [The actual tank diagnostic](status/marine-2026-10-15.md)
measures surge/depth/Stop with a simulator-pose observer. Native Tethys AUV is
also installed; independent marine sensing/estimation and volumetric missions remain separate;
do not treat a water scene or ideal pressure emulation as underwater SLAM.

October 15 also implements [six/eight/many-legged Gait controls](tutorials/multi-leg-gaits.md):
a pinned MIT source hexapod and separate authored octopod/centipede models,
source IK, bounded physical PD and measured contact/tilt guards. Normal GUI
nav_empty forward/Stop diagnostics exist for each. [Receipt and gaps](status/gaits-2026-10-15.md).
Remaining marine and fleet/swarm features precede the older roadmap;
terrain/recovery, independent legged estimation/SLAM/navigation and other
backends remain separate initial integrations/qualification.

## Deliver in this order

| Task | Concrete outcome | First implementation target |
|---|---|---|
| R10.1 | Several individually controlled robots in one physical world and ROS clock | Bumperbot + Labbot, Gazebo; per-robot names, TF, sensors, controllers and Nav2 |
| R10.2 | Fleet assignments, task queue, traffic coordination, pause/resume/cancel and robot health | Reuse Open-RMF through adapters to the existing real Nav2 actions |
| R10.3 | Swarm formation, rendezvous, dispersion, flocking and cooperative coverage | Ground robots first; PX4 multiple vehicles after distinct FCU identities/ports/leases exist |
| R10.4 | A complete ROV and AUV with real submerged dynamics and depth controls | Gazebo buoyancy/thrusters/hydrodynamics; source-matched controllers and force bounds |
| R10.5 | Underwater environments, sensing, estimation, 3D mapping and navigation | Tank, seabed/reef, harbor pilings, cave/tunnel and wreck inspection worlds |
| R10.6 | Complete six- and eight-legged robots with stable source-matched walking | Reviewed hexapod first; octopod with its own model/controller contract |
| R10.7 | Segmented many-legged/centipede robot and additional forms | Procedural complete segmented robot if a licensed maintained source is unavailable; snake, tracked, surface/amphibious variants |
| R10.8 | Native GUI workflows, taxonomy, tutorials and reproducible experiment adapters | Fleet/Swarm and Marine control panels, complete model families and capability-based commands |

Implement one complete model and usable workflow per family before increasing
asset counts. Follow the current implementation-first priority: build real
controllers/workflows, then hand off broad validation. Keep source bounds,
Stop, stale-command handling and truthful status in the initial implementation.

## Upstream candidates researched on October 14

These are sources to inspect and pin, not installed Robot Lab integrations.
Resolve their exact licenses, submodules, model resources, release compatibility
and checkpoint availability before provisioning. Store checkouts, builds,
assets and trials on `/workspace`, with `scripts/ssd_env.sh`. Agents provision
models into the existing registry; do not add user-facing download buttons.

| Area | Primary source | Reuse and compatibility work |
|---|---|---|
| Fleet coordination | [Open-RMF demos](https://github.com/open-rmf/rmf_demos), [fleet adapter template](https://github.com/open-rmf/fleet_adapter_template) | Traffic/task coordination and adapter APIs. Current demos target Kilted/Ionic; select compatible Humble dependencies and simulator branches. Existing Robot Lab Nav2 remains the vehicle controller. |
| Ground swarm | [ROS2swarm](https://github.com/ROS2swarm/ROS2swarm) | Inspect distributed behaviors and message contracts; verify the chosen ROS/Gazebo release and avoid assuming classic-Gazebo launch files run in Harmonic. |
| Drone swarm | [PX4 v1.16 multi-vehicle Gazebo guide](https://docs.px4.io/v1.16/en/sim_gazebo_gz/multi_vehicle_simulation), [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) | PX4 supports distinct vehicle instances. Keep Crazyflie and X500/FCU contracts separate; controller or model interchange requires an explicit adapter. |
| Native marine physics | [Gazebo Harmonic underwater tutorial](https://gazebosim.org/api/sim/8/underwater_vehicles.html) | Buoyancy, thrusters, hydrodynamics and fin models. Verify the local physics engine/SDF/plugin versions before choosing force parameters or added-mass support. |
| Marine sensing/dynamics | [Stonefish](https://github.com/patrykcieslak/stonefish), [ROS 2 interface](https://github.com/patrykcieslak/stonefish_ros2) | Specialized marine simulation and scenario/message interfaces. Assess arm64/OpenGL/ROS compatibility and licenses; an optional backend is distinct from porting marine dynamics to the existing four backends. |
| ROS hexapod | [Tiffany Gazebo](https://github.com/SENAI4LIFE/tiffany_gazebo), [Hexapod ROS 2](https://github.com/ariegweomamerie/hexapod_ros2) | Complete described hexapod, gait/IK and ROS control/navigation sources; these target Jazzy, so a Humble port and model/license review are required. |
| MuJoCo hexapod | [C-1N spider](https://github.com/haidmoham/spider) | Inspect exact standing/walking model and named policy artifacts. License/checkpoint audit is unresolved; upstream performance claims do not qualify Robot Lab. |
| Hexapod geometry/IK reference | [mithi hexapod](https://github.com/mithi/hexapod) | Kinematics reference only; browser animation does not establish physical walking. |

The selected hexapod source is now pinned at
`b146f84d58123caabfcea8fe2a5e2a353acc33de`, with retained MIT notice, original
CAD and unchanged KDL conversion. Robot Lab uses an authored MuJoCo gait/PD
adapter, not upstream Jazzy/Harmonic runtime claims. No suitable licensed
octopod/centipede source was selected: the complete first-party primitive
models have distinct eight- and twelve-leg configurations and documented
design limits. Their segments/limbs stay under complete families. Count legs,
not motors, and do not reuse a learned four-leg policy by padding vectors.

## R10.1: shared-world sessions are the foundation

The existing experiment queue isolates independent simulations. It does not
implement a fleet whose robots collide, negotiate space and observe each
other in the same world.

Add a session manifest with a selected canonical world, one backend/physics
clock, unique entity ids and a list of robot profiles with non-overlapping
spawn poses. Reuse existing asset resolution and controlled physics plants.
Introduce namespaces through bringup, controllers, sensors, map/localization,
Nav2, reset and GUI commands. Audit hard-coded absolute topics and frame names.
Each robot owns its odom/base/sensor frames; shared map/world transforms are
explicit. A namespace alone does not rename TF frames or Gazebo entities.

Expose measured per-robot pose, state age, motion/controller status, owned
task/action and resource use. Single-robot Drive should select an instance;
group commands must require explicit group membership and controller readiness.
All-stop cancels actions and clears latched manual commands for every selected
member. Reset acts on a documented robot/session scope without resetting other
members accidentally. Record contacts, not only trajectory intersection plots.

For PX4, implement unique FCU instances/system ids, command routing, ports,
leases, model bindings and SSD run directories before removing the current
instance-0 exclusivity. Each vehicle has its own takeoff/landing/heartbeat
and frame-offset contract. Preserve `-d`, bounded logs and owned cleanup. A
single setpoint publisher must never send one vehicle's target to another FCU.

## R10.2–R10.3: fleet management and swarm behaviors

Fleet management dispatches missions and coordinates resources. Swarm control
coordinates movement from member/neighbor observations. Implement them as
separate components sharing the same real session/robot interfaces.

Start an Open-RMF adapter for two existing differential bases. Map its task,
navigation and pause/cancel callbacks to real Robot Lab/Nav2 actions. Report
actual measured robot state and task results. If battery simulation or charging
physics is absent, mark energy unknown; do not fabricate battery percentages,
completed deliveries or charging success. Add traffic/door/lift annotations
only where world infrastructure and a real adapter exist.

Implement swarm behaviors with frame-consistent, timestamped neighbor inputs,
per-robot acceleration/speed limits, collision-aware commands and bounded
communication-loss behavior. Formation targets are algorithm output, not body
pose writes. Include leader loss, member removal/join, stalled members and
obstacle separation. Each behavior records its actual kernel/upstream revision
and inputs; a configuration name alone does not implement an algorithm.

## R10.4–R10.5: underwater robots and environments

Provision at least one complete ROV and one AUV: hull collision/inertia,
displaced volume/center of buoyancy, thruster placement/direction, signed
thrust curves/limits, fins where present, frames and sensors. Visual models
without water forces stay Display-only. Marine modes require positive backend
capability checks for buoyancy, drag/added mass, current and actuator dynamics.
Porting those forces to MuJoCo/PyBullet/Isaac is explicit implementation work;
do not infer it from mesh import or a blue world background.

Keep ROS world coordinates/frame conventions explicit and convert NED/body
coordinates at the adapter boundary. GUI Marine controls include the familiar
latched directional controls, Depth Up/Down, heading/depth hold, surface/stop
and bounded thruster commands. Enabling the panel publishes no travel command.
Implement depth and attitude feedback before submerged waypoint navigation.

Worlds need water volumes/surface level, seabed bathymetry and real collision
obstacles, with current/turbidity/light profiles and reset definitions. Marine
world tags distinguish water from collision solids and prevent aerial/ground
spawn logic from placing vehicles inside the seabed. Generate appropriate 3D
maps/bathymetry; a flat 2D occupancy grid does not represent free water volumes,
caves or overhangs. Keep water dynamics in source conversion manifests.

Integrate measured pressure/depth, IMU, DVL/velocity and sonar/camera streams
where the source model/backend supplies them. Navigation and localization
cannot depend on invented underwater GPS. Sensor-source and truth-based
diagnostic modes are separate. Reuse 3D planning and filter kernels only after
their marine units, force/time limits and sensor inputs are actually connected.

## R10.6–R10.7: six/eight-legged and segmented robots

Count physical legs, not joints. Preserve source link masses, collisions,
joint directions/ranges, contact surfaces and actuator laws. A six-legged
tripod policy cannot be applied to eight legs by padding its input vector.
Each complete variant gets its own controller/model identity.

Start with stable stance and source-bound joint/foot commands, then forward,
reverse, turning, lateral stepping where supported, neutral enablement, Stop,
heartbeat loss and reset. Implement real IK and phase-based tripod/ripple/wave
or alternating-tetrapod gait kernels, using measured body/foot contacts and
tilt/effort interruption. Preserve earlier failed walking/terrain evidence.

A centipede model needs articulated body segments and several real leg pairs,
with wave phase, segment-coupling limits, body clearance and contact-aware
turning. Provide segment/leg count as a source-generated variant, not separate
partial limbs crowding the robot list. Snake, tracked, surface-vessel and
amphibious forms can use the same model-family/controller/readiness machinery.

After motion, connect actual sensors, odometry/estimation, saved/reloaded SLAM
and obstacle navigation. Foot/trunk geometry, dynamic footprint and reachable
terrain matter; ground occupancy alone does not establish a usable foothold.

## R10.8: GUI and evidence contract

Add native Fleet/Swarm and Marine panels as runnable workflows arrive. Keep
queued extensions visible in Health/roadmap and unavailable in launch/control
until their runtime adapters exist. Group complete robot families and maps;
add structural tags for ROV/AUV/USV, hexapod/octopod/many-legged, segmented and
tracked models. Fleet membership is an instance/task property, not a robot
structural category. Commands autofill from actual compatible capabilities.

Extend experiment producers to shared-world fleet/swarm and marine/legged
workloads. Retain actual robot/model/world/backend/algorithm revisions, requested
and applied seeds, measured contacts/clearance/mission outcomes, communication
loss, resets, resource budgets and owned cleanup. Later validation covers
two-robot conflict resolution, member loss, underwater station keeping/depth
and current rejection, terrain gait stability, object/obstacle contacts, and
class-appropriate saved-map missions. No new runtime support is claimed by
this planning document.
