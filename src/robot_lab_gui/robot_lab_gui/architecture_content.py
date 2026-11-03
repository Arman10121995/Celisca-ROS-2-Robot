"""Source-linked teaching model. Designed wiring is never a live ROS observation."""

COMPONENTS = {}


def component(key, title, layer, summary, detail, sources, guide='docs/architecture/overview.md', ports=''):
    COMPONENTS[key] = dict(id=key, title=title, layer=layer, summary=summary,
                           detail=detail, sources=sources.split(), guide=guide, ports=ports)


component('gui', 'Control center', 'Interface', 'Select, launch, control and learn in one native Tk workspace.',
    'Launch selects the exact robot variant, map, backend, mode and algorithms. The right control column changes with robot class. '
    'The command preview is the reviewable launch request. Registry inspects assets; Fleet and Aerial Swarm create shared-world sessions; '
    'Health explains architecture and status. Merely opening a control page must remain neutral. Buttons that move a robot require the appropriate live ownership/readiness.',
    'src/robot_lab_gui/robot_lab_gui/launcher.py src/robot_lab_gui/robot_lab_gui/workspace_ui.py', 'docs/tutorials/gui-workspace.md')
component('profiles', 'Profiles & capabilities', 'Configuration', 'Robot, world, mode and algorithm selections become explicit configuration.',
    'robots.yaml declares physical/control profiles. sim_maps.yaml chooses world assets, occupancy and spawn conventions; sim_modes.yaml chooses stacks. '
    'Grouping nests variants/components; taxonomy supplies structural tags. Neither robot tags nor an installed mesh grants working SLAM/navigation. '
    'Experimental candidates and exact source-qualified support are separate. Inspect effective overlays instead of assuming every backend shares one controller.',
    'src/robot_lab_robots/config/robots.yaml src/robot_lab_bringup/config/sim_maps.yaml src/robot_lab_bringup/config/sim_modes.yaml src/robot_lab_utils/robot_lab_utils/mode_capability.py')
component('registry', 'Typed registry', 'Configuration', 'Catalogs link robots, environments, algorithms, scenarios and experiments.',
    'The registry validates schema and references. The shared resolver produces a manifest and launch command for compatible selections. '
    'Legacy robot profiles and typed registry catalogs coexist; aliases and adapters reconcile them. A valid manifest is a reproducible request, '
    'not evidence that a robot completed a mission. Browse source/provenance and the support ledger for the exact combination.',
    'src/robot_lab/robot_lab_registry/robot_lab_registry/catalog.py src/robot_lab_adapter/robot_lab_adapter/resolver.py')
component('launch', 'Bringup & process ownership', 'Orchestration', 'Resolve selections, start the correct stack and own its shutdown.',
    'simulated_robot.launch.py is the main profile-driven route. It dispatches a backend and mode, applies robot/map/algorithm overlays and starts compatible nodes. '
    'Fleet and aerial sessions use dedicated shared-world launch paths rather than one independent simulator per member. '
    'Readiness involves active controllers/actions, advancing time, sensors and TF. Stop must cancel owned work and clean owned descendants; it must not kill unrelated ROS processes.',
    'src/robot_lab_bringup/launch/simulated_robot.launch.py src/robot_lab_gui/robot_lab_gui/process_control.py')
component('assets', 'Physical robot assets', 'Assets', 'URDF/Xacro/MJCF, meshes, joints, sensors and source limits.',
    'A complete robot includes geometry, inertias, collisions, articulations, limits and sensors. Vendor parts live under their complete family. '
    'Backend import must preserve frames, native joint conventions and actuator envelopes. Changing a source model can invalidate a controller/policy certificate. '
    'Registry 3D preview is inspection; a physically stable display and commanded movement are separate runtime checks.',
    'src/robot_lab_robots/config/robots.yaml src/robot_lab_utils/robot_lab_utils/robot_description.py src/robot_lab_gui/robot_lab_gui/registry_preview.py', 'docs/ASSET_EXTENSION_GUIDE.md')
component('world', 'Worlds & occupancy', 'Assets', 'Physical geometry and navigation grids must describe the same place.',
    'World assets supply surfaces, obstacles, actors and scale/origin. A 2D occupancy grid is a height/task-specific projection, not the whole 3D world. '
    'Robot footprint, free spawn, map origin and physical support must agree. World conversion is backend-specific; static geometry does not reproduce moving gates or underwater forces. '
    'Worlds offers map generation; the facility has distinct L0/L1 grids and guarded physical lift access.',
    'src/robot_lab_bringup/config/sim_maps.yaml src/robot_lab_utils/robot_lab_utils/sdf_world.py src/robot_lab_maps/tools/arena_clearance.py', 'docs/ASSET_EXTENSION_GUIDE.md')
component('plant', 'Simulator plant', 'Simulation', 'Physics advances the robot and produces actual simulated measurements.',
    'Gazebo, MuJoCo, PyBullet and Isaac have different import, contact, actuator and sensor semantics. '
    'Ground/marine shared sessions use Fortress; native PX4 uses Harmonic in its own workflow. Source-specific bridge/controller adapters unify useful ROS contracts, '
    'not every engine feature. A physics import or rendered model does not qualify movement or navigation.',
    'src/robot_lab_description/launch/gazebo.launch.py src/robot_lab_mujoco/python/robot_lab_mujoco/mujoco_spawner.py src/robot_lab_pybullet/python/robot_lab_pybullet/pybullet_spawner.py src/robot_lab_isaac/python/robot_lab_isaac/isaac_spawner.py')
component('bridge', 'Engine ↔ ROS adapters', 'Simulation', 'Translate native sensors, commands, clocks and observed truth.',
    'Gazebo uses ros_gz bridges and native control plugins; other spawners publish equivalent class-specific ROS data. '
    'Conversion must preserve units, frame IDs, camera optical axes and timestamps. Isaac can involve a separate runtime process. '
    'Transport discovery is distinct from ROS discovery: GZ_PARTITION / IGN_PARTITION isolate Gazebo transport, while ROS_DOMAIN_ID isolates DDS domains.',
    'src/robot_lab_bringup/launch/shared_world.launch.py src/robot_lab_isaac/launch/isaac_simulator.launch.py src/robot_lab_utils/robot_lab_utils/sim_frames.py')
component('clock', 'Simulation time', 'ROS contract', 'A shared /clock coordinates simulation-time consumers.',
    'use_sim_time makes a node use the simulation clock. Paused simulation is different from a failed wall-time heartbeat. '
    'Sensor stamps and TF must describe the same time; extrapolation can occur even when topic names are correct. '
    'Clock resets invalidate old buffers, tasks and observations. A shared-world fleet has one plant clock; independent trials require deliberate ROS/transport isolation.',
    'src/robot_lab_utils/robot_lab_utils/fleet_controller.py src/robot_lab_bringup/launch/shared_world.launch.py', ports='/clock : rosgraph_msgs/msg/Clock')
component('sensors', 'Sensor streams', 'ROS contract', 'Scan, RGB-D, IMU and joint feedback feed different algorithms.',
    '2D mapping/localization needs a useful planar scan and odometry. RGB-D mapping additionally needs synchronized images/depth/camera calibration and correct optical TF. '
    'Missing or stale inputs cannot be fixed by enabling a GUI mode. Sensor data often uses best-effort QoS; requested/offered QoS must be compatible. '
    'Joint states drive robot_state_publisher so articulated motion is visible in RViz.',
    'src/robot_lab_description/scripts/gz_pointcloud_to_optical.py src/robot_lab_mapping', ports='/scan : sensor_msgs/msg/LaserScan\n/joint_states : sensor_msgs/msg/JointState\nRGB/depth : sensor_msgs/msg/Image; calibration : CameraInfo\nExact names vary by profile/namespace.')
component('tf', 'TF frame ownership', 'ROS contract', 'Every transform edge needs one authoritative publisher.',
    'Typical ground chain: map → odom → base_footprint → sensor links. Localization/SLAM owns map→odom; controller/estimator owns odom→body; '
    'robot_state_publisher uses joint states and the robot description for body→links. Fleet facility adds map→alpha/map for vertical floor placement, '
    'then AMCL owns alpha/map→alpha/odom. Namespacing topics does not automatically prefix frame IDs. Duplicate TF owners and mismatched stamps cause jumps.',
    'src/robot_lab_utils/robot_lab_utils/fleet_infrastructure.py src/robot_lab_utils/robot_lab_utils/fleet_session.py', 'docs/tutorials/fleet-infrastructure.md', '/tf : tf2_msgs/msg/TFMessage\n/tf_static : static transforms; matching durability matters.')
component('odom', 'Odometry & estimation', 'Estimation', 'Estimate motion from actual wheel/contact/IMU/visual measurements.',
    'Odometry provides a locally continuous motion estimate; localization corrects global drift. Wheeled, rolling snake, marine and contact-gait robots use different measurement models. '
    'Keep controller odometry distinct from simulator body truth. A truth observer may measure an endpoint but must not silently become the estimator. '
    'Check covariance, heading convention, timestamp freshness and the actual TF owner when a map appears to move around the robot.',
    'src/robot_lab_localization src/robot_lab_utils/robot_lab_utils/fleet_controller.py', ports='/odom : nav_msgs/msg/Odometry\n/odom/ground_truth : separate simulator observer (profile-specific).')
component('amcl', 'Known-map localization', 'Estimation', 'AMCL aligns planar scans with an existing occupancy grid.',
    'AMCL estimates map→odom from scan/map observations and motion. It needs the correct floor grid, robot scan frame and initial pose. '
    'Localization does not create a map and does not implement a gait. During a lift ride the planar observation model is invalid; the facility pauses only the AMCL scan relay, '
    'retains the odometric XY prior and waits for map/costmap acknowledgments before releasing motion.',
    'src/robot_lab_localization src/robot_lab_utils/robot_lab_utils/fleet_session.py', 'docs/tutorials/fleet-infrastructure.md', 'AMCL pose / map→odom TF\nFleet AMCL input: /<member>/localization_scan; raw obstacle scans remain live.')
component('slam2', '2D SLAM', 'Estimation', 'SLAM Toolbox builds a planar occupancy map while estimating pose.',
    'Drive supplies motion; scans and odometry allow the mapper to register observations and close loops. '
    'The mapper owns the global correction TF in this mode. Save a real occupancy image/YAML; next launch can localize against it. '
    'A loaded 3D world alone is not a saved 2D map, and a map export is separate from a successful saved-map navigation task.',
    'src/robot_lab_mapping src/robot_lab_bringup/config/sim_modes.yaml', ports='scan + odometry/TF → occupancy + map correction TF')
component('slam3', 'RGB-D / 3D mapping', 'Estimation', 'RTAB-Map builds a database and spatial representation from calibrated observations.',
    'Images, depth, camera information and capture-time transforms feed visual/RGB-D mapping. '
    'RTAB-Map stores actual keyframes/graph data; export and protected reload must preserve a populated database. '
    'A point cloud view is not automatically a usable navigation map. Marine, aerial and ground workflows add different source-heading and sensor constraints.',
    'src/robot_lab_mapping src/robot_lab_bringup/scripts/export_3d_map.py', 'docs/tutorials/marine-rov.md', 'RGB + depth + CameraInfo + odometry/TF → database / point cloud / optional occupancy')
component('goal', 'Task / RViz goal', 'Planning', 'A desired destination enters an asynchronous action workflow.',
    'A Nav2 goal is a PoseStamped plus an action lifecycle: accepted, feedback, result or cancellation. '
    'RViz uses the configured goal frame. Fleet tasks and native mobile workflows wrap real action results with independent settled-body gates. '
    'Goal accepted is not goal reached; a Nav2 success alone does not establish a collision-free, settled mission.',
    'src/robot_lab_navigation src/robot_lab_utils/robot_lab_utils/fleet_controller.py', ports='NavigateToPose / NavigateThroughPoses actions for compatible Nav2 workflows')
component('planner', 'Global planner', 'Planning', 'Find a feasible route through the global costmap.',
    'Nav2 planner plugins consume the selected map, estimated pose and robot footprint. Source-aware car overlays impose turning constraints. '
    'An algorithm selector must change the effective plugin/parameters. A Path is a geometric plan, not motor torque. '
    'Occupied/unknown cells, wrong floor maps, an oversized footprint or a disconnected free region can make a valid goal unreachable.',
    'src/robot_lab_navigation src/robot_lab_planning src/robot_lab_adapter/robot_lab_adapter/resolver.py', ports='nav_msgs/msg/Path + costmap → path follower')
component('follower', 'Local controller', 'Planning', 'Track the route while reacting to nearby obstacles and kinematic constraints.',
    'Nav2 controller plugins combine a global path, local costmap and estimated motion to produce velocity commands. '
    'Steering, holonomic and differential robots need suitable controllers/critics. Behavior-tree recovery may clear, wait, rotate or replan. '
    'A stopped controller may indicate inactive lifecycle, stale TF, an interlock or an infeasible path; inspect the actual action and node states.',
    'src/robot_lab_navigation src/robot_lab_motion', ports='Path / costmap → geometry_msgs/msg/Twist (or workflow-specific command)')
component('drive', 'Drive input & arbitration', 'Control', 'Latched GUI/keyboard/joystick commands pass through ownership and limits.',
    'GUI Drive uses ramped linear/angular increments and explicit input enablement. Ground manual input commonly enters /key_vel before the selected multiplexer/controller route. '
    'Autonomous and manual publishers must not compete for one actuator. Fleet sessions use an operator lease and member-specific commands. '
    'Stop clears latched intent, cancels tasks and applies the controller-specific safe behavior; losing a publisher must not imply continued motion.',
    'src/robot_lab_gui/robot_lab_gui/drive_control.py src/robot_lab_controller src/robot_lab_utils/robot_lab_utils/fleet_controller.py', 'docs/tutorials/gui-workspace.md', '/key_vel : geometry_msgs/msg/Twist (ordinary ground manual route)\nFleet uses per-member control and /fleet/* ownership messages.')
component('actuator', 'Actuation adapters', 'Control', 'Convert valid commands into native wheel/joint/rotor/thruster forces.',
    'Wheels may use ros2_control velocity interfaces or engine-native efforts. Four-wheel steering resolves wheel angles/rates; mecanum uses source roller geometry. '
    'Legged policies generate bounded joint efforts, arms use source-native actuator contracts and marine robots allocate thrust/fins. '
    'A cmd_vel topic by itself does not prove any native actuator is active. Joint feedback and actual body response close the loop.',
    'src/robot_lab_controller src/robot_lab_utils/robot_lab_utils/drive_kinematics.py src/robot_lab_mujoco/python/robot_lab_mujoco/joint_effort.py')
component('policy', 'Legged policies & gaits', 'Control', 'Robot-specific observations become bounded joint actions.',
    'Imported/trained policies require the exact robot, joint order, observation normalization, control period and PD/action scaling. '
    'Multi-leg gait controllers instead use source FK/contact support and an authored gait. Passive standing, a loaded checkpoint, walking, terrain recovery and Nav2 integration are distinct stages. '
    'Preserve source limits and expose experimental policy selection honestly; never route wheel commands to a humanoid and assume it walks.',
    'src/robot_lab_adapter/robot_lab_adapter/humanoid_policy_controller.py src/robot_lab_gui/robot_lab_gui/gait_controls.py src/robot_lab_gui/robot_lab_gui/training_tab.py', 'docs/tutorials/multi-leg-gaits.md')
component('terrain', 'Measured low-relief adaptation', 'Control', 'Observed support ranges become bounded source-foot corrections.',
    'MuJoCo, Bullet and PhysX use native collision rays; the Fortress adapter uses a labeled selected-static-world range model and native contact forces. '
    'Source joint frames, mass, effort and position bounds are retained. Freshness, 2 cm differential reach, support, slip and body/tilt gates can stop the gait. '
    'The source hexapod, octopod and centipede have backend adapters; each measured mission remains a separate support claim. A passive-wheel snake uses its separate MuJoCo adapter.',
    'src/robot_lab_utils/robot_lab_utils/gait_terrain.py src/robot_lab_utils/robot_lab_utils/gait_gazebo_adapter.py src/robot_lab_maps/systems/gait_contact_frame.cc src/robot_lab_isaac/python/robot_lab_isaac/gait_observation.py', 'docs/tutorials/curves-terrain-concurrency.md',
    '/gait/terrain : observed range JSON\n/gait/contacts : actual force JSON\n/gait/odometry_state : independent contact/FK consistency\n/gait/joint_targets : source-order Float64MultiArray')
component('arm', 'Arm planning & Servo', 'Manipulation', 'Joint targets, Cartesian planning and incremental end-effector controls.',
    'A 3D target is interpreted in a named planning frame. MoveIt/native planning uses source joint limits and collision geometry. '
    'Cartesian jog/Servo and taught poses help choose reachable targets; IK failure can mean unreachable pose, orientation or collision constraints. '
    'Preview and explicit Execute are separate. Actual native action feedback and measured joints, not the requested coordinates, establish a reach.',
    'src/robot_lab_gui/robot_lab_gui/cartesian_controls.py src/robot_lab_gui/robot_lab_gui/native_planning_controls.py src/robot_lab_bringup/scripts/native_arm_scene.py', 'docs/tutorials/native-arm-planning.md', 'FollowJointTrajectory / MoveIt requests; exact native interface depends on profile.')
component('hand', 'Hands & mobile tasks', 'Manipulation', 'Source hand couplings and coordinated base/arm/hand sequences.',
    'Hand controls respect coupled tendons, finger limits and native actuator mappings. A grasp requires actual object/contact behavior, not just a closed-finger pose. '
    'Mobile manipulation adds base ownership and frame changes; taught sequences store measured articulation states and explicit transitions. '
    'Stop invalidates pending execution. Stationary arms do not acquire ground navigation simply by being registered.',
    'src/robot_lab_gui/robot_lab_gui/native_hand_controls.py src/robot_lab_gui/robot_lab_gui/mobile_task_controls.py', 'docs/tutorials/native-hand-controls.md')
component('px4', 'PX4 flight controller', 'Aerial', 'The FCU controls a real simulated X500 rotor plant.',
    'The native Flight route starts PX4 SITL and Gazebo Harmonic. Offboard control, estimator health, arming, takeoff clearance, setpoints, landing and disarm are different stages. '
    'The current Robot Lab ROS controller uses MAVLink via pymavlink; do not assume its active link is a generic PX4 DDS bridge. '
    'Native plant truth is a separate observer. Multi-FCU sessions require unique system IDs, ports, namespaces and owned lifetime.',
    'src/robot_lab_adapter/robot_lab_adapter/px4_sitl_runtime.py src/robot_lab_adapter/robot_lab_adapter/px4_ros_controller.py', 'docs/tutorials/px4_x500.md', 'Ordinary FCU endpoint defaults to udpout:127.0.0.1:14580; inspect resolved session for actual ports.')
component('swarm', 'Aerial swarm owner', 'Aerial', 'One selected world, multiple independently identified FCUs.',
    'The aerial session model assigns member IDs, PX4 instances, MAVLink ports and ROS namespaces. '
    'Formation/workload requests become per-vehicle setpoints while the runtime monitors member telemetry and operator ownership. '
    'The native itinerary editor separates background source/swept-peer Plan from explicit Execute. Advancing FCU timestamps and low estimated speed gate each formation slot; Stop/loss invalidates the route generation. '
    'A two-drone measured case is not a larger-swarm certificate. Landing on loss differs from simply disarming in mid-air; source flight checks remain active.',
    'src/robot_lab_bringup/launch/aerial_swarm.launch.py src/robot_lab_adapter/robot_lab_adapter/aerial_swarm_model.py src/robot_lab_adapter/robot_lab_adapter/aerial_swarm_runtime.py src/robot_lab_adapter/robot_lab_adapter/aerial_swarm_route.py', 'docs/tutorials/formation-routes.md', 'Default session member ports 14580 + instance; distinct system IDs and namespaces.')
component('aerial_plan', 'Aerial scene planning', 'Aerial', 'Selected-world geometry and observed depth constrain explicit route execution.',
    'Takeoff checking binds a landed pose and source envelope to a proposed corridor. Static/observed 3D planning and live lookahead can interrupt stale or blocked execution. '
    'Direct manual/offboard paths have different guarantees from an explicitly checked plan. Actor prediction, unknown geometry and broader maps require their own implementation/evidence. '
    'A planned route is never proof of completed flight.',
    'src/robot_lab_utils/robot_lab_utils/aerial_planning.py src/robot_lab_utils/robot_lab_utils/aerial_takeoff.py src/robot_lab_gui/robot_lab_gui/drone_planning_controls.py', 'docs/tutorials/mapped-drone-planning.md')
component('fleet', 'Ground fleet owner', 'Fleet', 'Member queues, ownership, traffic and real Nav2 results in one world.',
    'shared_world.launch.py creates one Fortress plant/clock with named Bumperbot/Labbot instances and per-member controllers/Nav2. '
    'The fleet controller arbitrates manual input, queues, reservations and operator heartbeats. State includes task identity, actual acknowledgments and separate truth/estimated poses. '
    'Concurrent routes retain per-member acknowledgments and measured settling. Fresh observed floors separate planar traffic; post-lift parking uses source-clear Nav2 goals. '
    'A shared world is essential for physical interactions; several isolated simulations are not a fleet.',
    'src/robot_lab_bringup/launch/shared_world.launch.py src/robot_lab_utils/robot_lab_utils/fleet_session.py src/robot_lab_utils/robot_lab_utils/fleet_controller.py', 'docs/tutorials/fleet-and-swarm.md', '/fleet/state, /fleet/command, /fleet/operator_heartbeat : std_msgs/msg/String JSON\nPer-member Nav2 actions and /<member>/* sensor/control namespaces.')
component('rmf', 'Open-RMF traffic & tasks', 'Fleet', 'Optional native scheduling translates routes into actual member missions.',
    'The Humble SDK consumes a clearance-derived graph. The official building parser creates real lift-session events across L0/L1; door/lift supervisors aggregate protocol requests. '
    'The adapter converts follow-path callbacks into fleet routes and only completes from actual outcomes. A pinned process-scoped serializer correction fixes the upstream exit-event crash. '
    'SDK nominal battery costs stay disabled; explicit control epochs latch Stop/loss until re-enabled.',
    'src/robot_lab_utils/robot_lab_utils/fleet_rmf_adapter.py src/robot_lab_utils/robot_lab_utils/fleet_building_graph.py src/robot_lab_utils/vendor/rmf_graph', 'docs/tutorials/fleet-infrastructure.md', '/door_requests /door_states : rmf_door_msgs\n/lift_requests /lift_states : rmf_lift_msgs\n/fleet/rmf/state and /fleet/rmf/control : String JSON')
component('facility', 'Doors, lift & energy', 'Fleet', 'Physical interlocks, observed floors and source-bound modeled energy.',
    'Force-driven car/landing gates and lift position feedback guard boarding and travel. Body height selects each member floor; map load plus both costmap resets hold movement until acknowledged. '
    'AMCL retains XY/yaw ownership; floor placement adds only vertical translation. Car setpoints hold after request cessation. '
    'A bounded FIFO reserves native floor itineraries and waits for clear peer landings. Live SDK session identity persists until release/loss. Recovery is explicit, with whole-passenger footprint and gate/level interlocks; only empty-car recovery has a new receipt. '
    'Power uses complete actual effort/rate samples and source/runtime certificates. Berth charging and SOC are authored models; missing coverage makes SOC unknown, not zero.',
    'src/robot_lab_utils/robot_lab_utils/fleet_infrastructure.py src/robot_lab_utils/robot_lab_utils/fleet_transport.py src/robot_lab_utils/robot_lab_utils/fleet_transport_queue.py src/robot_lab_utils/robot_lab_utils/fleet_lift_recovery.py src/robot_lab_utils/robot_lab_utils/fleet_energy.py', 'docs/tutorials/fleet-infrastructure.md', '/fleet/infrastructure/state : String JSON\nPhysical fixture targets /facility/<joint>/target : std_msgs/msg/Float64')
component('marine', 'Marine force controller', 'Marine', 'ROV thrusters, AUV propulsion/fins and surface/amphibious control.',
    'Marine workflows use native buoyancy, hydrodynamics and force/fin allocation, not a ground-wheel controller. '
    'ROVs can actively trim/hold differently from forward-moving AUVs; neutral thrust may coast. Source heading, depth convention and pressure/vision assumptions must be explicit. '
    'Marine controls include bounded manual and estimated-route/map workflows. Source force bounds and native watchdogs remain part of control.',
    'src/robot_lab_bringup/launch/marine_simulator.launch.py src/robot_lab_gui/robot_lab_gui/marine_controls.py', 'docs/tutorials/marine-rov.md')
component('marine_map', 'Marine mapping & routes', 'Marine', 'Sensor-based acquisition, protected database reload and estimated missions.',
    'RGB-D acquisition exports a real database; saved localization reuses it with appropriate source frames and odometry. '
    'Ahead waypoints are interpreted by a marine mission controller rather than assuming planar Nav2 works underwater. '
    'A blocked segment publishes neutral input before cancelable replanning. Generation, live ownership and start-position checks reject late results. Tethys uses bounded Dubins curves, a straight flow run-up and a clear braking corridor; blocked AUV routes require a new explicit plan. Authored current commands are not current sensors or optimized route costs. '
    'Native body traces measure routes independently. The measured ROV/surface/AUV cases have different scope; currents, obstacles, loss and additional maps remain separate.',
    'src/robot_lab_mapping src/robot_lab_utils/robot_lab_utils/auv_routes.py src/robot_lab_gui/robot_lab_gui/marine_controls.py', 'docs/tutorials/curves-terrain-concurrency.md')
component('queue', 'Experiments & workloads', 'Evidence', 'Snapshot selected workflows and execute owned, bounded trials.',
    'The GUI queue records resolved selections and starts real producers for compatible Nav2/fleet/marine/gait/aerial cases. '
    'Concurrency needs isolated processes, ROS domains, transport partitions, clocks and artifact paths. '
    'A scheduled job is not a result; reset, actual endpoint, errors and cleanup must be observed. Seeds must be applied by the relevant simulator/algorithm, not only saved in metadata.',
    'src/robot_lab_gui/robot_lab_gui/experiment_queue.py src/robot_lab/robot_lab_benchmark/robot_lab_benchmark/extension_workload.py', 'docs/tutorials/concurrent-experiments.md')
component('evidence', 'Measurements & status', 'Evidence', 'Scope every support claim to actual source/model/map/backend/task evidence.',
    'Manifests, traces, body observations, source hashes and measured outcomes support a named case. The ledger and current status expose implemented, experimental, measured and remaining work. '
    'CI verifies builds/regressions; it does not drive every robot mission. Historical demonstration PASS flags and random metrics do not close roadmap acceptance. '
    'Ground truth may evaluate estimation or settling but must stay labeled and outside the estimator inputs.',
    'docs/status/platform-status.yaml docs/status/CURRENT_STATUS.md src/robot_lab/robot_lab_benchmark', 'docs/status/CHECKLIST.md')
component('storage', 'SSD artifacts & provenance', 'Evidence', 'Keep source, generated models, builds, caches, logs and bags on the SSD.',
    'On this Jetson /workspace is the SSD, while / and /tmp are internal eMMC. Source scripts/ssd_env.sh before large runs; explicit /tmp paths ignore TMPDIR. '
    'The old PX4 relocation is complete and compatibility symlinks should remain. Asset licenses and upstream hashes stay with sources. '
    'Unattended native PX4 uses -d; bound trials and monitor log growth. Another host must choose its own valid runtime storage configuration.',
    'scripts/ssd_env.sh docs/STORAGE.md', 'docs/STORAGE.md')
component('dds', 'ROS networking / DDS', 'Networking', 'Domain, discovery, names, types and QoS all affect communication.',
    'ROS_DOMAIN_ID selects a DDS domain; matching it is necessary but not sufficient across hosts. Reachability, discovery/interface settings, security and compatible middleware matter. '
    'Namespaces resolve relative names and frame IDs require their own naming policy. A matching topic name still needs the same message type and compatible requested/offered QoS. '
    'Gazebo partitions and MAVLink ports are separate transport boundaries. The Networking lesson changes examples only, never host configuration.',
    'docs/architecture/networking.md src/robot_lab_bringup/launch/shared_world.launch.py', 'docs/architecture/networking.md')
component('lifecycle', 'Readiness, Stop & recovery', 'Orchestration', 'A process running is only the first stage of a usable system.',
    'Resolve → validate → launch → wait for real readiness → explicit command → measured result → cancel/stop → owned cleanup. '
    'Nav2 lifecycle activation, TF, sensors and advancing clock are required before a route. Wall-time ownership watchdogs still run when simulation pauses. '
    'Stop and feedback loss invalidate pending commands; returning feedback must not silently reacquire ownership. Recovery is a new explicit, bounded operation.',
    'src/robot_lab_gui/robot_lab_gui/process_control.py src/robot_lab_utils/robot_lab_utils/fleet_controller.py', 'docs/REMAINING_WORK_GUIDE.md')


def view(key, title, subtitle, rows, links, lesson):
    positions = {}
    for row, items in enumerate(rows):
        for col, item in enumerate(items):
            positions[item] = (70 + col * 300, 90 + row * 155)
    return dict(id=key, title=title, subtitle=subtitle, positions=positions,
                edges=[dict(source=a, target=b, label=label) for a, b, label in links], lesson=lesson)


VIEWS = [
    view('overview', '01 · System at a glance', 'C4 context / containers · click a component to trace its role and sources',
         [['gui', 'profiles', 'registry'], ['launch', 'plant', 'bridge'], ['sensors', 'odom', 'actuator'], ['queue', 'evidence', 'storage']],
         [('gui','profiles','select'),('profiles','launch','resolve'),('registry','launch','manifest'),('launch','plant','start'),('plant','bridge','native state'),('bridge','sensors','ROS data'),('sensors','odom','measurements'),('actuator','plant','forces'),('queue','launch','owned run'),('queue','evidence','actual result'),('evidence','storage','artifacts')],
         'Start with configuration, then follow launch into physics. The feedback loop returns sensor measurements to algorithms; commands go through source-specific actuation. Finally record a measured outcome.'),
    view('launch', '02 · From selection to a running robot', 'Configuration → composition → lifetime; diagrammed flow is a design explanation',
         [['gui','profiles','registry'],['launch','assets','world'],['lifecycle','plant','clock']],
         [('gui','profiles','robot / map / mode'),('registry','launch','typed manifest'),('profiles','launch','overlays / gates'),('assets','plant','physical model'),('world','plant','environment'),('launch','plant','backend dispatch'),('plant','clock','advance'),('launch','lifecycle','readiness / Stop')],
         'Inspect the command before Run. Startup is complete only after the selected controller and required sensors/actions/TF are ready. Experiment exit and robot Stop have distinct ownership responsibilities.'),
    view('navigation','03 · Navigation feedback loop','Compatible ground Nav2 route · poses, plans and commands have different roles',
         [['goal','planner','follower'],['amcl','tf','drive'],['sensors','odom','actuator'],['world','bridge','plant']],
         [('goal','planner','goal action'),('world','planner','occupancy / footprint'),('amcl','tf','map→odom'),('odom','tf','odom→body'),('tf','planner','estimated start'),('planner','follower','Path'),('sensors','follower','local obstacles'),('follower','drive','autonomous command'),('drive','actuator','owned / bounded input'),('actuator','plant','wheel efforts'),('plant','bridge','physical state'),('bridge','sensors','scan / IMU'),('sensors','amcl','scan likelihood'),('bridge','odom','measured motion')],
         'Give a goal; watch the path, local tracking and actual robot response. If movement fails, follow the loop: action/lifecycle → TF/clock → costmaps → command arbitration → native actuators.'),
    view('mapping','04 · Localization versus SLAM','Choose one owner for global correction; map products must be saved and reloaded explicitly',
         [['world','amcl','goal'],['sensors','slam2','slam3'],['odom','tf','evidence']],
         [('world','amcl','known occupancy'),('sensors','amcl','planar scan'),('sensors','slam2','scan'),('sensors','slam3','RGB / depth / info'),('odom','slam2','motion prior'),('odom','slam3','odometry / TF'),('amcl','tf','global correction'),('slam2','tf','alternative owner'),('slam3','evidence','database / exports'),('slam2','evidence','saved grid'),('tf','goal','correct frame')],
         'Known-map localization consumes a map. SLAM creates a map while estimating motion. In a given mode only its selected estimator owns map→odom. A cloud, an occupancy grid and a mapping database are different artifacts.'),
    view('control','05 · Robot classes & actuation','A shared GUI does not mean every robot uses a wheel controller',
         [['gui','drive','actuator'],['policy','arm','hand'],['px4','marine','plant'],['terrain','sensors','clock']],
         [('gui','drive','manual / autonomous ownership'),('drive','actuator','wheeled inputs'),('gui','policy','exact-model policy'),('policy','actuator','joint effort'),('arm','actuator','bounded joint action'),('hand','actuator','coupled finger targets'),('gui','px4','flight commands'),('gui','marine','thrust / depth'),('px4','plant','rotors'),('marine','plant','thrusters / fins'),('actuator','plant','native actuation'),('plant','sensors','native feedback'),('sensors','terrain','support / range / slip'),('clock','terrain','fresh observations'),('terrain','policy','bounded source-foot targets')],
         'Choose the class-specific control page. Wheels, gaits, arms, hands, aircraft and marine vehicles have different limits and safe Stop behavior. Start with small explicit commands and inspect actual feedback.'),
    view('frames','06 · Frames, clocks & truth','One owner per TF edge · timestamps are part of the contract',
         [['clock','tf','sensors'],['amcl','odom','assets'],['facility','plant','evidence']],
         [('clock','sensors','stamp'),('clock','tf','common time'),('assets','tf','body→sensor links'),('amcl','tf','map→odom'),('odom','tf','odom→body'),('facility','tf','vertical floor placement'),('plant','evidence','independent body truth'),('sensors','odom','measurements'),('tf','evidence','inspect ownership')],
         'For a jumping or invisible robot, inspect TF owner, frame prefix and time before tuning navigation. Truth evaluates the system; it must not silently replace an estimate.'),
    view('fleet','07 · Fleet, doors & multi-floor lift','One physical world, separate members and explicit ownership',
         [['gui','rmf','fleet'],['world','facility','amcl'],['clock','plant','evidence']],
         [('gui','fleet','operator lease / tasks'),('rmf','fleet','real route callbacks'),('rmf','facility','door / lift sessions'),('facility','fleet','reservations / holds'),('facility','amcl','observed floor → map load'),('world','rmf','clearance graph'),('fleet','plant','member commands'),('plant','facility','joint / body observations'),('plant','clock','one clock'),('fleet','evidence','settled outcomes'),('facility','evidence','power / floor state')],
         'Transport reserves the member and car, approaches, boards, rides, changes member maps and exits. Gates are physical. SDK process-loss holds the car and invalidates the old enable epoch. A return to feedback is not automatic recovery.'),
    view('aerial','08 · Flight & aerial swarm','Separate FCUs, MAVLink identities and native rotor physics',
         [['gui','swarm','dds'],['aerial_plan','px4','bridge'],['world','plant','evidence']],
         [('gui','swarm','explicit formation / flight'),('swarm','px4','per-FCU MAVLink setpoints'),('dds','swarm','namespaced ROS control'),('aerial_plan','px4','checked execution'),('world','aerial_plan','geometry / observations'),('px4','plant','native rotor control'),('plant','bridge','truth / sensors'),('bridge','evidence','independent flight measure'),('px4','swarm','telemetry / estimator health')],
         'Inspect selected-world clearance before takeoff. Each FCU has its own system ID and UDP endpoint. Arming, takeoff, convergence, landing and disarm are separate observations.'),
    view('marine','09 · Underwater, surface & amphibious','Native force models and source-appropriate estimation',
         [['gui','marine','plant'],['world','marine_map','sensors'],['drive','odom','evidence']],
         [('gui','marine','explicit manual / mission'),('marine','plant','thrusters / fins / trim'),('world','plant','water / shore geometry'),('plant','sensors','simulated measurements'),('sensors','marine_map','RGB-D / heading / depth'),('odom','marine_map','motion estimate'),('marine_map','marine','estimated route'),('plant','evidence','independent body trace'),('drive','marine','hybrid rover input')],
         'ROV depth hold is different from AUV forward flight or amphibious wheel/thrust blending. Save and reload actual acquired maps; use estimated feedback for routing and separate body truth for evaluation.'),
    view('manipulation','10 · Manipulation & mobile tasks','Planning frame → preview → explicit execution → measured joints/contact',
         [['gui','arm','hand'],['assets','tf','actuator'],['drive','plant','evidence']],
         [('gui','arm','Cartesian jog / taught pose'),('assets','arm','joint limits / collisions'),('tf','arm','target frame'),('arm','actuator','planned native action'),('hand','actuator','coupled grasp command'),('drive','arm','coordinated mobile task'),('actuator','plant','bounded effort / targets'),('plant','evidence','measured reach / contact')],
         'A coordinate must be reachable in the chosen frame and orientation. Jog or teach a pose, inspect the plan, then Execute. Closing fingers is not the same as a stable object grasp.'),
    view('network','11 · Networking & isolation','ROS DDS, Gazebo transport and MAVLink are three different networks',
         [['gui','dds','fleet'],['bridge','plant','px4'],['clock','swarm','storage']],
         [('gui','dds','ROS domain / topic / type / QoS'),('dds','fleet','/<member>/*'),('dds','bridge','ROS side'),('bridge','plant','Gazebo partition'),('px4','swarm','MAVLink UDP + system ID'),('plant','clock','one clock per shared world'),('fleet','storage','owned run artifacts')],
         'Use the Networking playground to break and repair a teaching connection. Then inspect the live graph in the GUI process domain; a matching domain is not a firewall, rate or QoS proof.'),
    view('evidence','12 · Experiments, artifacts & readiness','Reproducibility and support claims follow real observed work',
         [['registry','queue','launch'],['lifecycle','plant','evidence'],['profiles','storage','gui']],
         [('registry','queue','resolved manifest'),('queue','launch','isolated producer'),('launch','lifecycle','start / ready / cleanup'),('plant','evidence','actual observations'),('queue','evidence','task / outcome identity'),('profiles','evidence','source / config hashes'),('evidence','storage','traces / bags / reports'),('evidence','gui','scope / remaining work')],
         'Follow one exact run from manifest to measured endpoint and saved artifacts. Unit tests, scene loading and mission success prove different things. Missing or stale evidence remains unknown, not an invented success.'),
]
VIEW_BY_ID = {v['id']: v for v in VIEWS}
LAYERS = ['Interface', 'Configuration', 'Orchestration', 'Assets', 'Simulation', 'ROS contract',
          'Estimation', 'Planning', 'Control', 'Manipulation', 'Aerial', 'Fleet', 'Marine', 'Evidence', 'Networking']

JOURNEYS = {
    'New here · follow a navigation command': [('overview','gui'),('launch','profiles'),('launch','launch'),('navigation','goal'),('navigation','amcl'),('navigation','planner'),('navigation','follower'),('navigation','drive'),('navigation','actuator'),('navigation','plant'),('evidence','evidence')],
    'Why will my robot not move?': [('launch','lifecycle'),('frames','clock'),('frames','tf'),('navigation','amcl'),('navigation','planner'),('navigation','drive'),('control','actuator'),('evidence','evidence')],
    'Make a map, then navigate': [('mapping','sensors'),('mapping','odom'),('mapping','slam2'),('mapping','slam3'),('evidence','storage'),('mapping','amcl'),('navigation','goal')],
    'Understand the lift round-trip': [('fleet','fleet'),('fleet','rmf'),('fleet','facility'),('frames','tf'),('mapping','amcl'),('fleet','evidence')],
    'Learn aircraft, arms and underwater robots': [('control','policy'),('manipulation','arm'),('manipulation','hand'),('aerial','px4'),('aerial','swarm'),('marine','marine'),('marine','marine_map')],
    'Debug a network connection': [('network','dds'),('frames','clock'),('frames','tf'),('network','bridge'),('network','px4'),('evidence','lifecycle')],
}


def repository_packages(root):
    """Top-level maintained package inventory, without treating vendored parts as stacks."""
    import xml.etree.ElementTree as ET
    paths = sorted((root / 'src').glob('*/package.xml')) + sorted((root / 'src/robot_lab').glob('*/package.xml'))
    rows = []
    for path in paths:
        try:
            xml = ET.parse(path).getroot()
            rows.append(dict(name=xml.findtext('name'), path=str(path.relative_to(root)),
                             description=(xml.findtext('description') or '').strip(),
                             dependencies=sorted({e.text.strip() for e in xml if e.tag.endswith('depend') and e.text})))
        except (OSError, ET.ParseError):
            continue
    return rows
