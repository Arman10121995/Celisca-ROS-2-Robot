# Project goal and integration audit — 2026-09-15

The user wants a working robot laboratory for learning, exercising and comparing
robots, maps and algorithms. Retain Bumperbot and Labbot, and deliver a humanoid,
multirotor, quadruped and four-wheel robot with Ackermann and reverse Ackermann
steering. Reusing working upstream implementations or developing missing code is
authorized. More diverse maps are part of the deliverable.

"Working" requires physical simulation, usable control, actual sensor output,
class-appropriate tasks, limits, stop/reset behavior, and repeatable measurements
through the shared CLI/GUI workflow. Model import, controller unit tests and
catalog entries establish narrower facts. Compatible modes include display,
teleoperation, mapping, autonomous tasks and benchmarking; flight and legged
terrain require suitable representations. An unsupported combination must report
why it cannot run. Physical hardware operation remains a separate milestone.

## Current evidence at checkout `2cb2bfe`

| Requirement | Current evidence | Remaining acceptance |
|---|---|---|
| Two differential-drive robots | Existing Bumperbot/Labbot workflows and description checks; Bumperbot drive/RGB-D/reset checks and five open-arena trials each on MuJoCo and PyBullet | Fresh Labbot missions and obstacle-world tests; distinguish deterministic repeated trials from randomized conditions |
| Humanoid | Berkeley Humanoid Lite model imports; 79 balance/description unit checks | Simulated stance and perturbation recovery, commanded walking/turning/stopping, fall handling, then terrain |
| Quadruped | Go2 model imports and 61 locomotion-core unit checks | Measured stance, walking, turning, stop and terrain; R5.2 reopened from done to partial |
| Drone | Quadrotor description and offboard wrapper | Real thrust/rotor and FCU-SITL integration, takeoff/hover/3D waypoints/landing/failsafe |
| Four-wheel robot | No current Ackermann robot/control integration found | Model, steering, wheel dynamics/odometry, sensors and complete driving missions (R5.5) |
| Maps | 26 existing environment entries; assets and conversion checks | Class-specific geometry/route/reset qualification and at least six diverse additions (R6.4) |
| Algorithms and modes | Shared composition/CLI/GUI and algorithm entry points exist | Physically compatible robot-specific execution and measured comparisons of distinct methods |

Reverse Ackermann is ambiguous: rear-wheel steering and anti-Ackermann geometry
are different mechanisms. A clarification was requested. Neither should be
silently substituted with driving a normal Ackermann vehicle backwards.

## Upstream integration candidates

These primary sources were checked on 2026-09-15. They are candidate foundations,
not claims of successful integration on this ARM64/Humble host. Pin commits,
dependencies, model versions, checkpoint hashes and applicable licenses before
adoption; compare joint order, actuator limits and observation conventions.

- **Humanoid:** [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite)
  includes training, sim-to-sim validation and deployment code, with robot assets
  and checkpoints. Its code is MIT; other assets have separate CC BY-SA terms.
  A source copy already exists under `src/robot_lab_robots/_upstream/`.
  Audit of that copy found the ONNX checkpoints, but the asset and low-level
  controller submodules named in `.gitmodules` are absent. Resolve/pin those
  dependencies and match their robot/configuration before running the policy.
- **Quadruped:** [Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym)
  documents Go2 training and a MuJoCo deployment workflow. Verify availability
  and suitability of a Go2 checkpoint rather than inferring it from humanoid
  examples. [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco)
  supplies a low-level simulation interface; that interface alone is not a gait.
- **Drone:** [PX4 Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
  provides flight simulation and custom-world selection. Choose a pinned PX4/Gazebo
  version that runs on this host, and connect world/sensor/frame/reset contracts.
  The lab currently launches Fortress for Gazebo; do not assume its installation
  matches the requirements of PX4's current development branch.
- **Four-wheel control:** [ROS 2 Control's Humble Ackermann controller](https://control.ros.org/humble/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html)
  is a candidate for conventional steering. Verify its backend interfaces and
  required geometry against the chosen model; qualify the reverse variant separately.

## Implementation order and completion evidence

1. Finish the current R6.1 navigation-fixture repair. The old reference routes
   cross obstacles in all four nonempty box arenas; checking waypoint occupancy
   alone missed them. Check full swept footprints against geometry and maps.
2. Continue MuJoCo humanoid balance/walking (R5.3), then Go2 locomotion (R5.2),
   obtaining one complete measured workflow per class. Reuse compatible policies
   where they provide a stronger starting point than the current analytic core.
3. Add four-wheel steering/control (R5.5) and complete flight SITL (R5.4), each
   through the common resolver and launch workflow.
4. Expand maps alongside each class: ground intersections/parking, uneven
   slopes/rubble/stairs, and indoor/outdoor 3D flight courses. Finish dynamic
   obstacle and disturbed-sensor behavior with reproducible seeds.
5. Qualify compatible mode/map/robot combinations and algorithm comparisons.
   Provide reproducible commands, configuration explanations, trajectories,
   measured outcomes and failure examples so the lab supports learning.

For each qualified cell record the revision, robot/model/controller or policy,
backend, mode, map, seed protocol, task, limits, commands, bag/trace provenance,
success/failure and metrics. Include negative cases such as command loss, falling,
collision, infeasible routes and missing data. Keep missing metrics unavailable.
At final acceptance, each class must work across at least three applicable map
families; every advertised combination requires its own evidence. The task ledger
and ROADMAP remain authoritative for state and acceptance respectively.
