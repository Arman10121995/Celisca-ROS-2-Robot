# Implementation-first checkpoint — October 9, 2026

The user prioritizes **remaining initial implementation before testing and
validation**. Work stays on `master`, based on published `c6e438d`. Codex/Claude
implement; follow-up validation agents are unassigned. Historical October 8
measurements keep their original source stages. This checkpoint records code
and installation; it does not certify new robot missions.

## Delivered implementation

| Feature | New implementation | Validation |
|---|---|---|
| Unitree G1/H1/H1_2 | Exact native model/config/checkpoint adapter, TorchScript observation and PD torque contract, per-physics-step feedback, limits, neutral startup, watchdog/fall latch/reset; three SSD-installed grouped GUI variants | Pending |
| Native arms/hands | 28 compatible fixed models gain authored bounded position-actuator/coupled-tendon targets, measured feedback, heartbeat/Stop/reset and an owned GUI channel panel; source-matched measured teach/save/load/replay with settling and interruption | Pending |
| Cartesian jogging | Panda/generic local Jacobian Servo, authored sites or actual terminal body origins, joint/rate bounds and private-data contact prediction; press-and-hold translation/rotation buttons | Pending |
| Panda object planning | Measured moving cube/pedestal geometry, fresh two-finger contact and joint/FK snapshot, acknowledged planning attach/detach, object-loss Stop, scene revision and plan invalidation; physical object remains contact-driven | Pending |
| Mobile manipulation | Stretch and Stretch 3 original wheel/tendon transmissions, base/articulation ownership, retracted-arm Drive, watchdog/reset, actual source-mounted lidar/camera RGB-D and conservative changing full-body footprint and estimator/mapper reset notifications | Pending |
| Stretch higher workflows | Explicit sensor-gated AMCL, slam_toolbox, RTAB-Map and Nav2 Smac2D/RPP composition with selected occupancy map, map save/database paths and simulator-state EKF bootstrap | Pending |
| Drone routes | Asynchronous static-world 3D A*, continuous segment clearance against primitives/mesh/terrain, measured FCU XYZ targets, separate Plan/Execute, pose/resource/target invalidation, bounded waypoint setpoints and manual/Stop/Hold/Land interruption | Pending |
| MuJoCo heavy worlds | Cached coplanar collision retriangulation accepts equivalent planar surface unions and preserves holes; curved/nonmanifold patches retain original triangles, visuals/raycast inputs unchanged | Performance/contact measurements pending |
| Experiments | Real resource-bounded subprocess queue, isolated leased ROS domains/partitions, actual clock or FCU readiness, physical traces/RSS/CPU/artifacts, owned cancel/reset/cleanup and provenance; invented legacy executor metrics removed | Concurrent-plant and overload qualification pending |
| PX4 process lifecycle | Single instance-0 MAVLink lease, SSD log budget, queued logs inside job artifact budget and cleanup during startup exceptions | Pending |

Counts describe installation eligibility: 28 fixed profiles, two native mobile
profiles and three additional Unitree variants. Current extension inventory is
116 profiles. They are not 116 working missions. Default native Display stays
passive unless explicit experimental controls are selected; Panda retains its
existing controller default. Missing controllers are not enabled by taxonomy.
Qualified higher-mode certificates remain unchanged.

## GUI paths

- G1/H1 family → **RL Gym policy** variant → MuJoCo → Display → enable
  **Native walking policy (experimental)** before Run → Drive/WASD/strafe.
- Compatible fixed model → MuJoCo → Display → **Native joint controls** →
  Arm/Hand channel selector, measured value, slider/target, Apply/Jog/Home/Stop,
  Teach current and explicit bounded Replay.
- Panda or compatible direct-joint controller → **Cartesian jog** → select
  an authored site or source terminal body origin → enable → hold an axis.
- Panda + Hand grasp fixture → **Cartesian Plan/Execute** and measured
  **Attach grasped object / Detach object**. These change collision planning,
  not physical attachment. Scene updates invalidate cached plans.
- Stretch/Stretch 3 → MuJoCo → Display → **Native mobile controls** → choose
  **Native workflow** display/loc/slam/3d_slam/nav. Run command autofills explicit
  `native_task`; experimental higher tasks require a selected map. Stop the
  base before articulation and retract the arm before Drive.
- PX4 X500 → Gazebo → Flight → Takeoff → Drone **Use current FCU position**,
  target steps, **Plan route**, **Execute route**, **Cancel / Hold**. Manual
  buttons and Space interrupt tracking. No autonomous takeoff or execution.
- Benchmark → **Concurrent experiments** → Queue Launch selection → Run
  queue. Capture completion is separate from mission success; PX4 jobs are
  serial because native instance 0 shares MAVLink port 14580.

Read [native controls](../tutorials/native-controls.md),
[drone controls](../tutorials/px4_x500.md) and
[experiment queue](../tutorials/concurrent-experiments.md). Reopen the GUI after
rebuilding so it loads installed metadata and new widgets.

## Installation and build receipt

All large sources, controllers, caches and output remain on the SSD. Existing
pinned RL Gym files are reused. Normal provisioning reproduces the policy
variants and mobile manifests; the standalone scripts refresh installed older
snapshots without reimporting models:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/install_geometry_dependencies.sh
source scripts/ssd_env.sh
python3 scripts/provision_extension_assets.py --kind robots --source unitree_rl_gym --no-download
python3 scripts/configure_native_articulation.py
python3 scripts/configure_native_mobile.py
colcon build --symlink-install --packages-select \
  robot_lab_utils robot_lab_mujoco robot_lab_bringup robot_lab_gui \
  robot_lab_adapter robot_lab_navigation robot_lab_localization \
  robot_lab_mapping robot_lab_benchmark
```

The three policy variants, 28 fixed controller registrations and two Stretch
registrations completed. Nine changed packages built successfully; the final installation repeat took
11.3 s (earlier full reconfiguration: 29.9 s).
Python/YAML/package XML parsing completed. Necessary build diagnostics are implementation
work; no new test suite, live GUI/physics mission campaign or CI result is
claimed. Colcon overlay/setuptools warnings and CMake pytest-discovery warnings
remain. Rebuild affected packages when adding further files.

The [installation receipt](implementation-receipt-2026-10-09.json) records real
build logs, current source hashes and 19 installed implementation modules
matching source bytes. It explicitly records no new physical/test/CI result.

Use normal `--kind all` provisioning on a new host; `--no-download` requires the
pinned SSD checkouts. Native policies require CPU TorchScript support. Source
model/resource/config/checkpoint/license hashes are retained in installed
manifests. Static aerial planning uses trimesh/rtree; collision simplification
and Stretch footprints use Shapely. The SSD bootstrap installs Rtree 1.4.1
and Shapely 2.1.2 without changing the host NumPy/ROS ABI. Sourcing
`scripts/ssd_env.sh` exposes these dependencies to ROS and the PX4 venv;
source it again after installation. Optional collision simplification falls
back to original geometry when Shapely is unavailable.

## Still missing initial implementations

1. **Other complete robots and backends:** exact-model locomotion/checkpoints,
   additional vendor/mobile bases, compatible sensor kits and control adapters
   outside the native MuJoCo candidates. Missing checkpoints cannot be replaced
   with another robot's network. RL Gym's configured Go2 checkpoint is absent;
   existing Go2/BHL controllers and failure histories remain separate.
2. **Manipulation:** full planning adapters for additional arms/backends,
   dexterous hand-specific controllers, arbitrary object scenes, repeated
   pick/place and coordinated navigate/reach/grasp task execution. Generic
   position jogging alone does not solve these tasks.
3. **Aerial workflows:** safe takeoff/floor/ceiling corridor selection, aerial
   sensor/3D-map save/reload, moving-obstacle avoidance and other flight backends.
   Current 0.7 m spherical static planning is deliberately conservative; narrow
   indoor routes can be rejected and manual/direct goals remain unchecked.
4. **World/terrain breadth:** source actor/material parity, multi-floor world
   registration/resource repairs and the provider/GUI terrain workflow owned
   separately by cline-agent. Preserve that lane; do not mark it complete here.
5. **Algorithms/experiments:** substantive remaining upstream method adapters,
   matched live-input datasets, explicit simulator-seed and workload mission
   result contracts, comparison capture/export/plots and hardware deployment.

The coplanar optimization is implemented, but P4 remains open until matched
speed/contact/geometry measurements. Existing 4WS/mecanum and selected wheeled
higher-mode implementations remain; broader physical mission coverage is the
later validation lane, not a new synthetic success flag.

## Handoff after initial implementation

Use [robot readiness](../AI_ROBOT_READINESS_GUIDE.md) and the actual existing
producers. Record robot variant, world, backend, controller/asset/config/source
revision, GUI command, raw artifacts and failures. Unitree needs stance,
forward/reverse/turn, Stop/loss/fall/reset and measured joint/body feedback.
Articulation/Servo/objects need bounds, measured motion/contacts, release/loss/
reset and real grasp missions. Stretch needs sensors/TF, moving footprint,
saved/reloaded SLAM and physical obstacle routes. Drone needs independent body
truth, actual route tracking/contacts, interventions and landed cleanup.
Concurrent runs need two real plants, overload, scoped cancel/reset and no
orphan processes. Compare P4 with matched clocks/sensors/control and original
collision surfaces; a smaller mesh does not prove a faster correct simulation.

Keep implementation and validation states separate in the ledger/GUI.
Do not refresh historical support certificates after code changes without
corresponding source-matched physical evidence. Preserve independent
`scripts/review_occupancy_maps.py` and cline-agent's R6.7 terrain work. Return
concrete defects to implementation; catalog counts and generated scores are
not physical evidence.
