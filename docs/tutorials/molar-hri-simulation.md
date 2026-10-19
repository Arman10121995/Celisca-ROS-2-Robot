# MOLAR / ProBOT-L simulation controls

This integration adds two complete robot variants and two separate detailed
CELISCA floors to the normal Robot Lab GUI. The workflow provides differential
Drive, conveyor lift, coupled telescope and transfer-roller controls in
**Display** mode. These are implemented simulator adapters with estimated
dynamics. The normal-GUI check (`scripts/qualify_molar_hri_gui.py`) covers
drive, lift and telescope to their limits, roller, Stop and heartbeat loss.
It passes for `molar_hri` in Gazebo, MuJoCo, PyBullet and Isaac on
`nav_empty` and on both detailed floors, and for `probot_l` on `nav_empty`
([evidence](../status/evidence/molar-hri-2026-10-16/README.md)). Each pass
covers that exact robot/backend/map cell only. Payload handling,
sensors, SLAM/Nav2 and physical behaviour are not qualified.

Use the [task ledger](../status/platform-status.yaml) for current ownership,
acceptance and evidence. Follow [Storage](../STORAGE.md) and keep sources,
asset caches, builds and trial outputs on `/workspace`.

## Run from the normal GUI

Start the existing installation from its sourced workspace:

```bash
cd /workspace/molar/ros_ws/bumperbot_ws
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_lab_gui robot_lab_gui
```

1. In Launch, select the **MOLAR / ProBOT-L** family. Choose **Complete MOLAR
   HRI assembly** (`molar_hri`) or **Complete ProBOT-L carrier** (`probot_l`).
2. Select **Display**, a backend, and a world. Gazebo, MuJoCo, PyBullet and
   Isaac have candidate articulation adapters; no higher MOLAR mode is enabled.
   `nav_empty` is available for initial mechanics checks. For detailed CELISCA,
   choose the exact map ID below, rather than a legacy/plain/furnished variant.
3. Inspect the generated command and press **Run**. Wait for the selected
   model's measured feedback before issuing input. Opening the GUI, selecting
   a profile, editing targets or dragging a slider sends no articulation target.
4. Use **Drive & limits** for the base and **MOLAR** for the conveyor assembly.
   Leaving the MOLAR page requests articulation Stop and withdraws its heartbeat.

| Map ID | Source alias | Physical building floor / FMS floor |
| --- | --- | --- |
| `celisca_floor_1_detailed` | F1 | 2.OG / 2 |
| `celisca_floor_2_detailed` | F2 | 3.OG / 3 |

Each map is an independent local floor. The integration does not stack them,
invent an inter-floor height, or operate a door or lift. Existing CELISCA map
entries and their registered transforms remain separate.

## Drive and articulation

Drive uses the existing latched direction buttons, ramps and optional
WASD/joystick input. Keyboard enablement starts neutral; actual operator input
requests travel. Combined forward/turn input is available, for example W+A.
Click a latched button again to release it. **Stop motion** or Space uses the
existing base Stop path. These controls drive the simulator's wheel interface.

Stop the base before articulation. The backend rejects articulation commands
while the measured base is moving. Base Drive is inhibited during articulation
activity/fault, while either measured telescope stage exceeds **0.005 m**, or
while measured stage-coupling deviation exceeds **0.01 m**. Both actual slide
measurements matter; retracting the first stage alone is insufficient.

The MOLAR page exposes these independent targets:

| Control | Target range | Interpretation |
| --- | --- | --- |
| Conveyor lift | 0–0.680 m | Lift above the nominal 0.420 m transport deck; nominal deck height is 0.420 m + measured lift |
| Telescope | 0–0.140 m | First-stage target; the second stage is coupled 1:1, giving at most 0.280 m total tip extension |
| Transfer roller | −8 to +8 rad/s | Velocity target for the represented roller, not an internal belt/payload controller |

Position references have source rate limits: **0.035 m/s** for lift and
**0.100 m/s** for each slide. Finite effort limits and backend motor settings
also apply. These are experiment bounds, not identified hardware performance.

**Apply** sends the displayed target. **Jog** makes an explicit bounded target
step and sends it; it is not automatic motion from slider dragging. Jog size
must be positive and no larger than 10% of the channel's range. **Use measured**
updates the editable fields without sending motion: positions and measured
roller velocity are clamped to source bounds for editing, while raw telemetry
remains unchanged. The roller's actual angle and actual velocity are distinct.

**Stop / hold** stops the roller and holds each measured lift/slide position
within source bounds. It does not snap a displaced coupled slide to the other
slide. A later explicit command recouples through bounded references. Inspect
actual motion and settling; a requested Stop is not an instantaneous physical
arrest or a hardware emergency-stop certificate.

### Operator handling sequence

Use a deliberate **lift → extend → transfer → Stop → retract → lower** sequence:

1. Stop the base and wait for fresh, settled feedback. Use measured if needed.
2. Apply a suitable lift target and wait for the measured deck height.
3. Apply the telescope target and inspect **both** stages, their total extension
   and coupling error. Keep the base stopped.
4. Explicitly apply a bounded roller velocity for the experiment. Stop / hold
   before continuing; confirm actual roller velocity falls.
5. Apply telescope zero and wait until both measured stages are retracted and
   coupling feedback is acceptable.
6. Apply lift zero and inspect settling before returning to base Drive.

This is an operator workflow, not an automatic transport sequence, collision-free
handling planner, internal conveyor model or demonstrated payload transfer.
The GUI does not queue these steps or automatically raise before extension.

## Ownership, loss and reset

The panel requires an owned, still-running main launch whose robot, backend,
Display mode and map match the current selection. It also checks the exact
resolved source-model path/SHA-256, the articulation schema, one state publisher
and one command/heartbeat subscriber. State additionally reports the executed
URDF hash for provenance; this is distinct from the selected source-file hash.
Ambiguous graphs do not enable control.

State carries `state_monotonic`, measured on the same host as the GUI. Its age
must be less than **0.5 wall seconds**; a future offset beyond 0.05 s is rejected.
Callback arrival alone does not make an old queued message fresh. DDS identity
and source age are checked additionally when metadata is available. This clock
contract assumes the GUI and plant share the host's monotonic clock; a remote
GUI on another OS host needs a separately designed/validated freshness contract.

The GUI supplies heartbeat only while the owned MOLAR page has fresh initialized
feedback and no fault. The backend holds on heartbeat absence beyond **0.5 wall
seconds**, feedback/ownership loss or an applicable fault. Selection changes,
leaving the control/workspace page, launch death and GUI close withdraw control.
Changing pages is therefore not a way to leave the roller running unattended.

Stop/reset changes the control `plant_id` and clears heartbeat admission. Old
command packets cannot revive the previous control generation. Re-enter the
page, obtain fresh state and make a new explicit request. The existing Reset
Robot workflow is separate from Stop; after reset, inspect the new plant and
actual pose before continuing. No cached target is automatically replayed.

## Source and backend architecture

Both variants retain **15 links and 14 moving joints**: two drive wheels, four
caster swivels, four caster spins, one conveyor lift, two telescope stages and
one transfer roller. Casters remain passive. The HRI assembly is attached to
the carrier; the adapter does not invent hidden motors, belts or suspension.

| Component | Source path / responsibility |
| --- | --- |
| Robot derivatives | `src/robot_lab_robots/{probot_l,molar_hri}/`: `source/robot.urdf`, `urdf/<variant>.urdf.xacro`, meshes, configuration and `provenance.json` |
| Robot profiles | `src/robot_lab_robots/config/robots.yaml`; family/type selectors in `src/robot_lab_bringup/config/asset_groups.yaml` and `robot_taxonomy.yaml` |
| GUI | `src/robot_lab_gui/robot_lab_gui/molar_controls.py`: explicit input, ownership, freshness, limits and measured display |
| Shared contract | `src/robot_lab_utils/robot_lab_utils/urdf_articulation.py`: `robot_lab.urdf_articulation.v1`, bounded references, coupling, Stop/watchdog/control generations and base inhibition |
| Normal composition | `src/robot_lab_bringup/launch/simulated_robot.launch.py`: selected source model and articulation configuration |
| Detailed maps | `src/robot_lab_maps/maps/<map_id>/`: native `worlds/`, source `adapters/`, offline `maps/` and `PROVENANCE.json` |

| Backend candidate | Actuation path |
| --- | --- |
| Gazebo | Humble's native Fortress path, `ign_ros2_control` and configured differential/position/velocity controllers; `src/robot_lab_bringup/scripts/molar_gazebo_control.py` bridges actual joint feedback to the shared contract |
| MuJoCo | `src/robot_lab_mujoco/python/robot_lab_mujoco/mujoco_spawner.py`; finite-force implicit PD actuators and source-bounded references |
| PyBullet | `src/robot_lab_pybullet/python/robot_lab_pybullet/pybullet_spawner.py`; native implicit position/velocity motors, finite forces and slide-rate caps |
| Isaac | `src/robot_lab_isaac/python/robot_lab_isaac/isaac_spawner.py` and `isaac_runtime.py`; linear/angular USD drives with unit conversion and passive caster-bearing damping |

The Gazebo control adapter belongs only to Gazebo. Native MuJoCo, PyBullet and
Isaac plants expose their own shared-contract endpoints; starting a duplicate
Gazebo adapter on those backends creates conflicting graph/feedback ownership.
PyBullet's earlier explicit roller PD trial was unstable; the current candidate
uses its native implicit motor. Gains, damping and other backend solver settings
remain estimates requiring actual per-backend measurement.

Backend settings found by the October 16-19 runs:

- MuJoCo integrates a robot with an articulation implicitly (`implicitfast`).
  Under explicit Euler the light roller misread and kept the base inhibited.
- MuJoCo and Isaac add a gravity feedforward to the lift and telescope
  servos. The servo and the feedforward share the joint's source effort
  limit. Without it, a pure PD held the 14.2 kg carriage 7 mm low, and each
  Stop lowered it further. MuJoCo ramps the feedforward in over the first
  second after a spawn and gives the slides stiff stops (5 ms), so the spawn
  landing stays inside the contract's measurement allowance.
- "Articulation active" and Isaac's "base moving" are judged from the
  position change over 0.25 s of simulated time. Isaac's reported joint and
  body velocities jitter at rest.
- In Gazebo (Fortress/DART) the caster tyres have friction 0.1, so they
  swivel. The lift and telescope hard stops sit 5 mm below the controlled
  range, because DART's velocity servo cannot leave a stop it rests on.
- Isaac leaves a world mesh's triangles lying on its z = 0 ground plane to the
  plane: the detailed floors' long duplicated floor slivers threw the base.

### ROS debugging interfaces

The normal GUI is the operator entry point. Diagnostic interfaces are:

| Name | Type | Contract |
| --- | --- | --- |
| `/molar/articulation/state` | `std_msgs/msg/String` | JSON with model/URDF hashes, publication time, current `plant_id`, readiness/fault/status, raw measured channels, targets/references, coupling error and base inhibition |
| `/molar/articulation/command` | `std_msgs/msg/String` | JSON `{"plant_id":"<fresh ID>","targets":{"conveyor_lift_joint":0.18}}`; independent joints only, finite source bounds and current heartbeat required |
| `/molar/articulation/heartbeat` | `std_msgs/msg/Empty` | GUI publishes at about 10 Hz while admission is valid; it is not a command to move |
| `/molar/articulation/stop` | `std_srvs/srv/Trigger` | Hold measured articulation, stop roller and revoke the prior control generation |

For a read-only check in the running launch's ROS domain:

```bash
ros2 topic echo --once /molar/articulation/state std_msgs/msg/String
ros2 topic info -v /molar/articulation/state
```

Do not replay a captured command after Stop/reset, publish to an unrelated
domain, or run another controller against the same plant. Raw ROS debugging
does not prove the GUI ownership and input workflow passed.

## Detailed-map reconstruction and limitations

The native worlds use full detailed visual derivatives plus separate collision
geometry. Visuals retain the original furniture and added bathroom details.
They bake GLB node transforms into metric Z-up coordinates with `(x, -z, y)`;
there is no added scale, centering shift, floor stacking or FMS registration.
PBR-to-Phong appearance is approximate. Bathroom dimensions and unsurveyed
fixtures are research illustrations, not a measured building survey.

Collision retains original structural triangles and conservative per-object
furniture/fixture boxes. Gazebo and MuJoCo read them from
`native/meshes/collision_normals_v1.stl`, which has explicit normals and the
source OBJ's exact float64 triangles, because Fortress/DART ignored the OBJ's
normal-less submeshes. Decorative tile/grout has no collision. Doors and lifts
are static. Full visual detail is not full geometric collision fidelity.
The occupancy map is an **offline** 0.06–2.3 m collision-height-band projection
at 0.05 m/cell; only the source-supported spawn component is free. Disconnected
or unsupported regions are unknown. It is not a live SLAM map, a Fortress map
plugin export, a complete multi-floor map or Nav2 acceptance.

The full visual DAEs are outside Git on the SSD:

```text
/workspace/molar/robot_lab_runtime/external_assets/celisca_detailed/<enhanced-GLB-SHA256>/z_up_visual_v1/visual_full.dae
```

F1's DAE is 217,382,228 bytes (about 217 MB); F2's is 319,727,955 bytes (about
320 MB), excluding the other cache contents/textures. Native SDF worlds reference
these absolute paths. A Git checkout alone is therefore **not self-contained**
for detailed-world rendering or clean-host reproduction.

Reconstruction uses `scripts/provision_celisca_detailed.py` and the pinned native
HRI tree at `/workspace/molar_voice_super_gui_redesign`, checkpoint `a98ff3c`.
Required inputs include the original registered
`map/Celisca_F1_SeperateMap.glb` and `map/Celisca_F2_SeperateMap.glb`, the detailed
GLBs, source manifests and complete relative adapters under
`simulation/celisca/exports/`. Supplying only a detailed GLB or the portable ZIP
does not supply the original source files used by the retention check.

The expected native enhanced-GLB hashes are:

```text
F1 d697a399fb54150ad4a8e56e5e8ba6347c43b7d815270e12d4689b79635f3066
F2 01d18338a90a41b4823e88d5f7b6a4e34a5f65653ecf0f2d5ac6e3c675ef90cc
```

These pins differ from the Windows detailed-GLB deliverable. Preserve the native
inputs and use their manifests; renaming a different export cannot satisfy the
byte/retention checks. On a prepared native checkout with dependencies installed:

```bash
cd /workspace/molar/ros_ws/bumperbot_ws
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
python3 scripts/provision_celisca_detailed.py \
  --source /workspace/molar_voice_super_gui_redesign/simulation/celisca \
  --runtime-root /workspace/molar/robot_lab_runtime
```

The provisioner checks original/enhanced hashes and source preservation, writes
the SSD cache, native worlds, collision-supported occupancy and registration.
It launches no simulator and refuses conflicting existing generated map bytes.
Its current paths and `/workspace` requirements are native-host assumptions;
reconstruction elsewhere needs reviewed path/configuration adaptation. See each
map's `README.md` and `PROVENANCE.json` for exact retained assets and checks.

Robot derivatives and meshes are present in this repository. The conversion
tool `scripts/provision_molar_hri.py` reads
`/workspace/molar_voice_super_gui_redesign/simulation/robot` by default and writes
the robot derivative/configuration files. It is not a routine GUI refresh step;
do not regenerate over unreviewed local edits. Original/derived redistribution
rights are not established by either provisioner. Build affected packages using
[the normal workflow](../WORKFLOW.md), source the installed overlay and reopen
the GUI. Verify installed Python/model bytes, since some packages copy files
despite a symlink-install build.

## Verification and continuation

On October 19 the shared-contract, backend-contract, Gazebo-watchdog,
robot-profile and detailed-map suites passed together: 81 tests at `ca218dd`.
The GUI suite (55 tests) passed on October 16; `molar_controls.py` has not
changed since. These suites exercise source/interface contracts and actual
Tk input behaviour. Travel, load handling and stability come from the
runtime checks below, not from the tests.

Relevant source suites are `src/robot_lab_gui/test/test_molar_controls.py`,
`src/robot_lab_utils/test/test_urdf_articulation.py`,
`src/robot_lab_bringup/test/test_molar_backend_contract.py`,
`src/robot_lab_bringup/test/test_molar_robot.py`,
`src/robot_lab_bringup/test/test_molar_gazebo_watchdog.py` and
`src/robot_lab_maps/test/test_celisca_detailed.py`. Use the repository's focused
test instructions and Xvfb for Tk. No runtime result is inferred from those tests.

Runtime handoff is **one engine at a time**, with an isolated ROS domain,
SSD output directory and the normal GUI producer
`scripts/qualify_molar_hri_gui.py`. Record the exact robot variant, backend,
map, generated command, source/installed hashes, raw body/joint/articulation
samples, phase results, failures and cleanup. Preserve earlier failed trials,
including unstable control candidates. Xvfb supplies a display, not physical
acceptance. Verify actual forward/reverse/combined turn, lift, both slides,
roller, Stop, heartbeat loss, selection/launch loss, reset and limits before
claiming that exact cell; detailed-floor contacts and import/visual fidelity need
their own checks. Missing/unrun backends stay pending.

Measured on October 19 (round M3b, `ca218dd`): the producer passes for
`molar_hri` in Gazebo, MuJoCo, PyBullet and Isaac on `nav_empty`,
`celisca_floor_1_detailed` and `celisca_floor_2_detailed`, and for
`probot_l` on `nav_empty`. Per cell:

- the lift holds 0.0600 m and reaches 0.6800 m;
- both stages reach 0.14 m (0.28 m tip), coupled within 0.21 mm;
- Stop drift stays under 0.4 mm;
- the roller stops on heartbeat loss (|ω| < 0.09 rad/s);
- with the telescope out the base stays within 1.4 mm.

The [evidence README](../status/evidence/molar-hri-2026-10-16/README.md) has
the table, the reports and the nine fixes. Selection loss, reset and
reverse driving are not in the producer yet.

Masses, inertias, caster parameters, servo gains, friction, joint travel and HRI
load allocation include estimates. Internal conveyor mechanics, payload transfer,
sensor/SLAM/Nav2 workflows, transport scheduling and physical qualification are
not supplied by this integration. It contains no physical robot, FMS, AIC, door,
building lift or PLC command path. Simulator diagnostics must not be reported as
hardware, clearance, stability or safety certification.
