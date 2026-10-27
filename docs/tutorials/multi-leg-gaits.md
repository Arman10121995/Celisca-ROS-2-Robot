# Six-, eight- and many-legged controls

## October 27 saved maps and goals

The source hexapod, octopod and centipede now share native Gait controls for
Drive, SLAM, 3D SLAM, localization and navigation. Contact/FK odometry keeps its
KDL chains alive and uses continuous supporting feet. Real map export/reload,
source phase/spine settings and explicit Nav2 goals are implemented.
[Current scoped measurements](../status/workflows-2026-10-27.md) include a
hexapod saved-map Nav2 result and an octopod 3D database; per-model terrain/
recovery/mission/backend acceptance remains open.

Select the **Gait task** in the control column. Connect and Arm before using
Drive or sending a navigation goal. Acquire a map in SLAM/3D SLAM, Save it,
and wait for the actual save result. Pick the saved occupancy YAML for 2D
localization/navigation, or the saved RTAB-Map database plus 3D localization
for a 3D reload. Paths and task choices fill the launch command automatically.
The goal panel can copy the estimated map pose and submits a real Nav2 action.

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=hexapod_ros2 simulator:=mujoco mode:=gait \
  map_name:=nav_empty gait_task:=slam
# After acquiring and saving a real grid:
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=hexapod_ros2 simulator:=mujoco mode:=gait \
  map_name:=nav_empty gait_task:=nav gait_map_yaml:=/workspace/.../map.yaml
```

3D mapping uses `gait_task:=3d_slam`; reload adds
`gait_database:=/workspace/.../rtabmap.db gait_localization:=true`.
Map and costmap subscriptions use `/gait/map`; map/odom transforms and source
spawn geometry remain separate. Gait or spine changes stop motion and require
re-arm. Simulator pose remains visualization/tilt/independent measurement only.

The original October 15 motion-only description below is historical.

The initial extension adds three complete, separately configured robots in
the normal GUI and registry. They use **MuJoCo Display/Gait**, bounded physical
joint PD and source FK/IK. Their bodies stay free in physics. No learned policy,
independent estimation, SLAM, navigation or other-backend support is implied.

| Robot | Physical structure | Source/default schedule |
|---|---|---|
| `hexapod_ros2` | Six legs, 18 leg + two head joints | Pinned MIT CAD/KDL source; authored tripod adapter |
| `lab_octopod` | Eight legs, 24 actuators | First-party primitive geometry; alternating tetrapod |
| `lab_centipede` | Twelve legs, six bodies, five spine joints; 41 actuators | First-party segmented geometry; propagated wave |

Legs and segments stay nested under these complete families. Registry
**Preview 3D** uses the configured source stance. Structural leg-count tags
never grant another robot's controller or mission support.

## Use the GUI

1. Source the SSD/ROS/workspace environment and launch
   `ros2 run robot_lab_gui robot_lab_gui`.
2. Select **Legged robots → Six legs / Eight legs / Many legs and segments**,
   then the complete robot. MuJoCo, Gait and a compatible ground map are
   selected and the command fills automatically. Start with `nav_empty`.
3. Run. In the right **Gait** page, Connect. Wait for fresh actual joints,
   body and foot forces, then **Arm walking**. Both actions start neutral.
4. Open **Drive** for the existing latched seven-button pad, WASD/gamepad,
   increments and strafe. Click again to release a latch. Space/Stop halts
   gait phases and holds the last bounded joint targets. Body momentum and
   contact settling remain physical. Re-arm after Stop/loss.
5. **Return to source stance** moves joint targets at bounded rates. It is
   a posture request, not a get-up/recovery controller. Apply a matching
   tripod/tetrapod default, ripple or wave schedule while held, then re-arm.

Display maintains a physical source stance and refuses walking ownership.
The models do not gain Localization/SLAM/Nav2 merely from having legs or a
MuJoCo range sensor. Terrain, independent odometry/perception, saved-map and
obstacle missions need their own integration and later qualification.

## Run

The same octopod session from a terminal:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=lab_octopod simulator:=mujoco mode:=gait map_name:=nav_empty
```

## Source and physical contracts

The [MIT upstream hexapod](https://github.com/ariegweomamerie/hexapod_ros2)
is pinned to `b146f84d58123caabfcea8fe2a5e2a353acc33de`. Original CAD
descriptions/meshes and license are retained under `robot_lab_robots/hexapod_ros2`.
The KDL conversion is reused unchanged with notice in `robot_lab_utils/vendor`.
The Robot Lab gait, MuJoCo actuator adapter and two other geometries are
authored. They are not the upstream Jazzy/Harmonic runtime or a learned policy.

`scripts/generate_gait_assets.py` generates each complete URDF/configuration.
The identity footprint joint is oriented so the massive trunk is root;
otherwise importing a massless root can discard trunk inertia. Link masses,
inertias, axes and limits are retained. The imported robot's −Y front is
handled by a whole-plant spawn rotation, not altered leg axes. `MjSpec.to_xml`
rounds child masses to six significant digits; the focused mass check uses
that stated serialization bound, while primitive-model masses match tightly.
Source mass totals are 27.4553523 kg, 4.64 kg and 8.16 kg respectively.

PD stays inside each physics step through the existing `JointPDCommand`, with
explicit gains/torque/rotor-inertia contracts. ROS publishes finite joint
targets in the exact source order. The controller bounds joint target rates,
body speed and stride, and interrupts on excessive IK error, actual body
contact, tilt or stale clock/joint/body/contact/operator observations.
Stop/loss freezes gait advancement and holds the current targets; it does not
pin a body or set its velocity directly. Plant-side target loss returns to
the declared source stance; broad loss/reset qualification is still open.

Normal forces on `/gait/contacts` come from actual MuJoCo static-world contacts,
including rigid world flex surfaces. Planned stance phases are a different
field. Initial/reset foot placement casts against the selected world's actual
surfaces with the robot excluded and rejects more than 4 cm initial support
variation. Explicit numeric `spawn_z` bypasses that automatic foot placement.
No physics-time placement correction occurs during walking.

The panel's body pose/tilt is explicitly simulator truth. It supplies the
visualization TF and interruption guard, not independent odometry/SLAM.
Actual reset detection disarms on the engine-time restart even when ROS time
continues. GUI latches clear on observed interruption and require new input.
Real joystick hardware validation remains separate.

## Agent handoff

Read [the actual scope/receipt](../status/gaits-2026-10-15.md), ledger and
provenance beside each model. Reuse `gait_model.py`, `gait_controller.py`,
`gait_simulator.launch.py`, the opt-in `gait_contacts.py` and the original
MuJoCo in-step actuator loop. New gait flags default off for existing robots.
The selected world's collision conversion must exist; no silent empty-world
fallback is accepted in this branch. External worlds whose conversion cannot
be resolved need an explicit adapter before being advertised.

Keep exact-model schedules: do not pad a hexapod policy for eight or twelve
legs. Extend force/contact-aware gait feedback and recovery before terrain
claims, then add independent odometry/IMU/range/RGB-D inputs, saved/reloaded
SLAM and routes. Add other backends with their own physical actuator/clock/
contact contracts. Current runs are short GUI diagnostics, not full acceptance.
Retain the failed inherited-string spawn and discarded-root-mass runs, source
hashes and serialization-bound test failure. Broad campaigns belong to the
later validation lane; remaining R10 marine/fleet features precede R3–R9.
