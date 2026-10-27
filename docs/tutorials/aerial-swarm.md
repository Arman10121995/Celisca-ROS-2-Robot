# Shared-world aerial swarm

Robot Lab has a native **Aerial Swarm** page for two to four X500 vehicles in
one Gazebo Harmonic world. Each vehicle runs its own native PX4 SITL binary,
FCU identity, UDP port, ROS namespace, frame tree and command worker. Ground
Fleet uses Fortress separately. These are experimental controllers; consult
the latest [workflow receipt](../status/workflows-2026-10-27.md) for measured
missions and remaining coverage.

1. Open **Aerial Swarm** in the sidebar. Choose an installed map family and
   variant, vehicle count and spacing. Blank X/Y requests separate free spawn
   positions near that world's default. Enter explicit XYZ for elevated
   platforms. Z is the model spawn height, not the requested flight altitude.
2. Save the session if wanted. The Run command fills automatically. **Run**
   starts the selected world once and creates all FCUs. Missing world geometry,
   insufficient space or a busy PX4 instance fails visibly. It does not silently
   replace the selected world with an empty scene.
3. **Connect · neutral** acquires operator ownership and displays actual FCU
   readiness and simulator truth separately. Connecting or enabling keyboard/
   joystick input does not request takeoff or motion.
4. Select one vehicle or **All vehicles**, then **Check clearance**. Wait for
   each selected worker's actual clear reply. **Takeoff** consumes each fresh,
   single-use check. A rejected peer aborts the group takeoff. Do not infer
   flight from an accepted request or an armed FCU; watch actual height and
   the reported ascending/flying state.
5. Once airborne, choose **Arm manual**. The six direction buttons latch when
   clicked and release when clicked again. **Stop/Hold** clears the latches.
   Altitude +/− moves vertically. With keyboard control enabled, W/S moves
   forward/backward, A/D changes yaw, arrow Up/Down changes altitude and Space
   requests Hold. Input release ramps velocity down. The displayed increments
   are changes per control step; yaw has a reduced 0.2 rad/s cap.
6. Formation, rendezvous, dispersion and flocking use current FCU poses to
   generate targets. Selected-world collision corridors and physical peer
   separation guard movement. These centralized experimental behaviors do not
   implement a distributed radio or collision-avoidance guarantee.
7. **Hold** clears movement. **Land** requests native PX4 landing. Wait for
   actual landed/disarmed state before closing. Loss of operator ownership,
   vehicle telemetry or session feedback clears GUI inputs; worker lease loss
   requests native Land.

The startup map transform uses the known settled spawn as a prior while the
vehicle remains disarmed, landed and level. Calibration waits for stable raw
FCU velocity. The first accepted Takeoff freezes that transform; subsequent
position comes from FCU odometry. Simulator truth is logged independently and
is not fed back during flight. A startup prior is not aerial localization.

## Terminal launch and saved sessions

```bash
source scripts/ssd_env.sh
source install/setup.bash
ros2 launch robot_lab_bringup aerial_swarm.launch.py \
  map_name:=nav_empty gui:=true
```

An optional `session:=/workspace/.../session.yaml` uses this format:

```yaml
schema_version: 1
map: nav_empty
spacing: 3.0
members:
  - {id: air1}
  - {id: air2}
```

The launch accepts `px4_root` and `px4_python` overrides. The default dedicated
SSD Python environment supplies MAVLink bindings; system Python need not have
them. Generated session manifests, per-FCU logs and origin calibration records
stay under `$ROBOT_LAB_RUNTIME_ROOT/aerial_sessions`. PX4 runs with `-d`,
bounded logs and per-instance locks. Teardown signals only the session's owned
process groups and releases locks even if ROS node cleanup fails. Runtime
startup failures propagate a nonzero launch result.

ROS inspection uses `/aerial_swarm/state`, `/aerial_swarm/command`,
`/aerial_swarm/operator_heartbeat` and `/aerial_swarm/key_vel`. Commands include
the live session ID, operator ID and explicit target IDs. Workers have separate
`/swarm/<member>/px4/...` topics and generation-scoped replies. Preserve these identity
and lease checks when adding new behaviors or experiment producers.
