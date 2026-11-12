# Source segmented octopod

The installed `asset_light_robotics_octopod` is the complete MIT
[Light Robotics octopod](https://github.com/light-robotics/robot_rl_lab/tree/c2cc2437393155760eeea5f588c5912f4654067c/robot_rl/bodies/octopod):
four body segments, eight three-joint legs and three pitch-spine joints.
It runs the original 7.5 kg MuJoCo plant with source position actuators,
joint damping, contact geometry and 2.45 N·m servo force limits.

## Run

In Launch, choose **Legged robots → Eight legs → Light Robotics octopod**,
MuJoCo, Display and no map. Enable **Native locomotion (experimental)**
before Run. The command fills automatically. Use the existing Drive pad,
WASD, Stop and Reset. The source flat gait supports forward, reverse and
differential turning; it has no lateral gait. Without opt-in it is a passive
source-stance display.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_light_robotics_octopod simulator:=mujoco \
  mode:=display map_name:=none enable_native_locomotion:=true
```

The NumPy controller ports the source metachronal flat-gait prior, without
requiring the upstream JAX training environment or downloading a checkpoint.
The lab adds bounded side scaling and transitions, a 0.5 s wall-clock command
lease, finite-state/fall fault latch and explicit reset. Only initialization
and reset place the root; physics moves the robot thereafter. The controller
writes position targets to the source servos, rather than treating them as
torque motors. Faults remove commanded restoring torque and require Reset.

**Speed tracking is uncalibrated:** a 0.12 m/s request produced roughly
0.25 m/s in the first screen. Requests scale the gait amplitude; they are not
verified measured speed. Turning also translates the front reference point.
Stop ramps into the source neutral stance, with finite stopping travel.

## What is measured

One actual MuJoCo no-map run through `/key_vel` and the normal twist mux
recorded 1.498 m forward, −1.535 m reverse and 1.237 rad turn. Startup remained
upright; actual force stayed within the source cap. Publisher loss ended with
zero requested velocity and neutral stance, with 0.1765 rad additional yaw
over its three-second observation. Real joint feedback and all native-body TF
were recorded. See the [receipt](../status/evidence/counterparts-2026-11-12/README.md).

This does not qualify calibrated velocity tracking, uneven terrain, recovery,
SLAM, navigation or other backends. Those modes remain unavailable for this
source variant; the existing authored octopod and its receipts are unchanged.

## Reproduce the installation

The robot is already installed locally; no GUI download is necessary. On a
fresh host, the pinned SSD installer is:

```bash
python3 scripts/provision_robot_counterparts.py --robot octopod --download
```

The source license and hashes accompany the content-addressed model. The
selected world owns its floor. The importer explicitly freezes the source's
compiled masses/inertias before removing its standalone plane; otherwise
`settotalmass` could silently rescale the robot when map geometry is attached.
The URDF export is measured-body RViz visualization, not replacement physics.
