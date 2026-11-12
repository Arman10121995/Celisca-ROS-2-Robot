# BlueROV2 Heavy candidate in Robot Lab

## Run

The installed `asset_bluerov2_heavy` is a separate source-based experimental Fortress
plant. Select **Run a robot → Underwater robots → BlueROV2 → Source Heavy configuration**,
check experimental source assets, select Gazebo/Marine and `marine_tank`. The command
fills automatically. In the Marine control column, Connect, wait for fresh feedback,
then Arm depth/heading hold. Arming starts with neutral movement input.

The latched pad provides forward/reverse, turning and strafe; depth controls change the
hold target. Stop removes thrust; passive buoyancy and coasting remain physical.
Optional Marine RGB-D SLAM/database settings reach the same launch command. Mapping,
route missions, depth/strafe acceptance and other worlds are not qualified by the initial
forward/turn run.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_robot_counterparts.py --robot bluerov2_heavy --download
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_bluerov2_heavy simulator:=gazebo mode:=marine \
  map_name:=marine_tank allow_experimental_assets:=true
```

## Source and measured scope

The derivative preserves the pinned Heavy hull's 13.016 kg total mass, original mesh
scales, collision displacement, propeller link/joint frames, inertias and water-force
coefficients. The lab's six-axis force allocator replaces ArduSub; this does not run or
qualify the original firmware. The original FRD IMU remains separate from the declared
massless lab ENU IMU/RGB-D kit. A native force watchdog bounds each motor to the source
normalized-control envelope of ±50 N and removes stale commands.

One real Fortress run recorded 1.1373 m forward travel and 1.4250 rad turning, real
320×240 images and eight propeller states. Final neutral heading hold continued
0.1718 rad over 3.90 s; this is not an instantaneous stop measurement. No navigation,
contact, publisher-loss or full underwater mission qualification follows from this run.

Source package metadata declares MIT; the root license text and third-party mesh
redistribution review remain open. Assets stay in the pinned local SSD cache; the
original authored ROV and its measurements remain available.

[Exact source/physical receipt](../status/evidence/counterparts-2026-11-12/README.md) ·
[Upstream model](https://github.com/clydemcqueen/bluerov2_gz/tree/4451a5d1b15d204581d43ef7d93df4a0508eddbb).
