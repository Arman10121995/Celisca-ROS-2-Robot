# VRX WAM-V in Robot Lab

## Run

The agent-provisioned `asset_vrx_wamv_h` is the complete pinned VRX H configuration,
with upstream water-force equations ported privately to Fortress. Choose **Run a robot →
Surface vessels → VRX WAM-V → Source H layout**, enable **experimental source assets**,
then Gazebo, Marine and `surface_harbor`. The command and source-specific clear-water
spawn fill automatically. Restart the GUI or refresh installed assets after provisioning.

Connect in the **Marine** control column and wait for fresh measured feedback. Arm
starts neutral; use the latched pad or WASD/joystick for surge and yaw. Source-specific
limits are 0.4 m/s and 0.1 rad/s. Stop disarms propulsion. This is a surface craft:
no depth or strafe actuation. Real buoyancy and passive drift continue after Stop.

Optional Marine RGB-D mapping and saved-map fields are passed through the normal run
command; source-sized route clearance is declared as 3.3 m. These higher workflows
are implemented but remain unqualified for this source vessel.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/provision_robot_counterparts.py --robot wamv --download
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=asset_vrx_wamv_h simulator:=gazebo mode:=marine \
  map_name:=surface_harbor allow_experimental_assets:=true
```

## Measured scope

One actual Fortress run recorded 1.8785 m forward travel, 0.7413 rad turning,
source joint feedback within 100 rad/s and real RGB-D images. Initial integration
preserves source mass, hulls, motors and water-force equations; the controller and
massless sensor kit are declared lab additions. Other maps, waves/currents, contact,
message-loss trials and acquired-map navigation remain unqualified. Original `lab_usv`
and its historical measurements are preserved.

[Source/mission receipt](../status/evidence/counterparts-2026-11-12/README.md) ·
[Upstream VRX source](https://github.com/osrf/vrx/tree/dc30ed8d17aa1083fd872edad9c77c69896d2b07).
