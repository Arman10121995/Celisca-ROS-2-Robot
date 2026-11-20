# Pro630 with F100 gripper in MuJoCo

One normal Robot Lab GUI run on the Jetson used
`asset_elephant_myagv_pro_pro630_f100`, MuJoCo, `nav_empty` and Display.
The **Mobile arm → Gripper** Open and Close buttons drove the actual shaft
and five observed followers. The operator used **End session** and the owned
launch exited cleanly. The raw values and exact generated launch command are
in [the GUI report](gui-gripper-result.json). [Runtime hashes](runtime-sha256.txt).

| Command | Target | Measured leader | Largest settled follower error | First within 0.03 rad |
|---|---:|---:|---:|---:|
| Open | 1.000 rad | 0.999997 rad | 0.0000068 rad | 2.21 s |
| Close | 0.000 rad | 0.000003 rad | 0.0000071 rad | 2.23 s |

Both Pro630 variants also passed a private source-flange planning check for
an 8.46 mm move, with 0.00165 mm forward-kinematics error at the planned
joint endpoint. Live Cartesian execution, contact grasp, transport and
other simulator versions of the F100 remain open.

## Source and retained diagnostics

The bare arm and its wheel/arm measurements use the existing source pin.
The F100 mount, collision meshes and linkage come from the vendor Pro630
Gazebo description at `b8176c264e583b073e1c4293c2edb1241b20e1aa`. Three
follower joint coordinates reverse sign with their axes to preserve the
same spatial linkage in the shared positive-ratio contract. One MuJoCo
position actuator drives the five passive equality joints. Lab caps are
0.8 Nm and 0.5 rad/s. The vendor package declares BSD and the mesh package
contains the BSD-2-Clause license. [Vendor source](https://github.com/elephantrobotics/mycobot_ros2/tree/b8176c264e583b073e1c4293c2edb1241b20e1aa/mycobot_pro/mycobotpro630_gazeboros2).

The initial fixed-base jaw bench reached its targets but showed 0.267 rad
maximum follower error with the default soft constraint. With the final
coupling, the bench targets 0.8 and 0.05 rad both settled to the printed
precision, with 0.00552 rad maximum follower error during motion and
0.240 Nm peak motor effort. The first GUI trial reached model import and
was stopped after the launch rejected the optional planner channel list.
The later launch gate accepts that list and validates it in the planner.
GUI cleanup also keeps ownership when publishing neutral fails after ROS
context shutdown.

79 focused source and GUI checks and two targeted shutdown checks passed.
Five packages built. Provisioning logs, the bench probes, screenshots and
failed and passing launch outputs are on SSD under
`$ROBOT_LAB_RUNTIME_ROOT/pro630-gripper-20261120/`.

## Reproduce the GUI gripper screen

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source scripts/ssd_env.sh
export ROS_DOMAIN_ID=143
export IGN_PARTITION=robot_lab_pro630_f100_gui_2 GZ_PARTITION=robot_lab_pro630_f100_gui_2
xvfb-run -a python3 scripts/qualify_urdf_arm_gui.py \
  --robot asset_elephant_myagv_pro_pro630_f100 --simulator mujoco \
  --map nav_empty --gripper-presets --output "$ROBOT_LAB_RUNTIME_ROOT/pro630-f100-repeat"
```
