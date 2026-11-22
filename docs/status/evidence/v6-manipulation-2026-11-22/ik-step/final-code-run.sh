#!/bin/bash
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v6-manipulation-20261122/final-code
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"; cd "$HERE"
export ROS_DOMAIN_ID=117; LOCK=/workspace/molar/robot_lab_runtime/physics-qualification.lock
flock -w 1800 "$LOCK" timeout --signal=INT 1200 xvfb-run -a python3 scripts/qualify_urdf_arm_gui.py --robot asset_franka_panda_panda --simulator mujoco \
  --map arm_pick_place_lab --pick-place pickup_cube place_table --output "$RUN/panda-cube-gui-mujoco" > "$RUN/panda-cube-gui-mujoco.log" 2>&1
echo "panda-cube-gui-mujoco exit=$?" >> $RUN/progress.txt
IGN_PARTITION=v6_final GZ_PARTITION=v6_final flock -w 1800 "$LOCK" python3 scripts/run_with_launch.py --timeout 900 --log "$RUN/panda-can-gazebo.launch.log" \
  --launch "robot_model:=asset_franka_panda_panda simulator:=gazebo mode:=display map_name:=arm_pick_place_can gui:=false perception:=none" \
  -- python3 scripts/arm_pick_place_check.py --robot asset_franka_panda_panda --object pickup_can --support place_table > "$RUN/panda-can-gazebo.json" 2> "$RUN/panda-can-gazebo.err"
echo "panda-can-gazebo exit=$?" >> $RUN/progress.txt; echo ALLDONE >> $RUN/progress.txt
