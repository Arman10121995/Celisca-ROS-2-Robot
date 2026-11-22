#!/bin/bash
# After the nearest-branch IK rule: TIAGo RGB-D fetch on MuJoCo, then the R14.3 Isaac F100 fetch.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v6-manipulation-20261122/nearest-branch; mkdir -p $RUN
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"; cd "$HERE"
export ROS_DOMAIN_ID=117; LOCK=/workspace/molar/robot_lab_runtime/physics-qualification.lock
flock -w 1800 "$LOCK" python3 scripts/run_with_launch.py --timeout 1200 --log "$RUN/tiago-mujoco.launch.log" \
  --launch "robot_model:=asset_pal_tiago simulator:=mujoco mode:=display map_name:=mobile_fetch_lab allow_experimental_assets:=true spawn_x:=-1.0 spawn_y:=-1.0" \
  -- python3 scripts/arm_pick_place_check.py --robot asset_pal_tiago --object fetch_block --support drop_table --pose-source rgbd > "$RUN/tiago-mujoco.json" 2> "$RUN/tiago-mujoco.err"
echo "tiago-mujoco exit=$?" >> $RUN/progress.txt
/workspace/molar/robot_lab_runtime/pro630-gripper-20261120/claude-isaac-align-run.sh
echo "f100-isaac done" >> $RUN/progress.txt
