#!/bin/bash
# V6 diagnosis: run the TIAGo RGB-D fetch on MuJoCo with urdf_pick_place.py and arm_pick_place.py taken from <rev>,
# then restore both files from HEAD and rebuild. usage: bisect_tiago.sh <rev> <tag>
REV=$1; TAG=$2; HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v6-manipulation-20261122/bisect
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"; cd "$HERE"
FILES="src/robot_lab_bringup/scripts/urdf_pick_place.py src/robot_lab_mujoco/python/robot_lab_mujoco/arm_pick_place.py"
source scripts/ssd_env.sh >/dev/null; source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=117; LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"
restore() { git checkout HEAD -- $FILES; colcon build --symlink-install --packages-select robot_lab_mujoco > "$RUN/$TAG.restore-build.log" 2>&1; }
trap restore EXIT
git checkout "$REV" -- $FILES && colcon build --symlink-install --packages-select robot_lab_mujoco > "$RUN/$TAG.build.log" 2>&1 || exit 3
source install/setup.bash
flock -w 1800 "$LOCK" python3 scripts/run_with_launch.py --timeout 1200 --log "$RUN/$TAG.launch.log" \
  --launch "robot_model:=asset_pal_tiago simulator:=mujoco mode:=display map_name:=mobile_fetch_lab allow_experimental_assets:=true spawn_x:=-1.0 spawn_y:=-1.0" \
  -- python3 scripts/arm_pick_place_check.py --robot asset_pal_tiago --object fetch_block --support drop_table --pose-source rgbd > "$RUN/$TAG.json" 2> "$RUN/$TAG.err"
echo "$TAG $REV exit=$?" >> "$DIR/bisect-progress.txt"
