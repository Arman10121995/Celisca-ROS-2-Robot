#!/bin/bash
# V6 diagnosis: HEAD with only the IK rotation error reverted to the pre-f7f476cf cross-product form; TIAGo RGB-D fetch
# on MuJoCo; then restore arm_pick_place.py from HEAD and rebuild.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v6-manipulation-20261122/bisect
DIR=$(cd "$(dirname "$0")" && pwd); cd "$HERE"; F=src/robot_lab_mujoco/python/robot_lab_mujoco/arm_pick_place.py
source scripts/ssd_env.sh >/dev/null; source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=117; LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"
restore() { git checkout HEAD -- $F; colcon build --symlink-install --packages-select robot_lab_mujoco > "$RUN/rotvec-restore-build.log" 2>&1; }
trap restore EXIT
sed -i 's|error_rotation = Rotation.from_matrix(target_rotation @ tcp_rotation.T).as_rotvec()|error_rotation = 0.5 * sum(np.cross(tcp_rotation[:, k], target_rotation[:, k]) for k in range(3))|' $F
grep -q "0.5 \* sum(np.cross(tcp_rotation" $F || exit 3
colcon build --symlink-install --packages-select robot_lab_mujoco > "$RUN/rotvec-build.log" 2>&1 || exit 4
source install/setup.bash
flock -w 1800 "$LOCK" python3 scripts/run_with_launch.py --timeout 1200 --log "$RUN/rotvec-cross.launch.log" \
  --launch "robot_model:=asset_pal_tiago simulator:=mujoco mode:=display map_name:=mobile_fetch_lab allow_experimental_assets:=true spawn_x:=-1.0 spawn_y:=-1.0" \
  -- python3 scripts/arm_pick_place_check.py --robot asset_pal_tiago --object fetch_block --support drop_table --pose-source rgbd > "$RUN/rotvec-cross.json" 2> "$RUN/rotvec-cross.err"
echo "rotvec-cross exit=$?" >> "$DIR/bisect-progress.txt"
