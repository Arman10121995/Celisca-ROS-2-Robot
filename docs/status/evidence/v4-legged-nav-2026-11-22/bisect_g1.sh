#!/bin/bash
# V4 diagnosis: G1 leg-odometry screen (MuJoCo, nav_obstacle) with the G1 gait-path files taken from <rev>,
# then restore them from HEAD and rebuild. Pass = leg drift under 5 % of the distance walked.
# usage: bisect_g1.sh <rev> <tag>
REV=$1; TAG=$2; HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v4-legged-nav-20261122/bisect-g1
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"; cd "$HERE"
FILES="src/robot_lab_adapter/robot_lab_adapter/legged_policy.py src/robot_lab_adapter/robot_lab_adapter/legged_policy_controller.py src/robot_lab_mujoco/python/robot_lab_mujoco/mujoco_spawner.py src/robot_lab_mujoco/python/robot_lab_mujoco/joint_pd.py src/robot_lab_robots/config/robots.yaml src/robot_lab_bringup/launch/simulated_robot.launch.py"
source scripts/ssd_env.sh >/dev/null; source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=117; LOCK=/workspace/molar/robot_lab_runtime/physics-qualification.lock
restore() { git checkout HEAD -- $FILES; colcon build --symlink-install --packages-select robot_lab_mujoco robot_lab_adapter robot_lab_robots robot_lab_bringup > "$RUN/$TAG.restore-build.log" 2>&1; }
trap restore EXIT
git checkout "$REV" -- $FILES && colcon build --symlink-install --packages-select robot_lab_mujoco robot_lab_adapter robot_lab_robots robot_lab_bringup > "$RUN/$TAG.build.log" 2>&1 || { echo "$TAG $REV build-failed" >> "$DIR/bisect-g1-progress.txt"; exit 3; }
source install/setup.bash
flock -w 1800 "$LOCK" timeout --signal=INT 1200 scripts/leg_odometry_check.sh mujoco unitree_g1_29dof nav_obstacle "$RUN/$TAG.json" > "$RUN/$TAG.run.log" 2>&1
drift=$(python3 -c "import json;print(json.load(open('$RUN/$TAG.json')).get('leg_drift_fraction'))" 2>/dev/null)
echo "$TAG $REV drift=$drift" >> "$DIR/bisect-g1-progress.txt"
