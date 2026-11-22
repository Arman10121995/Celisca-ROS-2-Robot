#!/bin/bash
# V4 diagnosis: HEAD with only the G1 rl_sar policy config taken from before 1c665db4 (rl_kp/rl_kd changed there),
# G1 leg-odometry screen on MuJoCo nav_obstacle; then restore the config from HEAD and rebuild.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v4-legged-nav-20261122/bisect-g1
F=src/robot_lab_adapter/policies/rl_sar/g1_robomimic_locomotion/config.yaml; cd "$HERE"
source scripts/ssd_env.sh >/dev/null; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=117
restore() { git checkout HEAD -- $F; colcon build --symlink-install --packages-select robot_lab_adapter > "$RUN/gains-restore-build.log" 2>&1; }
trap restore EXIT
git checkout 1c665db4~1 -- $F && colcon build --symlink-install --packages-select robot_lab_adapter > "$RUN/gains-build.log" 2>&1 || exit 3
source install/setup.bash
flock -w 1800 /workspace/molar/robot_lab_runtime/physics-qualification.lock timeout --signal=INT 1200 scripts/leg_odometry_check.sh mujoco unitree_g1_29dof nav_obstacle "$RUN/october-gains.json" > "$RUN/october-gains.run.log" 2>&1
echo done > "$RUN/october-gains.done"
