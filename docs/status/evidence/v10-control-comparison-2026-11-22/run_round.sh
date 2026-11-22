#!/bin/bash
# V10: the R7.8 Gazebo control comparison (October 13 protocol) re-run at one revision, one trial per lock hold.
# Six FollowPath trackers with global_planning:=a_star_planner local_planning:=pure_pursuit, seeds interleaved.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v10-control-cmp-20261122
source "$HERE/scripts/ssd_env.sh" >/dev/null; export ROS_DOMAIN_ID=117 GOALS=3 ROBOT_RADIUS=0.12
LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"; DIR=$(cd "$(dirname "$0")" && pwd)
for seed in 11 23 37 41 53; do
  for p in pid_controller lqr_controller mpc_controller nonlinear_mpc feedback_linearization backstepping_controller; do
    flock -w 1800 "$LOCK" "$HERE/scripts/planner_comparison.sh" "$RUN" gazebo bumperbot nav_maze control "$p" "$seed" \
      global_planning:=a_star_planner local_planning:=pure_pursuit > /dev/null 2>&1
    echo "control $p $seed exit=$?" >> "$DIR/progress.txt"
  done
done
echo ALLDONE >> "$DIR/progress.txt"
