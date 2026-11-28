#!/bin/bash
# V10: MPPI with a shorter Arrival hand-over (10 s, then 20 s; navigation.launch.py trial, reverted), on the
# 11-22 R7.7 protocol (Bumperbot, Gazebo nav_maze, global_planning:=a_star_planner, seeds 11-53, 3 goals each).
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v10-mppi-arrival-20261128
source "$HERE/scripts/ssd_env.sh" >/dev/null; export ROS_DOMAIN_ID=117 GOALS=3 ROBOT_RADIUS=0.12
LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"; DIR=$(cd "$(dirname "$0")" && pwd)
for seed in 11 23 37 41 53; do
  flock -w 1800 "$LOCK" "$HERE/scripts/planner_comparison.sh" "$RUN" gazebo bumperbot nav_maze local_planning mppi_controller "$seed" \
    global_planning:=a_star_planner > /dev/null 2>&1
  echo "mppi_controller $seed exit=$?" >> "$DIR/progress.txt"
done
echo ALLDONE >> "$DIR/progress.txt"
