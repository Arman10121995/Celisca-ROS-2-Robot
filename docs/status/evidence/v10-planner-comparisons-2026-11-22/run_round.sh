#!/bin/bash
# V10: the R7.6 (global) and R7.7 (local) comparisons re-run at one revision, one trial per lock hold.
# Global: Dijkstra, A*, PRM, RRT, RRT* with local_planning:=pure_pursuit (Nav2 RPP, as in October); local: the five October local methods with A*.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v10-planner-cmp-20261122
source "$HERE/scripts/ssd_env.sh" >/dev/null; export ROS_DOMAIN_ID=117 GOALS=3 ROBOT_RADIUS=0.12
LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"; DIR=$(cd "$(dirname "$0")" && pwd)
trial() { # category dir planner seed fixed-arg
  flock -w 1800 "$LOCK" "$HERE/scripts/planner_comparison.sh" "$2" gazebo bumperbot nav_maze "$1" "$3" "$4" "$5" > /dev/null 2>&1
  echo "$1 $3 $4 exit=$?" >> "$DIR/progress.txt"; }
for seed in 11 23 37 41 53; do
  for p in dijkstra_planner a_star_planner prm_planner rrt_planner rrt_star_planner; do trial global_planning $RUN/global $p $seed local_planning:=pure_pursuit; done
done
for seed in 11 23 37 41 53; do
  for p in pure_pursuit dwb_local_planner mppi_controller regulated_pure_pursuit follow_the_gap; do trial local_planning $RUN/local $p $seed global_planning:=a_star_planner; done
done
echo ALLDONE >> "$DIR/progress.txt"
