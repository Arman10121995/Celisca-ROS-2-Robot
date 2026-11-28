#!/bin/bash
# The cells that failed goal B near an obstacle, rerun with the along-obstacle goal heading for unscored headings
# (sim_nav_mission.py, 2026-11-28), each with its original arguments (V3 car protocol; V4 legged protocol).
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v4-tight-goal-heading-20261128
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"; export ROS_DOMAIN_ID=117
LOCK=/workspace/molar/robot_lab_runtime/physics-qualification.lock
cell() { # name sim robot check-args
  local out="$RUN/$1.json"
  [ -s "$out" ] || CHECK_ARGS="$4" flock -w 3600 "$LOCK" timeout --signal=INT 1800 "$HERE/scripts/sim_nav_mission.sh" $2 $3 nav_obstacle "$out" > "$out.run.log" 2>&1
  echo "$1 exit=$?" >> "$DIR/progress.txt"; sleep 3; }
cell isaac_ackermann_car_seed1 isaac ackermann_car "--max-yaw-error-deg 0"
cell mujoco_unitree_go2_seed2 mujoco unitree_go2 "--base base --max-yaw-error-deg 0 --seed 2"
cell isaac_unitree_go2_seed3 isaac unitree_go2 "--base base --max-yaw-error-deg 0 --seed 3"
cell gazebo_unitree_go2_seed1 gazebo unitree_go2 "--base base --max-yaw-error-deg 0 --seed 1"
for s in 1 2 3; do cell mujoco_unitree_b2_seed$s mujoco unitree_b2 "--base base --max-yaw-error-deg 0 --seed $s"; done
echo ALLDONE >> "$DIR/progress.txt"
