#!/bin/bash
# The 10-13 4WS/mecanum nav_obstacle mission (sim_nav_mission.sh, --robot-radius 0.35) over the four backends.
DIR=$1; HERE=$(cd "$(dirname "$0")/../../../../scripts" && pwd)
export CHECK_ARGS="--robot-radius 0.35"
for sim in mujoco pybullet gazebo isaac; do
  for pattern in ackermann in_phase crab pivot mecanum; do
    out="$DIR/${sim}_${pattern}_nav_obstacle.json"; [ -s "$out" ] && continue
    if [ $pattern = mecanum ]; then
      timeout 1800 "$HERE/sim_nav_mission.sh" $sim mecanum_car nav_obstacle "$out"
    else
      timeout 1800 "$HERE/sim_nav_mission.sh" $sim four_wheel_steer_car nav_obstacle "$out" steering_mode:=$pattern
    fi
    echo "$sim $pattern exit=$?" >> "$DIR/progress.txt"; sleep 3
  done
done
echo ALLDONE >> "$DIR/progress.txt"
